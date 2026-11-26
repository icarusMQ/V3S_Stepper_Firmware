/**
 * @file    espnow_comm.cpp
 * @brief   Implementation of CommBase over ESP-NOW.
 */

#include "espnow_comm.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include <string.h>

ESPNowComm *ESPNowComm::s_instance = nullptr;


ESPNowComm::ESPNowComm(const char *description, espnow_comm_config_t *config)
    : CommBase(description), cfg(config), rx_Queue(nullptr) {}


ESPNowComm::~ESPNowComm() {
    this->deinit();
}


CommRet_t ESPNowComm::init() {
    if (!cfg) {
        ESP_LOGE("ESPNOW INIT", "Configuracao nao fornecida");
        set_last_error(COMM_RET_INVALID_ARG);
        return COMM_RET_INVALID_ARG;
    }
    s_instance = this;

    rx_Queue = xQueueCreate(20, sizeof(CommMessage_t));
    if (!rx_Queue) {
        ESP_LOGE("ESPNOW INIT", "Falha ao criar rx_Queue");
        set_last_error(COMM_RET_NO_MEMORY);
        return COMM_RET_NO_MEMORY;
    }

    // NVS - required by the WiFi driver even though we do not associate.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW("ESPNOW INIT", "nvs_flash_init: %s (continuando)",
            esp_err_to_name(ret));
    }

    // WiFi PHY init. We never associate to an AP - ESP-NOW only needs the MAC
    // layer up in STA mode.
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ESPNOW INIT", "esp_netif_init: %s", esp_err_to_name(ret));
        set_last_error(COMM_RET_IO_ERROR);
        return COMM_RET_IO_ERROR;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ESPNOW INIT", "esp_event_loop_create_default: %s",
            esp_err_to_name(ret));
        set_last_error(COMM_RET_IO_ERROR);
        return COMM_RET_IO_ERROR;
    }

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&wcfg);
    if (ret != ESP_OK) {
        ESP_LOGE("ESPNOW INIT", "esp_wifi_init: %s", esp_err_to_name(ret));
        set_last_error(COMM_RET_IO_ERROR);
        return COMM_RET_IO_ERROR;
    }

    if (!peer_mutex) peer_mutex = xSemaphoreCreateMutex();

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    // Allow channels 1..13 regardless of the default country policy.
    wifi_country_t country = { .cc = "BR", .schan = 1, .nchan = 13,
                               .max_tx_power = 20, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);
    esp_wifi_start();
    // No modem sleep: the radio must always be listening (latency + no lost frames).
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(cfg->tx_power ? cfg->tx_power : 80);

    // Fix the channel (no HT40). Must match the controller / bridge.
    ret = esp_wifi_set_channel(cfg->channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW("ESPNOW INIT", "esp_wifi_set_channel(%u): %s",
            (unsigned)cfg->channel, esp_err_to_name(ret));
    }

    // PHY rate for ESP-NOW frames. Higher rate = shorter airtime = fewer
    // collisions when many robots share a channel (receivers accept any rate).
    if (cfg->phy_rate) {
        ret = esp_wifi_config_espnow_rate(WIFI_IF_STA, (wifi_phy_rate_t)cfg->phy_rate);
        if (ret != ESP_OK) {
            ESP_LOGW("ESPNOW INIT", "esp_wifi_config_espnow_rate(%u): %s",
                (unsigned)cfg->phy_rate, esp_err_to_name(ret));
        }
    }

    // ESP-NOW init + callbacks
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE("ESPNOW INIT", "esp_now_init falhou");
        set_last_error(COMM_RET_IO_ERROR);
        return COMM_RET_IO_ERROR;
    }

    esp_now_register_recv_cb(&ESPNowComm::on_recv);
    esp_now_register_send_cb(&ESPNowComm::on_send);

    // Add the peer (broadcast or unicast). For broadcast peer, encryption
    // must be false.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, cfg->peer_mac, 6);
    peer.channel = 0;   // 0 = follow the current channel (so set_channel() just works)
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t pe = esp_now_add_peer(&peer);
    if (pe != ESP_OK && pe != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW("ESPNOW INIT", "esp_now_add_peer: %s", esp_err_to_name(pe));
    }

    // Log own MAC for the gateway side.
    uint8_t self_mac[6] = {0};
    esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
    ESP_LOGI("ESPNOW INIT",
        "Self MAC %02X:%02X:%02X:%02X:%02X:%02X channel=%u peer=%02X:%02X:%02X:%02X:%02X:%02X",
        self_mac[0], self_mac[1], self_mac[2], self_mac[3], self_mac[4], self_mac[5],
        (unsigned)cfg->channel,
        cfg->peer_mac[0], cfg->peer_mac[1], cfg->peer_mac[2],
        cfg->peer_mac[3], cfg->peer_mac[4], cfg->peer_mac[5]);

    set_state(COMM_STATE_IDLE);
    connected = true;  // ESP-NOW is connectionless; treat as always "up"

    if (cfg->auto_start) {
        return start();
    }
    set_last_error(COMM_RET_OK);
    return COMM_RET_OK;
}


CommRet_t ESPNowComm::start() {
    set_state(COMM_STATE_RUNNING);
    set_last_error(COMM_RET_OK);
    return COMM_RET_OK;
}


CommRet_t ESPNowComm::stop() {
    set_state(COMM_STATE_STOPPED);
    return COMM_RET_OK;
}


CommRet_t ESPNowComm::deinit() {
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
    if (rx_Queue) {
        vQueueDelete(rx_Queue);
        rx_Queue = nullptr;
    }
    if (s_instance == this) {
        s_instance = nullptr;
    }
    set_state(COMM_STATE_UNINITIALIZED);
    return COMM_RET_OK;
}


CommRet_t ESPNowComm::send(const CommMessage_t &msg, uint32_t timeout_ms) {
    return send_to(msg, msg.dest, timeout_ms);
}


CommRet_t ESPNowComm::send_to(const CommMessage_t &msg,
                              const CommAddr_t &addr,
                              uint32_t timeout_ms)
{
    (void)timeout_ms;  // ESP-NOW send is fire-and-forget at this layer

    if (msg.length == 0 || msg.length > ESP_NOW_MAX_DATA_LEN) {
        set_last_error(COMM_RET_INVALID_ARG);
        return COMM_RET_INVALID_ARG;
    }

    // Resolve destination MAC: explicit ESP_NOW address if provided, else
    // fall back to the configured peer (broadcast in our default setup).
    const uint8_t *target_mac;
    if (addr.type == COMM_TYPE_ESP_NOW && addr.len == 6) {
        target_mac = addr.data;
    } else {
        target_mac = cfg->peer_mac;
    }

    // Unicast needs the peer registered (MAC-layer ACK + hardware retries).
    static const uint8_t bcast[6] = ESPNOW_BROADCAST_MAC;
    if (memcmp(target_mac, bcast, 6) != 0 && !ensure_peer(target_mac)) {
        tx_fail++;
        set_last_error(COMM_RET_IO_ERROR);
        return COMM_RET_IO_ERROR;
    }

    esp_err_t e = esp_now_send(target_mac, msg.payload, msg.length);
    if (e != ESP_OK) {
        tx_fail++;
        set_last_error(COMM_RET_IO_ERROR);
        return COMM_RET_IO_ERROR;
    }
    msg_sent++;
    return COMM_RET_OK;
}


bool ESPNowComm::ensure_peer(const uint8_t mac[6]) {
    if (!peer_mutex) return false;
    xSemaphoreTake(peer_mutex, portMAX_DELAY);
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    int free_slot = -1, lru_slot = 0;
    for (int i = 0; i < ESPNOW_MAX_UNICAST_PEERS; i++) {
        if (peer_used[i] && memcmp(peer_mac[i], mac, 6) == 0) {
            peer_last_use[i] = now;
            xSemaphoreGive(peer_mutex);
            return true;
        }
        if (!peer_used[i] && free_slot < 0) free_slot = i;
        if (peer_used[i] && (now - peer_last_use[i]) > (now - peer_last_use[lru_slot])) lru_slot = i;
    }
    int slot = free_slot;
    if (slot < 0) {                       // evict least recently used
        slot = lru_slot;
        esp_now_del_peer(peer_mac[slot]);
        peer_used[slot] = false;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;                     // follow current channel
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t e = esp_now_add_peer(&peer);
    bool ok = (e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST);
    if (ok) {
        memcpy(peer_mac[slot], mac, 6);
        peer_last_use[slot] = now;
        peer_used[slot] = true;
    }
    xSemaphoreGive(peer_mutex);
    return ok;
}


CommRet_t ESPNowComm::set_channel(uint8_t channel) {
    if (!cfg || channel < 1 || channel > 13) return COMM_RET_INVALID_ARG;
    esp_err_t e = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (e != ESP_OK) {
        ESP_LOGW("ESPNOW", "set_channel(%u): %s", (unsigned)channel, esp_err_to_name(e));
        return COMM_RET_IO_ERROR;
    }
    cfg->channel = channel;
    return COMM_RET_OK;
}


void ESPNowComm::get_self_mac(uint8_t mac[6]) const {
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
}


CommRet_t ESPNowComm::receive(CommMessage_t &out_msg, uint32_t timeout_ms) {
    if (!rx_Queue) {
        set_last_error(COMM_RET_NOT_INITIALIZED);
        return COMM_RET_NOT_INITIALIZED;
    }
    // timeout_ms == 0 means "block forever" to match the existing parser_Task
    // semantics (it calls receive() with no timeout and expects to sleep until
    // a packet arrives).
    TickType_t to = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (xQueueReceive(rx_Queue, &out_msg, to) == pdTRUE) {
        msg_received++;
        return COMM_RET_OK;
    }
    return COMM_RET_TIMEOUT;
}


CommRet_t ESPNowComm::receive_from(CommMessage_t &out_msg,
                                   CommAddr_t &addr,
                                   uint32_t timeout)
{
    CommRet_t r = receive(out_msg, timeout);
    if (r == COMM_RET_OK) {
        addr = out_msg.src;
    }
    return r;
}


size_t ESPNowComm::available() const {
    if (!rx_Queue) return 0;
    return (size_t)uxQueueMessagesWaiting(rx_Queue);
}


// -----------------------------------------------------------------------------
// ESP-NOW callbacks - run in the WiFi task context. Keep them short: copy the
// bytes, enqueue, return. No printf, no malloc, no blocking calls.
// -----------------------------------------------------------------------------

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
void ESPNowComm::on_recv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len)
{
    const uint8_t *src_mac = info ? info->src_addr : nullptr;
#else
void ESPNowComm::on_recv(const uint8_t *src_mac,
                         const uint8_t *data, int len)
{
#endif
    ESPNowComm *self = s_instance;
    if (!self || !self->rx_Queue || !data || len <= 0) {
        return;
    }
    if ((size_t)len > COMM_MAX_PAYLOAD) {
        // Oversized for our protocol packet buffer - drop.
        return;
    }

    CommMessage_t msg = {};
    msg.type   = COMM_MSG_TYPE_RAW;
    msg.length = (uint32_t)len;
    memcpy(msg.payload, data, (size_t)len);
    if (src_mac) {
        msg.src.type = COMM_TYPE_ESP_NOW;
        msg.src.len  = 6;
        memcpy(msg.src.data, src_mac, 6);
    }
    // Best-effort enqueue. If the queue is full we drop the packet rather than
    // block the radio task - upper layer / retransmit logic handles loss.
    if (xQueueSend(self->rx_Queue, &msg, 0) != pdTRUE) {
        self->rx_drop++;
    }
}


void ESPNowComm::on_send(const uint8_t *mac_addr,
                         esp_now_send_status_t status)
{
    (void)mac_addr;
    ESPNowComm *self = s_instance;
    if (!self) return;
    // For unicast this reflects the MAC-layer ACK after hardware retries.
    if (status == ESP_NOW_SEND_SUCCESS) self->tx_ok++;
    else                                self->tx_fail++;
}
