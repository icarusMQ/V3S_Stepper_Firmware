#include "link/robot_link.h"
#include "board_config.h"
#include "Serial/SerialDebugger.h"
#include <Arduino.h>
#include <Preferences.h>
#include "esp_mac.h"
#include "esp_random.h"
#include <string.h>

namespace v3s {

static const char *NVS_NS = "v3s";

static inline uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

static CommAddr_t espnow_addr(const uint8_t mac[6]) {
    CommAddr_t a = {};
    a.type = COMM_TYPE_ESP_NOW;
    a.len  = 6;
    memcpy(a.data, mac, 6);
    return a;
}

static CommAddr_t serial_addr() {
    CommAddr_t a = {};
    a.type = COMM_TYPE_USB_SERIAL;
    return a;
}

// ---------------------------------------------------------------- persistence

uint8_t RobotLink::stored_id() {
    Preferences p;
    uint8_t id = 0;
    if (p.begin(NVS_NS, true)) { id = p.getUChar("id", 0); p.end(); }
    if (id == 0 || id == PROTO_ID_BROADCAST) {
        // Default: derived from the MAC so robots out of the box rarely collide.
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        id = (uint8_t)((mac[5] % 0xFD) + 1);          // 1..0xFD
    }
    return id;
}

uint8_t RobotLink::stored_channel(uint8_t fallback) {
    Preferences p;
    uint8_t ch = fallback;
    if (p.begin(NVS_NS, true)) { ch = p.getUChar("ch", fallback); p.end(); }
    return (ch >= 1 && ch <= 13) ? ch : fallback;
}

void RobotLink::persist_id(uint8_t id) {
    Preferences p;
    if (p.begin(NVS_NS, false)) { p.putUChar("id", id); p.end(); }
}

void RobotLink::persist_channel(uint8_t ch) {
    Preferences p;
    if (p.begin(NVS_NS, false)) { p.putUChar("ch", ch); p.end(); }
}

// ---------------------------------------------------------------- setup

void RobotLink::begin(ProtocolComm *espnow_, ESPNowComm *espnow_base_,
                      ProtocolComm *serial_, const RobotHooks &hooks_) {
    espnow = espnow_;
    espnow_base = espnow_base_;
    serial = serial_;
    hooks = hooks_;
    robot_id = stored_id();
    esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
    lock = xSemaphoreCreateMutex();

    for (auto &pd : pending) {
        pd.link = this;
        esp_timer_create_args_t a = {};
        a.callback = &RobotLink::discover_timer_cb;
        a.arg = &pd;
        a.name = "v3s_disc";
        esp_timer_create(&a, &pd.timer);
    }
    esp_timer_create_args_t ca = {};
    ca.callback = &RobotLink::channel_timer_cb;
    ca.arg = this;
    ca.name = "v3s_ch";
    esp_timer_create(&ca, &channel_timer);

    if (espnow) {
        espnow->set_id_device(robot_id);
        ctx_espnow = { this, espnow };
        espnow->register_event_loop_callback(&RobotLink::on_event, ESP_EVENT_ANY_ID, &ctx_espnow);
    }
    if (serial) {
        serial->set_id_device(robot_id);
        ctx_serial = { this, serial };
        serial->register_event_loop_callback(&RobotLink::on_event, ESP_EVENT_ANY_ID, &ctx_serial);
    }
    xTaskCreate(&RobotLink::telemetry_task, "V3SLinkTelem", 3072, this, 3, nullptr);

    DEBUG_SERIAL("LINK", "V3S Link v%u: id=0x%02X ch=%u mac=%02X:%02X:%02X:%02X:%02X:%02X",
        LINK_PROTO_VERSION, robot_id, channel(),
        self_mac[0], self_mac[1], self_mac[2], self_mac[3], self_mac[4], self_mac[5]);
}

uint8_t RobotLink::channel() const {
    return espnow_base ? espnow_base->get_channel() : 0;
}

// ---------------------------------------------------------------- helpers

bool RobotLink::from_serial(const ProtocolParsed_t *m) const {
    return m->src_addr.type == COMM_TYPE_USB_SERIAL;
}

bool RobotLink::is_owner(const ProtocolParsed_t *m) {
    return m->src_addr.type == COMM_TYPE_ESP_NOW && m->src_addr.len == 6 &&
           owner_set && memcmp(m->src_addr.data, owner_mac, 6) == 0;
}

bool RobotLink::has_owner() {
    return owner_set && (now_ms() - owner_last_ms) < lease_ms;
}

bool RobotLink::authorized(const ProtocolParsed_t *m) {
    if (from_serial(m)) return true;           // physical connection
    return is_owner(m) && has_owner();
}

// Drop duplicates (MAC-layer retries) and out-of-order motion commands.
bool RobotLink::fresh_seq(const ProtocolParsed_t *m) {
    bool ser = from_serial(m);
    uint16_t &last = ser ? last_seq_serial : last_seq_espnow;
    bool &valid    = ser ? seq_valid_serial : seq_valid_espnow;
    uint16_t seq = m->packet.header.seq;
    if (valid) {
        int16_t diff = (int16_t)(seq - last);
        if (diff <= 0 && diff > -1000) return false;   // big jump back = sender restarted
    }
    last = seq;
    valid = true;
    return true;
}

void RobotLink::fill_info(RobotInfo &info, const uint8_t *requester_mac) {
    memset(&info, 0, sizeof(info));
    info.info_version = LINK_PROTO_VERSION;
    info.robot_id = robot_id;
    memcpy(info.mac, self_mac, 6);
    info.channel = channel();
    if (has_owner()) {
        bool you = requester_mac && memcmp(requester_mac, owner_mac, 6) == 0;
        info.lock_state = you ? LOCK_YOU : LOCK_OTHER;
        memcpy(info.owner_mac, owner_mac, 6);
    } else {
        info.lock_state = LOCK_FREE;
    }
    info.fw_major = FW_VERSION_MAJOR;
    info.fw_minor = FW_VERSION_MINOR;
    info.capabilities = CAP_DIFF_DRIVE | CAP_STEPPER | CAP_OLED | CAP_SET_CHANNEL | CAP_SERIAL_LINK;
    snprintf(info.name, sizeof(info.name), "V3S-%02X", robot_id);
}

void RobotLink::reply(ProtocolComm *comm, const ProtocolParsed_t *req, const void *p, size_t n) {
    comm->send_packet(PROTO_FLAG_RESP, req->packet.header.cmd, req->packet.header.seq,
                      p, n, req->src_addr);
}

void RobotLink::release_owner(const char *why) {
    if (!owner_set) return;
    owner_set = false;
    seq_valid_espnow = false;
    if (hooks.stop) hooks.stop();
    DEBUG_SERIAL("LINK", "Controller released (%s)", why);
}

// ---------------------------------------------------------------- dispatch

void RobotLink::on_event(void *arg, esp_event_base_t, int32_t, void *data) {
    Ctx *ctx = static_cast<Ctx *>(arg);
    ProtocolParsed_t *msg = *(ProtocolParsed_t **)data;
    if (ctx && ctx->link && msg) ctx->link->handle(ctx->comm, msg);
}

void RobotLink::handle(ProtocolComm *comm, ProtocolParsed_t *m) {
    const uint32_t now = now_ms();
    const bool ser = from_serial(m);
    const uint8_t cmd = m->packet.header.cmd;
    const uint8_t *p  = m->packet.payload;
    const uint8_t n   = m->packet.header.payload_len;

    if (ser) serial_last_ms = now;
    else if (is_owner(m)) owner_last_ms = now;      // any traffic from the owner renews the lease

    switch (cmd) {
    case PROTO_CMD_DISCOVER: {
        if (ser) {
            RobotInfo info;
            fill_info(info, nullptr);
            reply(comm, m, &info, sizeof(info));
            break;
        }
        // Many robots answer the same broadcast: spread replies over ~20 ms so
        // they don't collide, and answer unicast (MAC ACK + retries).
        for (auto &pd : pending) {
            if (pd.busy) continue;
            pd.busy = true;
            pd.seq = m->packet.header.seq;
            memcpy(pd.mac, m->src_addr.data, 6);
            esp_timer_start_once(pd.timer, 300 + (esp_random() % 20000));
            break;
        }
        break;
    }

    case PROTO_CMD_CONNECT: {
        ConnectReq req = { 0, LEASE_DEFAULT_MS };
        if (n >= sizeof(ConnectReq)) memcpy(&req, p, sizeof(req));
        ConnectResp r = {};
        r.robot_id = robot_id;
        r.session = req.session;
        if (ser) {
            serial_session = true;
            seq_valid_serial = false;
            r.status = ST_OK;
            r.lease_ms = (uint16_t)SERIAL_SESSION_MS;
            DEBUG_SERIAL("LINK", "Serial host connected");
        } else {
            xSemaphoreTake(lock, portMAX_DELAY);
            bool same = owner_set && memcmp(owner_mac, m->src_addr.data, 6) == 0;
            if (!has_owner() || same) {
                if (!same) DEBUG_SERIAL("LINK", "Controller %02X:%02X:%02X:%02X:%02X:%02X connected",
                    m->src_addr.data[0], m->src_addr.data[1], m->src_addr.data[2],
                    m->src_addr.data[3], m->src_addr.data[4], m->src_addr.data[5]);
                owner_set = true;
                memcpy(owner_mac, m->src_addr.data, 6);
                owner_session = req.session;
                uint16_t l = req.lease_ms ? req.lease_ms : LEASE_DEFAULT_MS;
                lease_ms = l < LEASE_MIN_MS ? LEASE_MIN_MS : (l > LEASE_MAX_MS ? LEASE_MAX_MS : l);
                owner_last_ms = now;
                seq_valid_espnow = false;
                r.status = ST_OK;
            } else {
                r.status = ST_BUSY;
            }
            r.lease_ms = lease_ms;
            memcpy(r.owner_mac, owner_mac, 6);
            xSemaphoreGive(lock);
        }
        reply(comm, m, &r, sizeof(r));
        break;
    }

    case PROTO_CMD_DISCONNECT: {
        ConnectResp r = {};
        r.robot_id = robot_id;
        if (ser) {
            serial_session = false;
            if (hooks.stop) hooks.stop();
            r.status = ST_OK;
        } else if (is_owner(m)) {
            xSemaphoreTake(lock, portMAX_DELAY);
            release_owner("disconnect");
            xSemaphoreGive(lock);
            r.status = ST_OK;
        } else {
            r.status = ST_NOT_OWNER;
        }
        if (m->packet.header.flags & PROTO_FLAG_REQ_RESP) reply(comm, m, &r, sizeof(r));
        break;
    }

    case PROTO_CMD_HEARTBEAT:
        break;                                      // lease renewal handled above

    case PROTO_CMD_PING:
        if (m->packet.header.flags & PROTO_FLAG_REQ_RESP) reply(comm, m, p, n);   // echo
        break;

    case PROTO_CMD_GET_ID_DEVICE:
        reply(comm, m, &robot_id, 1);
        break;

    case PROTO_CMD_SET_ID_DEVICE: {
        uint8_t r[2] = { ST_NOT_OWNER, robot_id };
        if (authorized(m)) {
            if (n >= 1 && p[0] >= 1 && p[0] <= 0xFE) {
                robot_id = p[0];
                persist_id(robot_id);
                if (espnow) espnow->set_id_device(robot_id);
                if (serial) serial->set_id_device(robot_id);
                r[0] = ST_OK; r[1] = robot_id;
                DEBUG_SERIAL("LINK", "Robot id -> 0x%02X", robot_id);
            } else {
                r[0] = ST_BAD_REQUEST;
            }
        }
        reply(comm, m, r, sizeof(r));               // reply carries the new id in the header
        break;
    }

    case PROTO_CMD_SET_CHANNEL: {
        uint8_t r[2] = { ST_NOT_OWNER, channel() };
        if (authorized(m)) {
            if (n >= 1 && p[0] >= 1 && p[0] <= 13) {
                persist_channel(p[0]);
                pending_channel = p[0];
                r[0] = ST_OK; r[1] = p[0];
                esp_timer_stop(channel_timer);
                esp_timer_start_once(channel_timer, 30 * 1000);   // reply first, on the old channel
                DEBUG_SERIAL("LINK", "Channel -> %u", p[0]);
            } else {
                r[0] = ST_BAD_REQUEST;
            }
        }
        reply(comm, m, r, sizeof(r));
        break;
    }

    case PROTO_CMD_SET_TARGET_VEL: {
        if (n != sizeof(float) * 2) break;
        if (!authorized(m) || !fresh_seq(m)) { rx_rejected++; break; }
        float l, r;
        memcpy(&l, p, sizeof(float));
        memcpy(&r, p + sizeof(float), sizeof(float));
        last_cmd_l = l; last_cmd_r = r; last_cmd_ms = now;
        rx_cmds++;
        if (hooks.drive) hooks.drive(l, r);
        break;
    }

    case PROTO_CMD_STOP:
        if (!authorized(m)) { rx_rejected++; break; }
        last_cmd_l = last_cmd_r = 0; last_cmd_ms = now;
        if (hooks.stop) hooks.stop();
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------- timers / tasks

void RobotLink::discover_timer_cb(void *arg) {
    PendingDiscover *pd = static_cast<PendingDiscover *>(arg);
    RobotLink *self = pd->link;
    if (self && self->espnow) {
        RobotInfo info;
        self->fill_info(info, pd->mac);
        self->espnow->send_packet(PROTO_FLAG_RESP, PROTO_CMD_DISCOVER, pd->seq,
                                  &info, sizeof(info), espnow_addr(pd->mac));
    }
    pd->busy = false;
}

void RobotLink::channel_timer_cb(void *arg) {
    RobotLink *self = static_cast<RobotLink *>(arg);
    if (self->espnow_base && self->pending_channel) {
        self->espnow_base->set_channel(self->pending_channel);
    }
}

void RobotLink::telemetry_task(void *arg) {
    RobotLink *self = static_cast<RobotLink *>(arg);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        const uint32_t now = now_ms();

        xSemaphoreTake(self->lock, portMAX_DELAY);
        if (self->owner_set && (now - self->owner_last_ms) >= self->lease_ms) {
            self->release_owner("lease expired");
        }
        bool owner = self->owner_set;
        uint8_t mac[6];
        memcpy(mac, self->owner_mac, 6);
        xSemaphoreGive(self->lock);

        if (self->serial_session && (now - self->serial_last_ms) > SERIAL_SESSION_MS) {
            self->serial_session = false;
        }
        if (!owner && !self->serial_session) continue;

        Telemetry t = {};
        t.version = LINK_PROTO_VERSION;
        t.robot_id = self->robot_id;
        t.uptime_ms = now;
        bool torque = false;
        float rpm_l = 0, rpm_r = 0;
        if (self->hooks.state) self->hooks.state(rpm_l, rpm_r, torque);
        t.rpm_l = rpm_l;
        t.rpm_r = rpm_r;
        t.torque = torque;
        t.cmd_l = self->last_cmd_l;
        t.cmd_r = self->last_cmd_r;
        t.link_flags = (owner ? 1 : 0) | (self->serial_session ? 2 : 0);
        t.rx_cmds = self->rx_cmds;
        t.rx_rejected = self->rx_rejected;
        t.tx_fail = self->espnow_base ? (uint16_t)self->espnow_base->tx_fail_count() : 0;
        uint32_t age = now - self->last_cmd_ms;
        t.cmd_age_ms = age > 0xFFFF ? 0xFFFF : (uint16_t)age;

        if (owner && self->espnow) {
            self->espnow->send_packet(PROTO_FLAG_NOP, PROTO_CMD_TELEMETRY, self->espnow->next_seq(),
                                      &t, sizeof(t), espnow_addr(mac));
        }
        if (self->serial_session && self->serial) {
            self->serial->send_packet(PROTO_FLAG_NOP, PROTO_CMD_TELEMETRY, self->serial->next_seq(),
                                      &t, sizeof(t), serial_addr());
        }
    }
}

}  // namespace v3s
