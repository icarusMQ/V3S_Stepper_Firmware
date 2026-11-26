#include "usb_serial_comm.h"
#include "board_config.h"
#include "utils/cobs.h"
#include "freertos/semphr.h"
#include <Arduino.h>
#include <string.h>

extern SemaphoreHandle_t serialDebuggerMutex;   // shared with DEBUG_SERIAL

static constexpr size_t RX_ENC_MAX = COMM_MAX_PAYLOAD + 8;

CommRet_t UsbSerialComm::init() {
    if (rx_queue) return COMM_RET_OK;
    rx_queue = xQueueCreate(16, sizeof(CommMessage_t));
    if (!rx_queue) return COMM_RET_NO_MEMORY;
    // Priority above the motor task so host commands are parsed promptly.
    if (xTaskCreate(&UsbSerialComm::rx_task, "UsbSerialRx", 3072, this, 5, &task) != pdPASS) {
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
        return COMM_RET_NO_MEMORY;
    }
    connected = true;
    set_state(COMM_STATE_RUNNING);
    return COMM_RET_OK;
}

CommRet_t UsbSerialComm::deinit() {
    if (task)     { vTaskDelete(task); task = nullptr; }
    if (rx_queue) { vQueueDelete(rx_queue); rx_queue = nullptr; }
    set_state(COMM_STATE_UNINITIALIZED);
    return COMM_RET_OK;
}

void UsbSerialComm::rx_task(void *arg) {
    UsbSerialComm *self = static_cast<UsbSerialComm *>(arg);
    uint8_t buf[RX_ENC_MAX];
    size_t  len = 0;
    bool    overflow = false;
    for (;;) {
        int n = USB_BUS.available();
        if (n <= 0) {
            vTaskDelay(1);          // ~1 ms poll keeps latency low with negligible CPU
            continue;
        }
        while (n-- > 0) {
            int c = USB_BUS.read();
            if (c < 0) break;
            if (c == 0x00) {
                if (len > 0 && !overflow) self->handle_frame(buf, len);
                len = 0;
                overflow = false;
            } else if (len < sizeof(buf)) {
                buf[len++] = (uint8_t)c;
            } else {
                overflow = true;    // not a frame of ours (or garbage): skip to next 0x00
            }
        }
    }
}

void UsbSerialComm::handle_frame(const uint8_t *enc, size_t len) {
    CommMessage_t msg = {};
    size_t dec = cobs_decode(enc, len, msg.payload);
    if (dec == 0 || dec > COMM_MAX_PAYLOAD) { rx_bad++; return; }
    msg.type     = COMM_MSG_TYPE_RAW;
    msg.length   = (uint32_t)dec;
    msg.src.type = COMM_TYPE_USB_SERIAL;
    msg.src.len  = 0;
    if (xQueueSend(rx_queue, &msg, 0) == pdTRUE) rx_ok++;
}

CommRet_t UsbSerialComm::send(const CommMessage_t &msg, uint32_t timeout_ms) {
    return send_to(msg, msg.dest, timeout_ms);
}

CommRet_t UsbSerialComm::send_to(const CommMessage_t &msg, const CommAddr_t &addr,
                                 uint32_t timeout_ms) {
    (void)addr;
    if (msg.length == 0 || msg.length > COMM_MAX_PAYLOAD) return COMM_RET_INVALID_ARG;
    uint8_t out[COMM_MAX_PAYLOAD + 8];
    out[0] = 0x00;
    size_t n = cobs_encode(msg.payload, msg.length, out + 1);
    out[1 + n] = 0x00;
    TickType_t to = pdMS_TO_TICKS(timeout_ms ? timeout_ms : 20);
    if (serialDebuggerMutex && xSemaphoreTake(serialDebuggerMutex, to) != pdTRUE) {
        return COMM_RET_TIMEOUT;
    }
    size_t w = USB_BUS.write(out, n + 2);
    if (serialDebuggerMutex) xSemaphoreGive(serialDebuggerMutex);
    if (w != n + 2) return COMM_RET_IO_ERROR;
    msg_sent++;
    return COMM_RET_OK;
}

CommRet_t UsbSerialComm::receive(CommMessage_t &out_msg, uint32_t timeout_ms) {
    if (!rx_queue) return COMM_RET_NOT_INITIALIZED;
    TickType_t to = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (xQueueReceive(rx_queue, &out_msg, to) == pdTRUE) {
        msg_received++;
        return COMM_RET_OK;
    }
    return COMM_RET_TIMEOUT;
}

size_t UsbSerialComm::available() const {
    return rx_queue ? (size_t)uxQueueMessagesWaiting(rx_queue) : 0;
}
