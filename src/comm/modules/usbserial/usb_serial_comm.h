/**
 * @file    usb_serial_comm.h
 * @brief   CommBase transport over the ESP32-C3 USB CDC port (Arduino `Serial`).
 *
 * Protocol packets are COBS-encoded and wrapped in 0x00 delimiters:
 *     0x00 | COBS(packet) | 0x00
 * Debug text printed with DEBUG_SERIAL never contains 0x00, so the host can
 * split the stream on 0x00 and tell frames (valid COBS + CRC) from text.
 * Writes take serialDebuggerMutex so frames never interleave with debug lines.
 */
#pragma once
#include "interfaces/comm_base.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class UsbSerialComm : public CommBase {
  public:
    explicit UsbSerialComm(const char *description = "USB serial")
        : CommBase(description) {}
    ~UsbSerialComm() { deinit(); }

    CommRet_t init()   override;
    CommRet_t deinit() override;
    CommRet_t start()  override { set_state(COMM_STATE_RUNNING); return COMM_RET_OK; }
    CommRet_t stop()   override { set_state(COMM_STATE_STOPPED); return COMM_RET_OK; }

    CommRet_t send(const CommMessage_t &msg, uint32_t timeout_ms = 0) override;
    CommRet_t send_to(const CommMessage_t &msg, const CommAddr_t &addr,
                      uint32_t timeout_ms = 0) override;
    CommRet_t receive(CommMessage_t &out_msg, uint32_t timeout_ms = 0) override;
    size_t    available() const override;

    uint32_t  rx_frames()     const { return rx_ok; }
    uint32_t  rx_bad_frames() const { return rx_bad; }

  private:
    static void rx_task(void *arg);
    void handle_frame(const uint8_t *enc, size_t len);

    QueueHandle_t rx_queue = nullptr;
    TaskHandle_t  task     = nullptr;
    volatile uint32_t rx_ok = 0, rx_bad = 0;
};
