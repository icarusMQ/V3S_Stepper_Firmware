/**
 * @file    robot_link.h
 * @brief   V3S Link, robot side: discovery, exclusive controller lease,
 *          command authorization, telemetry. Spec: docs/protocol.md.
 *
 * Two transports feed the same handlers:
 *   - ESP-NOW: commands are accepted only from the controller MAC that holds
 *     the lease (CONNECT). The lease expires if that controller goes quiet.
 *   - USB serial: a physical connection, always authorized.
 */
#pragma once
#include <stdint.h>
#include "controllers/communication/communication.h"
#include "modules/espnow/espnow_comm.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

namespace v3s {

constexpr uint8_t  LINK_PROTO_VERSION  = 1;
constexpr uint8_t  FW_VERSION_MAJOR    = 1;
constexpr uint8_t  FW_VERSION_MINOR    = 1;
constexpr uint16_t LEASE_DEFAULT_MS    = 1000;
constexpr uint16_t LEASE_MIN_MS        = 200;
constexpr uint16_t LEASE_MAX_MS        = 5000;
constexpr uint32_t SERIAL_SESSION_MS   = 3000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;      // 20 Hz

enum LockState : uint8_t { LOCK_FREE = 0, LOCK_YOU = 1, LOCK_OTHER = 2 };
enum Status    : uint8_t { ST_OK = 0, ST_BUSY = 1, ST_BAD_REQUEST = 2, ST_NOT_OWNER = 3 };
enum Caps      : uint16_t { CAP_DIFF_DRIVE = 1 << 0, CAP_STEPPER = 1 << 1, CAP_OLED = 1 << 2,
                            CAP_SET_CHANNEL = 1 << 3, CAP_SERIAL_LINK = 1 << 4 };

struct __attribute__((packed)) RobotInfo {      // DISCOVER response (32 B)
    uint8_t  info_version;
    uint8_t  robot_id;
    uint8_t  mac[6];
    uint8_t  channel;
    uint8_t  lock_state;                        // relative to the requester
    uint8_t  owner_mac[6];
    uint8_t  fw_major, fw_minor;
    uint16_t capabilities;
    char     name[12];
};
struct __attribute__((packed)) ConnectReq  { uint32_t session; uint16_t lease_ms; };
struct __attribute__((packed)) ConnectResp {    // also DISCONNECT response
    uint8_t  status;
    uint8_t  robot_id;
    uint32_t session;
    uint16_t lease_ms;
    uint8_t  owner_mac[6];
};
struct __attribute__((packed)) Telemetry {      // 32 B, streamed to the lease holder
    uint8_t  version;
    uint8_t  robot_id;
    uint32_t uptime_ms;
    float    rpm_l, rpm_r;                      // measured / commanded wheel RPM
    float    cmd_l, cmd_r;                      // last accepted normalized command
    uint8_t  torque;
    uint8_t  link_flags;                        // bit0 ESP-NOW lease, bit1 serial session
    uint16_t rx_cmds;                           // accepted motion commands
    uint16_t rx_rejected;                       // dropped: not owner / stale seq
    uint16_t tx_fail;                           // ESP-NOW MAC-layer failures
    uint16_t cmd_age_ms;                        // since last accepted motion command
};

struct RobotHooks {
    void (*drive)(float left, float right);     // normalized [-1, 1]
    void (*stop)();
    void (*state)(float &rpm_l, float &rpm_r, bool &torque);
};

class RobotLink {
  public:
    /** Loads id/channel from NVS. Call before creating the transports. */
    static uint8_t stored_channel(uint8_t fallback);
    static uint8_t stored_id();

    void begin(ProtocolComm *espnow, ESPNowComm *espnow_base,
               ProtocolComm *serial, const RobotHooks &hooks);

    uint8_t id()      const { return robot_id; }
    uint8_t channel() const;
    bool    has_owner();

  private:
    struct Ctx { RobotLink *link; ProtocolComm *comm; };
    struct PendingDiscover {
        volatile bool    busy;
        uint16_t         seq;
        uint8_t          mac[6];
        esp_timer_handle_t timer;
        RobotLink       *link;
    };

    static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data);
    static void discover_timer_cb(void *arg);
    static void channel_timer_cb(void *arg);
    static void telemetry_task(void *arg);

    void handle(ProtocolComm *comm, ProtocolParsed_t *msg);
    bool from_serial(const ProtocolParsed_t *m) const;
    bool is_owner(const ProtocolParsed_t *m);
    bool authorized(const ProtocolParsed_t *m);
    bool fresh_seq(const ProtocolParsed_t *m);
    void fill_info(RobotInfo &info, const uint8_t *requester_mac);
    void reply(ProtocolComm *comm, const ProtocolParsed_t *req, const void *p, size_t n);
    void release_owner(const char *why);
    void persist_id(uint8_t id);
    void persist_channel(uint8_t ch);

    ProtocolComm *espnow = nullptr;
    ProtocolComm *serial = nullptr;
    ESPNowComm   *espnow_base = nullptr;
    RobotHooks    hooks = {};
    Ctx           ctx_espnow = {}, ctx_serial = {};

    uint8_t  robot_id = 1;
    uint8_t  self_mac[6] = {};

    // ESP-NOW lease
    SemaphoreHandle_t lock = nullptr;
    bool     owner_set = false;
    uint8_t  owner_mac[6] = {};
    uint32_t owner_session = 0;
    uint16_t lease_ms = LEASE_DEFAULT_MS;
    uint32_t owner_last_ms = 0;
    uint16_t last_seq_espnow = 0;  bool seq_valid_espnow = false;

    // Serial session (host sent CONNECT over USB)
    bool     serial_session = false;
    uint32_t serial_last_ms = 0;
    uint16_t last_seq_serial = 0;  bool seq_valid_serial = false;

    // Stats for telemetry
    float    last_cmd_l = 0, last_cmd_r = 0;
    uint32_t last_cmd_ms = 0;
    uint16_t rx_cmds = 0, rx_rejected = 0;

    PendingDiscover pending[4] = {};
    esp_timer_handle_t channel_timer = nullptr;
    volatile uint8_t pending_channel = 0;
};

}  // namespace v3s
