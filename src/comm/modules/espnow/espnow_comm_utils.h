/**
 * @file    espnow_comm_utils.h
 * @brief   Auxiliary types for ESPNowComm (broadcast, fixed channel).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default broadcast MAC. Use this as peer_mac for broadcast operation.
#define ESPNOW_BROADCAST_MAC { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

typedef struct {
    uint8_t channel;       // 1..13, fixed. Must match peer.
    uint8_t peer_mac[6];   // FF:FF:FF:FF:FF:FF for broadcast.
    bool    auto_start;    // If true, init() automatically calls start().
    uint8_t phy_rate;      // wifi_phy_rate_t for ESP-NOW TX; 0 = WIFI_PHY_RATE_1M_L (IDF default)
    int8_t  tx_power;      // esp_wifi_set_max_tx_power units (0.25 dBm); 0 = 80 (20 dBm)
} espnow_comm_config_t;

// Max unicast peers kept registered at once (LRU). Broadcast peer is extra.
#define ESPNOW_MAX_UNICAST_PEERS 8

#ifdef __cplusplus
}
#endif
