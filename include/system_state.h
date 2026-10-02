#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @file system_state.h
 * @brief Thread-safe Global System State for MicroLink Node
 */

enum class WifiState : uint8_t
{
    DISCONNECTED = 0,
    CONNECTING,
    CONNECTED,
    AP_MODE,
    FAILED
};

enum class EchoLinkState : uint8_t
{
    DISCONNECTED = 0,
    CONNECTING_DIR,
    LOGGED_IN,
    AUTH_FAILED,
    ERROR
};

enum class StationState : uint8_t
{
    IDLE = 0,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
};

struct SystemState
{
    WifiState wifi_state;
    EchoLinkState echolink_state;
    StationState station_state;
    char connected_callsign[16];
    uint32_t connected_node;
    bool tx_active;         // True when transmitting (PTT active)
    bool rx_active;         // True when receiving audio from EchoLink
    uint16_t mic_raw_level; // Recent raw ADC sample
    int16_t mic_level_pct;  // Normalized mic volume (0 - 100%)
    uint16_t rx_raw_level;  // Recent RX PCM peak sample (0 - 32767)
    int16_t rx_level_pct;   // Normalized RX volume (0 - 100%)
    uint32_t ptt_press_count;
    uint32_t last_tx_duration_ms;
    size_t free_heap;
    uint32_t uptime_sec;
    uint32_t jitter_depth;
    uint32_t jitter_underflows;
    uint32_t jitter_overflows;
    bool loopback_active;
};

// State Manager API
void system_state_init();
bool system_state_lock(TickType_t wait_ticks = portMAX_DELAY);
void system_state_unlock();

// Thread-safe copy getter
SystemState system_state_get();

// Mutex-protected setters for common state changes
void system_state_set_ptt(bool active);
void system_state_set_rx(bool active);
void system_state_set_wifi(WifiState state);
void system_state_set_echolink(EchoLinkState state);
void system_state_set_station(StationState state, const char *callsign = nullptr, uint32_t node = 0);
void system_state_update_metrics(size_t free_heap, uint32_t uptime_sec);
void system_state_update_mic(uint16_t raw_level, int16_t level_pct);
void system_state_update_rx_level(uint16_t raw_level, int16_t level_pct);
void system_state_update_jitter(uint32_t depth, uint32_t underflows, uint32_t overflows);
void system_state_set_loopback(bool active);
bool system_state_get_loopback();

// Helper string convertors
const char *wifi_state_str(WifiState state);
const char *echolink_state_str(EchoLinkState state);
const char *station_state_str(StationState state);
