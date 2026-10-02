#include "system_state.h"
#include <cstring>

static SemaphoreHandle_t s_state_mutex = nullptr;
static SystemState s_state = {
    .wifi_state = WifiState::DISCONNECTED,
    .echolink_state = EchoLinkState::DISCONNECTED,
    .station_state = StationState::IDLE,
    .connected_callsign = {0},
    .connected_node = 0,
    .tx_active = false,
    .rx_active = false,
    .mic_raw_level = 0,
    .mic_level_pct = 0,
    .rx_raw_level = 0,
    .rx_level_pct = 0,
    .ptt_press_count = 0,
    .last_tx_duration_ms = 0,
    .free_heap = 0,
    .uptime_sec = 0};

static uint32_t s_tx_start_time_ms = 0;

void system_state_init()
{
    if (!s_state_mutex)
    {
        s_state_mutex = xSemaphoreCreateRecursiveMutex();
    }
}

bool system_state_lock(TickType_t wait_ticks)
{
    if (!s_state_mutex)
        return false;
    return xSemaphoreTakeRecursive(s_state_mutex, wait_ticks) == pdTRUE;
}

void system_state_unlock()
{
    if (s_state_mutex)
    {
        xSemaphoreGiveRecursive(s_state_mutex);
    }
}

SystemState system_state_get()
{
    SystemState copy{};
    if (system_state_lock())
    {
        copy = s_state;
        system_state_unlock();
    }
    return copy;
}

void system_state_set_ptt(bool active)
{
    if (system_state_lock())
    {
        if (s_state.tx_active != active)
        {
            s_state.tx_active = active;
            if (active)
            {
                s_state.ptt_press_count++;
                s_tx_start_time_ms = millis();
            }
            else
            {
                if (s_tx_start_time_ms > 0)
                {
                    s_state.last_tx_duration_ms = millis() - s_tx_start_time_ms;
                }
            }
        }
        system_state_unlock();
    }
}

void system_state_set_rx(bool active)
{
    if (system_state_lock())
    {
        s_state.rx_active = active;
        if (!active)
        {
            s_state.rx_raw_level = 0;
            s_state.rx_level_pct = 0;
        }
        system_state_unlock();
    }
}

void system_state_set_wifi(WifiState state)
{
    if (system_state_lock())
    {
        s_state.wifi_state = state;
        system_state_unlock();
    }
}

void system_state_set_echolink(EchoLinkState state)
{
    if (system_state_lock())
    {
        s_state.echolink_state = state;
        system_state_unlock();
    }
}

void system_state_set_station(StationState state, const char *callsign, uint32_t node)
{
    if (system_state_lock())
    {
        s_state.station_state = state;
        if (callsign)
        {
            strncpy(s_state.connected_callsign, callsign, sizeof(s_state.connected_callsign) - 1);
            s_state.connected_callsign[sizeof(s_state.connected_callsign) - 1] = '\0';
        }
        else if (state == StationState::IDLE)
        {
            s_state.connected_callsign[0] = '\0';
        }
        s_state.connected_node = node;
        system_state_unlock();
    }
}

void system_state_update_metrics(size_t free_heap, uint32_t uptime_sec)
{
    if (system_state_lock(pdMS_TO_TICKS(10)))
    {
        s_state.free_heap = free_heap;
        s_state.uptime_sec = uptime_sec;
        system_state_unlock();
    }
}

void system_state_update_mic(uint16_t raw_level, int16_t level_pct)
{
    if (system_state_lock(pdMS_TO_TICKS(10)))
    {
        s_state.mic_raw_level = raw_level;
        s_state.mic_level_pct = level_pct;
        system_state_unlock();
    }
}

void system_state_update_rx_level(uint16_t raw_level, int16_t level_pct)
{
    if (system_state_lock(pdMS_TO_TICKS(10)))
    {
        s_state.rx_raw_level = raw_level;
        s_state.rx_level_pct = level_pct;
        system_state_unlock();
    }
}

void system_state_update_jitter(uint32_t depth, uint32_t underflows, uint32_t overflows)
{
    if (system_state_lock(pdMS_TO_TICKS(10)))
    {
        s_state.jitter_depth = depth;
        s_state.jitter_underflows = underflows;
        s_state.jitter_overflows = overflows;
        system_state_unlock();
    }
}

void system_state_set_loopback(bool active)
{
    if (system_state_lock(pdMS_TO_TICKS(10)))
    {
        s_state.loopback_active = active;
        system_state_unlock();
    }
}

bool system_state_get_loopback()
{
    bool active = false;
    if (system_state_lock(pdMS_TO_TICKS(10)))
    {
        active = s_state.loopback_active;
        system_state_unlock();
    }
    return active;
}

const char *wifi_state_str(WifiState state)
{
    switch (state)
    {
    case WifiState::DISCONNECTED:
        return "DISCONNECTED";
    case WifiState::CONNECTING:
        return "CONNECTING";
    case WifiState::CONNECTED:
        return "CONNECTED";
    case WifiState::AP_MODE:
        return "AP_MODE";
    case WifiState::FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

const char *echolink_state_str(EchoLinkState state)
{
    switch (state)
    {
    case EchoLinkState::DISCONNECTED:
        return "DISCONNECTED";
    case EchoLinkState::CONNECTING_DIR:
        return "CONNECTING_DIR";
    case EchoLinkState::LOGGED_IN:
        return "LOGGED_IN";
    case EchoLinkState::AUTH_FAILED:
        return "AUTH_FAILED";
    case EchoLinkState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

const char *station_state_str(StationState state)
{
    switch (state)
    {
    case StationState::IDLE:
        return "IDLE";
    case StationState::CONNECTING:
        return "CONNECTING";
    case StationState::CONNECTED:
        return "CONNECTED";
    case StationState::DISCONNECTING:
        return "DISCONNECTING";
    default:
        return "UNKNOWN";
    }
}
