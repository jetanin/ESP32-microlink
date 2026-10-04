#pragma once

#include <Arduino.h>

/**
 * @file config_manager.h
 * @brief Persistent Configuration Manager using ESP32 NVS (Preferences)
 *
 * Stores secrets and node preferences in NVS.
 * Guarantees passwords are never hard-coded and never exposed in plain-text to APIs.
 */

struct ConfigData
{
    char wifi_ssid[33];
    char wifi_pass[64];
    char callsign[16];
    char el_password[32];
    char station_name[32];
    char location[64];
    char web_user[32];
    char web_pass[32];
    bool proxy_enabled;
    char proxy_host[64];
    uint16_t proxy_port;
    char proxy_password[32];
    uint8_t vox_pre; // VOX pre-roll delay in frames (0..6, default 3 = 60 ms)
};

void config_manager_init();
ConfigData config_manager_get();
bool config_manager_save(const ConfigData &new_cfg);
bool config_manager_set_vox_preroll(uint8_t frames);
uint8_t config_manager_get_vox_preroll();

// Favorites list management (JSON array of nodes/conferences)
String config_manager_get_favorites();
bool config_manager_save_favorites(const String &favs_json);

// Helper to mask secret strings for safe JSON serialization
inline String mask_secret(const char *secret)
{
    if (secret && strlen(secret) > 0)
    {
        return "********";
    }
    return "";
}
