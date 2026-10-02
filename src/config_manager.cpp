#include "config_manager.h"
#include <Preferences.h>
#include <cstring>

static Preferences s_prefs_config;
static Preferences s_prefs_favs;
static ConfigData s_cached_config;

// Default favorites list for ham radio EchoLink testing
static const char *DEFAULT_FAVORITES_JSON =
    "["
    "{\"node\":9999,\"call\":\"*ECHOTEST*\",\"desc\":\"EchoLink Audio Test Server\"},"
    "{\"node\":123456,\"call\":\"*THAILAND*\",\"desc\":\"Thailand Conference Server\"},"
    "{\"node\":304439,\"call\":\"*HAM-CU*\",\"desc\":\"\"},"
    "{\"node\":346613,\"call\":\"*10WATTS*\",\"desc\":\"10WATTS GROUP THAILAND\"},"
    "{\"node\":211881,\"call\":\"*HS1AB-R*\",\"desc\":\"[Svx] E2HUB Bangkok TH(1)\"},"
    "{\"node\":211880,\"call\":\"*HS1AB-L*\",\"desc\":\"[Svx] E2HUB Bangkok TH(4)\"}"
    "]";

void config_manager_init()
{
    memset(&s_cached_config, 0, sizeof(s_cached_config));

    s_prefs_config.begin("ml_config", false);

    // Read stored settings with defaults
    String ssid = s_prefs_config.getString("wifi_ssid", "");
    String pass = s_prefs_config.getString("wifi_pass", "");
    String call = s_prefs_config.getString("callsign", "N0CALL");
    String el_p = s_prefs_config.getString("el_pass", "");
    String name = s_prefs_config.getString("st_name", "MicroLink Node");
    String loc = s_prefs_config.getString("location", "Bangkok, Thailand");
    String w_u = s_prefs_config.getString("web_user", "admin");
    String w_p = s_prefs_config.getString("web_pass", "admin");
    bool   p_en = s_prefs_config.getBool("proxy_en", false);
    String p_host = s_prefs_config.getString("proxy_host", "");
    uint16_t p_port = s_prefs_config.getUShort("proxy_port", 8100);
    String p_pass = s_prefs_config.getString("proxy_pass", "PUBLIC");

    strncpy(s_cached_config.wifi_ssid, ssid.c_str(), sizeof(s_cached_config.wifi_ssid) - 1);
    strncpy(s_cached_config.wifi_pass, pass.c_str(), sizeof(s_cached_config.wifi_pass) - 1);
    strncpy(s_cached_config.callsign, call.c_str(), sizeof(s_cached_config.callsign) - 1);
    strncpy(s_cached_config.el_password, el_p.c_str(), sizeof(s_cached_config.el_password) - 1);
    strncpy(s_cached_config.station_name, name.c_str(), sizeof(s_cached_config.station_name) - 1);
    strncpy(s_cached_config.location, loc.c_str(), sizeof(s_cached_config.location) - 1);
    strncpy(s_cached_config.web_user, w_u.c_str(), sizeof(s_cached_config.web_user) - 1);
    strncpy(s_cached_config.web_pass, w_p.c_str(), sizeof(s_cached_config.web_pass) - 1);
    s_cached_config.proxy_enabled = p_en;
    strncpy(s_cached_config.proxy_host, p_host.c_str(), sizeof(s_cached_config.proxy_host) - 1);
    s_cached_config.proxy_port = (p_port > 0) ? p_port : 8100;
    strncpy(s_cached_config.proxy_password, p_pass.c_str(), sizeof(s_cached_config.proxy_password) - 1);

    s_prefs_config.end();

    // Ensure favorites namespace exists
    s_prefs_favs.begin("ml_favs", false);
    if (!s_prefs_favs.isKey("fav_list"))
    {
        s_prefs_favs.putString("fav_list", DEFAULT_FAVORITES_JSON);
    }
    s_prefs_favs.end();
}

ConfigData config_manager_get()
{
    return s_cached_config;
}

bool config_manager_save(const ConfigData &new_cfg)
{
    s_prefs_config.begin("ml_config", false);

    strncpy(s_cached_config.wifi_ssid, new_cfg.wifi_ssid, sizeof(s_cached_config.wifi_ssid) - 1);
    s_prefs_config.putString("wifi_ssid", s_cached_config.wifi_ssid);

    // Only update WiFi password if non-empty and not masked
    if (strlen(new_cfg.wifi_pass) > 0 && strcmp(new_cfg.wifi_pass, "********") != 0)
    {
        strncpy(s_cached_config.wifi_pass, new_cfg.wifi_pass, sizeof(s_cached_config.wifi_pass) - 1);
        s_prefs_config.putString("wifi_pass", s_cached_config.wifi_pass);
    }

    strncpy(s_cached_config.callsign, new_cfg.callsign, sizeof(s_cached_config.callsign) - 1);
    s_prefs_config.putString("callsign", s_cached_config.callsign);

    // Only update EchoLink password if non-empty and not masked
    if (strlen(new_cfg.el_password) > 0 && strcmp(new_cfg.el_password, "********") != 0)
    {
        strncpy(s_cached_config.el_password, new_cfg.el_password, sizeof(s_cached_config.el_password) - 1);
        s_prefs_config.putString("el_pass", s_cached_config.el_password);
    }

    strncpy(s_cached_config.station_name, new_cfg.station_name, sizeof(s_cached_config.station_name) - 1);
    s_prefs_config.putString("st_name", s_cached_config.station_name);

    strncpy(s_cached_config.location, new_cfg.location, sizeof(s_cached_config.location) - 1);
    s_prefs_config.putString("location", s_cached_config.location);

    if (strlen(new_cfg.web_user) > 0)
    {
        strncpy(s_cached_config.web_user, new_cfg.web_user, sizeof(s_cached_config.web_user) - 1);
        s_prefs_config.putString("web_user", s_cached_config.web_user);
    }

    // Only update Web password if non-empty and not masked
    if (strlen(new_cfg.web_pass) > 0 && strcmp(new_cfg.web_pass, "********") != 0)
    {
        strncpy(s_cached_config.web_pass, new_cfg.web_pass, sizeof(s_cached_config.web_pass) - 1);
        s_prefs_config.putString("web_pass", s_cached_config.web_pass);
    }

    s_cached_config.proxy_enabled = new_cfg.proxy_enabled;
    s_prefs_config.putBool("proxy_en", s_cached_config.proxy_enabled);

    strncpy(s_cached_config.proxy_host, new_cfg.proxy_host, sizeof(s_cached_config.proxy_host) - 1);
    s_prefs_config.putString("proxy_host", s_cached_config.proxy_host);

    s_cached_config.proxy_port = (new_cfg.proxy_port > 0) ? new_cfg.proxy_port : 8100;
    s_prefs_config.putUShort("proxy_port", s_cached_config.proxy_port);

    if (strlen(new_cfg.proxy_password) > 0 && strcmp(new_cfg.proxy_password, "********") != 0)
    {
        strncpy(s_cached_config.proxy_password, new_cfg.proxy_password, sizeof(s_cached_config.proxy_password) - 1);
        s_prefs_config.putString("proxy_pass", s_cached_config.proxy_password);
    }

    s_prefs_config.end();
    return true;
}

String config_manager_get_favorites()
{
    s_prefs_favs.begin("ml_favs", true);
    String favs = s_prefs_favs.getString("fav_list", DEFAULT_FAVORITES_JSON);
    s_prefs_favs.end();
    return favs;
}

bool config_manager_save_favorites(const String &favs_json)
{
    s_prefs_favs.begin("ml_favs", false);
    size_t written = s_prefs_favs.putString("fav_list", favs_json);
    s_prefs_favs.end();
    return (written > 0);
}
