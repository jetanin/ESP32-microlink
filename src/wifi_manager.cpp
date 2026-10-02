#include "wifi_manager.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "config_manager.h"
#include "system_state.h"

static DNSServer s_dns_server;
static bool s_is_ap_mode = false;
static bool s_is_connected = false;
static const IPAddress AP_IP(192, 168, 4, 1);
static const char *HOSTNAME = "microlink";

void wifi_manager_init()
{
    s_is_ap_mode = false;
    s_is_connected = false;
}

void wifi_manager_start()
{
    ConfigData cfg = config_manager_get();

    WiFi.setHostname(HOSTNAME);

    // If WiFi credentials exist, attempt STA connection
    if (strlen(cfg.wifi_ssid) > 0)
    {
        Serial.printf("[WIFI] Connecting to SSID: '%s'...\n", cfg.wifi_ssid);
        system_state_set_wifi(WifiState::CONNECTING);

        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

        uint32_t start_ms = millis();
        constexpr uint32_t TIMEOUT_MS = 15000;

        while (WiFi.status() != WL_CONNECTED && (millis() - start_ms < TIMEOUT_MS))
        {
            delay(250);
            Serial.print(F("."));
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED)
        {
            s_is_connected = true;
            s_is_ap_mode = false;
            system_state_set_wifi(WifiState::CONNECTED);

            Serial.printf("[WIFI] Connected! IP: %s | RSSI: %d dBm\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());

            if (MDNS.begin(HOSTNAME))
            {
                MDNS.addService("http", "tcp", 80);
                Serial.printf("[mDNS] Responder started at http://%s.local\n", HOSTNAME);
            }
            return;
        }

        Serial.println(F("[WIFI] Connection timed out. Falling back to SoftAP."));
    }
    else
    {
        Serial.println(F("[WIFI] No WiFi credentials configured. Starting SoftAP setup."));
    }

    // SoftAP Fallback mode
    s_is_connected = false;
    s_is_ap_mode = true;
    system_state_set_wifi(WifiState::AP_MODE);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP("MicroLink-Setup");

    // Captive portal DNS redirect
    s_dns_server.start(53, "*", AP_IP);

    Serial.println(F("[WIFI] Fallback SoftAP started!"));
    Serial.println(F("[WIFI] SSID : MicroLink-Setup (Open)"));
    Serial.println(F("[WIFI] IP   : 192.168.4.1 (Captive Portal enabled)"));
}

void wifi_manager_process()
{
    if (s_is_ap_mode)
    {
        s_dns_server.processNextRequest();
    }
}

bool wifi_manager_is_connected()
{
    return (WiFi.status() == WL_CONNECTED);
}

bool wifi_manager_is_ap()
{
    return s_is_ap_mode;
}

IPAddress wifi_manager_get_ip()
{
    if (s_is_ap_mode)
    {
        return AP_IP;
    }
    return WiFi.localIP();
}

String wifi_manager_get_ssid()
{
    if (s_is_ap_mode)
    {
        return "MicroLink-Setup";
    }
    return WiFi.SSID();
}

int8_t wifi_manager_get_rssi()
{
    if (s_is_ap_mode)
    {
        return 0;
    }
    return WiFi.RSSI();
}

const char *wifi_manager_get_hostname()
{
    return HOSTNAME;
}

void wifi_manager_reconnect()
{
    WiFi.disconnect(true);
    delay(100);
    wifi_manager_start();
}
