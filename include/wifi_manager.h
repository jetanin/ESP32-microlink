#pragma once

#include <Arduino.h>
#include <IPAddress.h>

/**
 * @file wifi_manager.h
 * @brief WiFi Connection and Fallback SoftAP / Captive Portal Manager
 */

void wifi_manager_init();
void wifi_manager_start();
void wifi_manager_reconnect();
void wifi_manager_process(); // Polled for DNS captive portal

bool wifi_manager_is_connected();
bool wifi_manager_is_ap();
IPAddress wifi_manager_get_ip();
String wifi_manager_get_ssid();
int8_t wifi_manager_get_rssi();
const char *wifi_manager_get_hostname();
