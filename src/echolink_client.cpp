/**
 * @file echolink_client.cpp
 * @brief EchoLink client implementation for ESP32-C6.
 *
 * MicroLink ESP32-C6 EchoLink Node
 * Adapted from Bruce MacKinnon (KC1FSZ) MicroLink
 * GPL-3.0 License
 */

#include "echolink_client.h"
#include "echolink_protocol.h"
#include "gsm_codec.h"
#include "audio_pipeline.h"
#include "config_manager.h"
#include "system_state.h"
#include "pins.h"
#include "echolink_proxy.h"
#include "dtmf_detector.h"
#include "announcer.h"

#include <Arduino.h>
#include <WiFi.h>

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#define TAG "EchoLink"

// Timing constants
static const uint32_t LOGON_INTERVAL_MS = 5 * 60 * 1000;  // 5 minutes
static const uint32_t KEEP_ALIVE_INTERVAL_MS = 10 * 1000; // 10 seconds
static const uint32_t SESSION_TIMEOUT_MS = 30 * 1000;     // 30 seconds
static const uint32_t RX_ACTIVE_TIMEOUT_MS = 600;         // 600 ms silence -> RX inactive

// Mutex & task handle
static SemaphoreHandle_t s_el_mutex = nullptr;
static TaskHandle_t s_el_task_handle = nullptr;
static volatile bool s_el_running = false;

// Registration state
static bool s_is_registered = false;
static uint32_t s_last_logon_time = 0;
static uint32_t s_next_logon_attempt = 0;
static uint32_t s_last_warning_time = 0;
static volatile bool s_force_registration = false;

// Session state
enum class ClientSessionState
{
    IDLE,
    LOOKUP,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
};

static ClientSessionState s_session_state = ClientSessionState::IDLE;
static char s_target_input[32] = {0};
static char s_remote_callsign[32] = {0};
static uint32_t s_remote_node = 0;
static struct sockaddr_in s_remote_rtp_addr;
static struct sockaddr_in s_remote_rtcp_addr;

static int s_rtp_sock = -1;
static int s_rtcp_sock = -1;

static uint32_t s_ssrc = 0x12345678;
static uint16_t s_tx_seq = 0;
static uint32_t s_last_keepalive_time = 0;
static uint32_t s_last_rx_packet_time = 0;
static uint32_t s_last_audio_rx_time = 0;

// PTT and TX buffering
static volatile bool s_ptt_requested = false;
static bool s_tx_active = false;
static volatile TxSource s_tx_source = TxSource::NONE;
static int16_t s_tx_pcm_buf[GSM_BUNDLE_SAMPLES];
static size_t s_tx_pcm_samples_accum = 0;

// Forward declarations
static bool addressing_server_logon();
static bool lookup_station(const char *target, char *out_call, uint32_t *out_node, struct in_addr *out_ip);
static bool setup_udp_sockets();
static void close_udp_sockets();
static void send_keepalive();
static void send_session_bye();
static void service_rx_traffic();
static void service_tx_audio();

bool echolink_client_init()
{
    if (s_el_mutex == nullptr)
    {
        s_el_mutex = xSemaphoreCreateMutex();
    }
    gsm_codec_init();
    s_session_state = ClientSessionState::IDLE;
    s_is_registered = false;
    s_last_logon_time = 0;
    return (s_el_mutex != nullptr);
}

static void echolink_task(void *pvParameters)
{
    Serial.printf("[EchoLink] Background task started on core %d (Priority %d)\n",
                  xPortGetCoreID(), uxTaskPriorityGet(nullptr));

    while (s_el_running)
    {
        // Check WiFi connectivity
        if (WiFi.status() == WL_CONNECTED)
        {
            uint32_t now = millis();

            // 1. Periodic Addressing Server Keep-Alive / Registration
            bool due = (now >= s_next_logon_attempt);
            if (s_force_registration || !s_is_registered || due)
            {
                s_force_registration = false;
                ConfigData cfg = config_manager_get();
                if (strlen(cfg.callsign) == 0 || strlen(cfg.el_password) == 0 || strcmp(cfg.callsign, "N0CALL") == 0)
                {
                    if (s_last_warning_time == 0 || (now - s_last_warning_time >= 30000))
                    {
                        Serial.println(F("[EchoLink] Registration pending: Callsign or EchoLink Password not configured."));
                        Serial.println(F("[EchoLink] Set via Web UI Settings or Serial: 'set callsign <CALL>' and 'set password <PASS>'"));
                        s_last_warning_time = now;
                    }
                    s_next_logon_attempt = now + 30000;
                    system_state_set_echolink(EchoLinkState::AUTH_FAILED);
                    system_state_set_reg_failed(true);
                }
                else
                {
                    if (cfg.proxy_enabled)
                    {
                        Serial.printf("[EchoLink] Connecting through Proxy %s:%u for registration...\n", cfg.proxy_host, cfg.proxy_port);
                    }
                    else
                    {
                        Serial.println(F("[EchoLink] Connecting to Addressing Server for registration..."));
                    }
                    system_state_set_reg_failed(false);
                    system_state_set_echolink(EchoLinkState::CONNECTING_DIR);
                    if (addressing_server_logon())
                    {
                        s_is_registered = true;
                        s_last_logon_time = now;
                        s_next_logon_attempt = now + LOGON_INTERVAL_MS;
                        system_state_set_echolink(EchoLinkState::LOGGED_IN);
                        system_state_set_reg_failed(false);
                        Serial.println(F("[EchoLink] Registered with EchoLink directory"));
                    }
                    else
                    {
                        s_next_logon_attempt = now + 30000; // Backoff 30 seconds
                        system_state_set_echolink(EchoLinkState::AUTH_FAILED);
                        system_state_set_reg_failed(true);
                        Serial.println(F("[EchoLink] Addressing server registration failed (retrying in 30s)"));
                    }
                }
            }

            // 2. State Machine for VoIP Session
            if (xSemaphoreTake(s_el_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                switch (s_session_state)
                {
                case ClientSessionState::IDLE:
                    // Nothing active
                    break;

                case ClientSessionState::LOOKUP:
                {
                    Serial.printf("[EchoLink] Looking up station: %s\n", s_target_input);
                    system_state_set_station(StationState::CONNECTING, s_target_input, 0);

                    char resolved_call[32] = {0};
                    uint32_t resolved_node = 0;
                    struct in_addr resolved_ip;

                    xSemaphoreGive(s_el_mutex);
                    bool ok = lookup_station(s_target_input, resolved_call, &resolved_node, &resolved_ip);
                    xSemaphoreTake(s_el_mutex, portMAX_DELAY);

                    if (ok)
                    {
                        strncpy(s_remote_callsign, resolved_call, sizeof(s_remote_callsign) - 1);
                        s_remote_node = resolved_node;

                        memset(&s_remote_rtp_addr, 0, sizeof(s_remote_rtp_addr));
                        s_remote_rtp_addr.sin_family = AF_INET;
                        s_remote_rtp_addr.sin_port = htons(ECHOLINK_RTP_PORT);
                        s_remote_rtp_addr.sin_addr = resolved_ip;

                        memset(&s_remote_rtcp_addr, 0, sizeof(s_remote_rtcp_addr));
                        s_remote_rtcp_addr.sin_family = AF_INET;
                        s_remote_rtcp_addr.sin_port = htons(ECHOLINK_RTCP_PORT);
                        s_remote_rtcp_addr.sin_addr = resolved_ip;

                        char ip_str[INET_ADDRSTRLEN];
                        inet_ntop(AF_INET, &resolved_ip, ip_str, sizeof(ip_str));
                        Serial.printf("[EchoLink] Station %s resolved to IP %s (Node %lu)\n",
                                      s_remote_callsign, ip_str, (unsigned long)s_remote_node);

                        if (setup_udp_sockets())
                        {
                            s_session_state = ClientSessionState::CONNECTING;
                            s_last_keepalive_time = millis();
                            s_last_audio_rx_time = 0;
                            s_last_rx_packet_time = millis();
                            s_tx_seq = 0;
                            s_tx_pcm_samples_accum = 0;
                            gsm_codec_reset();
                            audio_pipeline_clear();

                            // Send initial greeting
                            send_keepalive();
                            system_state_set_station(StationState::CONNECTING, s_remote_callsign, s_remote_node);
                            Serial.printf("[EchoLink] UDP sockets bound. Greeting sent to %s. Awaiting connection...\n", s_remote_callsign);
                        }
                        else
                        {
                            Serial.println("[EchoLink] Failed to setup UDP sockets");
                            s_session_state = ClientSessionState::IDLE;
                            system_state_set_station(StationState::IDLE);
                        }
                    }
                    else
                    {
                        Serial.printf("[EchoLink] Station lookup failed for %s\n", s_target_input);
                        s_session_state = ClientSessionState::IDLE;
                        system_state_set_station(StationState::IDLE);
                        Announcer::instance().announceNotConnected();
                    }
                    break;
                }

                case ClientSessionState::CONNECTING:
                case ClientSessionState::CONNECTED:
                {
                    // Check session timeout
                    if (now - s_last_rx_packet_time > SESSION_TIMEOUT_MS)
                    {
                        Serial.printf("[EchoLink] Session timed out with %s (no traffic in %lu ms)\n",
                                      s_remote_callsign, (unsigned long)(now - s_last_rx_packet_time));
                        send_session_bye();
                        close_udp_sockets();
                        s_session_state = ClientSessionState::IDLE;
                        system_state_set_station(StationState::IDLE);
                        system_state_set_rx(false);
                        audio_pipeline_set_mode(PipelineMode::IDLE);
                        audio_pipeline_clear();
                        Announcer::instance().announceDisconnected(s_remote_node);
                        break;
                    }

                    // Check keep-alive timer (every 10s)
                    if (now - s_last_keepalive_time >= KEEP_ALIVE_INTERVAL_MS)
                    {
                        send_keepalive();
                        s_last_keepalive_time = now;
                    }

                    // Process Inbound Audio & Control packets
                    service_rx_traffic();

                    // Process Outbound Audio (PTT)
                    service_tx_audio();

                    // Update RX active indicator timeout
                    uint32_t rx_check_now = millis();
                    if (s_last_audio_rx_time > 0 && (rx_check_now - s_last_audio_rx_time > RX_ACTIVE_TIMEOUT_MS))
                    {
                        system_state_set_rx(false);
                        s_last_audio_rx_time = 0;
                    }
                    break;
                }

                case ClientSessionState::DISCONNECTING:
                    send_session_bye();
                    close_udp_sockets();
                    s_session_state = ClientSessionState::IDLE;
                    system_state_set_station(StationState::IDLE);
                    system_state_set_rx(false);
                    audio_pipeline_set_mode(PipelineMode::IDLE);
                    audio_pipeline_clear();
                    Announcer::instance().announceDisconnected(s_remote_node);
                    Serial.println("[EchoLink] Disconnected from station");
                    break;
                }
                xSemaphoreGive(s_el_mutex);
            }
        }
        else
        {
            // WiFi not connected
            s_is_registered = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to lower priority tasks
    }

    close_udp_sockets();
    vTaskDelete(nullptr);
}

bool echolink_client_start()
{
    if (s_el_running)
        return true;
    s_el_running = true;

    // Stack: 6 KB, Priority 3 (higher than web server at 1 and system status at 2, below audio at 4 & 5)
    BaseType_t ret = xTaskCreate(echolink_task, "echolink_task", 6144, nullptr, 3, &s_el_task_handle);
    return (ret == pdPASS);
}

void echolink_client_stop()
{
    s_el_running = false;
    if (s_el_task_handle)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        s_el_task_handle = nullptr;
    }
    echolink_proxy_disconnect();
}

bool echolink_client_connect(const char *callsign_or_node)
{
    if (!callsign_or_node || strlen(callsign_or_node) == 0)
        return false;

    if (xSemaphoreTake(s_el_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        strncpy(s_target_input, callsign_or_node, sizeof(s_target_input) - 1);
        s_target_input[sizeof(s_target_input) - 1] = 0;
        // Trim whitespace
        char *p = s_target_input;
        while (*p == ' ')
            p++;
        if (p != s_target_input)
            memmove(s_target_input, p, strlen(p) + 1);
        size_t l = strlen(s_target_input);
        while (l > 0 && s_target_input[l - 1] == ' ')
        {
            s_target_input[l - 1] = 0;
            l--;
        }

        s_session_state = ClientSessionState::LOOKUP;
        xSemaphoreGive(s_el_mutex);
        return true;
    }
    return false;
}

void echolink_client_disconnect()
{
    if (xSemaphoreTake(s_el_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (s_session_state != ClientSessionState::IDLE)
        {
            s_session_state = ClientSessionState::DISCONNECTING;
        }
        xSemaphoreGive(s_el_mutex);
    }
}

void echolink_client_trigger_registration()
{
    s_force_registration = true;
    s_next_logon_attempt = 0;
    system_state_set_reg_failed(false);
}

bool echolink_client_is_registered()
{
    return s_is_registered;
}

bool echolink_client_is_connected()
{
    return (s_session_state == ClientSessionState::CONNECTED);
}

const char *echolink_client_get_connected_callsign()
{
    return s_remote_callsign;
}

uint32_t echolink_client_get_connected_node()
{
    return s_remote_node;
}

bool echolink_client_is_tx_active()
{
    return s_tx_active || s_ptt_requested;
}

TxSource echolink_client_get_tx_source()
{
    return s_tx_source;
}

bool echolink_client_tx_request(TxSource source)
{
    // Operator priority: if operator PTT or VOX is active, reject lower priority requests
    if ((s_tx_source == TxSource::PTT_BUTTON || s_tx_source == TxSource::VOX) && source == TxSource::ANNOUNCEMENT)
    {
        return false;
    }

    s_tx_source = source;
    s_ptt_requested = true;
    return true;
}

void echolink_client_tx_release(TxSource source)
{
    if (s_tx_source == source)
    {
        s_tx_source = TxSource::NONE;
        s_ptt_requested = false;
    }
}

void echolink_client_tx_switch_to(TxSource new_source)
{
    s_tx_source = new_source;
    s_ptt_requested = true;
    system_state_set_ptt(true);
}

void echolink_client_tx_force_off()
{
    s_tx_source = TxSource::NONE;
    s_ptt_requested = false;
}

void echolink_client_set_ptt(bool active)
{
    if (active)
    {
        // User priority: if an announcement is active, abort it immediately and hand TX over
        if (s_tx_source == TxSource::ANNOUNCEMENT && Announcer::instance().busy())
        {
            Announcer::instance().abort();
            echolink_client_tx_switch_to(TxSource::PTT_BUTTON);
            Serial.println(F("[EchoLink] Operator PTT pressed during announcement - handed TX to operator seamlessly"));
        }
        else
        {
            echolink_client_tx_request(TxSource::PTT_BUTTON);
        }
    }
    else
    {
        echolink_client_tx_release(TxSource::PTT_BUTTON);
    }
}

void echolink_client_feed_announcement_pcm(const int16_t *samples, size_t count)
{
    if (!samples || count == 0) return;
    if (s_tx_source != TxSource::ANNOUNCEMENT) return;

    ConfigData cfg = config_manager_get();
    size_t offset = 0;
    while (offset < count)
    {
        size_t needed = GSM_BUNDLE_SAMPLES - s_tx_pcm_samples_accum;
        size_t to_copy = (count - offset < needed) ? (count - offset) : needed;
        memcpy(s_tx_pcm_buf + s_tx_pcm_samples_accum, samples + offset, to_copy * sizeof(int16_t));
        s_tx_pcm_samples_accum += to_copy;
        offset += to_copy;

        if (s_tx_pcm_samples_accum >= GSM_BUNDLE_SAMPLES)
        {
            uint8_t gsm_payload[GSM_BUNDLE_BYTES];
            gsm_encode_4frames(s_tx_pcm_buf, gsm_payload);

            uint8_t rtp_packet[144];
            uint32_t rtp_len = echolink::formatRTPPacket(s_tx_seq++, s_ssrc,
                                                         gsm_payload, rtp_packet, sizeof(rtp_packet));
            if (rtp_len == 144)
            {
                if (cfg.proxy_enabled)
                {
                    echolink_proxy_send_udp_data(s_remote_rtp_addr.sin_addr, rtp_packet, 144);
                }
                else if (s_rtp_sock >= 0)
                {
                    sendto(s_rtp_sock, rtp_packet, 144, 0,
                           (struct sockaddr *)&s_remote_rtp_addr, sizeof(s_remote_rtp_addr));
                }
            }
            s_tx_pcm_samples_accum = 0;
        }
    }
}

// =========================================================================
// Network & Protocol Internals
// =========================================================================

static bool addressing_server_logon()
{
    ConfigData cfg = config_manager_get();
    if (strlen(cfg.callsign) == 0 || strlen(cfg.el_password) == 0)
    {
        Serial.println("[EchoLink] Cannot logon: Callsign or Password not configured");
        return false;
    }

    struct hostent *he = gethostbyname(ECHOLINK_DEFAULT_ADDR_SERVER);
    if (!he || !he->h_addr_list[0])
    {
        Serial.println("[EchoLink] DNS lookup failed for addressing server");
        return false;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(ECHOLINK_DEFAULT_ADDR_PORT);
    memcpy(&serv_addr.sin_addr, he->h_addr_list[0], sizeof(serv_addr.sin_addr));

    uint8_t msg[256];
    uint32_t msg_len = echolink::createOnlineMessage(msg, sizeof(msg),
                                                     cfg.callsign, cfg.el_password,
                                                     cfg.location, ECHOLINK_VERSION_ID);

    if (cfg.proxy_enabled)
    {
        if (!echolink_proxy_is_connected())
        {
            if (!echolink_proxy_connect(cfg.proxy_host, cfg.proxy_port, cfg.callsign, cfg.proxy_password))
            {
                return false;
            }
        }

        char resp[128];
        size_t resp_len = 0;
        if (echolink_proxy_tcp_transaction(serv_addr.sin_addr, msg, msg_len, resp, sizeof(resp) - 1, &resp_len, 5000))
        {
            resp[resp_len] = 0;
            if (strncmp(resp, "OK", 2) == 0)
            {
                return true;
            }
            else
            {
                Serial.printf("[EchoLink] Server rejected registration: %s\n", resp);
            }
        }
        return false;
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        return false;

    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        close(sock);
        return false;
    }

    if (send(sock, msg, msg_len, 0) != (ssize_t)msg_len)
    {
        close(sock);
        return false;
    }

    char resp[128];
    ssize_t r = recv(sock, resp, sizeof(resp) - 1, 0);
    close(sock);

    if (r > 0)
    {
        resp[r] = 0;
        if (strncmp(resp, "OK", 2) == 0)
        {
            return true;
        }
        else
        {
            Serial.printf("[EchoLink] Server rejected registration: %s\n", resp);
        }
    }
    else
    {
        Serial.println(F("[EchoLink] Timeout or no response from addressing server"));
    }
    return false;
}

static bool lookup_station(const char *target, char *out_call, uint32_t *out_node, struct in_addr *out_ip)
{
    // Check if user specified 9999 (ECHOTEST)
    char query_call[32];
    strncpy(query_call, target, sizeof(query_call) - 1);
    query_call[sizeof(query_call) - 1] = 0;

    if (strcmp(query_call, "9999") == 0)
    {
        strcpy(query_call, "*ECHOTEST*");
    }

    struct hostent *he = gethostbyname(ECHOLINK_DEFAULT_ADDR_SERVER);
    if (!he || !he->h_addr_list[0])
        return false;

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(ECHOLINK_DEFAULT_ADDR_PORT);
    memcpy(&serv_addr.sin_addr, he->h_addr_list[0], sizeof(struct in_addr));

    char req[64];
    snprintf(req, sizeof(req), "V%s\n", query_call);

    char buf[512];
    ssize_t total = 0;

    ConfigData cfg = config_manager_get();
    if (cfg.proxy_enabled)
    {
        if (echolink_proxy_is_connected())
        {
            size_t resp_len = 0;
            if (echolink_proxy_tcp_transaction(serv_addr.sin_addr, req, strlen(req), buf, sizeof(buf) - 1, &resp_len, 5000))
            {
                total = resp_len;
            }
        }
    }

    if (total <= 0)
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
            return false;

        struct timeval tv;
        tv.tv_sec = 6;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv));

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            close(sock);
            return false;
        }

        send(sock, req, strlen(req), 0);

        while (total < (ssize_t)(sizeof(buf) - 1))
        {
            ssize_t n = recv(sock, buf + total, sizeof(buf) - 1 - total, 0);
            if (n <= 0)
                break;
            total += n;
        }
        close(sock);
    }

    if (total <= 0)
        return false;
    buf[total] = 0;

    // Tokenize response lines delimited by '\r' and '\n'
    char lines[8][64];
    uint16_t line_count = 0;
    char *p = buf;
    while (*p && line_count < 8)
    {
        while (*p == '\r' || *p == '\n')
            p++;
        if (!*p)
            break;
        char *line_start = p;
        while (*p && *p != '\r' && *p != '\n')
            p++;
        size_t len = p - line_start;
        if (len >= sizeof(lines[line_count]))
            len = sizeof(lines[line_count]) - 1;
        memcpy(lines[line_count], line_start, len);
        lines[line_count][len] = '\0';
        line_count++;
    }

    if (line_count < 2)
        return false;

    // Line 0 is always the station callsign
    strncpy(out_call, lines[0], 31);
    out_call[31] = '\0';
    *out_node = 0;
    out_ip->s_addr = 0;

    // Scan subsequent lines for IP address and Node number
    for (uint16_t i = 1; i < line_count; i++)
    {
        struct in_addr temp_ip;
        if (inet_pton(AF_INET, lines[i], &temp_ip) == 1)
        {
            *out_ip = temp_ip;
        }
        else
        {
            char *endp = nullptr;
            unsigned long n = strtoul(lines[i], &endp, 10);
            if (endp != lines[i] && *endp == '\0' && n > 0 && *out_node == 0)
            {
                *out_node = (uint32_t)n;
            }
        }
    }

    // Fallback: if target was numeric (e.g. "9999") and out_node is still 0
    if (*out_node == 0)
    {
        char *endp = nullptr;
        unsigned long n = strtoul(target, &endp, 10);
        if (endp != target && *endp == '\0')
        {
            *out_node = (uint32_t)n;
        }
    }

    return (out_ip->s_addr != 0);
}

static bool setup_udp_sockets()
{
    ConfigData cfg = config_manager_get();
    if (cfg.proxy_enabled)
    {
        if (!echolink_proxy_is_connected())
        {
            return echolink_proxy_connect(cfg.proxy_host, cfg.proxy_port, cfg.callsign, cfg.proxy_password);
        }
        return true;
    }

    close_udp_sockets();

    // 1. RTP Socket (UDP 5198)
    s_rtp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_rtp_sock < 0)
        return false;

    int opt = 1;
    setsockopt(s_rtp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in rtp_local;
    memset(&rtp_local, 0, sizeof(rtp_local));
    rtp_local.sin_family = AF_INET;
    rtp_local.sin_port = htons(ECHOLINK_RTP_PORT);
    rtp_local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_rtp_sock, (struct sockaddr *)&rtp_local, sizeof(rtp_local)) < 0)
    {
        Serial.printf("[EchoLink] Failed to bind RTP socket to port %d\n", ECHOLINK_RTP_PORT);
        close(s_rtp_sock);
        s_rtp_sock = -1;
        return false;
    }

    // 2. RTCP Socket (UDP 5199)
    s_rtcp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_rtcp_sock < 0)
    {
        close(s_rtp_sock);
        s_rtp_sock = -1;
        return false;
    }

    setsockopt(s_rtcp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in rtcp_local;
    memset(&rtcp_local, 0, sizeof(rtcp_local));
    rtcp_local.sin_family = AF_INET;
    rtcp_local.sin_port = htons(ECHOLINK_RTCP_PORT);
    rtcp_local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_rtcp_sock, (struct sockaddr *)&rtcp_local, sizeof(rtcp_local)) < 0)
    {
        Serial.printf("[EchoLink] Failed to bind RTCP socket to port %d\n", ECHOLINK_RTCP_PORT);
        close(s_rtp_sock);
        close(s_rtcp_sock);
        s_rtp_sock = -1;
        s_rtcp_sock = -1;
        return false;
    }

    // Set non-blocking
    fcntl(s_rtp_sock, F_SETFL, O_NONBLOCK);
    fcntl(s_rtcp_sock, F_SETFL, O_NONBLOCK);

    return true;
}

static void close_udp_sockets()
{
    if (s_rtp_sock >= 0)
    {
        close(s_rtp_sock);
        s_rtp_sock = -1;
    }
    if (s_rtcp_sock >= 0)
    {
        close(s_rtcp_sock);
        s_rtcp_sock = -1;
    }
}

static void send_keepalive()
{
    ConfigData cfg = config_manager_get();

    if (!cfg.proxy_enabled && (s_rtcp_sock < 0 || s_rtp_sock < 0))
        return;
    if (cfg.proxy_enabled && !echolink_proxy_is_connected())
        return;

    // 1. RTCP SDES packet to remote_ip:5199
    uint8_t rtcp_buf[256];
    uint32_t rtcp_len = echolink::formatRTCPPacket_SDES(s_ssrc, cfg.callsign,
                                                        cfg.station_name, s_ssrc,
                                                        rtcp_buf, sizeof(rtcp_buf));
    if (rtcp_len > 0)
    {
        if (cfg.proxy_enabled)
        {
            echolink_proxy_send_udp_ctrl(s_remote_rtcp_addr.sin_addr, rtcp_buf, rtcp_len);
        }
        else
        {
            // Send to RTCP port on remote from RTCP local socket
            sendto(s_rtcp_sock, rtcp_buf, rtcp_len, 0,
                   (struct sockaddr *)&s_remote_rtcp_addr, sizeof(s_remote_rtcp_addr));
            // Also send to RTCP port from RTP local socket for UDP hole-punching through NAT
            sendto(s_rtp_sock, rtcp_buf, rtcp_len, 0,
                   (struct sockaddr *)&s_remote_rtcp_addr, sizeof(s_remote_rtcp_addr));
        }
    }

    // 2. RTP oNDATA packet to remote_ip:5198
    char ondata_msg[128];
    snprintf(ondata_msg, sizeof(ondata_msg), "oNDATA\r%s\r%s\r%s\rMicroLink ESP32-C6\r",
             cfg.callsign, cfg.station_name, cfg.location);

    uint8_t rtp_buf[256];
    uint32_t rtp_len = echolink::formatOnDataPacket(ondata_msg, s_ssrc, rtp_buf, sizeof(rtp_buf));
    if (rtp_len > 0)
    {
        if (cfg.proxy_enabled)
        {
            echolink_proxy_send_udp_data(s_remote_rtp_addr.sin_addr, rtp_buf, rtp_len);
        }
        else
        {
            sendto(s_rtp_sock, rtp_buf, rtp_len, 0,
                   (struct sockaddr *)&s_remote_rtp_addr, sizeof(s_remote_rtp_addr));
        }
    }
}

static void send_session_bye()
{
    ConfigData cfg = config_manager_get();
    uint8_t bye_buf[64];
    uint32_t bye_len = echolink::formatRTCPPacket_BYE(s_ssrc, bye_buf, sizeof(bye_buf));
    if (bye_len > 0)
    {
        if (cfg.proxy_enabled)
        {
            echolink_proxy_send_udp_ctrl(s_remote_rtcp_addr.sin_addr, bye_buf, bye_len);
        }
        else if (s_rtcp_sock >= 0)
        {
            sendto(s_rtcp_sock, bye_buf, bye_len, 0,
                   (struct sockaddr *)&s_remote_rtcp_addr, sizeof(s_remote_rtcp_addr));
        }
    }
}

static void service_rx_traffic()
{
    ConfigData cfg = config_manager_get();

    if (cfg.proxy_enabled)
    {
        int p_sock = echolink_proxy_get_socket();
        if (p_sock < 0 || !echolink_proxy_is_connected())
            return;

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(p_sock, &read_fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 2000;

        if (select(p_sock + 1, &read_fds, nullptr, nullptr, &tv) > 0 && FD_ISSET(p_sock, &read_fds))
        {
            EchoLinkProxyMsgType type;
            struct in_addr from_ip;
            uint8_t buf[512];
            size_t n = 0;
            while (echolink_proxy_recv_msg(&type, &from_ip, buf, sizeof(buf), &n))
            {
                if (n == 0)
                    continue;
                s_last_rx_packet_time = millis();

                if (type == EchoLinkProxyMsgType::UDP_CONTROL)
                {
                    if (echolink::isRTCPByePacket(buf, n))
                    {
                        Serial.printf("[EchoLink] Received BYE from %s - closing session\n", s_remote_callsign);
                        s_session_state = ClientSessionState::DISCONNECTING;
                        return;
                    }
                    else if (echolink::isRTCPSDESPacket(buf, n) || echolink::isRTCPPINGPacket(buf, n))
                    {
                        if (s_session_state == ClientSessionState::CONNECTING)
                        {
                            s_session_state = ClientSessionState::CONNECTED;
                            system_state_set_station(StationState::CONNECTED, s_remote_callsign, s_remote_node);
                            Serial.printf("[EchoLink] Connected to %s!\n", s_remote_callsign);
                            Announcer::instance().announceConnected(s_remote_node);
                        }
                    }
                }
                else if (type == EchoLinkProxyMsgType::UDP_DATA)
                {
                    if (echolink::isRTPAudioPacket(buf, n))
                    {
                        if (s_session_state == ClientSessionState::CONNECTING)
                        {
                            s_session_state = ClientSessionState::CONNECTED;
                            system_state_set_station(StationState::CONNECTED, s_remote_callsign, s_remote_node);
                            Serial.printf("[EchoLink] Connected to %s!\n", s_remote_callsign);
                            Announcer::instance().announceConnected(s_remote_node);
                        }

                        // Extract 4 GSM frames (132 bytes) starting at byte 12
                        const uint8_t *gsm_payload = buf + 12;
                        int16_t pcm_bundle[GSM_BUNDLE_SAMPLES];

                        // Decode 4 frames (640 samples / 80 ms)
                        gsm_decode_4frames(gsm_payload, pcm_bundle);
                        dtmf_detector_process(pcm_bundle, GSM_BUNDLE_SAMPLES);

                        // Push each of the 4 160-sample frames into the Jitter Buffer
                        const int16_t *pcm_frame = pcm_bundle;
                        for (int i = 0; i < GSM_BUNDLE_FRAMES; i++)
                        {
                            audio_pipeline_write_frame(pcm_frame);
                            pcm_frame += GSM_FRAME_SAMPLES;
                        }

                        // Calculate Peak RX Level for audio meter
                        int16_t max_peak = 0;
                        for (int i = 0; i < GSM_BUNDLE_SAMPLES; i++)
                        {
                            int16_t abs_s = abs(pcm_bundle[i]);
                            if (abs_s > max_peak)
                                max_peak = abs_s;
                        }
                        int16_t rx_pct = (max_peak * 100) / 24000;
                        if (rx_pct > 100)
                            rx_pct = 100;
                        system_state_update_rx_level((uint16_t)max_peak, rx_pct);

                        s_last_audio_rx_time = millis();
                        system_state_set_rx(true);

                        static uint32_t s_last_proxy_rx_log = 0;
                        if (millis() - s_last_proxy_rx_log >= 1000)
                        {
                            s_last_proxy_rx_log = millis();
                            Serial.printf("[EchoLink] Audio RX from %s: peak %d (%d%%)\n",
                                          s_remote_callsign, max_peak, rx_pct);
                        }
                    }
                    else if (echolink::isOnDataPacket(buf, n))
                    {
                        if (s_session_state == ClientSessionState::CONNECTING)
                        {
                            s_session_state = ClientSessionState::CONNECTED;
                            system_state_set_station(StationState::CONNECTED, s_remote_callsign, s_remote_node);
                            Serial.printf("[EchoLink] Connected to %s!\n", s_remote_callsign);
                        }
                        Serial.printf("[EchoLink] Data message: %.*s\n", (int)n, (const char *)buf);
                    }
                }
                else if (type == EchoLinkProxyMsgType::SYSTEM)
                {
                    if (buf[0] == 1)
                    {
                        Serial.println(F("[EchoLink Proxy] System: Incorrect password supplied to proxy!"));
                    }
                    else if (buf[0] == 2)
                    {
                        Serial.println(F("[EchoLink Proxy] System: Callsign access denied by proxy!"));
                    }
                }
            }
        }
        return;
    }

    if (s_rtp_sock < 0 || s_rtcp_sock < 0)
        return;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(s_rtp_sock, &read_fds);
    FD_SET(s_rtcp_sock, &read_fds);
    int max_fd = std::max(s_rtp_sock, s_rtcp_sock);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 2000; // 2 ms non-blocking poll

    int sel = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
    if (sel <= 0)
        return;

    // Check RTCP control traffic
    if (FD_ISSET(s_rtcp_sock, &read_fds))
    {
        uint8_t buf[256];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t n = recvfrom(s_rtcp_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        if (n > 0)
        {
            s_last_rx_packet_time = millis();

            if (echolink::isRTCPByePacket(buf, n))
            {
                Serial.printf("[EchoLink] Received BYE from %s - closing session\n", s_remote_callsign);
                s_session_state = ClientSessionState::DISCONNECTING;
                return;
            }
            else if (echolink::isRTCPSDESPacket(buf, n) || echolink::isRTCPPINGPacket(buf, n))
            {
                if (s_session_state == ClientSessionState::CONNECTING)
                {
                    s_session_state = ClientSessionState::CONNECTED;
                    system_state_set_station(StationState::CONNECTED, s_remote_callsign, s_remote_node);
                    Serial.printf("[EchoLink] Connected to %s!\n", s_remote_callsign);
                }
            }
        }
    }

    // Check RTP audio traffic
    if (FD_ISSET(s_rtp_sock, &read_fds))
    {
        uint8_t buf[256];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t n = recvfrom(s_rtp_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        if (n > 0)
        {
            s_last_rx_packet_time = millis();

            if (echolink::isRTPAudioPacket(buf, n))
            {
                if (s_session_state == ClientSessionState::CONNECTING)
                {
                    s_session_state = ClientSessionState::CONNECTED;
                    system_state_set_station(StationState::CONNECTED, s_remote_callsign, s_remote_node);
                    Serial.printf("[EchoLink] Connected to %s!\n", s_remote_callsign);
                    Announcer::instance().announceConnected(s_remote_node);
                }

                // Extract 4 GSM frames (132 bytes) starting at byte 12
                const uint8_t *gsm_payload = buf + 12;
                int16_t pcm_bundle[GSM_BUNDLE_SAMPLES];

                // Decode 4 frames (640 samples / 80 ms)
                gsm_decode_4frames(gsm_payload, pcm_bundle);
                dtmf_detector_process(pcm_bundle, GSM_BUNDLE_SAMPLES);

                // Push each of the 4 160-sample frames into the Jitter Buffer
                const int16_t *pcm_frame = pcm_bundle;
                for (int i = 0; i < GSM_BUNDLE_FRAMES; i++)
                {
                    audio_pipeline_write_frame(pcm_frame);
                    pcm_frame += GSM_FRAME_SAMPLES;
                }

                // Calculate Peak RX Level for audio meter
                int16_t max_peak = 0;
                for (int i = 0; i < GSM_BUNDLE_SAMPLES; i++)
                {
                    int16_t abs_s = abs(pcm_bundle[i]);
                    if (abs_s > max_peak)
                        max_peak = abs_s;
                }
                int16_t rx_pct = (max_peak * 100) / 24000;
                if (rx_pct > 100)
                    rx_pct = 100;
                system_state_update_rx_level((uint16_t)max_peak, rx_pct);

                s_last_audio_rx_time = millis();
                system_state_set_rx(true);

                static uint32_t s_last_direct_rx_log = 0;
                if (millis() - s_last_direct_rx_log >= 1000)
                {
                    s_last_direct_rx_log = millis();
                    Serial.printf("[EchoLink] Audio RX from %s: peak %d (%d%%)\n",
                                  s_remote_callsign, max_peak, rx_pct);
                }
            }
            else if (echolink::isOnDataPacket(buf, n))
            {
                if (s_session_state == ClientSessionState::CONNECTING)
                {
                    s_session_state = ClientSessionState::CONNECTED;
                    system_state_set_station(StationState::CONNECTED, s_remote_callsign, s_remote_node);
                    Serial.printf("[EchoLink] Connected to %s!\n", s_remote_callsign);
                }
                // Station text or chat message
                Serial.printf("[EchoLink] Data message: %.*s\n", (int)n, (const char *)buf);
            }
        }
    }
}

static void service_tx_audio()
{
    ConfigData cfg = config_manager_get();
    if (!cfg.proxy_enabled && (s_rtp_sock < 0 || s_rtcp_sock < 0))
        return;
    if (cfg.proxy_enabled && !echolink_proxy_is_connected())
        return;

    // Check PTT state
    if (s_ptt_requested)
    {
        if (!s_tx_active)
        {
            s_tx_active = true;
            s_tx_pcm_samples_accum = 0;
            system_state_set_ptt(true);
            Serial.printf("[EchoLink] TX activated (Source: %u) - transmitting audio\n", (unsigned int)s_tx_source);
            if (s_tx_source == TxSource::VOX)
            {
                uint8_t pre_frames = audio_pipeline_get_vox_preroll_delay();
                Serial.printf("[EchoLink] VOX TX started: sending %u pre-roll frames (%u ms) first\n",
                              pre_frames, (unsigned int)(pre_frames * 20));
            }
        }

        // While an announcement is going out, ignore the microphone (mute mic path into TX)
        if (s_tx_source == TxSource::ANNOUNCEMENT)
        {
            return;
        }

        // Pull available samples from ADC audio capture into 640-sample bundle
        for (;;)
        {
            size_t needed = GSM_BUNDLE_SAMPLES - s_tx_pcm_samples_accum;
            size_t got = audio_pipeline_read_samples(s_tx_pcm_buf + s_tx_pcm_samples_accum, needed, 0);
            if (got == 0)
                break;
            s_tx_pcm_samples_accum += got;

            if (s_tx_pcm_samples_accum >= GSM_BUNDLE_SAMPLES)
            {
                // Encode 4 frames into 132 GSM bytes
                uint8_t gsm_payload[GSM_BUNDLE_BYTES];
                gsm_encode_4frames(s_tx_pcm_buf, gsm_payload);

                // Build 144-byte RTP packet
                uint8_t rtp_packet[144];
                uint32_t rtp_len = echolink::formatRTPPacket(s_tx_seq++, s_ssrc,
                                                             gsm_payload, rtp_packet, sizeof(rtp_packet));
                if (rtp_len == 144)
                {
                    if (cfg.proxy_enabled)
                    {
                        echolink_proxy_send_udp_data(s_remote_rtp_addr.sin_addr, rtp_packet, 144);
                    }
                    else if (s_rtp_sock >= 0)
                    {
                        sendto(s_rtp_sock, rtp_packet, 144, 0,
                               (struct sockaddr *)&s_remote_rtp_addr, sizeof(s_remote_rtp_addr));
                    }
                }

                s_tx_pcm_samples_accum = 0;
            }
        }
    }
    else
    {
        if (s_tx_active)
        {
            s_tx_active = false;
            s_tx_source = TxSource::NONE;
            system_state_set_ptt(false);
            audio_pipeline_clear_tx();
            Serial.println("[EchoLink] PTT released - transmission ended");

            // Flush remaining partial audio buffer if any (zero pad to full 640-sample bundle)
            if (s_tx_pcm_samples_accum > 0)
            {
                memset(s_tx_pcm_buf + s_tx_pcm_samples_accum, 0,
                       (GSM_BUNDLE_SAMPLES - s_tx_pcm_samples_accum) * sizeof(int16_t));

                uint8_t gsm_payload[GSM_BUNDLE_BYTES];
                gsm_encode_4frames(s_tx_pcm_buf, gsm_payload);

                uint8_t rtp_packet[144];
                uint32_t rtp_len = echolink::formatRTPPacket(s_tx_seq++, s_ssrc,
                                                             gsm_payload, rtp_packet, sizeof(rtp_packet));
                if (rtp_len == 144)
                {
                    if (cfg.proxy_enabled)
                    {
                        echolink_proxy_send_udp_data(s_remote_rtp_addr.sin_addr, rtp_packet, 144);
                    }
                    else if (s_rtp_sock >= 0)
                    {
                        sendto(s_rtp_sock, rtp_packet, 144, 0,
                               (struct sockaddr *)&s_remote_rtp_addr, sizeof(s_remote_rtp_addr));
                    }
                }
                s_tx_pcm_samples_accum = 0;
            }

            // Send RTCP OVER packet
            uint8_t over_buf[16];
            uint32_t over_len = echolink::formatRTCPPacket_OVER(s_ssrc, over_buf, sizeof(over_buf));
            if (over_len > 0)
            {
                if (cfg.proxy_enabled)
                {
                    echolink_proxy_send_udp_ctrl(s_remote_rtcp_addr.sin_addr, over_buf, over_len);
                }
                else if (s_rtcp_sock >= 0)
                {
                    sendto(s_rtcp_sock, over_buf, over_len, 0,
                           (struct sockaddr *)&s_remote_rtcp_addr, sizeof(s_remote_rtcp_addr));
                }
            }
        }
    }
}
