#pragma once

#include <Arduino.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

/**
 * @file echolink_proxy.h
 * @brief EchoLink Public & Private Proxy Protocol Client for ESP32-C6
 *
 * Implements the EchoLink Proxy Protocol (co-authored by K1RFD & SM0SVX)
 * to tunnel EchoLink VoIP and directory traffic over a single TCP connection (port 8100).
 * Bypasses NAT, CGNAT (4G/5G), and restrictive firewalls without port forwarding.
 */

enum class EchoLinkProxyMsgType : uint8_t
{
    TCP_OPEN    = 1,
    TCP_DATA    = 2,
    TCP_CLOSE   = 3,
    TCP_STATUS  = 4,
    UDP_DATA    = 5,
    UDP_CONTROL = 6,
    SYSTEM      = 7
};

/**
 * @brief Connect to an EchoLink proxy server and authenticate using MD5 challenge-response.
 * @param host IP address or hostname of proxy server
 * @param port TCP port of proxy server (typically 8100)
 * @param callsign Client station callsign (e.g. "HS1ABC-L")
 * @param password Proxy password (defaults to "PUBLIC" for public proxies)
 * @return true if connected and authenticated successfully
 */
bool echolink_proxy_connect(const char *host, uint16_t port, const char *callsign, const char *password);

/**
 * @brief Disconnect from proxy server and close socket.
 */
void echolink_proxy_disconnect();

/**
 * @brief Check if currently connected and authenticated with proxy.
 */
bool echolink_proxy_is_connected();

/**
 * @brief Get the underlying TCP socket descriptor for select() polling.
 */
int echolink_proxy_get_socket();

/**
 * @brief Send RTP audio / oNDATA packet to remote station via proxy tunnel (UDP 5198 emulation).
 */
bool echolink_proxy_send_udp_data(struct in_addr remote_ip, const void *data, uint32_t len);

/**
 * @brief Send RTCP control packet to remote station via proxy tunnel (UDP 5199 emulation).
 */
bool echolink_proxy_send_udp_ctrl(struct in_addr remote_ip, const void *data, uint32_t len);

/**
 * @brief Perform a complete TCP transaction to a destination through proxy tunnel (TCP 5200 emulation).
 * Used for addressing server registration and station lookup.
 * Performs TCP_OPEN -> sends request via TCP_DATA -> receives response -> sends TCP_CLOSE.
 */
bool echolink_proxy_tcp_transaction(struct in_addr dest_ip, const void *req, size_t req_len,
                                    void *resp, size_t max_resp_len, size_t *out_resp_len,
                                    uint32_t timeout_ms = 5000);

/**
 * @brief Non-blocking check and read of a single proxy message frame.
 * @return true if a complete message was received and unpacked.
 */
bool echolink_proxy_recv_msg(EchoLinkProxyMsgType *out_type, struct in_addr *out_remote_ip,
                             uint8_t *out_data, size_t max_data_len, size_t *out_data_len);
