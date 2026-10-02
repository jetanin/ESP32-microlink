#include "echolink_proxy.h"
#include <MD5Builder.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>

static int s_proxy_sock = -1;
static bool s_proxy_connected = false;

static bool send_msg_block(EchoLinkProxyMsgType type, struct in_addr remote_ip, const void *data, uint32_t len)
{
    if (s_proxy_sock < 0 || !s_proxy_connected)
    {
        return false;
    }

    // Proxy message header: 1 byte type, 4 bytes IP address, 4 bytes length (little-endian)
    uint8_t hdr[9];
    hdr[0] = static_cast<uint8_t>(type);
    memcpy(&hdr[1], &remote_ip.s_addr, 4);
    hdr[5] = static_cast<uint8_t>(len & 0xff);
    hdr[6] = static_cast<uint8_t>((len >> 8) & 0xff);
    hdr[7] = static_cast<uint8_t>((len >> 16) & 0xff);
    hdr[8] = static_cast<uint8_t>((len >> 24) & 0xff);

    ssize_t ret = send(s_proxy_sock, hdr, sizeof(hdr), 0);
    if (ret != sizeof(hdr))
    {
        echolink_proxy_disconnect();
        return false;
    }

    if (len > 0 && data != nullptr)
    {
        ret = send(s_proxy_sock, data, len, 0);
        if (ret != (ssize_t)len)
        {
            echolink_proxy_disconnect();
            return false;
        }
    }
    return true;
}

bool echolink_proxy_connect(const char *host, uint16_t port, const char *callsign, const char *password)
{
    echolink_proxy_disconnect();

    if (!host || strlen(host) == 0)
    {
        return false;
    }
    if (port == 0)
    {
        port = 8100;
    }

    const char *pwd = (password && strlen(password) > 0) ? password : "PUBLIC";

    Serial.printf("[EchoLink Proxy] Resolving proxy host: %s...\n", host);
    struct hostent *he = gethostbyname(host);
    if (!he || !he->h_addr_list[0])
    {
        Serial.printf("[EchoLink Proxy] DNS resolution failed for %s\n", host);
        return false;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    memcpy(&serv_addr.sin_addr, he->h_addr_list[0], sizeof(struct in_addr));

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &serv_addr.sin_addr, ip_str, sizeof(ip_str));
    Serial.printf("[EchoLink Proxy] Connecting to TCP %s:%u...\n", ip_str, port);

    s_proxy_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (s_proxy_sock < 0)
    {
        return false;
    }

    // Set connection timeout (5 seconds)
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(s_proxy_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
    setsockopt(s_proxy_sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv));

    if (connect(s_proxy_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        Serial.println(F("[EchoLink Proxy] Connection failed or timed out"));
        close(s_proxy_sock);
        s_proxy_sock = -1;
        return false;
    }

    // Step 1: Wait for 8-byte ASCII nonce from proxy server
    char nonce[8] = {0};
    size_t n_read = 0;
    uint32_t t0 = millis();
    while (n_read < 8 && (millis() - t0 < 5000))
    {
        ssize_t r = recv(s_proxy_sock, nonce + n_read, 8 - n_read, 0);
        if (r > 0)
        {
            n_read += r;
        }
        else if (r < 0 && errno != EAGAIN)
        {
            break;
        }
        delay(10);
    }

    if (n_read < 8)
    {
        Serial.println(F("[EchoLink Proxy] Failed to receive 8-byte challenge nonce"));
        close(s_proxy_sock);
        s_proxy_sock = -1;
        return false;
    }

    // Step 2: Compute 16-byte MD5 digest = MD5( UPPERCASE(password) + nonce[8] )
    String pass_upper = pwd;
    pass_upper.toUpperCase();

    MD5Builder md5;
    md5.begin();
    md5.add((const uint8_t *)pass_upper.c_str(), pass_upper.length());
    md5.add((const uint8_t *)nonce, 8);
    md5.calculate();

    uint8_t digest[16];
    md5.getBytes(digest);

    // Step 3: Send authentication payload: <CALLSIGN>\n<16_BYTE_MD5>
    uint8_t auth_buf[64];
    size_t call_len = strlen(callsign);
    memcpy(auth_buf, callsign, call_len);
    auth_buf[call_len] = '\n';
    memcpy(auth_buf + call_len + 1, digest, 16);
    size_t total_auth_len = call_len + 1 + 16;

    if (send(s_proxy_sock, auth_buf, total_auth_len, 0) != (ssize_t)total_auth_len)
    {
        Serial.println(F("[EchoLink Proxy] Failed to send authentication credentials"));
        close(s_proxy_sock);
        s_proxy_sock = -1;
        return false;
    }

    // Step 4: Configure socket for non-blocking operation
    fcntl(s_proxy_sock, F_SETFL, O_NONBLOCK);
    s_proxy_connected = true;

    Serial.printf("[EchoLink Proxy] Connected and authenticated to %s:%u (Callsign: %s)\n",
                  host, port, callsign);
    return true;
}

void echolink_proxy_disconnect()
{
    s_proxy_connected = false;
    if (s_proxy_sock >= 0)
    {
        close(s_proxy_sock);
        s_proxy_sock = -1;
        Serial.println(F("[EchoLink Proxy] Tunnel closed"));
    }
}

bool echolink_proxy_is_connected()
{
    return (s_proxy_sock >= 0 && s_proxy_connected);
}

int echolink_proxy_get_socket()
{
    return s_proxy_sock;
}

bool echolink_proxy_send_udp_data(struct in_addr remote_ip, const void *data, uint32_t len)
{
    return send_msg_block(EchoLinkProxyMsgType::UDP_DATA, remote_ip, data, len);
}

bool echolink_proxy_send_udp_ctrl(struct in_addr remote_ip, const void *data, uint32_t len)
{
    return send_msg_block(EchoLinkProxyMsgType::UDP_CONTROL, remote_ip, data, len);
}

bool echolink_proxy_tcp_open(struct in_addr dest_ip)
{
    return send_msg_block(EchoLinkProxyMsgType::TCP_OPEN, dest_ip, nullptr, 0);
}

bool echolink_proxy_tcp_send(struct in_addr dest_ip, const void *data, uint32_t len)
{
    return send_msg_block(EchoLinkProxyMsgType::TCP_DATA, dest_ip, data, len);
}

bool echolink_proxy_tcp_close(struct in_addr dest_ip)
{
    return send_msg_block(EchoLinkProxyMsgType::TCP_CLOSE, dest_ip, nullptr, 0);
}

bool echolink_proxy_tcp_transaction(struct in_addr dest_ip, const void *req, size_t req_len,
                                    void *resp, size_t max_resp_len, size_t *out_resp_len,
                                    uint32_t timeout_ms)
{
    if (s_proxy_sock < 0 || !s_proxy_connected)
    {
        return false;
    }

    // 1. Request proxy to open TCP port 5200 to dest_ip
    if (!send_msg_block(EchoLinkProxyMsgType::TCP_OPEN, dest_ip, nullptr, 0))
    {
        return false;
    }

    // 2. Wait for TCP_STATUS
    uint32_t t_start = millis();
    bool opened = false;
    while (millis() - t_start < timeout_ms)
    {
        EchoLinkProxyMsgType type;
        struct in_addr from_ip;
        uint8_t buf[256];
        size_t len = 0;
        if (echolink_proxy_recv_msg(&type, &from_ip, buf, sizeof(buf), &len))
        {
            if (type == EchoLinkProxyMsgType::TCP_STATUS && len >= 4)
            {
                uint32_t status = buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
                if (status == 0)
                {
                    opened = true;
                    break;
                }
                else
                {
                    Serial.printf("[EchoLink Proxy] Destination TCP_OPEN rejected (status %u)\n", status);
                    return false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!opened)
    {
        Serial.println(F("[EchoLink Proxy] Destination TCP_OPEN timed out"));
        send_msg_block(EchoLinkProxyMsgType::TCP_CLOSE, dest_ip, nullptr, 0);
        return false;
    }

    // 3. Send Request Data
    if (!send_msg_block(EchoLinkProxyMsgType::TCP_DATA, dest_ip, req, req_len))
    {
        send_msg_block(EchoLinkProxyMsgType::TCP_CLOSE, dest_ip, nullptr, 0);
        return false;
    }

    // 4. Receive Response Data
    bool received = false;
    size_t resp_accum = 0;
    t_start = millis();
    while (millis() - t_start < timeout_ms)
    {
        EchoLinkProxyMsgType type;
        struct in_addr from_ip;
        uint8_t buf[512];
        size_t len = 0;
        if (echolink_proxy_recv_msg(&type, &from_ip, buf, sizeof(buf), &len))
        {
            if (type == EchoLinkProxyMsgType::TCP_DATA && len > 0)
            {
                size_t copy_len = std::min(len, max_resp_len - 1 - resp_accum);
                memcpy((uint8_t *)resp + resp_accum, buf, copy_len);
                resp_accum += copy_len;
                received = true;
                break;
            }
            else if (type == EchoLinkProxyMsgType::TCP_CLOSE)
            {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 5. Close TCP
    send_msg_block(EchoLinkProxyMsgType::TCP_CLOSE, dest_ip, nullptr, 0);

    if (received)
    {
        if (out_resp_len)
        {
            *out_resp_len = resp_accum;
        }
        ((char *)resp)[resp_accum] = '\0';
    }
    return received;
}

static bool read_exact(int sock, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t total = 0;
    uint32_t t0 = millis();
    while (total < len)
    {
        ssize_t r = recv(sock, buf + total, len - total, 0);
        if (r > 0)
        {
            total += r;
        }
        else if (r == 0)
        {
            return false; // Socket closed
        }
        else if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            return false; // Socket error
        }
        else
        {
            if (millis() - t0 > timeout_ms)
            {
                return false; // Timeout
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return true;
}

bool echolink_proxy_recv_msg(EchoLinkProxyMsgType *out_type, struct in_addr *out_remote_ip,
                             uint8_t *out_data, size_t max_data_len, size_t *out_data_len)
{
    if (s_proxy_sock < 0 || !s_proxy_connected)
    {
        return false;
    }

    // Peek 9-byte header
    uint8_t hdr[9];
    ssize_t n = recv(s_proxy_sock, hdr, 9, MSG_PEEK);
    if (n < 9)
    {
        return false;
    }

    // Safely consume the full 9-byte header
    if (!read_exact(s_proxy_sock, hdr, 9, 200))
    {
        Serial.println(F("[EchoLink Proxy] Failed to read header"));
        echolink_proxy_disconnect();
        return false;
    }

    uint32_t msg_len = hdr[5] | ((uint32_t)hdr[6] << 8) | ((uint32_t)hdr[7] << 16) | ((uint32_t)hdr[8] << 24);
    if (msg_len > 4096)
    {
        Serial.printf("[EchoLink Proxy] Corrupted frame (size %u) - resetting proxy socket\n", msg_len);
        echolink_proxy_disconnect();
        return false;
    }

    *out_type = static_cast<EchoLinkProxyMsgType>(hdr[0]);
    memcpy(&out_remote_ip->s_addr, &hdr[1], 4);

    if (msg_len == 0)
    {
        *out_data_len = 0;
        return true;
    }

    // Read payload into destination buffer
    size_t to_copy = std::min((size_t)msg_len, max_data_len);
    if (!read_exact(s_proxy_sock, out_data, to_copy, 500))
    {
        Serial.println(F("[EchoLink Proxy] Failed to read payload"));
        echolink_proxy_disconnect();
        return false;
    }

    // If packet size exceeds out_data buffer, consume excess bytes to avoid stream misalignment
    if (msg_len > to_copy)
    {
        size_t excess = msg_len - to_copy;
        uint8_t discard[64];
        while (excess > 0)
        {
            size_t c = std::min(excess, sizeof(discard));
            if (!read_exact(s_proxy_sock, discard, c, 200))
            {
                echolink_proxy_disconnect();
                return false;
            }
            excess -= c;
        }
    }

    *out_data_len = to_copy;
    return true;
}
