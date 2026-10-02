/**
 * @file echolink_protocol.h
 * @brief EchoLink protocol packet formatters and parsers.
 *
 * MicroLink ESP32-C6 EchoLink Node
 * Adapted from Bruce MacKinnon (KC1FSZ) MicroLink
 * GPL-3.0 License
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>

#define ECHOLINK_VERSION_ID "0.02MLZ"
#define ECHOLINK_DEFAULT_ADDR_SERVER "naeast.echolink.org"
#define ECHOLINK_DEFAULT_ADDR_PORT 5200
#define ECHOLINK_RTP_PORT 5198
#define ECHOLINK_RTCP_PORT 5199

namespace echolink
{

    // Basic network endianness helpers
    void writeInt32(uint8_t *buf, uint32_t d);
    uint32_t readInt32(const uint8_t *buf);

    // Packet inspectors
    bool isRTPAudioPacket(const uint8_t *d, uint32_t len);
    bool isRTCPPacket(const uint8_t *d, uint32_t len);
    bool isRTCPSDESPacket(const uint8_t *d, uint32_t len);
    bool isRTCPPINGPacket(const uint8_t *d, uint32_t len);
    bool isRTCPOPENPacket(const uint8_t *d, uint32_t len);
    bool isRTCPByePacket(const uint8_t *d, uint32_t len);
    bool isOnDataPacket(const uint8_t *d, uint32_t len);

    // Packet formatters
    uint32_t formatRTPPacket(uint16_t seq, uint32_t ssrc,
                             const uint8_t *gsmFrames4x33,
                             uint8_t *packet, uint32_t packetSize);

    uint32_t formatRTCPPacket_SDES(uint32_t ssrc,
                                   const char *callSign,
                                   const char *fullName,
                                   uint32_t ssrc2,
                                   uint8_t *packet, uint32_t packetSize);

    uint32_t formatRTCPPacket_PING(uint32_t ssrc,
                                   const char *callSign,
                                   uint8_t *packet, uint32_t packetSize);

    uint32_t formatRTCPPacket_OVER(uint32_t ssrc,
                                   uint8_t *packet, uint32_t packetSize);

    uint32_t formatRTCPPacket_BYE(uint32_t ssrc,
                                  uint8_t *packet, uint32_t packetSize);

    uint32_t formatOnDataPacket(const char *msg, uint32_t ssrc,
                                uint8_t *packet, uint32_t packetSize);

    uint32_t createOnlineMessage(uint8_t *buf, uint32_t bufLen,
                                 const char *callsign, const char *password,
                                 const char *location, const char *versionId);

    // SDES Item parser
    struct SDESItem
    {
        uint8_t type;
        uint8_t len;
        uint8_t content[256];

        void toString(char *str, uint32_t strSize) const
        {
            uint32_t n = (len < strSize - 1) ? len : (strSize - 1);
            memcpy(str, content, n);
            str[n] = 0;
        }
    };

    uint32_t parseSDES(const uint8_t *packet, uint32_t packetLen,
                       uint32_t *ssrc,
                       SDESItem *items, uint32_t itemsSize);

    bool extractCallSignFromSDES(const uint8_t *data, uint32_t dataLen,
                                 char *outCallSign, size_t outSize);

} // namespace echolink
