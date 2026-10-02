/**
 * @file echolink_protocol.cpp
 * @brief EchoLink protocol packet formatters and parsers.
 *
 * MicroLink ESP32-C6 EchoLink Node
 * Adapted from Bruce MacKinnon (KC1FSZ) MicroLink
 * GPL-3.0 License
 */

#include "echolink_protocol.h"
#include <cstdio>
#include <ctime>
#include <algorithm>

namespace echolink
{

    void writeInt32(uint8_t *buf, uint32_t d)
    {
        buf[0] = (d >> 24) & 0xff;
        buf[1] = (d >> 16) & 0xff;
        buf[2] = (d >> 8) & 0xff;
        buf[3] = (d) & 0xff;
    }

    uint32_t readInt32(const uint8_t *buf)
    {
        uint32_t r = 0;
        r |= buf[0];
        r <<= 8;
        r |= buf[1];
        r <<= 8;
        r |= buf[2];
        r <<= 8;
        r |= buf[3];
        return r;
    }

    bool isRTPAudioPacket(const uint8_t *d, uint32_t len)
    {
        if (len < 144 || d == nullptr)
            return false;
        // RTP Version 2 (0x80) or Version 3 (0xc0)
        uint8_t ver = (d[0] >> 6) & 3;
        if (ver != 2 && ver != 3 && ver != 1)
            return false;
        // GSM 06.10 payload type is 3 (masking out marker bit 0x80 in d[1])
        return ((d[1] & 0x7f) == 0x03);
    }

    bool isRTCPPacket(const uint8_t *d, uint32_t len)
    {
        if (len < 4 || d == nullptr)
            return false;
        uint8_t ver = (d[0] >> 6) & 3;
        if (ver != 2 && ver != 3 && ver != 1)
            return false;
        // RTCP payload types: 200 (SR), 201 (RR), 202 (SDES), 203 (BYE), 204 (APP)
        return (d[1] >= 200 && d[1] <= 204);
    }

    bool isRTCPSDESPacket(const uint8_t *d, uint32_t len)
    {
        if (!isRTCPPacket(d, len) || len < 8)
            return false;
        // Scan compound RTCP packets for type 202 (SDES)
        uint32_t offset = 0;
        while (offset + 4 <= len)
        {
            uint8_t pt = d[offset + 1];
            if (pt == 202 || pt == 0xca)
                return true;
            uint16_t plen = (d[offset + 2] << 8) | d[offset + 3];
            uint32_t step = (plen + 1) * 4;
            if (step == 0 || offset + step > len)
                break;
            offset += step;
        }
        return false;
    }

    bool isRTCPPINGPacket(const uint8_t *d, uint32_t len)
    {
        if (!isRTCPSDESPacket(d, len) || len < 16)
            return false;
        const uint8_t target[6] = {0x05, 0x04, 'P', 'I', 'N', 'G'};
        for (uint32_t i = 8; i <= len - 6; i++)
        {
            if (memcmp(d + i, target, 6) == 0)
                return true;
        }
        return false;
    }

    bool isRTCPOPENPacket(const uint8_t *d, uint32_t len)
    {
        if (!isRTCPSDESPacket(d, len) || len < 16)
            return false;
        const uint8_t target[6] = {0x05, 0x04, 'O', 'P', 'E', 'N'};
        for (uint32_t i = 8; i <= len - 6; i++)
        {
            if (memcmp(d + i, target, 6) == 0)
                return true;
        }
        return false;
    }

    bool isRTCPByePacket(const uint8_t *d, uint32_t len)
    {
        if (!isRTCPPacket(d, len) || len < 8)
            return false;
        // Scan compound RTCP packets for type 203 (BYE)
        uint32_t offset = 0;
        while (offset + 4 <= len)
        {
            uint8_t pt = d[offset + 1];
            if (pt == 203 || pt == 0xcb)
                return true;
            uint16_t plen = (d[offset + 2] << 8) | d[offset + 3];
            uint32_t step = (plen + 1) * 4;
            if (step == 0 || offset + step > len)
                break;
            offset += step;
        }
        return false;
    }

    bool isOnDataPacket(const uint8_t *d, uint32_t len)
    {
        return (len >= 6 && d != nullptr && memcmp(d, "oNDATA", 6) == 0);
    }

    static uint32_t addRTCPPad(uint32_t unpaddedLength, uint8_t *p, uint32_t packetSize)
    {
        uint32_t padSize = 4 - (unpaddedLength % 4);
        if (packetSize < unpaddedLength + padSize)
            return 0;

        p += unpaddedLength;
        for (uint32_t i = 0; i < padSize; i++)
        {
            *(p++) = 0x00;
        }
        *(p - 1) = (uint8_t)padSize;
        return padSize;
    }

    uint32_t formatRTPPacket(uint16_t seq, uint32_t ssrc,
                             const uint8_t *gsmFrames4x33,
                             uint8_t *p, uint32_t packetSize)
    {
        if (packetSize < 144 || !gsmFrames4x33 || !p)
            return 0;

        *(p++) = 0xc0;
        *(p++) = 0x03;

        // Sequence # (big endian)
        *(p++) = (seq >> 8) & 0xff;
        *(p++) = (seq) & 0xff;

        // Timestamp
        *(p++) = 0x00;
        *(p++) = 0x00;
        *(p++) = 0x00;
        *(p++) = 0x00;

        // SSRC
        writeInt32(p, ssrc);
        p += 4;

        // 4 GSM frames (132 bytes)
        memcpy(p, gsmFrames4x33, 4 * 33);
        return 144;
    }

    uint32_t formatRTCPPacket_SDES(uint32_t ssrc,
                                   const char *callSign,
                                   const char *fullName,
                                   uint32_t ssrc2,
                                   uint8_t *p, uint32_t packetSize)
    {
        if (!callSign || !fullName || !p)
            return 0;

        uint32_t callSignLen = strlen(callSign);
        uint32_t fullNameLen = strlen(fullName);
        const uint32_t spacesLen = 9;
        const uint32_t versionLen = strlen(ECHOLINK_VERSION_ID);

        uint32_t unpaddedLength = 8 + 4 + 4;
        unpaddedLength += 10;                                        // Token 1
        unpaddedLength += 2 + callSignLen + spacesLen + fullNameLen; // Token 2
        unpaddedLength += 2 + 8;                                     // Token 3
        unpaddedLength += 2 + 8;                                     // Token 4
        unpaddedLength += 2 + versionLen;                            // Token 6
        unpaddedLength += 2 + 6;                                     // Token 8a
        unpaddedLength += 2 + 3;                                     // Token 8b

        uint32_t sdesPadSize = 4 - (unpaddedLength % 4);
        unpaddedLength += sdesPadSize;

        uint32_t padSize = addRTCPPad(unpaddedLength, p, packetSize);
        uint32_t sdesLength = (unpaddedLength + padSize - 12) / 4;

        uint8_t *out = p;
        *(out++) = 0xc0;
        *(out++) = 0xc9;
        *(out++) = 0x00;
        *(out++) = 0x01;
        writeInt32(out, ssrc);
        out += 4;

        *(out++) = 0xe1;
        *(out++) = 0xca;
        *(out++) = (sdesLength >> 8) & 0xff;
        *(out++) = (sdesLength) & 0xff;
        writeInt32(out, ssrc);
        out += 4;

        // Token 1: CALLSIGN
        *(out++) = 0x01;
        *(out++) = 0x08;
        memcpy(out, "CALLSIGN", 8);
        out += 8;

        // Token 2: CALLSIGN + 9 spaces + Full Name
        *(out++) = 0x02;
        *(out++) = (uint8_t)(callSignLen + spacesLen + fullNameLen);
        memcpy(out, callSign, callSignLen);
        out += callSignLen;
        memset(out, ' ', spacesLen);
        out += spacesLen;
        memcpy(out, fullName, fullNameLen);
        out += fullNameLen;

        // Token 3: CALLSIGN
        *(out++) = 0x03;
        *(out++) = 0x08;
        memcpy(out, "CALLSIGN", 8);
        out += 8;

        // Token 4: Hex SSRC
        char buf[9];
        snprintf(buf, 9, "%08X", (unsigned int)ssrc2);
        *(out++) = 0x04;
        *(out++) = 0x08;
        memcpy(out, buf, 8);
        out += 8;

        // Token 6: Version
        *(out++) = 0x06;
        *(out++) = versionLen;
        memcpy(out, ECHOLINK_VERSION_ID, versionLen);
        out += versionLen;

        // Token 8a: PORT 5198
        *(out++) = 0x08;
        *(out++) = 0x06;
        *(out++) = 0x01;
        *(out++) = 'P';
        *(out++) = '5';
        *(out++) = '1';
        *(out++) = '9';
        *(out++) = '8';

        // Token 8b: DTMF support (D1)
        *(out++) = 0x08;
        *(out++) = 0x03;
        *(out++) = 0x01;
        *(out++) = 'D';
        *(out++) = '1';

        // SDES padding
        for (uint32_t i = 0; i < sdesPadSize; i++)
        {
            *(out++) = 0x00;
        }

        return unpaddedLength + padSize;
    }

    uint32_t formatRTCPPacket_PING(uint32_t ssrc,
                                   const char *callSign,
                                   uint8_t *p, uint32_t packetSize)
    {
        if (!callSign || !p)
            return 0;

        uint32_t callSignLen = strlen(callSign);
        uint32_t unpaddedLength = 8 + 4 + 4;
        unpaddedLength += 2 + callSignLen; // Item 1
        unpaddedLength += 2;               // Item 2
        unpaddedLength += 2;               // Item 3
        unpaddedLength += 2;               // Item 4
        unpaddedLength += 2 + 4;           // Item 5 (PING)
        unpaddedLength += 2;               // Item 6
        unpaddedLength += 2 + 6;           // Token 8a
        unpaddedLength += 2 + 3;           // Token 8b

        uint32_t sdesPadSize = 4 - (unpaddedLength % 4);
        unpaddedLength += sdesPadSize;

        uint32_t padSize = addRTCPPad(unpaddedLength, p, packetSize);
        uint32_t sdesLength = (unpaddedLength + padSize - 12) / 4;

        uint8_t *out = p;
        *(out++) = 0xc0;
        *(out++) = 0xc9;
        *(out++) = 0x00;
        *(out++) = 0x01;
        writeInt32(out, ssrc);
        out += 4;

        *(out++) = 0xe1;
        *(out++) = 0xca;
        *(out++) = (sdesLength >> 8) & 0xff;
        *(out++) = (sdesLength) & 0xff;
        writeInt32(out, ssrc);
        out += 4;

        // Item 1: Call
        *(out++) = 0x01;
        *(out++) = (uint8_t)callSignLen;
        memcpy(out, callSign, callSignLen);
        out += callSignLen;

        *(out++) = 0x02;
        *(out++) = 0x00;
        *(out++) = 0x03;
        *(out++) = 0x00;
        *(out++) = 0x04;
        *(out++) = 0x00;

        // Item 5: PING
        *(out++) = 0x05;
        *(out++) = 0x04;
        memcpy(out, "PING", 4);
        out += 4;

        *(out++) = 0x06;
        *(out++) = 0x00;

        // Token 8a: PORT 5198
        *(out++) = 0x08;
        *(out++) = 0x06;
        *(out++) = 0x01;
        *(out++) = 'P';
        *(out++) = '5';
        *(out++) = '1';
        *(out++) = '9';
        *(out++) = '8';

        // Token 8b: DTMF
        *(out++) = 0x08;
        *(out++) = 0x03;
        *(out++) = 0x01;
        *(out++) = 'D';
        *(out++) = '1';

        for (uint32_t i = 0; i < sdesPadSize; i++)
        {
            *(out++) = 0x00;
        }

        return unpaddedLength + padSize;
    }

    uint32_t formatRTCPPacket_OVER(uint32_t ssrc, uint8_t *p, uint32_t packetSize)
    {
        if (packetSize < 12 || !p)
            return 0;
        p[0] = 0xc0;
        p[1] = 0xcc;
        p[2] = 0x00;
        p[3] = 0x02;
        writeInt32(p + 4, ssrc);
        p[8] = 'O';
        p[9] = 'V';
        p[10] = 'E';
        p[11] = 'R';
        return 12;
    }

    uint32_t formatRTCPPacket_BYE(uint32_t ssrc, uint8_t *p, uint32_t packetSize)
    {
        uint32_t unpaddedLength = 8 + 4 + 4 + 1 + 7;
        uint32_t padSize = addRTCPPad(unpaddedLength, p, packetSize);

        uint8_t *out = p;
        *(out++) = 0xc0;
        *(out++) = 0xc9;
        *(out++) = 0x00;
        *(out++) = 0x01;
        writeInt32(out, ssrc);
        out += 4;

        *(out++) = 0xe1;
        *(out++) = 0xcb;
        *(out++) = 0x00;
        *(out++) = 0x04;
        writeInt32(out, ssrc);
        out += 4;

        *(out++) = 7;
        memcpy(out, "jan2002", 7);
        out += 7;

        return unpaddedLength + padSize;
    }

    uint32_t formatOnDataPacket(const char *msg, uint32_t ssrc,
                                uint8_t *packet, uint32_t packetSize)
    {
        if (!msg || !packet)
            return 0;
        uint32_t msgLen = strlen(msg);
        uint32_t len = msgLen + 1 + 4;
        if (len > packetSize)
            return 0;

        memcpy(packet, msg, msgLen);
        packet[msgLen] = 0;
        writeInt32(packet + msgLen + 1, ssrc);
        return len;
    }

    uint32_t createOnlineMessage(uint8_t *buf, uint32_t bufLen,
                                 const char *callsign, const char *password,
                                 const char *location, const char *versionId)
    {
        if (!buf || !callsign || !password)
            return 0;

        time_t now = time(nullptr);
        struct tm tm_info;
        char time_str[8] = "12:00";
        if (localtime_r(&now, &tm_info))
        {
            strftime(time_str, sizeof(time_str), "%H:%M", &tm_info);
        }

        uint8_t *p = buf;
        (*p++) = 'l';

        size_t cs_len = strlen(callsign);
        memcpy(p, callsign, cs_len);
        p += cs_len;

        (*p++) = 0xac;
        (*p++) = 0xac;

        size_t pwd_len = strlen(password);
        memcpy(p, password, pwd_len);
        p += pwd_len;

        (*p++) = 0x0d;
        memcpy(p, "ONLINE", 6);
        p += 6;

        size_t ver_len = versionId ? strlen(versionId) : strlen(ECHOLINK_VERSION_ID);
        memcpy(p, versionId ? versionId : ECHOLINK_VERSION_ID, ver_len);
        p += ver_len;

        (*p++) = '(';
        memcpy(p, time_str, 5);
        p += 5;
        (*p++) = ')';

        (*p++) = 0x0d;
        const char *loc = location ? location : "MicroLink Node";
        size_t loc_len = strlen(loc);
        memcpy(p, loc, loc_len);
        p += loc_len;
        (*p++) = 0x0d;

        return (p - buf);
    }

    uint32_t parseSDES(const uint8_t *packet, uint32_t packetLen,
                       uint32_t *ssrc,
                       SDESItem *items, uint32_t itemsSize)
    {
        if (!packet || packetLen < 16 || !ssrc || !items || itemsSize == 0)
            return 0;

        uint8_t padCount = packet[packetLen - 1];
        if (padCount >= packetLen)
            padCount = 0;
        uint32_t len = packetLen - (uint32_t)padCount;

        *ssrc = readInt32(packet + 4);

        const uint8_t *p = packet + 16;
        uint32_t itemCount = 0;
        uint8_t itemPtr = 0;
        int state = 0;

        while (p < packet + len && itemCount < itemsSize)
        {
            if (state == 0)
            {
                items[itemCount].type = *p;
                state = 1;
            }
            else if (state == 1)
            {
                items[itemCount].len = *p;
                itemPtr = 0;
                if (*p == 0)
                    state = 0;
                else
                    state = 2;
            }
            else if (state == 2)
            {
                items[itemCount].content[itemPtr++] = *p;
                if (itemPtr == items[itemCount].len)
                {
                    state = 0;
                    itemCount++;
                }
            }
            p++;
        }
        return itemCount;
    }

    bool extractCallSignFromSDES(const uint8_t *data, uint32_t dataLen,
                                 char *outCallSign, size_t outSize)
    {
        if (!isRTCPSDESPacket(data, dataLen) || !outCallSign || outSize == 0)
            return false;

        SDESItem items[8];
        uint32_t ssrc = 0;
        uint32_t itemCount = parseSDES(data, dataLen, &ssrc, items, 8);

        for (uint32_t i = 0; i < itemCount; i++)
        {
            if (items[i].type == 2)
            { // CallSign + Name
                char buf[64];
                items[i].toString(buf, sizeof(buf));
                size_t j = 0;
                while (j < sizeof(buf) && buf[j] != 0 && buf[j] != ' ' && j < outSize - 1)
                {
                    outCallSign[j] = buf[j];
                    j++;
                }
                outCallSign[j] = 0;
                return (j > 0);
            }
        }
        return false;
    }

} // namespace echolink
