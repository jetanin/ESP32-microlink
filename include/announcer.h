#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstddef>
#include "audio_common.h"

/**
 * @file announcer.h
 * @brief Spoken Voice & Beep Announcement Engine for MicroLink
 *
 * Supports multi-destination audio routing:
 *   - ROUTE_LOCAL: Local PCM5102A DAC / Speaker output
 *   - ROUTE_TX   : Transmit audio stream via VoIP RTP to EchoLink network
 *
 * Rules:
 *   - ROUTE_TX requests TX through txRequest(source=ANNOUNCEMENT) / txRelease()
 *   - Transmitter kept keyed for the entire announcement duration + 200 ms tail
 *   - Microphone is muted during announcement to avoid speaker bleed and noise
 *   - VOX and DTMF detectors are inhibited while busy() and for 300 ms after
 *   - Channel busy deferral up to 5 s (drops sensitivity/mode, keeps station events)
 *   - Operator PTT/VOX priority immediately aborts announcement and hands TX without gap
 */

// Audio Routing Flags (combinable bitmask)
constexpr uint8_t ROUTE_LOCAL = 0x01; // PCM5102A speaker
constexpr uint8_t ROUTE_TX    = 0x02; // Transmit audio path

#ifndef ROUTE_DEFAULT_STATION_EVENTS
#define ROUTE_DEFAULT_STATION_EVENTS (ROUTE_LOCAL | ROUTE_TX)
#endif

#ifndef ROUTE_DEFAULT_MODE_EVENTS
#define ROUTE_DEFAULT_MODE_EVENTS (ROUTE_LOCAL)
#endif

// TX Request Source
enum class TxSource : uint8_t {
    NONE = 0,
    PTT_BUTTON,
    VOX,
    ANNOUNCEMENT,
    WEB_UI
};

// Audio Sink Callback (called with 160-sample PCM chunks = 20 ms at 8 kHz)
typedef void (*AnnouncerAudioSink)(const int16_t *samples, size_t count);
typedef bool (*ChannelBusyHook)();

struct AnnouncementItem {
    char text[64];
    uint8_t route;
    bool is_station_event;
    uint32_t queued_time_ms;
};

class Announcer {
public:
    static Announcer& instance();

    /**
     * @brief Initialize announcer engine with local and TX audio sinks
     * @param local_sink Callback to deliver audio chunks to local speaker (DAC)
     * @param tx_sink Callback to deliver audio chunks to EchoLink TX stream (RTP)
     * @param channel_busy_fn Optional hook returning true if RF/VoIP channel is busy
     */
    void begin(AnnouncerAudioSink local_sink, AnnouncerAudioSink tx_sink, ChannelBusyHook channel_busy_fn = nullptr);

    /**
     * @brief Queue a spoken/beep announcement
     * @param text Words or digits separated by space (e.g. "connected 9999", "vox mode")
     * @param route Bitmask of ROUTE_LOCAL, ROUTE_TX
     */
    void say(const char *text, uint8_t route = ROUTE_LOCAL);

    /**
     * @brief Abort current announcement within 1 audio chunk and clear queue
     */
    void abort();

    /**
     * @brief Check if announcer is currently active
     */
    bool busy() const;

    /**
     * @brief Check if VOX / DTMF input should remain inhibited (busy + 300 ms)
     */
    bool is_vox_inhibited() const;

    // Station Event Announcements
    void announceConnected(uint32_t node);
    void announceConnected(const char* node_str);
    void announceDisconnected(uint32_t node);
    void announceDisconnected(const char* node_str);
    void announceDisconnectedAll();
    void announceNotConnected();

    // Mode & Sensitivity Announcements
    void announcePttMode();
    void announceVoxMode();
    void announceSensitivity(uint8_t digit);

    // Main update loop (pumps audio chunks, handles TX tail and timeouts)
    void update();

    // Beep Fallback Generator for words
    static size_t generateWordBeeps(const char *word, int16_t *out_samples, size_t max_samples);

    Announcer();
    ~Announcer() = default;

private:
    void processNextItem();
    void generateAudioChunk(const int16_t *chunk, size_t count);
    void releaseTxTail();

    AnnouncerAudioSink m_local_sink;
    AnnouncerAudioSink m_tx_sink;
    ChannelBusyHook m_channel_busy;

    // Queue storage (small depth to prevent flooding)
    static constexpr size_t QUEUE_CAPACITY = 4;
    AnnouncementItem m_queue[QUEUE_CAPACITY];
    size_t m_queue_head;
    size_t m_queue_tail;
    size_t m_queue_count;

    // Playback state
    AnnouncementItem m_current_item;
    bool m_active;
    volatile bool m_abort_requested;
    bool m_tx_keyed;
    uint32_t m_tail_start_time;
    uint32_t m_inhibit_until_ms;

    // Audio generation buffer for current word/sound
    int16_t m_word_samples[1600]; // up to 200 ms of audio (10 frames)
    size_t m_word_sample_count;
    size_t m_word_sample_pos;

    // Word parsing within current item
    char m_remaining_words[64];
    uint32_t m_last_chunk_time;
};

// Global convenience functions
void announcer_init(AnnouncerAudioSink local_sink, AnnouncerAudioSink tx_sink, ChannelBusyHook channel_busy_fn = nullptr);
void announcer_say(const char *text, uint8_t route);
void announcer_abort();
bool announcer_busy();
bool announcer_is_vox_inhibited();
void announcer_update();
