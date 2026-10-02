#include "announcer.h"
#include "audio_pipeline.h"
#include "echolink_client.h"
#include "dtmf_detector.h"
#include "system_state.h"
#include <cmath>
#include <cstring>
#include <cctype>

static Announcer s_announcer;

Announcer& Announcer::instance()
{
    return s_announcer;
}

Announcer::Announcer()
    : m_local_sink(nullptr),
      m_tx_sink(nullptr),
      m_channel_busy(nullptr),
      m_queue_head(0),
      m_queue_tail(0),
      m_queue_count(0),
      m_active(false),
      m_abort_requested(false),
      m_tx_keyed(false),
      m_tail_start_time(0),
      m_inhibit_until_ms(0),
      m_word_sample_count(0),
      m_word_sample_pos(0),
      m_last_chunk_time(0)
{
    memset(&m_current_item, 0, sizeof(m_current_item));
    memset(m_word_samples, 0, sizeof(m_word_samples));
    m_remaining_words[0] = '\0';
}

void Announcer::begin(AnnouncerAudioSink local_sink, AnnouncerAudioSink tx_sink, ChannelBusyHook channel_busy_fn)
{
    m_local_sink = local_sink;
    m_tx_sink = tx_sink;
    m_channel_busy = channel_busy_fn;
    m_queue_head = 0;
    m_queue_tail = 0;
    m_queue_count = 0;
    m_active = false;
    m_abort_requested = false;
    m_tx_keyed = false;
    m_inhibit_until_ms = 0;

    Serial.println(F("[ANNOUNCER] Engine initialized with dual-route sinks (LOCAL + TX)"));
}

bool Announcer::busy() const
{
    return m_active || (m_queue_count > 0);
}

bool Announcer::is_vox_inhibited() const
{
    return busy() || (millis() < m_inhibit_until_ms);
}

void Announcer::say(const char *text, uint8_t route)
{
    if (!text || strlen(text) == 0) return;

    // Check queue overflow
    if (m_queue_count >= QUEUE_CAPACITY)
    {
        Serial.printf("[ANNOUNCER] Queue full (%u items), dropping oldest item\n", (unsigned int)QUEUE_CAPACITY);
        m_queue_head = (m_queue_head + 1) % QUEUE_CAPACITY;
        m_queue_count--;
    }

    AnnouncementItem item{};
    strncpy(item.text, text, sizeof(item.text) - 1);
    item.route = route;
    item.queued_time_ms = millis();

    // Identify if this is a station event vs mode/sensitivity
    if (strstr(text, "connected") != nullptr || strstr(text, "disconnected") != nullptr)
    {
        item.is_station_event = true;
    }
    else
    {
        item.is_station_event = false;
    }

    m_queue[m_queue_tail] = item;
    m_queue_tail = (m_queue_tail + 1) % QUEUE_CAPACITY;
    m_queue_count++;

    Serial.printf("[ANNOUNCER] Queued '%s' (Route: 0x%02X, Queue: %u)\n",
                  item.text, item.route, (unsigned int)m_queue_count);
}

void Announcer::abort()
{
    m_abort_requested = true;
    m_queue_head = 0;
    m_queue_tail = 0;
    m_queue_count = 0;
    m_word_sample_count = 0;
    m_word_sample_pos = 0;
    m_remaining_words[0] = '\0';

    if (m_tx_keyed)
    {
        echolink_client_tx_release(TxSource::ANNOUNCEMENT);
        m_tx_keyed = false;
    }

    m_active = false;
    m_inhibit_until_ms = millis() + 300;
    Serial.println(F("[ANNOUNCER] Aborted immediately, queue cleared, TX released"));
}

void Announcer::announceConnected(uint32_t node)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "connected %lu", (unsigned long)node);
    say(buf, ROUTE_DEFAULT_STATION_EVENTS);
}

void Announcer::announceConnected(const char* node_str)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "connected %s", node_str ? node_str : "");
    say(buf, ROUTE_DEFAULT_STATION_EVENTS);
}

void Announcer::announceDisconnected(uint32_t node)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "disconnected %lu", (unsigned long)node);
    say(buf, ROUTE_DEFAULT_STATION_EVENTS);
}

void Announcer::announceDisconnected(const char* node_str)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "disconnected %s", node_str ? node_str : "");
    say(buf, ROUTE_DEFAULT_STATION_EVENTS);
}

void Announcer::announceDisconnectedAll()
{
    say("disconnected all", ROUTE_DEFAULT_STATION_EVENTS);
}

void Announcer::announceNotConnected()
{
    say("not connected", ROUTE_DEFAULT_STATION_EVENTS);
}

void Announcer::announcePttMode()
{
    say("ptt mode", ROUTE_DEFAULT_MODE_EVENTS);
}

void Announcer::announceVoxMode()
{
    say("vox mode", ROUTE_DEFAULT_MODE_EVENTS);
}

void Announcer::announceSensitivity(uint8_t digit)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "sensitivity %u", digit);
    say(buf, ROUTE_DEFAULT_MODE_EVENTS);
}

// Helper: synthesize sine wave with envelope into sample buffer
static size_t synthesizeSine(int16_t *dest, size_t max_len, float freq_hz, uint32_t duration_ms, int16_t amplitude = 18000)
{
    size_t num_samples = (duration_ms * AUDIO_SAMPLE_RATE_HZ) / 1000;
    if (num_samples > max_len) num_samples = max_len;

    const float phase_step = (2.0f * (float)M_PI * freq_hz) / (float)AUDIO_SAMPLE_RATE_HZ;
    float phase = 0.0f;
    size_t env_samples = 40; // 5 ms attack / decay

    for (size_t i = 0; i < num_samples; ++i)
    {
        float scale = 1.0f;
        if (i < env_samples) scale = (float)i / (float)env_samples;
        else if (i + env_samples > num_samples) scale = (float)(num_samples - i) / (float)env_samples;

        dest[i] = (int16_t)(sinf(phase) * (float)amplitude * scale);
        phase += phase_step;
        if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
    }
    return num_samples;
}

// Fallback beep generator for words
size_t Announcer::generateWordBeeps(const char *word, int16_t *out_samples, size_t max_samples)
{
    if (!word || !out_samples || max_samples < 160) return 0;

    size_t pos = 0;

    if (strcasecmp(word, "vox") == 0)
    {
        // vox = two short beeps (900 Hz: 60 ms on, 40 ms off, 60 ms on)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 900.0f, 60);
        size_t pause = (40 * AUDIO_SAMPLE_RATE_HZ) / 1000;
        if (pos + pause <= max_samples) { memset(out_samples + pos, 0, pause * sizeof(int16_t)); pos += pause; }
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 900.0f, 60);
    }
    else if (strcasecmp(word, "ptt") == 0)
    {
        // ptt = one long beep (1000 Hz: 220 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 1000.0f, 220);
    }
    else if (strcasecmp(word, "mode") == 0 || strcasecmp(word, "sensitivity") == 0)
    {
        // mode and sensitivity = short blip (1200 Hz: 40 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 1200.0f, 40);
    }
    else if (strcasecmp(word, "connected") == 0)
    {
        // Ascending chime: 880 Hz (90 ms) -> 1320 Hz (140 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 880.0f, 90);
        size_t pause = (20 * AUDIO_SAMPLE_RATE_HZ) / 1000;
        if (pos + pause <= max_samples) { memset(out_samples + pos, 0, pause * sizeof(int16_t)); pos += pause; }
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 1320.0f, 140);
    }
    else if (strcasecmp(word, "disconnected") == 0)
    {
        // Descending chime: 1320 Hz (90 ms) -> 880 Hz (140 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 1320.0f, 90);
        size_t pause = (20 * AUDIO_SAMPLE_RATE_HZ) / 1000;
        if (pos + pause <= max_samples) { memset(out_samples + pos, 0, pause * sizeof(int16_t)); pos += pause; }
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 880.0f, 140);
    }
    else if (strcasecmp(word, "not") == 0)
    {
        // Low buzz: 400 Hz (120 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 400.0f, 120);
    }
    else if (strcasecmp(word, "all") == 0)
    {
        // High blip: 1000 Hz (80 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 1000.0f, 80);
    }
    else if (isdigit((unsigned char)word[0]))
    {
        // Digit 0..9: distinct frequencies (600 Hz + digit * 80 Hz)
        uint8_t d = word[0] - '0';
        float f = 600.0f + ((float)d * 80.0f);
        pos += synthesizeSine(out_samples + pos, max_samples - pos, f, 100);
    }
    else
    {
        // Generic word tone: 800 Hz (80 ms)
        pos += synthesizeSine(out_samples + pos, max_samples - pos, 800.0f, 80);
    }

    // Add trailing silence gap between words (60 ms)
    size_t gap = (60 * AUDIO_SAMPLE_RATE_HZ) / 1000;
    if (pos + gap <= max_samples)
    {
        memset(out_samples + pos, 0, gap * sizeof(int16_t));
        pos += gap;
    }

    // Pad to multiple of AUDIO_FRAME_SAMPLES (160)
    size_t rem = pos % AUDIO_FRAME_SAMPLES;
    if (rem != 0)
    {
        size_t pad = AUDIO_FRAME_SAMPLES - rem;
        if (pos + pad <= max_samples)
        {
            memset(out_samples + pos, 0, pad * sizeof(int16_t));
            pos += pad;
        }
    }

    return pos;
}

void Announcer::processNextItem()
{
    if (m_queue_count == 0) return;

    AnnouncementItem &next = m_queue[m_queue_head];

    // Check ROUTE_TX channel availability
    if (next.route & ROUTE_TX)
    {
        // Downgrade to LOCAL-only if not connected to any EchoLink station
        if (!echolink_client_is_connected())
        {
            next.route &= ~ROUTE_TX; // Remove TX flag, keep LOCAL
        }
        else
        {
            bool busy_chan = (m_channel_busy && m_channel_busy());
            bool busy_tx = echolink_client_is_tx_active();

            if (busy_chan || busy_tx)
            {
                uint32_t waited = millis() - next.queued_time_ms;
                if (waited >= 5000)
                {
                    if (!next.is_station_event)
                    {
                        Serial.printf("[ANNOUNCER] Channel busy for %lu ms, dropping '%s'\n", (unsigned long)waited, next.text);
                        m_queue_head = (m_queue_head + 1) % QUEUE_CAPACITY;
                        m_queue_count--;
                        return;
                    }
                    // For station events (connected/disconnected), keep waiting
                }
                return; // Defer
            }

            // Request TX through clearly marked hook
            if (!echolink_client_tx_request(TxSource::ANNOUNCEMENT))
            {
                return; // Could not acquire TX (e.g. operator has PTT engaged)
            }
            m_tx_keyed = true;
        }
    }

    // Pop from queue into active item
    m_current_item = next;
    m_queue_head = (m_queue_head + 1) % QUEUE_CAPACITY;
    m_queue_count--;

    strncpy(m_remaining_words, m_current_item.text, sizeof(m_remaining_words) - 1);
    m_remaining_words[sizeof(m_remaining_words) - 1] = '\0';
    m_word_sample_count = 0;
    m_word_sample_pos = 0;
    m_active = true;
    m_abort_requested = false;

    // Inhibit DTMF detector during announcement to prevent self-triggering
    dtmf_detector_set_enabled(false);

    Serial.printf("[ANNOUNCER] Starting announcement: '%s' (Route: 0x%02X)\n",
                  m_current_item.text, m_current_item.route);
}

void Announcer::update()
{
    // Check if abort was requested
    if (m_abort_requested)
    {
        abort();
        return;
    }

    // Inhibit maintenance: re-enable DTMF once inhibit window has expired and announcer is idle
    // Use m_inhibit_until_ms == 0 as "already re-enabled" sentinel to avoid calling every loop
    if (!busy() && m_inhibit_until_ms > 0 && millis() >= m_inhibit_until_ms)
    {
        m_inhibit_until_ms = 0; // Clear sentinel first
        dtmf_detector_set_enabled(true);
    }

    uint32_t now = millis();
    if (now - m_last_chunk_time < 20)
    {
        return; // Paced at 20 ms per frame
    }
    m_last_chunk_time = now;

    if (!m_active)
    {
        processNextItem();
        return;
    }

    // If current word buffer is empty, generate next word
    if (m_word_sample_pos >= m_word_sample_count)
    {
        // Parse next word from m_remaining_words
        char *p = m_remaining_words;
        while (*p == ' ') p++;
        if (*p == '\0')
        {
            // All words complete! Enter TX tail or finish
            if (m_tx_keyed)
            {
                // Send 200 ms of silence tail (10 frames)
                static int16_t tail_silence[AUDIO_FRAME_SAMPLES] = {0};
                if (m_tail_start_time == 0)
                {
                    m_tail_start_time = now;
                }
                if (now - m_tail_start_time < 200)
                {
                    if (m_tx_sink) m_tx_sink(tail_silence, AUDIO_FRAME_SAMPLES);
                    return;
                }
                m_tail_start_time = 0;
                echolink_client_tx_release(TxSource::ANNOUNCEMENT);
                m_tx_keyed = false;
            }

            m_active = false;
            m_inhibit_until_ms = now + 300;
            Serial.printf("[ANNOUNCER] Finished announcement: '%s'\n", m_current_item.text);
            return;
        }

        // Extract single word or single digit
        char word[32];
        size_t len = 0;
        if (isdigit((unsigned char)*p))
        {
            // Read digits one by one!
            word[0] = *p;
            word[1] = '\0';
            p++;
        }
        else
        {
            while (*p && *p != ' ' && len < sizeof(word) - 1)
            {
                word[len++] = *p++;
            }
            word[len] = '\0';
        }
        // Update remaining words
        while (*p == ' ') p++;
        memmove(m_remaining_words, p, strlen(p) + 1);

        // Generate audio for this word (using playBeeps fallback)
        m_word_sample_count = generateWordBeeps(word, m_word_samples, sizeof(m_word_samples) / sizeof(m_word_samples[0]));
        m_word_sample_pos = 0;
    }

    // Stream 1 frame (160 samples = 20 ms)
    if (m_word_sample_pos < m_word_sample_count)
    {
        const int16_t *chunk = m_word_samples + m_word_sample_pos;
        size_t count = AUDIO_FRAME_SAMPLES;
        if (m_word_sample_pos + count > m_word_sample_count)
        {
            count = m_word_sample_count - m_word_sample_pos;
        }

        // Route to local DAC
        if ((m_current_item.route & ROUTE_LOCAL) && m_local_sink)
        {
            m_local_sink(chunk, count);
        }

        // Route to EchoLink TX
        if ((m_current_item.route & ROUTE_TX) && m_tx_sink)
        {
            m_tx_sink(chunk, count);
        }

        m_word_sample_pos += count;
    }
}

// Global wrappers
void announcer_init(AnnouncerAudioSink local_sink, AnnouncerAudioSink tx_sink, ChannelBusyHook channel_busy_fn)
{
    Announcer::instance().begin(local_sink, tx_sink, channel_busy_fn);
}

void announcer_say(const char *text, uint8_t route)
{
    Announcer::instance().say(text, route);
}

void announcer_abort()
{
    Announcer::instance().abort();
}

bool announcer_busy()
{
    return Announcer::instance().busy();
}

bool announcer_is_vox_inhibited()
{
    return Announcer::instance().is_vox_inhibited();
}

void announcer_update()
{
    Announcer::instance().update();
}
