#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "audio_common.h"

/**
 * @file jitter_buffer.h
 * @brief Thread-safe Audio Jitter Buffer for 8 kHz EchoLink VoIP frames
 *
 * EchoLink packets arrive with variable network jitter.
 * This jitter buffer absorbs packet timing variations and feeds smooth,
 * gap-free 20 ms audio frames (160 samples at 8 kHz) to the I2S DMA output.
 */

struct AudioFrame
{
    int16_t samples[AUDIO_FRAME_SAMPLES];
    uint32_t seq;
};

struct JitterBufferStats
{
    uint32_t frames_pushed;
    uint32_t frames_popped;
    uint32_t underflow_count;
    uint32_t overflow_count;
    size_t current_depth_frames;
    size_t watermark_frames;
    bool is_buffering;
};

class JitterBuffer
{
public:
    static constexpr size_t CAPACITY_FRAMES = 32;         // 32 * 20 ms = 640 ms buffer capacity
    static constexpr size_t DEFAULT_WATERMARK_FRAMES = 4; // Pre-buffer 4 frames (80 ms) before playout starts

    JitterBuffer(size_t watermark = DEFAULT_WATERMARK_FRAMES);
    ~JitterBuffer();

    void reset();
    void set_watermark(size_t watermark);

    /**
     * @brief Push a 20 ms frame into the buffer (called from network or capture task)
     * @param frame Pointer to AudioFrame (160 int16 samples)
     * @return true if stored, false if dropped due to overflow
     */
    bool push_frame(const AudioFrame &frame);

    /**
     * @brief Pop a 20 ms frame for playback (called from high-priority audio task)
     * @param out_frame Destination buffer
     * @return true if valid frame returned, false if buffer underflow (out_frame zeroed)
     */
    bool pop_frame(AudioFrame &out_frame);

    /**
     * @brief Get current snapshot of buffer statistics
     */
    JitterBufferStats get_stats();

private:
    AudioFrame m_queue[CAPACITY_FRAMES];
    size_t m_head;
    size_t m_tail;
    size_t m_count;
    size_t m_watermark;
    bool m_buffering;

    SemaphoreHandle_t m_mutex;

    // Cumulative stats
    uint32_t m_frames_pushed;
    uint32_t m_frames_popped;
    uint32_t m_underflow_count;
    uint32_t m_overflow_count;
    size_t m_consecutive_underflows;
};
