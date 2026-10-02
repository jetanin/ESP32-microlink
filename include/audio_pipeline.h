#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "audio_common.h"
#include "audio_in.h"
#include "audio_out.h"
#include "jitter_buffer.h"

/**
 * @file audio_pipeline.h
 * @brief High-Priority Audio Pipeline clocked by I2S DMA with Jitter Buffer
 */

enum class PipelineMode : uint8_t {
    IDLE = 0,
    LOOPBACK,
    TONE,
    NETWORK
};

class AudioPipeline {
public:
    AudioPipeline(AudioIn &audio_in, AudioOut &audio_out);
    ~AudioPipeline();

    static AudioPipeline* instance();

    bool init();
    void start();
    void start_loopback();
    void stop_loopback();
    void play_tone(float freq_hz, uint32_t duration_ms);

    void set_mode(PipelineMode mode) { m_mode = mode; }
    PipelineMode get_mode() const { return m_mode; }
    bool is_loopback_active() const { return m_mode == PipelineMode::LOOPBACK; }

    void write_frame(const int16_t* pcm160);
    bool read_frame(int16_t* pcm160, uint32_t wait_ms);
    size_t read_samples(int16_t *dest, size_t count, uint32_t wait_ms);
    void clear();

    JitterBuffer& get_jitter_buffer() { return m_jitter_buffer; }
    JitterBufferStats get_stats() { return m_jitter_buffer.get_stats(); }

private:
    static void audio_playout_task(void *pvParameters);
    static void audio_capture_task(void *pvParameters);

    AudioIn &m_audio_in;
    AudioOut &m_audio_out;
    JitterBuffer m_jitter_buffer;

    PipelineMode m_mode;
    TaskHandle_t m_playout_task_handle;
    TaskHandle_t m_capture_task_handle;
    bool m_initialized;
    uint32_t m_rx_seq;

    static AudioPipeline* s_instance;
};

// Global helper functions
void audio_pipeline_set_mode(PipelineMode mode);
void audio_pipeline_write_frame(const int16_t* pcm160);
bool audio_pipeline_read_frame(int16_t* pcm160, uint32_t wait_ms);
size_t audio_pipeline_read_samples(int16_t *dest, size_t count, uint32_t wait_ms);
void audio_pipeline_clear();
