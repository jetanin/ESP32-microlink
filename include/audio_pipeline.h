#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include "audio_common.h"
#include "audio_in.h"
#include "audio_out.h"
#include "jitter_buffer.h"
#include "vox_pre_roll.h"

/**
 * @file audio_pipeline.h
 * @brief High-Priority Audio Pipeline clocked by I2S DMA with Jitter Buffer and VOX Pre-roll
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
    void clear_tx();

    void set_vox_preroll_delay(uint8_t delay_frames);
    uint8_t get_vox_preroll_delay() const;
    void request_vox_reset();

    JitterBuffer& get_jitter_buffer() { return m_jitter_buffer; }
    JitterBufferStats get_stats() { return m_jitter_buffer.get_stats(); }

private:
    static void audio_playout_task(void *pvParameters);
    static void audio_capture_task(void *pvParameters);

    AudioIn &m_audio_in;
    AudioOut &m_audio_out;
    JitterBuffer m_jitter_buffer;

    VoxPreRoll m_vox_pre_roll;
    std::atomic<uint8_t> m_req_preroll_delay;
    std::atomic<bool> m_delay_change_pending;
    std::atomic<bool> m_reset_pending;

    // Static TX PCM FIFO connecting audio_capture_task to service_tx_audio
    static constexpr size_t TX_FIFO_CAPACITY = 1280; // 8 frames = 160 ms @ 8 kHz
    int16_t m_tx_fifo_buf[TX_FIFO_CAPACITY];
    size_t m_tx_fifo_head;
    size_t m_tx_fifo_tail;
    size_t m_tx_fifo_count;
    SemaphoreHandle_t m_tx_fifo_mutex;

    bool tx_fifo_write(const int16_t *samples, size_t count);
    size_t tx_fifo_read(int16_t *dest, size_t count);
    void tx_fifo_clear();

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
void audio_pipeline_clear_tx();
void audio_pipeline_set_vox_preroll_delay(uint8_t delay_frames);
uint8_t audio_pipeline_get_vox_preroll_delay();
void audio_pipeline_request_vox_reset();

