#pragma once

#include <Arduino.h>
#include <driver/i2s_std.h>
#include "audio_common.h"

/**
 * @file audio_out.h
 * @brief I2S Audio Output Driver (PCM5102A DAC via ESP-IDF i2s_std driver)
 *
 * Configured for 8 kHz 16-bit PCM playback clocked by I2S DMA.
 */

class AudioOut {
public:
    AudioOut();
    ~AudioOut();

    /**
     * @brief Initialize I2S peripheral and allocate DMA descriptors
     * @return true on success
     */
    bool init();

    /**
     * @brief Start I2S TX channel
     */
    bool start();

    /**
     * @brief Stop I2S TX channel
     */
    bool stop();

    /**
     * @brief Write mono 16-bit PCM samples to I2S DMA (replicated to L+R stereo)
     * @param mono_samples Pointer to array of int16_t samples
     * @param count Number of mono samples to write
     * @param timeout_ticks FreeRTOS timeout ticks
     * @return Number of samples successfully written
     */
    size_t write_samples(const int16_t *mono_samples, size_t count, TickType_t timeout_ticks = portMAX_DELAY);

    /**
     * @brief Generate and play a clean sine wave tone via I2S DMA
     * @param freq_hz Tone frequency in Hz (e.g., 1000.0f)
     * @param duration_ms Duration in milliseconds
     */
    void play_tone(float freq_hz, uint32_t duration_ms);

    bool is_running() const { return m_running; }

private:
    i2s_chan_handle_t m_tx_handle;
    bool m_initialized;
    bool m_running;
};
