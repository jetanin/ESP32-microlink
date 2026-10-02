#pragma once

#include <Arduino.h>
#include <esp_adc/adc_continuous.h>
#include "audio_common.h"

/**
 * @file audio_in.h
 * @brief Abstract Audio Input Interface and ADC-based Microphone Implementation
 *
 * Designed to abstract microphone hardware so KY-038 (ADC1) can be cleanly
 * swapped for an I2S MEMS mic (e.g. INMP441) in the future without affecting
 * higher layers.
 */

class AudioIn {
public:
    virtual ~AudioIn() = default;

    /**
     * @brief Initialize the audio input hardware
     * @return true on success
     */
    virtual bool init() = 0;

    /**
     * @brief Start sampling
     */
    virtual bool start() = 0;

    /**
     * @brief Stop sampling
     */
    virtual bool stop() = 0;

    /**
     * @brief Read 16-bit signed PCM samples at 8 kHz mono
     * @param dest Buffer to store int16_t PCM samples
     * @param count Number of samples requested
     * @param timeout_ticks Maximum FreeRTOS ticks to wait
     * @return Number of samples actually read
     */
    virtual size_t read_samples(int16_t *dest, size_t count, TickType_t timeout_ticks = portMAX_DELAY) = 0;

    /**
     * @brief Retrieve recent raw ADC / sample level for meters and debugging
     */
    virtual uint16_t get_last_raw() const = 0;

    /**
     * @brief Retrieve signal metrics for level measurement
     */
    virtual void get_metrics(uint16_t &raw_min, uint16_t &raw_max, uint16_t &raw_avg, uint16_t &peak_to_peak) = 0;

    virtual bool is_running() const = 0;
};

/**
 * @brief Concrete AudioIn implementation for analog electret microphone (KY-038 on ADC1)
 */
class AdcAudioIn : public AudioIn {
public:
    AdcAudioIn(uint8_t gpio_pin);
    ~AdcAudioIn() override;

    bool init() override;
    bool start() override;
    bool stop() override;
    size_t read_samples(int16_t *dest, size_t count, TickType_t timeout_ticks = portMAX_DELAY) override;
    uint16_t get_last_raw() const override { return m_last_raw; }
    void get_metrics(uint16_t &raw_min, uint16_t &raw_max, uint16_t &raw_avg, uint16_t &peak_to_peak) override;
    bool is_running() const override { return m_running; }

private:
    uint8_t m_gpio_pin;
    adc_continuous_handle_t m_adc_handle;
    bool m_initialized;
    bool m_running;

    // DC bias tracking and level metrics
    int32_t  m_dc_bias;
    uint16_t m_last_raw;
    uint16_t m_metric_min;
    uint16_t m_metric_max;
    uint32_t m_metric_sum;
    uint32_t m_metric_count;
};
