#include "audio_in.h"
#include "pins.h"

AdcAudioIn::AdcAudioIn(uint8_t gpio_pin)
    : m_gpio_pin(gpio_pin),
      m_adc_handle(nullptr),
      m_initialized(false),
      m_running(false),
      m_dc_bias(2048), // Initial DC bias guess for 12-bit ADC (Vcc/2 ~ 2048)
      m_last_raw(2048),
      m_metric_min(4095),
      m_metric_max(0),
      m_metric_sum(0),
      m_metric_count(0) {
}

AdcAudioIn::~AdcAudioIn() {
    stop();
    if (m_adc_handle) {
        adc_continuous_deinit(m_adc_handle);
        m_adc_handle = nullptr;
    }
}

bool AdcAudioIn::init() {
    if (m_initialized) {
        return true;
    }

    adc_continuous_handle_cfg_t hdl_cfg = {
        .max_store_buf_size = 2048,
        .conv_frame_size = AUDIO_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES, // 160 * 4 = 640 bytes
        .flags = {
            .flush_pool = 1,
        }
    };

    esp_err_t err = adc_continuous_new_handle(&hdl_cfg, &m_adc_handle);
    if (err != ESP_OK) {
        log_e("adc_continuous_new_handle failed: %s", esp_err_to_name(err));
        return false;
    }

    // Map GPIO to ADC1 Channel. On ESP32-C6: GPIO 1 is ADC1_CH1
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    if (m_gpio_pin == 1) {
        adc_ch = ADC_CHANNEL_1;
    } else {
        adc_ch = (adc_channel_t)m_gpio_pin;
    }

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = (uint8_t)adc_ch,
        .unit = (uint8_t)ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH, // 12-bit
    };

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = AUDIO_SAMPLE_RATE_HZ, // 8000 Hz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    err = adc_continuous_config(m_adc_handle, &dig_cfg);
    if (err != ESP_OK) {
        log_e("adc_continuous_config failed: %s", esp_err_to_name(err));
        adc_continuous_deinit(m_adc_handle);
        m_adc_handle = nullptr;
        return false;
    }

    m_initialized = true;
    return true;
}

bool AdcAudioIn::start() {
    if (!m_initialized && !init()) {
        return false;
    }
    if (m_running) {
        return true;
    }

    esp_err_t err = adc_continuous_start(m_adc_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        log_e("adc_continuous_start failed: %s", esp_err_to_name(err));
        return false;
    }

    m_running = true;
    return true;
}

bool AdcAudioIn::stop() {
    if (!m_running || !m_adc_handle) {
        return true;
    }

    esp_err_t err = adc_continuous_stop(m_adc_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        log_e("adc_continuous_stop failed: %s", esp_err_to_name(err));
        return false;
    }

    m_running = false;
    return true;
}

size_t AdcAudioIn::read_samples(int16_t *dest, size_t count, TickType_t timeout_ticks) {
    if (!m_running && !start()) {
        return 0;
    }

    constexpr size_t TEMP_FRAME_SAMPLES = 160;
    adc_digi_output_data_t raw_buf[TEMP_FRAME_SAMPLES];

    size_t total_samples = 0;
    while (total_samples < count) {
        size_t samples_to_read = count - total_samples;
        if (samples_to_read > TEMP_FRAME_SAMPLES) {
            samples_to_read = TEMP_FRAME_SAMPLES;
        }

        uint32_t bytes_to_read = samples_to_read * SOC_ADC_DIGI_RESULT_BYTES;
        uint32_t bytes_read = 0;

        esp_err_t err = adc_continuous_read(
            m_adc_handle,
            reinterpret_cast<uint8_t*>(raw_buf),
            bytes_to_read,
            &bytes_read,
            timeout_ticks
        );

        if (err != ESP_OK || bytes_read == 0) {
            break;
        }

        size_t samples_obtained = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;
        for (size_t i = 0; i < samples_obtained; ++i) {
            uint16_t raw_val = raw_buf[i].type2.data;
            m_last_raw = raw_val;

            // Metric accumulation
            if (raw_val < m_metric_min) m_metric_min = raw_val;
            if (raw_val > m_metric_max) m_metric_max = raw_val;
            m_metric_sum += raw_val;
            m_metric_count++;

            // Adaptive DC bias tracking (leaky integrator)
            m_dc_bias += (((int32_t)raw_val - m_dc_bias) >> 7);

            // Convert unipolar 12-bit [0..4095] with DC bias to bipolar 16-bit [-32768..32767]
            int32_t ac = (int32_t)raw_val - m_dc_bias;
            int32_t pcm = ac << 4; // 12-bit to 16-bit shift

            if (pcm > 32767)  pcm = 32767;
            if (pcm < -32768) pcm = -32768;

            dest[total_samples + i] = static_cast<int16_t>(pcm);
        }

        total_samples += samples_obtained;
    }

    return total_samples;
}

void AdcAudioIn::get_metrics(uint16_t &raw_min, uint16_t &raw_max, uint16_t &raw_avg, uint16_t &peak_to_peak) {
    if (m_metric_count == 0) {
        raw_min = m_last_raw;
        raw_max = m_last_raw;
        raw_avg = m_last_raw;
        peak_to_peak = 0;
        return;
    }

    raw_min = m_metric_min;
    raw_max = m_metric_max;
    raw_avg = static_cast<uint16_t>(m_metric_sum / m_metric_count);
    peak_to_peak = (m_metric_max >= m_metric_min) ? (m_metric_max - m_metric_min) : 0;

    // Reset metric window
    m_metric_min = 4095;
    m_metric_max = 0;
    m_metric_sum = 0;
    m_metric_count = 0;
}
