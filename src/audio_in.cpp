#include "audio_in.h"
#include "pins.h"
#include <cmath>

AdcAudioIn::AdcAudioIn(uint8_t gpio_pin, int8_t pot_pin)
    : m_gpio_pin(gpio_pin),
      m_pot_pin(pot_pin),
      m_adc_handle(nullptr),
      m_initialized(false),
      m_running(false),
      m_pot_raw(2048), // Default mid-scale (Sensitivity ~5)
      m_first_sample(true),
      m_dc_bias(2048), // Initial DC bias guess for 12-bit ADC (Vcc/2 ~ 2048)
      m_last_raw(2048),
      m_metric_min(4095),
      m_metric_max(0),
      m_metric_sum(0),
      m_metric_count(0),
      m_rms_window{0, 0, 0, 0},
      m_rms_idx(0),
      m_moving_rms(0),
      m_acc_energy(0),
      m_acc_count(0)
{
}

AdcAudioIn::~AdcAudioIn()
{
    stop();
    if (m_adc_handle)
    {
        adc_continuous_deinit(m_adc_handle);
        m_adc_handle = nullptr;
    }
}

bool AdcAudioIn::init()
{
    if (m_initialized)
    {
        return true;
    }

    adc_continuous_handle_cfg_t hdl_cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size = AUDIO_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES * (m_pot_pin >= 0 ? 2U : 1U),
        .flags = {
            .flush_pool = 1,
        }};

    esp_err_t err = adc_continuous_new_handle(&hdl_cfg, &m_adc_handle);
    if (err != ESP_OK)
    {
        log_e("adc_continuous_new_handle failed: %s", esp_err_to_name(err));
        return false;
    }

    // Map GPIO to ADC1 Channel. On ESP32-C6: GPIO 1 is ADC1_CH1
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    if (m_gpio_pin == 1)
    {
        adc_ch = ADC_CHANNEL_1;
    }
    else
    {
        adc_ch = (adc_channel_t)m_gpio_pin;
    }

    adc_digi_pattern_config_t pattern[2];
    uint32_t pat_cnt = 0;

    // Pattern 0: Microphone channel
    pattern[pat_cnt].atten = ADC_ATTEN_DB_12;
    pattern[pat_cnt].channel = (uint8_t)adc_ch;
    pattern[pat_cnt].unit = (uint8_t)ADC_UNIT_1;
    pattern[pat_cnt].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    pat_cnt++;

    // Pattern 1: Potentiometer channel (GPIO 3 = ADC1_CH3)
    if (m_pot_pin >= 0)
    {
        adc_channel_t pot_ch = ADC_CHANNEL_3;
        if (m_pot_pin == 3)
        {
            pot_ch = ADC_CHANNEL_3;
        }
        else
        {
            pot_ch = (adc_channel_t)m_pot_pin;
        }
        pattern[pat_cnt].atten = ADC_ATTEN_DB_12;
        pattern[pat_cnt].channel = (uint8_t)pot_ch;
        pattern[pat_cnt].unit = (uint8_t)ADC_UNIT_1;
        pattern[pat_cnt].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
        pat_cnt++;
    }

    adc_continuous_config_t dig_cfg = {
        .pattern_num = pat_cnt,
        .adc_pattern = pattern,
        .sample_freq_hz = AUDIO_SAMPLE_RATE_HZ * pat_cnt, // e.g. 16000 Hz total (8000 Hz per channel)
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    err = adc_continuous_config(m_adc_handle, &dig_cfg);
    if (err != ESP_OK)
    {
        log_e("adc_continuous_config failed: %s", esp_err_to_name(err));
        adc_continuous_deinit(m_adc_handle);
        m_adc_handle = nullptr;
        return false;
    }

    m_initialized = true;
    return true;
}

bool AdcAudioIn::start()
{
    if (!m_initialized && !init())
    {
        return false;
    }
    if (m_running)
    {
        return true;
    }

    esp_err_t err = adc_continuous_start(m_adc_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        log_e("adc_continuous_start failed: %s", esp_err_to_name(err));
        return false;
    }

    m_running = true;
    m_first_sample = true;
    return true;
}

bool AdcAudioIn::stop()
{
    if (!m_running || !m_adc_handle)
    {
        return true;
    }

    esp_err_t err = adc_continuous_stop(m_adc_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        log_e("adc_continuous_stop failed: %s", esp_err_to_name(err));
        return false;
    }

    m_running = false;
    return true;
}

size_t AdcAudioIn::read_samples(int16_t *dest, size_t count, TickType_t timeout_ticks)
{
    if (!m_running && !start())
    {
        return 0;
    }

    constexpr size_t TEMP_FRAME_SAMPLES = 320;
    adc_digi_output_data_t raw_buf[TEMP_FRAME_SAMPLES];

    size_t total_samples = 0;
    while (total_samples < count)
    {
        size_t samples_needed = count - total_samples;
        size_t samples_to_read = samples_needed * (m_pot_pin >= 0 ? 2U : 1U);
        if (samples_to_read > TEMP_FRAME_SAMPLES)
        {
            samples_to_read = TEMP_FRAME_SAMPLES;
        }

        uint32_t bytes_to_read = samples_to_read * SOC_ADC_DIGI_RESULT_BYTES;
        uint32_t bytes_read = 0;

        esp_err_t err = adc_continuous_read(
            m_adc_handle,
            reinterpret_cast<uint8_t *>(raw_buf),
            bytes_to_read,
            &bytes_read,
            timeout_ticks);

        if (err != ESP_OK || bytes_read == 0)
        {
            break;
        }

        size_t samples_obtained = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;
        for (size_t i = 0; i < samples_obtained; ++i)
        {
            uint8_t ch = raw_buf[i].type2.channel;
            uint16_t raw_val = raw_buf[i].type2.data;

            // Separate Potentiometer samples from Microphone
            // Pattern has only 2 channels: CH1 (Mic) and CH3 (Pot).
            // Any sample not belonging to Mic channel (CH1) belongs to Potentiometer.
            if (m_pot_pin >= 0 && (ch == ADC_CHANNEL_3 || ch != ADC_CHANNEL_1))
            {
                // Exponential moving average filter on pot reading (smoothes ADC noise)
                m_pot_raw = static_cast<uint16_t>(((uint32_t)m_pot_raw * 15 + raw_val) >> 4);
                continue;
            }

            // Microphone sample (ADC_CHANNEL_1)
            m_last_raw = raw_val;

            // Metric accumulation
            if (raw_val < m_metric_min)
                m_metric_min = raw_val;
            if (raw_val > m_metric_max)
                m_metric_max = raw_val;
            m_metric_sum += raw_val;
            m_metric_count++;

            // Fast lock on first sample so DC bias matches actual hardware resting voltage
            if (m_first_sample)
            {
                m_dc_bias = raw_val;
                m_first_sample = false;
            }
            else
            {
                // Adaptive DC bias tracking (leaky integrator)
                m_dc_bias += (((int32_t)raw_val - m_dc_bias) >> 7);
            }

            // Convert unipolar 12-bit [0..4095] with DC bias to bipolar 16-bit [-32768..32767]
            int32_t ac = (int32_t)raw_val - m_dc_bias;
            int32_t pcm = ac << 4; // 12-bit to 16-bit shift

            if (pcm > 32767)
                pcm = 32767;
            if (pcm < -32768)
                pcm = -32768;

            dest[total_samples++] = static_cast<int16_t>(pcm);

            // Accumulate energy for Moving Energy / RMS window (multi-frame window to reject short spikes/clicks)
            m_acc_energy += (uint64_t)((int64_t)ac * ac);
            m_acc_count++;
            if (m_acc_count >= AUDIO_FRAME_SAMPLES)
            {
                uint32_t mean_sq = static_cast<uint32_t>(m_acc_energy / m_acc_count);
                uint16_t frame_rms = static_cast<uint16_t>(sqrtf((float)mean_sq));
                m_rms_window[m_rms_idx] = frame_rms;
                m_rms_idx = (m_rms_idx + 1) % RMS_WINDOW_SIZE;

                uint32_t sum = 0;
                for (size_t k = 0; k < RMS_WINDOW_SIZE; ++k)
                {
                    sum += m_rms_window[k];
                }
                m_moving_rms = static_cast<uint16_t>(sum / RMS_WINDOW_SIZE);

                m_acc_energy = 0;
                m_acc_count = 0;
            }

            if (total_samples >= count)
                break;
        }
    }

    return total_samples;
}

uint8_t AdcAudioIn::get_pot_step() const
{
    if (m_pot_pin < 0)
        return 10;
    // Map 0..4095 to 1..20 fine-grained sensitivity steps
    uint32_t step = ((uint32_t)m_pot_raw * 20) / 4096 + 1;
    if (step < 1)
        step = 1;
    if (step > 20)
        step = 20;
    return static_cast<uint8_t>(step);
}

void AdcAudioIn::get_metrics(uint16_t &raw_min, uint16_t &raw_max, uint16_t &raw_avg, uint16_t &peak_to_peak)
{
    if (m_metric_count == 0)
    {
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
