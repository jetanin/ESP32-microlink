#include "audio_out.h"
#include "pins.h"
#include <cmath>

AudioOut::AudioOut()
    : m_tx_handle(nullptr),
      m_initialized(false),
      m_running(false)
{
}

AudioOut::~AudioOut()
{
    stop();
    if (m_tx_handle)
    {
        i2s_del_channel(m_tx_handle);
        m_tx_handle = nullptr;
    }
}

bool AudioOut::init()
{
    if (m_initialized)
    {
        return true;
    }

    // Channel configuration for I2S Master TX
    // 6 DMA descriptors × 160 frames (exactly 1 audio frame per descriptor) = 120 ms DMA buffer
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = AUDIO_FRAME_SAMPLES; // 160 samples per DMA descriptor (20 ms at 8 kHz)
    chan_cfg.auto_clear = true;                   // Output silence when buffer underflows

    esp_err_t err = i2s_new_channel(&chan_cfg, &m_tx_handle, nullptr);
    if (err != ESP_OK)
    {
        log_e("i2s_new_channel failed: %s", esp_err_to_name(err));
        return false;
    }

    // Configure standard mode (Philips format)
    // Note: PCM5102A in PLL mode (SCK grounded) requires >= 64 BCK per audio frame.
    // At 8 kHz, 32-bit slots yield BCK = 8000 * 2 * 32 = 512 kHz, allowing PCM5102A PLL to lock reliably.
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)PIN_I2S_BCK,
            .ws = (gpio_num_t)PIN_I2S_WS,
            .dout = (gpio_num_t)PIN_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(m_tx_handle, &std_cfg);
    if (err != ESP_OK)
    {
        log_e("i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
        i2s_del_channel(m_tx_handle);
        m_tx_handle = nullptr;
        return false;
    }

    m_initialized = true;
    return true;
}

bool AudioOut::start()
{
    if (!m_initialized && !init())
    {
        return false;
    }
    if (m_running)
    {
        return true;
    }

    esp_err_t err = i2s_channel_enable(m_tx_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        log_e("i2s_channel_enable failed: %s", esp_err_to_name(err));
        return false;
    }

    m_running = true;
    return true;
}

bool AudioOut::stop()
{
    if (!m_running || !m_tx_handle)
    {
        return true;
    }

    esp_err_t err = i2s_channel_disable(m_tx_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        log_e("i2s_channel_disable failed: %s", esp_err_to_name(err));
        return false;
    }

    m_running = false;
    return true;
}

size_t AudioOut::write_samples(const int16_t *mono_samples, size_t count, TickType_t timeout_ticks)
{
    if (!m_running && !start())
    {
        return 0;
    }

    constexpr size_t CHUNK_SIZE = 160;
    int32_t stereo_buf[CHUNK_SIZE * 2]; // Left + Right interleaved

    size_t total_written = 0;
    while (total_written < count)
    {
        size_t to_process = count - total_written;
        if (to_process > CHUNK_SIZE)
        {
            to_process = CHUNK_SIZE;
        }

        for (size_t i = 0; i < to_process; ++i)
        {
            int32_t s32 = ((int32_t)mono_samples[total_written + i]) << 16;
            stereo_buf[i * 2]     = s32; // Left
            stereo_buf[i * 2 + 1] = s32; // Right
        }

        size_t bytes_to_write = to_process * 2 * sizeof(int32_t);
        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(m_tx_handle, stereo_buf, bytes_to_write, &bytes_written, timeout_ticks);
        if (err != ESP_OK || bytes_written == 0)
        {
            break;
        }

        size_t samples_written = bytes_written / (2 * sizeof(int32_t));
        total_written += samples_written;
    }

    return total_written;
}

        void AudioOut::play_tone(float freq_hz, uint32_t duration_ms)
        {
            if (!start())
            {
                return;
            }

            const float phase_step = (2.0f * (float)M_PI * freq_hz) / (float)AUDIO_SAMPLE_RATE_HZ;
            float phase = 0.0f;
            constexpr int16_t AMPLITUDE = 23000; // ~70% of full scale 32767

            int16_t frame_buf[AUDIO_FRAME_SAMPLES];
            uint32_t total_frames = (duration_ms * AUDIO_SAMPLE_RATE_HZ) / (1000 * AUDIO_FRAME_SAMPLES);
            if (total_frames == 0)
                total_frames = 1;

            for (uint32_t f = 0; f < total_frames; ++f)
            {
                for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i)
                {
                    frame_buf[i] = (int16_t)(sinf(phase) * AMPLITUDE);
                    phase += phase_step;
                    if (phase >= 2.0f * (float)M_PI)
                    {
                        phase -= 2.0f * (float)M_PI;
                    }
                }
                write_samples(frame_buf, AUDIO_FRAME_SAMPLES, pdMS_TO_TICKS(100));
            }
        }
