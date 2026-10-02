#include "audio_pipeline.h"
#include "system_state.h"
#include "dtmf_detector.h"
#include "announcer.h"
#include <cmath>

static constexpr UBaseType_t TASK_PRIO_AUDIO_PLAYOUT = 5; // Highest system priority
static constexpr UBaseType_t TASK_PRIO_AUDIO_CAPTURE = 4;

AudioPipeline *AudioPipeline::s_instance = nullptr;

AudioPipeline::AudioPipeline(AudioIn &audio_in, AudioOut &audio_out)
    : m_audio_in(audio_in),
      m_audio_out(audio_out),
      m_jitter_buffer(4), // Pre-buffer 4 frames (80 ms) — matches 1 EchoLink packet exactly
      m_mode(PipelineMode::IDLE),
      m_playout_task_handle(nullptr),
      m_capture_task_handle(nullptr),
      m_initialized(false),
      m_rx_seq(0)
{
    s_instance = this;
}

AudioPipeline::~AudioPipeline()
{
    stop_loopback();
    if (m_playout_task_handle)
    {
        vTaskDelete(m_playout_task_handle);
        m_playout_task_handle = nullptr;
    }
    if (m_capture_task_handle)
    {
        vTaskDelete(m_capture_task_handle);
        m_capture_task_handle = nullptr;
    }
    s_instance = nullptr;
}

AudioPipeline *AudioPipeline::instance()
{
    return s_instance;
}

bool AudioPipeline::init()
{
    if (m_initialized)
        return true;

    if (!m_audio_out.init())
    {
        log_e("AudioOut init failed");
        return false;
    }
    if (!m_audio_in.init())
    {
        log_e("AudioIn init failed");
        return false;
    }

    // Start Audio Hardware
    m_audio_out.start();
    m_audio_in.start();

    // Spawn Playout Task (highest priority, paced by I2S DMA)
    BaseType_t res1 = xTaskCreate(
        audio_playout_task,
        "audio_playout",
        4096,
        this,
        TASK_PRIO_AUDIO_PLAYOUT,
        &m_playout_task_handle);

    // Spawn Capture Task
    BaseType_t res2 = xTaskCreate(
        audio_capture_task,
        "audio_capture",
        4096,
        this,
        TASK_PRIO_AUDIO_CAPTURE,
        &m_capture_task_handle);

    if (res1 != pdPASS || res2 != pdPASS)
    {
        log_e("Failed to create audio pipeline tasks!");
        return false;
    }

    m_initialized = true;
    return true;
}

void AudioPipeline::start()
{
    init();
}

void AudioPipeline::start_loopback()
{
    m_jitter_buffer.reset();
    m_mode = PipelineMode::LOOPBACK;
    Serial.println(F("[PIPELINE] Started real-time Loopback (Mic -> ADC DMA -> JitterBuffer -> I2S DMA -> DAC)"));
}

void AudioPipeline::stop_loopback()
{
    m_mode = PipelineMode::IDLE;
    m_jitter_buffer.reset();
    Serial.println(F("[PIPELINE] Stopped audio pipeline loopback."));
}

void AudioPipeline::play_tone(float freq_hz, uint32_t duration_ms)
{
    PipelineMode prev_mode = m_mode;
    m_mode = PipelineMode::TONE;

    m_audio_out.play_tone(freq_hz, duration_ms);

    m_mode = prev_mode;
}

void AudioPipeline::write_frame(const int16_t *pcm160)
{
    // Set NETWORK mode before pushing so playout_task starts driving
    // the I2S clock immediately on first frame received
    if (m_mode != PipelineMode::LOOPBACK)
    {
        m_mode = PipelineMode::NETWORK;
    }

    AudioFrame frame;
    memcpy(frame.samples, pcm160, sizeof(frame.samples));
    frame.seq = m_rx_seq++;
    m_jitter_buffer.push_frame(frame);
}

bool AudioPipeline::read_frame(int16_t *pcm160, uint32_t wait_ms)
{
    size_t got = m_audio_in.read_samples(pcm160, AUDIO_FRAME_SAMPLES, pdMS_TO_TICKS(wait_ms));
    return (got == AUDIO_FRAME_SAMPLES);
}

size_t AudioPipeline::read_samples(int16_t *dest, size_t count, uint32_t wait_ms)
{
    return m_audio_in.read_samples(dest, count, pdMS_TO_TICKS(wait_ms));
}

void AudioPipeline::clear()
{
    m_jitter_buffer.reset();
    m_rx_seq = 0;
}

void AudioPipeline::audio_playout_task(void *pvParameters)
{
    auto *pipeline = static_cast<AudioPipeline *>(pvParameters);
    AudioFrame frame;
    static int16_t silence[AUDIO_FRAME_SAMPLES] = {0};

    for (;;)
    {
            PipelineMode mode = pipeline->m_mode;

        if (mode == PipelineMode::LOOPBACK || mode == PipelineMode::NETWORK)
        {
            bool got_frame = pipeline->m_jitter_buffer.pop_frame(frame);

            if (got_frame && mode == PipelineMode::NETWORK)
            {
                system_state_set_rx(true);
            }

            // Always write to I2S (silence if underflow) — this keeps the DMA clock running
            // smoothly and prevents gaps/stutters caused by the task sleeping when empty
            const int16_t *samples = got_frame ? frame.samples : silence;
            pipeline->m_audio_out.write_samples(samples, AUDIO_FRAME_SAMPLES, portMAX_DELAY);
        }
        else
        {
            // IDLE or TONE: yield, don't drive I2S
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

void AudioPipeline::audio_capture_task(void *pvParameters)
{
    auto *pipeline = static_cast<AudioPipeline *>(pvParameters);
    AudioFrame frame;
    uint32_t seq = 0;

    for (;;)
    {
        if (pipeline->m_mode == PipelineMode::LOOPBACK)
        {
            // Read exactly 160 samples (20 ms) from ADC continuous DMA
            size_t read_cnt = pipeline->m_audio_in.read_samples(frame.samples, AUDIO_FRAME_SAMPLES, pdMS_TO_TICKS(50));
            if (read_cnt == AUDIO_FRAME_SAMPLES)
            {
                frame.seq = seq++;
                pipeline->m_jitter_buffer.push_frame(frame);
                dtmf_detector_process(frame.samples, AUDIO_FRAME_SAMPLES);
            }
        }
        else
        {
            // Sample for VU meter metrics and DTMF detection when NOT transmitting
            SystemState st = system_state_get();
            if (!st.tx_active)
            {
                int16_t dummy[AUDIO_FRAME_SAMPLES];
                size_t read_cnt = pipeline->m_audio_in.read_samples(dummy, AUDIO_FRAME_SAMPLES, pdMS_TO_TICKS(50));
                if (read_cnt > 0)
                {
                    // Gate DTMF when announcer is busy or in post-announcement inhibit window
                    // (prevents mic bleed and speaker echo from self-triggering detector)
                    if (!Announcer::instance().is_vox_inhibited())
                    {
                        dtmf_detector_process(dummy, read_cnt);
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

// Global helper wrappers
void audio_pipeline_set_mode(PipelineMode mode)
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->set_mode(mode);
}

void audio_pipeline_write_frame(const int16_t *pcm160)
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->write_frame(pcm160);
}

bool audio_pipeline_read_frame(int16_t *pcm160, uint32_t wait_ms)
{
    if (AudioPipeline::instance())
        return AudioPipeline::instance()->read_frame(pcm160, wait_ms);
    return false;
}

size_t audio_pipeline_read_samples(int16_t *dest, size_t count, uint32_t wait_ms)
{
    if (AudioPipeline::instance())
        return AudioPipeline::instance()->read_samples(dest, count, wait_ms);
    return 0;
}

void audio_pipeline_clear()
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->clear();
}
