#include "audio_pipeline.h"
#include "system_state.h"
#include "dtmf_detector.h"
#include "dtmf_controller.h"
#include "dtmf_gate.h"
#include "announcer.h"
#include "echolink_client.h"
#include <cmath>

static constexpr UBaseType_t TASK_PRIO_AUDIO_PLAYOUT = 5; // Highest system priority
static constexpr UBaseType_t TASK_PRIO_AUDIO_CAPTURE = 4;

static DtmfGate s_dtmf_gate;
DtmfGate &dtmf_gate_instance()
{
    return s_dtmf_gate;
}

AudioPipeline *AudioPipeline::s_instance = nullptr;

AudioPipeline::AudioPipeline(AudioIn &audio_in, AudioOut &audio_out)
    : m_audio_in(audio_in),
      m_audio_out(audio_out),
      m_jitter_buffer(4), // Pre-buffer 4 frames (80 ms) — matches 1 EchoLink packet exactly
      m_vox_pre_roll(3),
      m_req_preroll_delay(3),
      m_delay_change_pending(false),
      m_reset_pending(false),
      m_vox_tx_open(false),
      m_speech_start_time(0),
      m_vox_hang_end(0),
      m_tx_fifo_head(0),
      m_tx_fifo_tail(0),
      m_tx_fifo_count(0),
      m_tx_fifo_mutex(nullptr),
      m_mode(PipelineMode::IDLE),
      m_playout_task_handle(nullptr),
      m_capture_task_handle(nullptr),
      m_initialized(false),
      m_rx_seq(0)
{
    memset(m_tx_fifo_buf, 0, sizeof(m_tx_fifo_buf));
    m_tx_fifo_mutex = xSemaphoreCreateMutex();
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
    if (m_tx_fifo_mutex)
    {
        vSemaphoreDelete(m_tx_fifo_mutex);
        m_tx_fifo_mutex = nullptr;
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
    size_t got = read_samples(pcm160, AUDIO_FRAME_SAMPLES, wait_ms);
    return (got == AUDIO_FRAME_SAMPLES);
}

size_t AudioPipeline::read_samples(int16_t *dest, size_t count, uint32_t wait_ms)
{
    (void)wait_ms;
    return tx_fifo_read(dest, count);
}

void AudioPipeline::clear()
{
    m_jitter_buffer.reset();
    m_rx_seq = 0;
    tx_fifo_clear();
    request_vox_reset();
}

void AudioPipeline::clear_tx()
{
    tx_fifo_clear();
}

void AudioPipeline::set_vox_preroll_delay(uint8_t delay_frames)
{
    m_req_preroll_delay.store(delay_frames);
    m_delay_change_pending.store(true);
}

uint8_t AudioPipeline::get_vox_preroll_delay() const
{
    return m_vox_pre_roll.getDelayFrames();
}

void AudioPipeline::request_vox_reset()
{
    m_reset_pending.store(true);
}

bool AudioPipeline::tx_fifo_write(const int16_t *samples, size_t count)
{
    if (!samples || count == 0) return true;
    if (!m_tx_fifo_mutex || xSemaphoreTake(m_tx_fifo_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;

    if (TX_FIFO_CAPACITY - m_tx_fifo_count < count)
    {
        xSemaphoreGive(m_tx_fifo_mutex);
        return false;
    }

    for (size_t i = 0; i < count; ++i)
    {
        m_tx_fifo_buf[m_tx_fifo_head] = samples[i];
        m_tx_fifo_head = (m_tx_fifo_head + 1) % TX_FIFO_CAPACITY;
    }
    m_tx_fifo_count += count;

    xSemaphoreGive(m_tx_fifo_mutex);
    return true;
}

size_t AudioPipeline::tx_fifo_read(int16_t *dest, size_t count)
{
    if (!dest || count == 0) return 0;
    if (!m_tx_fifo_mutex || xSemaphoreTake(m_tx_fifo_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return 0;

    size_t to_read = (count < m_tx_fifo_count) ? count : m_tx_fifo_count;
    for (size_t i = 0; i < to_read; ++i)
    {
        dest[i] = m_tx_fifo_buf[m_tx_fifo_tail];
        m_tx_fifo_tail = (m_tx_fifo_tail + 1) % TX_FIFO_CAPACITY;
    }
    m_tx_fifo_count -= to_read;

    xSemaphoreGive(m_tx_fifo_mutex);
    return to_read;
}

void AudioPipeline::tx_fifo_clear()
{
    if (m_tx_fifo_mutex && xSemaphoreTake(m_tx_fifo_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        m_tx_fifo_head = 0;
        m_tx_fifo_tail = 0;
        m_tx_fifo_count = 0;
        xSemaphoreGive(m_tx_fifo_mutex);
    }
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

void AudioPipeline::force_close_vox()
{
    m_speech_start_time = 0;
    m_vox_hang_end = 0;
    if (m_vox_tx_open)
    {
        m_vox_tx_open = false;
        if (echolink_client_get_tx_source() == TxSource::VOX)
        {
            echolink_client_tx_release(TxSource::VOX);
            system_state_set_ptt(false);
            Serial.println(F("[VOX] TX force-closed"));
        }
    }
}

void AudioPipeline::reset_vox()
{
    force_close_vox();
    m_vox_pre_roll.reset();
}

void AudioPipeline::audio_capture_task(void *pvParameters)
{
    auto *pipeline = static_cast<AudioPipeline *>(pvParameters);
    AudioFrame frame;
    uint32_t seq = 0;

    for (;;)
    {
        // 1. Process atomic delay updates and atomic reset requests at frame boundary
        if (pipeline->m_delay_change_pending.exchange(false))
        {
            uint8_t d = pipeline->m_req_preroll_delay.load();
            pipeline->m_vox_pre_roll.setDelayFrames(d);
        }
        if (pipeline->m_reset_pending.exchange(false))
        {
            pipeline->m_vox_pre_roll.reset();
        }

        if (pipeline->m_mode == PipelineMode::LOOPBACK)
        {
            // Read exactly 160 samples (20 ms) from ADC continuous DMA
            size_t read_cnt = pipeline->m_audio_in.read_samples(frame.samples, AUDIO_FRAME_SAMPLES, pdMS_TO_TICKS(50));
            if (read_cnt == AUDIO_FRAME_SAMPLES)
            {
                frame.seq = seq++;
                pipeline->m_jitter_buffer.push_frame(frame);
                dtmf_detector_process_frame(frame.samples);
            }
        }
        else
        {
            // Read 160 samples (20 ms @ 8 kHz) from ADC continuous DMA
            int16_t mic_frame[AUDIO_FRAME_SAMPLES];
            size_t read_cnt = pipeline->m_audio_in.read_samples(mic_frame, AUDIO_FRAME_SAMPLES, pdMS_TO_TICKS(50));
            if (read_cnt == AUDIO_FRAME_SAMPLES)
            {
                uint32_t now = millis();

                // 1. DTMF detector runs on EVERY captured mic frame, independent of
                //    VOX state, TX state, and mode (PTT/VOX).
                DtmfFrameResult res = dtmf_detector_process_frame(mic_frame);

                // 2. Dispatch confirmed digits to DTMF controller immediately
                //    (inhibit only if local speaker is playing an announcement to avoid acoustic feedback)
                if (res.confirmed_digit != '\0')
                {
                    if (!announcer_is_vox_inhibited())
                    {
                        dtmf_controller_handle_digit(res.confirmed_digit);
                    }
                }

                // 3. Update DtmfGate with per-frame candidate and confirmed digit
                s_dtmf_gate.update(res.candidate, res.confirmed_digit, dtmf_controller_in_progress(), now);

                // Rising edge of candidate: zero newest 2 frames in pre-roll buffer
                // so the onset of the tone buffered before detection does not leak into TX.
                if (s_dtmf_gate.is_candidate_rising(res.candidate))
                {
                    pipeline->m_vox_pre_roll.muteRecentFrames(2);
                }

                // Periodic or on-event debug logging if enabled
                if (s_dtmf_gate.is_debug() && (res.candidate || res.confirmed_digit != '\0'))
                {
                    SystemState st_dbg = system_state_get();
                    Serial.printf("[dtmf] cand=%d conf=%d digit=%c tx=%d vox=%d pp=%u\n",
                                  res.candidate ? 1 : 0,
                                  res.confirmed_digit != '\0' ? 1 : 0,
                                  res.confirmed_digit != '\0' ? res.confirmed_digit : '-',
                                  st_dbg.tx_active ? 1 : 0,
                                  (st_dbg.op_mode == 1) ? 1 : 0,
                                  (unsigned int)pipeline->m_vox_pre_roll.getBufferedCount());
                }

                // Rule 2: Force-close VOX immediately on tone detection (no hang or attack wait)
                if (s_dtmf_gate.should_force_close_vox())
                {
                    pipeline->force_close_vox();
                }

                SystemState st = system_state_get();
                bool is_vox = (st.op_mode == 1);
                bool tx_active = echolink_client_is_tx_active();
                TxSource tx_src = echolink_client_get_tx_source();

                // Announcements mute the live microphone path into TX
                if (tx_src == TxSource::ANNOUNCEMENT)
                {
                    continue;
                }

                if (is_vox)
                {
                    // 4. VOX Mode: evaluate level, attack, and hang (if not inhibited by gate, TOT, or announcer)
                    if (!s_dtmf_gate.is_vox_inhibited() && !st.tot_triggered && !announcer_is_vox_inhibited())
                    {
                        uint16_t moving_rms = pipeline->m_audio_in.get_moving_rms();
                        uint8_t sens = st.vox_sensitivity;
                        if (sens < 1) sens = 1;
                        if (sens > 20) sens = 20;
                        uint16_t threshold = VOX_THRESHOLDS[sens];
                        bool voice_active = (moving_rms >= threshold);

                        if (voice_active)
                        {
                            if (pipeline->m_speech_start_time == 0)
                            {
                                pipeline->m_speech_start_time = now;
                            }
                            if (!pipeline->m_vox_tx_open && (now - pipeline->m_speech_start_time >= VOX_ATTACK_MS))
                            {
                                pipeline->m_vox_tx_open = true;
                                if (echolink_client_tx_request(TxSource::VOX))
                                {
                                    system_state_set_ptt(true);
                                    Serial.printf("[VOX] >>> TX OPENED (Moving RMS=%u >= Threshold=%u, sens=%u/20)\n",
                                                  moving_rms, threshold, sens);
                                }
                            }
                            pipeline->m_vox_hang_end = now + VOX_HANG_MS;
                        }
                        else
                        {
                            pipeline->m_speech_start_time = 0;
                        }

                        if (pipeline->m_vox_tx_open && now >= pipeline->m_vox_hang_end)
                        {
                            pipeline->m_vox_tx_open = false;
                            pipeline->m_speech_start_time = 0;
                            echolink_client_tx_release(TxSource::VOX);
                            system_state_set_ptt(false);
                            Serial.println(F("[VOX] TX closed (hang timer expired)"));
                        }
                    }
                    else
                    {
                        if (pipeline->m_vox_tx_open)
                        {
                            pipeline->force_close_vox();
                        }
                    }

                    // VOX Mode: push newest frame into VoxPreRoll delay line
                    int16_t delayed_frame[AUDIO_FRAME_SAMPLES];
                    bool primed = pipeline->m_vox_pre_roll.push(mic_frame, delayed_frame);

                    tx_active = echolink_client_is_tx_active();
                    tx_src = echolink_client_get_tx_source();

                    if (tx_active && (tx_src == TxSource::VOX))
                    {
                        // Once VOX is open, write delayed frame to TX FIFO (skip if not primed yet)
                        if (primed)
                        {
                            if (s_dtmf_gate.should_mute_tx())
                            {
                                memset(delayed_frame, 0, sizeof(delayed_frame));
                            }
                            pipeline->tx_fifo_write(delayed_frame, AUDIO_FRAME_SAMPLES);
                        }
                    }
                }
                else
                {
                    // 5. PTT Mode: bypass pre-roll buffer completely (0 added latency)
                    if (tx_active && (tx_src == TxSource::PTT_BUTTON))
                    {
                        if (s_dtmf_gate.should_mute_tx())
                        {
                            int16_t silence[AUDIO_FRAME_SAMPLES] = {0};
                            pipeline->tx_fifo_write(silence, AUDIO_FRAME_SAMPLES);
                        }
                        else
                        {
                            pipeline->tx_fifo_write(mic_frame, AUDIO_FRAME_SAMPLES);
                        }
                    }
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
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

void audio_pipeline_clear_tx()
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->clear_tx();
}

void audio_pipeline_set_vox_preroll_delay(uint8_t delay_frames)
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->set_vox_preroll_delay(delay_frames);
}

uint8_t audio_pipeline_get_vox_preroll_delay()
{
    if (AudioPipeline::instance())
        return AudioPipeline::instance()->get_vox_preroll_delay();
    return 3;
}

void audio_pipeline_request_vox_reset()
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->request_vox_reset();
}

bool audio_pipeline_is_vox_open()
{
    if (AudioPipeline::instance())
        return AudioPipeline::instance()->is_vox_tx_open();
    return false;
}

void audio_pipeline_force_close_vox()
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->force_close_vox();
}

void audio_pipeline_reset_vox()
{
    if (AudioPipeline::instance())
        AudioPipeline::instance()->reset_vox();
}


