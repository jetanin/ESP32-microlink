#include "jitter_buffer.h"
#include <cstring>

JitterBuffer::JitterBuffer(size_t watermark)
    : m_head(0),
      m_tail(0),
      m_count(0),
      m_watermark(watermark),
      m_buffering(true),
      m_mutex(nullptr),
      m_frames_pushed(0),
      m_frames_popped(0),
      m_underflow_count(0),
      m_overflow_count(0),
      m_consecutive_underflows(0)
{
    m_mutex = xSemaphoreCreateMutex();
    reset();
}

JitterBuffer::~JitterBuffer()
{
    if (m_mutex)
    {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

void JitterBuffer::reset()
{
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        m_head = 0;
        m_tail = 0;
        m_count = 0;
        m_buffering = true;
        m_consecutive_underflows = 0;
        xSemaphoreGive(m_mutex);
    }
}

void JitterBuffer::set_watermark(size_t watermark)
{
    if (watermark == 0)
        watermark = 1;
    if (watermark > CAPACITY_FRAMES / 2)
        watermark = CAPACITY_FRAMES / 2;

    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        m_watermark = watermark;
        xSemaphoreGive(m_mutex);
    }
}

bool JitterBuffer::push_frame(const AudioFrame &frame)
{
    if (!m_mutex || xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return false;
    }

    bool success = true;

    // If buffer is full, drop oldest frame to maintain low real-time latency
    if (m_count >= CAPACITY_FRAMES)
    {
        m_head = (m_head + 1) % CAPACITY_FRAMES;
        m_count--;
        m_overflow_count++;
        success = false;
    }

    m_queue[m_tail] = frame;
    m_tail = (m_tail + 1) % CAPACITY_FRAMES;
    m_count++;
    m_frames_pushed++;

    // Release buffering hold once watermark threshold is reached
    if (m_buffering && m_count >= m_watermark)
    {
        m_buffering = false;
    }

    xSemaphoreGive(m_mutex);
    return success;
}

bool JitterBuffer::pop_frame(AudioFrame &out_frame)
{
    if (!m_mutex || xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        memset(out_frame.samples, 0, sizeof(out_frame.samples));
        return false;
    }

    // If buffer is currently empty
    if (m_count == 0)
    {
        m_underflow_count++;
        m_consecutive_underflows++;

        // Only enter pre-buffering hold if empty for a prolonged silence period
        // (> 6 frames = 120 ms). A momentary 1-frame jitter gap will NOT block playback!
        if (m_consecutive_underflows >= 6)
        {
            m_buffering = true;
        }

        memset(out_frame.samples, 0, sizeof(out_frame.samples)); // Zero-fill silence
        xSemaphoreGive(m_mutex);
        return false;
    }

    // If still in initial pre-buffering phase (waiting for watermark), hold playout
    if (m_buffering)
    {
        memset(out_frame.samples, 0, sizeof(out_frame.samples));
        xSemaphoreGive(m_mutex);
        return false;
    }

    // Normal playout: pop frame
    out_frame = m_queue[m_head];
    m_head = (m_head + 1) % CAPACITY_FRAMES;
    m_count--;
    m_frames_popped++;
    m_consecutive_underflows = 0;

    xSemaphoreGive(m_mutex);
    return true;
}

JitterBufferStats JitterBuffer::get_stats()
{
    JitterBufferStats stats{};
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        stats.frames_pushed = m_frames_pushed;
        stats.frames_popped = m_frames_popped;
        stats.underflow_count = m_underflow_count;
        stats.overflow_count = m_overflow_count;
        stats.current_depth_frames = m_count;
        stats.watermark_frames = m_watermark;
        stats.is_buffering = m_buffering;
        xSemaphoreGive(m_mutex);
    }
    return stats;
}
