#ifndef VOX_PRE_ROLL_H
#define VOX_PRE_ROLL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Fixed-size pre-roll delay line for VOX audio transmission.
 *
 * Preserves the initial syllable of speech that occurs during VOX attack
 * detection time (typically 30-50 ms) by delaying the transmitted audio by
 * a configurable number of frames (0..MAX_FRAMES, default 3 = 60 ms).
 *
 * Pure C++ class with zero dynamic allocation, constant-time operations,
 * and no external dependencies (Arduino/FreeRTOS-free, host testable).
 */
class VoxPreRoll
{
public:
    static constexpr size_t MAX_FRAMES    = 8;   // Up to 160 ms pre-roll delay
    static constexpr size_t FRAME_SAMPLES = 160; // 20 ms @ 8 kHz

    explicit VoxPreRoll(uint8_t delay_frames = 3)
        : m_delay_frames(0),
          m_write_idx(0),
          m_count(0)
    {
        setDelayFrames(delay_frames);
    }

    /**
     * @brief Set delay length in frames (0..MAX_FRAMES).
     * Clamps values > MAX_FRAMES and resets internal buffer state.
     */
    void setDelayFrames(uint8_t n)
    {
        if (n > MAX_FRAMES)
        {
            n = static_cast<uint8_t>(MAX_FRAMES);
        }
        m_delay_frames = n;
        reset();
    }

    /**
     * @brief Get current delay setting in frames.
     */
    uint8_t getDelayFrames() const
    {
        return m_delay_frames;
    }

    /**
     * @brief Reset delay line state (empties buffer, un-primes).
     */
    void reset()
    {
        m_write_idx = 0;
        m_count = 0;
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    /**
     * @brief Check whether the buffer has primed enough frames to start outputting.
     */
    bool isPrimed() const
    {
        return (m_delay_frames == 0) || (m_count >= m_delay_frames);
    }

    /**
     * @brief Get number of frames currently primed in the buffer.
     */
    uint8_t getBufferedCount() const
    {
        return m_count;
    }

    /**
     * @brief Push newest input frame and retrieve delayed output frame.
     *
     * @param in  Pointer to 160 input samples (must not be null).
     * @param out Pointer to destination buffer for 160 output samples.
     * @return true if buffer is primed and `out` contains valid delayed frame;
     *         false if buffer is still priming (first `n` pushes).
     */
    bool push(const int16_t *in, int16_t *out)
    {
        if (!in)
        {
            return false;
        }

        // When delay is 0, act as an immediate pass-through
        if (m_delay_frames == 0)
        {
            if (out && out != in)
            {
                memcpy(out, in, FRAME_SAMPLES * sizeof(int16_t));
            }
            return true;
        }

        // Store newest frame in ring buffer
        memcpy(m_buffer[m_write_idx], in, FRAME_SAMPLES * sizeof(int16_t));

        if (m_count < m_delay_frames)
        {
            m_count++;
            m_write_idx = (m_write_idx + 1) % MAX_FRAMES;
            return false; // Still priming initial delay frames
        }

        // Buffer is primed (m_count == m_delay_frames).
        // Read oldest frame recorded m_delay_frames pushes ago.
        size_t read_idx = (m_write_idx + MAX_FRAMES - m_delay_frames) % MAX_FRAMES;
        if (out)
        {
            memcpy(out, m_buffer[read_idx], FRAME_SAMPLES * sizeof(int16_t));
        }

        m_write_idx = (m_write_idx + 1) % MAX_FRAMES;
        return true;
    }

    /**
     * @brief Zero out the most recently written n frames in the pre-roll buffer.
     * Used when a DTMF tone candidate is detected to prevent the onset of the tone
     * that was already buffered from leaking to TX.
     */
    void muteRecentFrames(uint8_t n)
    {
        if (n > MAX_FRAMES)
        {
            n = static_cast<uint8_t>(MAX_FRAMES);
        }
        if (n > m_count)
        {
            n = m_count;
        }
        for (uint8_t i = 0; i < n; i++)
        {
            size_t idx = (m_write_idx + MAX_FRAMES - 1 - i) % MAX_FRAMES;
            memset(m_buffer[idx], 0, FRAME_SAMPLES * sizeof(int16_t));
        }
    }

private:
    int16_t m_buffer[MAX_FRAMES][FRAME_SAMPLES]; // 8 * 160 * 2 = 2560 bytes static
    uint8_t m_delay_frames;
    uint8_t m_write_idx;
    uint8_t m_count;
};

#endif // VOX_PRE_ROLL_H
