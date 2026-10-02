#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

/**
 * @file dtmf_detector.h
 * @brief Real-time DSP DTMF Tone Detector using Goertzel Algorithm (8 kHz sample rate)
 *
 * Standard DTMF Frequencies:
 * Low Group  (Rows)   : 697 Hz, 770 Hz, 852 Hz, 941 Hz
 * High Group (Columns): 1209 Hz, 1336 Hz, 1477 Hz, 1633 Hz
 */

typedef void (*DTMFDigitCallback)(char digit);

class DTMFDetector {
public:
    DTMFDetector();

    void init();
    void reset();

    void set_enabled(bool enabled) { m_enabled = enabled; }
    bool is_enabled() const { return m_enabled; }

    void set_digit_callback(DTMFDigitCallback cb) { m_callback = cb; }

    /**
     * @brief Process an audio buffer of 16-bit PCM samples at 8 kHz.
     * @param samples Pointer to PCM samples
     * @param count Number of samples
     */
    void process(const int16_t *samples, size_t count);

    /**
     * @brief Process a single 20 ms frame (160 samples at 8 kHz).
     * @return Detected character if confirmed on this frame, or '\0'
     */
    char process_frame_160(const int16_t *samples);

    char get_last_digit() const { return m_last_digit; }
    uint32_t get_digit_count() const { return m_digit_count; }

private:
    bool m_enabled;
    DTMFDigitCallback m_callback;

    char m_candidate_char;
    uint8_t m_tone_hits;
    uint8_t m_pause_hits;
    bool m_reported;

    char m_last_digit;
    uint32_t m_digit_count;

    static const float DTMF_COEFFS[8];
    static const char DTMF_MAP[4][4];
};

// Global instance functions
void dtmf_detector_init();
void dtmf_detector_process(const int16_t *samples, size_t count);
void dtmf_detector_reset();
void dtmf_detector_set_enabled(bool enabled);
bool dtmf_detector_is_enabled();
char dtmf_detector_get_last_digit();
