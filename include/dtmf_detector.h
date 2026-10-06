#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cstdint>
#include <cstddef>

/**
 * @file dtmf_detector.h
 * @brief Real-time DSP DTMF Tone Detector using Integer Goertzel Algorithm (8 kHz sample rate)
 *
 * Standard DTMF Frequencies:
 * Low Group  (Rows)   : 697 Hz, 770 Hz, 852 Hz, 941 Hz
 * High Group (Columns): 1209 Hz, 1336 Hz, 1477 Hz, 1633 Hz
 */

typedef void (*DTMFDigitCallback)(char digit);

enum class DtmfRejectReason : uint8_t
{
    None = 0,
    EnergyLow,
    LowDominance,
    HighDominance,
    EnergyRatio,
    Harm2,
    Twist,
    FrequencyInstability,
    AmplitudeInstability
};

const char *dtmf_reject_reason_str(DtmfRejectReason reason);

struct DtmfFrameResult
{
    bool candidate;                 // Fast 1-frame tone pair detected
    bool candidate_strict;          // Strict 1-frame tone pair passing all 7 criteria
    char candidate_char;            // Tone pair character, e.g. '1', '*', or '\0'
    char confirmed_digit;           // Debounced confirmed digit, or '\0'
    DtmfRejectReason reject_reason; // Reason for rejection, or None if candidate_strict
};

class DTMFDetector
{
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
     * @return DtmfFrameResult containing candidate, strict candidate, and confirmed digit info.
     */
    DtmfFrameResult process_frame(const int16_t *samples);

    /**
     * @brief Legacy single-frame processing returning confirmed digit.
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

    // Temporal stability tracking for strict criteria
    bool m_prev_strict;
    int m_prev_row_best;
    int m_prev_col_best;
    int64_t m_prev_row_max;
    int64_t m_prev_col_max;

    static const int32_t DTMF_COEFFS_Q14[8];
    static const int32_t ROW_HARM2_COEFFS_Q14[4];
    static const char DTMF_MAP[4][4];
};

// Global instance functions
void dtmf_detector_init();
void dtmf_detector_process(const int16_t *samples, size_t count);
DtmfFrameResult dtmf_detector_process_frame(const int16_t *samples);
void dtmf_detector_reset();
void dtmf_detector_set_enabled(bool enabled);
bool dtmf_detector_is_enabled();
char dtmf_detector_get_last_digit();
