#include "dtmf_detector.h"
#include <cmath>
#include <cstdlib>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Pre-computed Goertzel coefficients in Q14 fixed-point (2 * cos(2 * pi * f / 8000) * 16384):
// Frequencies: 697, 770, 852, 941, 1209, 1336, 1477, 1633 Hz
    const int32_t DTMFDetector::DTMF_COEFFS_Q14[8] = {
        27980, // 697 Hz  (1.7077484 * 16384 = 27979.7)
        26956, // 770 Hz  (1.6452882 * 16384 = 26956.4)
        25703, // 852 Hz  (1.5688059 * 16384 = 25703.3)
        24217, // 941 Hz  (1.4781282 * 16384 = 24217.3)
        19077, // 1209 Hz (1.1643403 * 16384 = 19076.7)
        16332, // 1336 Hz (0.9968360 * 16384 = 16332.2)
        13091, // 1477 Hz (0.7990263 * 16384 = 13091.2)
        9314   // 1633 Hz (0.5684841 * 16384 =  9314.0)
    };

// 2nd harmonic coefficients for low group in Q14 (2 * cos(2 * pi * (2*f_low) / 8000) * 16384):
// Row 0: 2 * 697 = 1394 Hz -> 15007
// Row 1: 2 * 770 = 1540 Hz -> 11579
// Row 2: 2 * 852 = 1704 Hz -> 7560
// Row 3: 2 * 941 = 1882 Hz -> 3031
const int32_t DTMFDetector::ROW_HARM2_COEFFS_Q14[4] = {
    15007,
    11579,
    7560,
    3031};

const char DTMFDetector::DTMF_MAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

const char *dtmf_reject_reason_str(DtmfRejectReason reason)
{
    switch (reason)
    {
    case DtmfRejectReason::None:
        return "None";
    case DtmfRejectReason::EnergyLow:
        return "EnergyLow";
    case DtmfRejectReason::LowDominance:
        return "LowDominance";
    case DtmfRejectReason::HighDominance:
        return "HighDominance";
    case DtmfRejectReason::EnergyRatio:
        return "EnergyRatio";
    case DtmfRejectReason::Harm2:
        return "Harm2";
    case DtmfRejectReason::Twist:
        return "Twist";
    case DtmfRejectReason::FrequencyInstability:
        return "FreqInstability";
    case DtmfRejectReason::AmplitudeInstability:
        return "AmpInstability";
    default:
        return "Unknown";
    }
}

static int16_t s_hamming_q15[160];
static bool s_hamming_initialized = false;

static DTMFDetector s_detector;

DTMFDetector::DTMFDetector()
    : m_enabled(true),
      m_callback(nullptr),
      m_candidate_char('\0'),
      m_tone_hits(0),
      m_pause_hits(0),
      m_reported(false),
      m_last_digit('\0'),
          m_digit_count(0),
      m_prev_strict(false),
      m_prev_row_best(-1),
      m_prev_col_best(-1),
      m_prev_row_max(0),
      m_prev_col_max(0)
{
}

void DTMFDetector::init()
{
    reset();
    m_enabled = true;
    if (!s_hamming_initialized)
    {
        for (int i = 0; i < 160; ++i)
        {
            float h = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)i / 159.0f);
            s_hamming_q15[i] = (int16_t)roundf(h * 32767.0f);
        }
        s_hamming_initialized = true;
    }
}

void DTMFDetector::reset()
{
    m_candidate_char = '\0';
    m_tone_hits = 0;
    m_pause_hits = 0;
    m_reported = false;
    m_prev_strict = false;
    m_prev_row_best = -1;
    m_prev_col_best = -1;
    m_prev_row_max = 0;
    m_prev_col_max = 0;
}

DtmfFrameResult DTMFDetector::process_frame(const int16_t *samples)
{
    DtmfFrameResult result = {false, false, '\0', '\0', DtmfRejectReason::None};
    if (!m_enabled || !samples)
        return result;

    if (!s_hamming_initialized)
    {
        init();
    }

    // 1. Fast peak check
    int32_t peak = 0;
    for (int i = 0; i < 160; ++i)
    {
        int32_t a = abs((int32_t)samples[i]);
        if (a > peak)
            peak = a;
    }

        if (peak < 350)
        {
            result.reject_reason = DtmfRejectReason::EnergyLow;
            m_prev_strict = false;
            m_pause_hits++;
                if (m_pause_hits >= 3)
                {
                    m_candidate_char = '\0';
                    m_tone_hits = 0;
                    m_reported = false;
                }
            return result;
        }

        // 2. Integer Hamming window & total frame energy
        int32_t x_w[160];
    int64_t total_energy = 0;
    for (int i = 0; i < 160; ++i)
    {
        int32_t s = ((int32_t)samples[i] * (int32_t)s_hamming_q15[i]) >> 15;
        x_w[i] = s;
        total_energy += (int64_t)s * s;
    }

        if (total_energy < 500000LL)
        {
            result.reject_reason = DtmfRejectReason::EnergyLow;
            m_prev_strict = false;
            m_pause_hits++;
            if (m_pause_hits >= 3)
            {
                m_candidate_char = '\0';
                m_tone_hits = 0;
                m_reported = false;
            }
            return result;
        }

    // 3. Integer Goertzel algorithm for 8 DTMF bins
    int64_t power[8];
    for (int k = 0; k < 8; ++k)
    {
        int32_t coeff = DTMF_COEFFS_Q14[k];
        int32_t s0 = 0, s1 = 0, s2 = 0;

        for (int i = 0; i < 160; ++i)
        {
            int32_t term = (int32_t)(((int64_t)coeff * s1) >> 14);
            s0 = x_w[i] + term - s2;
            s2 = s1;
            s1 = s0;
        }

        int64_t s1_sq = (int64_t)s1 * s1;
        int64_t s2_sq = (int64_t)s2 * s2;
        int64_t prod = ((int64_t)coeff * s1) >> 14;
        int64_t p = s1_sq + s2_sq - prod * s2;
        if (p < 0)
            p = 0;
        power[k] = p;
    }

    // Best and second best in Low group (0..3)
    int row_best = 0;
    int64_t row_max = power[0];
    for (int r = 1; r < 4; ++r)
    {
        if (power[r] > row_max)
        {
            row_max = power[r];
            row_best = r;
        }
    }
    int64_t row_second = 0;
    for (int r = 0; r < 4; ++r)
    {
        if (r != row_best && power[r] > row_second)
        {
            row_second = power[r];
        }
    }

    // Best and second best in High group (4..7)
    int col_best = 4;
    int64_t col_max = power[4];
    for (int c = 5; c < 8; ++c)
    {
        if (power[c] > col_max)
        {
            col_max = power[c];
            col_best = c;
        }
    }
    int64_t col_second = 0;
    for (int c = 4; c < 8; ++c)
    {
        if (c != col_best && power[c] > col_second)
        {
            col_second = power[c];
        }
    }

    // STRICT CRITERIA VALIDATION:
    // 1. Absolute energy floor: both tones above noise threshold
    if (row_max < 15000000LL || col_max < 1500000LL)
    {
        result.reject_reason = DtmfRejectReason::EnergyLow;
        m_prev_strict = false;
        m_pause_hits++;
        if (m_pause_hits >= 3)
        {
            m_candidate_char = '\0';
            m_tone_hits = 0;
            m_reported = false;
        }
        return result;
    }

    // 2. Low group dominance: strongest bin >= 8.0x second strongest (~9 dB)
    if (row_max < 8 * row_second)
    {
        result.reject_reason = DtmfRejectReason::LowDominance;
        m_prev_strict = false;
        m_pause_hits++;
        if (m_pause_hits >= 3)
        {
            m_candidate_char = '\0';
            m_tone_hits = 0;
            m_reported = false;
        }
        return result;
    }

    // 3. High group dominance: strongest bin >= 8.0x second strongest (~9 dB)
    if (col_max < 8 * col_second)
    {
        result.reject_reason = DtmfRejectReason::HighDominance;
        m_prev_strict = false;
        m_pause_hits++;
        if (m_pause_hits >= 3)
        {
            m_candidate_char = '\0';
            m_tone_hits = 0;
            m_reported = false;
        }
        return result;
    }

    // 4. Twist limit: telecom limits (+/- 10 dB to accommodate radio pre-emphasis / de-emphasis)
    if ((row_max > 10 * col_max) || (col_max > 10 * row_max))
    {
        result.reject_reason = DtmfRejectReason::Twist;
        m_prev_strict = false;
        m_pause_hits++;
        if (m_pause_hits >= 3)
        {
            m_candidate_char = '\0';
            m_tone_hits = 0;
            m_reported = false;
        }
        return result;
    }

        // 5. Low group second harmonic check
        int32_t harm2_coeff = ROW_HARM2_COEFFS_Q14[row_best];
    int32_t h_s0 = 0, h_s1 = 0, h_s2 = 0;
    for (int i = 0; i < 160; ++i)
    {
        int32_t term = (int32_t)(((int64_t)harm2_coeff * h_s1) >> 14);
        h_s0 = x_w[i] + term - h_s2;
        h_s2 = h_s1;
        h_s1 = h_s0;
    }
    int64_t harm2_power = (int64_t)h_s1 * h_s1 + (int64_t)h_s2 * h_s2 -
                          (((int64_t)harm2_coeff * h_s1) >> 14) * h_s2;
    if (harm2_power < 0)
        harm2_power = 0;

        // Account for high group column tone sidelobe leakage into adjacent harmonic bins
        int64_t col_leakage = 0;
    if (row_best == 0 && col_best == 5) // '2': 1336 Hz leaks into 1394 Hz (~-12 dB)
    {
        col_leakage = col_max / 14;
    }
    else if (row_best == 1 && col_best == 6) // '6': 1477 Hz leaks into 1540 Hz (~-14 dB)
    {
        col_leakage = col_max / 18;
    }
    else if (row_best == 2 && col_best == 7) // 'C': 1633 Hz leaks into 1704 Hz (~-15 dB)
    {
        col_leakage = col_max / 22;
    }
    int64_t harm2_net = harm2_power - col_leakage;
    if (harm2_net < 0)
        harm2_net = 0;

        if (row_max < 10 * harm2_net)
        {
                result.reject_reason = DtmfRejectReason::Harm2;
            m_prev_strict = false;
            m_pause_hits++;
            if (m_pause_hits >= 3)
            {
                m_candidate_char = '\0';
                m_tone_hits = 0;
                m_reported = false;
            }
            return result;
        }

            // 6. Energy ratio: (low_peak + high_peak) >= 0.80 * total_energy (~45 * total_energy)
            int64_t tone_p_sum = row_max + col_max;
    if (tone_p_sum < 45 * total_energy)
    {
        result.reject_reason = DtmfRejectReason::EnergyRatio;
        m_prev_strict = false;
        m_pause_hits++;
            if (m_pause_hits >= 3)
            {
                m_candidate_char = '\0';
                m_tone_hits = 0;
                m_reported = false;
            }
        return result;
    }

    // 7. Temporal stability across consecutive frames
    if (m_prev_strict)
    {
        if (row_best != m_prev_row_best || col_best != m_prev_col_best)
        {
            result.reject_reason = DtmfRejectReason::FrequencyInstability;
            m_prev_strict = false;
            m_pause_hits++;
            if (m_pause_hits >= 3)
            {
                m_candidate_char = '\0';
                m_tone_hits = 0;
                m_reported = false;
            }
            return result;
        }
        if ((row_max > 4 * m_prev_row_max) || (m_prev_row_max > 4 * row_max) ||
            (col_max > 4 * m_prev_col_max) || (m_prev_col_max > 4 * col_max))
        {
            result.reject_reason = DtmfRejectReason::AmplitudeInstability;
            m_prev_strict = false;
            m_pause_hits++;
            if (m_pause_hits >= 3)
            {
                m_candidate_char = '\0';
                m_tone_hits = 0;
                m_reported = false;
            }
            return result;
        }
    }

    // All strict criteria passed!
    char detected = DTMF_MAP[row_best][col_best - 4];
    result.candidate = true;
    result.candidate_strict = true;
    result.candidate_char = detected;
    result.reject_reason = DtmfRejectReason::None;

    m_prev_strict = true;
    m_prev_row_best = row_best;
    m_prev_col_best = col_best;
    m_prev_row_max = row_max;
    m_prev_col_max = col_max;

    // Debouncing & Confirmation State Machine (Requires >= 40 ms tone)
    m_pause_hits = 0;
    if (detected == m_candidate_char)
    {
        m_tone_hits++;
        if (m_tone_hits >= 2 && !m_reported)
        {
            m_reported = true;
            result.confirmed_digit = detected;
            m_last_digit = detected;
            m_digit_count++;
            if (m_callback)
            {
                m_callback(detected);
            }
        }
    }
    else
    {
        m_candidate_char = detected;
        m_tone_hits = 1;
        m_reported = false;
    }

    return result;
}

char DTMFDetector::process_frame_160(const int16_t *samples)
{
    return process_frame(samples).confirmed_digit;
}

void DTMFDetector::process(const int16_t *samples, size_t count)
{
    if (!m_enabled || !samples || count < 160)
        return;

    size_t offset = 0;
    while (offset + 160 <= count)
    {
        process_frame(samples + offset);
        offset += 160;
    }
}

// Global wrappers
void dtmf_detector_init()
{
    s_detector.init();
    s_detector.set_digit_callback(nullptr);
}

void dtmf_detector_process(const int16_t *samples, size_t count)
{
    s_detector.process(samples, count);
}

DtmfFrameResult dtmf_detector_process_frame(const int16_t *samples)
{
    return s_detector.process_frame(samples);
}

void dtmf_detector_reset()
{
    s_detector.reset();
}

void dtmf_detector_set_enabled(bool enabled)
{
    s_detector.set_enabled(enabled);
}

bool dtmf_detector_is_enabled()
{
    return s_detector.is_enabled();
}

char dtmf_detector_get_last_digit()
{
    return s_detector.get_last_digit();
}
