#include "dtmf_detector.h"
#include "dtmf_controller.h"
#include <cmath>
#include <cstring>

// Pre-computed Goertzel coefficients for 8000 Hz sample rate:
// coeff = 2 * cos(2 * pi * f / 8000)
// Frequencies: 697, 770, 852, 941, 1209, 1336, 1477, 1633 Hz
const float DTMFDetector::DTMF_COEFFS[8] = {
    1.7077484f, // 697 Hz
    1.6452882f, // 770 Hz
    1.5688059f, // 852 Hz
    1.4781282f, // 941 Hz
    1.1643403f, // 1209 Hz
    0.9968360f, // 1336 Hz
    0.7990263f, // 1477 Hz
    0.5684841f  // 1633 Hz
};

const char DTMFDetector::DTMF_MAP[4][4] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '*', '0', '#', 'D' }
};

static float s_hamming[160];
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
      m_digit_count(0)
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
            s_hamming[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)i / 159.0f);
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
}

char DTMFDetector::process_frame_160(const int16_t *samples)
{
    if (!m_enabled || !samples) return '\0';

    // Fast integer peak check first! If the signal amplitude is low (silence / quiet),
    // skip Goertzel completely to save CPU on ESP32-C6 single-core RISC-V.
    int32_t peak = 0;
    for (int i = 0; i < 160; ++i)
    {
        int32_t a = abs((int32_t)samples[i]);
        if (a > peak) peak = a;
    }

    if (peak < 500) // ~1.5% of full scale (-36 dBFS) to reliably detect weak radio DTMF
    {
        m_pause_hits++;
        if (m_pause_hits >= 2)
        {
            m_candidate_char = '\0';
            m_tone_hits = 0;
            m_reported = false;
        }
        return '\0';
    }

    if (!s_hamming_initialized)
    {
        init();
    }

    // Apply Hamming window to reduce spectral leakage across adjacent DTMF bins
    float x[160];
    float total_energy = 0.0f;
    constexpr float INV_32768 = 1.0f / 32768.0f;
    for (int i = 0; i < 160; ++i)
    {
        x[i] = ((float)samples[i] * INV_32768) * s_hamming[i];
        total_energy += x[i] * x[i];
    }

    float power[8];
    // Run Goertzel algorithm on 8 DTMF frequencies
    for (int k = 0; k < 8; ++k)
    {
        float s0 = 0.0f;
        float s1 = 0.0f;
        float s2 = 0.0f;
        float coeff = DTMF_COEFFS[k];

        for (int i = 0; i < 160; ++i)
        {
            s0 = x[i] + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        power[k] = s1 * s1 + s2 * s2 - coeff * s1 * s2;
        if (power[k] < 0.0f) power[k] = 0.0f;
    }

    // Find best and second best in Low group (0..3)
    int row_best = 0;
    float row_max = power[0];
    for (int r = 1; r < 4; ++r)
    {
        if (power[r] > row_max)
        {
            row_max = power[r];
            row_best = r;
        }
    }
    float row_second = 0.0f;
    for (int r = 0; r < 4; ++r)
    {
        if (r != row_best && power[r] > row_second)
        {
            row_second = power[r];
        }
    }

    // Find best and second best in High group (4..7)
    int col_best = 4;
    float col_max = power[4];
    for (int c = 5; c < 8; ++c)
    {
        if (power[c] > col_max)
        {
            col_max = power[c];
            col_best = c;
        }
    }
    float col_second = 0.0f;
    for (int c = 4; c < 8; ++c)
    {
        if (c != col_best && power[c] > col_second)
        {
            col_second = power[c];
        }
    }

    char detected = '\0';

    // Standard DTMF Validation Tests adapted for amateur radio channels:
    // 1. Minimum energy threshold (row tone and column tone present)
    // 2. Selectivity (peak tone >= 1.8x second highest in group)
    // 3. Twist check: Expanded to [0.08 .. 35.0] (+/- 15 dB) to support FM receiver de-emphasis.
    //    Digits '2', '3', '6' have high twist (~15x to 20x) because of large frequency gap.
    // 4. SNR / Concentrated tone energy test: Total DTMF energy vs total signal energy
    bool energy_ok = (row_max > 0.05f) && (col_max > 0.005f);
    bool row_selective = (row_second <= 0.0005f) || (row_max >= 1.8f * row_second);
    bool col_selective = (col_second <= 0.0005f) || (col_max >= 1.8f * col_second);

    float twist = row_max / (col_max + 1e-6f);
    bool twist_ok = (twist >= 0.08f && twist <= 35.0f);

    float tone_energy = (row_max + col_max) / 58.5f;
    bool snr_ok = (tone_energy >= 0.35f * total_energy);

    if (energy_ok && row_selective && col_selective && twist_ok && snr_ok)
    {
        detected = DTMF_MAP[row_best][col_best - 4];
    }

    // Debouncing & Confirmation State Machine (Requires >= 40 ms tone + >= 40 ms pause)
    char confirmed_digit = '\0';
    if (detected != '\0')
    {
        m_pause_hits = 0;
        if (detected == m_candidate_char)
        {
            m_tone_hits++;
            if (m_tone_hits >= 2 && !m_reported) // 2 consecutive 20 ms frames = 40 ms
            {
                m_reported = true;
                confirmed_digit = detected;
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
    }
    else
    {
        m_pause_hits++;
        if (m_pause_hits >= 2) // 40 ms of pause/silence
        {
            m_candidate_char = '\0';
            m_tone_hits = 0;
            m_reported = false;
        }
    }

    return confirmed_digit;
}

void DTMFDetector::process(const int16_t *samples, size_t count)
{
    if (!m_enabled || !samples || count < 160) return;

    size_t offset = 0;
    while (offset + 160 <= count)
    {
        process_frame_160(samples + offset);
        offset += 160;
    }
}

// Global wrappers
void dtmf_detector_init()
{
    s_detector.init();
    s_detector.set_digit_callback(dtmf_controller_handle_digit);
}

void dtmf_detector_process(const int16_t *samples, size_t count)
{
    s_detector.process(samples, count);
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
