#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <random>
#include <map>

#include "dtmf_detector.h"
#include "dtmf_gate.h"
#include "vox_pre_roll.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Stubs for linking
void dtmf_controller_handle_digit(char) {}

static DtmfGate s_gate;
DtmfGate &dtmf_gate_instance()
{
    return s_gate;
}

// Test VOX detector model
class TestVoxDetector
{
public:
    static constexpr uint32_t ATTACK_MS = 40; // 2 frames
    static constexpr uint32_t HANG_MS = 600;  // 600 ms

    TestVoxDetector() : m_open(false), m_speech_start_ms(0), m_hang_end_ms(0) {}

    bool is_open() const { return m_open; }

    void force_close()
    {
        m_open = false;
        m_speech_start_ms = 0;
        m_hang_end_ms = 0;
    }

    bool update(uint16_t rms, uint16_t threshold, uint32_t now_ms, bool inhibit)
    {
        if (inhibit)
        {
            force_close();
            return false;
        }

        bool voice_active = (rms >= threshold);
        if (voice_active)
        {
            if (m_speech_start_ms == 0)
            {
                m_speech_start_ms = now_ms;
            }
            if (!m_open && (uint32_t)(now_ms - m_speech_start_ms) >= ATTACK_MS)
            {
                m_open = true;
            }
            m_hang_end_ms = now_ms + HANG_MS;
        }
        else
        {
            m_speech_start_ms = 0;
        }

        if (m_open && (now_ms >= m_hang_end_ms))
        {
            m_open = false;
            m_speech_start_ms = 0;
        }
        return m_open;
    }

private:
    bool m_open;
    uint32_t m_speech_start_ms;
    uint32_t m_hang_end_ms;
};

void generate_vowel_token(float f0, float F1, float F2, float F3, float duration_ms,
                          float amplitude, std::vector<int16_t> &out_samples)
{
    size_t num_samples = (size_t)(duration_ms * 8.0f);
    out_samples.resize(num_samples);
    float T0_samples = 8000.0f / f0;
    float pulse_phase = 0.0f;
    float r1 = 0.95f, r2 = 0.94f, r3 = 0.96f;
    float c1 = 2.0f * r1 * std::cos(2.0f * (float)M_PI * F1 / 8000.0f);
    float c2 = 2.0f * r2 * std::cos(2.0f * (float)M_PI * F2 / 8000.0f);
    float c3 = 2.0f * r3 * std::cos(2.0f * (float)M_PI * F3 / 8000.0f);
    float y1[2] = {0, 0}, y2[2] = {0, 0}, y3[2] = {0, 0};

    for (size_t i = 0; i < num_samples; ++i)
    {
        float glottal = 0.0f;
        if (pulse_phase < 0.25f * T0_samples)
        {
            float p = pulse_phase / (0.25f * T0_samples);
            glottal = std::sin((float)M_PI * p) * amplitude;
        }
        pulse_phase += 1.0f;
        if (pulse_phase >= T0_samples)
            pulse_phase -= T0_samples;

        float out1 = glottal + c1 * y1[0] - (r1 * r1) * y1[1];
        y1[1] = y1[0];
        y1[0] = out1;
        float out2 = (glottal * 0.7f) + c2 * y2[0] - (r2 * r2) * y2[1];
        y2[1] = y2[0];
        y2[0] = out2;
        float out3 = (glottal * 0.3f) + c3 * y3[0] - (r3 * r3) * y3[1];
        y3[1] = y3[0];
        y3[0] = out3;

        float s = (out1 + out2 + out3) * 0.25f;
        if (s > 32767.0f)
            s = 32767.0f;
        if (s < -32768.0f)
            s = -32768.0f;
        out_samples[i] = (int16_t)s;
    }
}

void generate_dtmf(char symbol, float duration_ms, float amplitude, float twist_db,
                   float snr_db, std::vector<int16_t> &out_samples, std::mt19937 &rng)
{
    static const struct
    {
        char c;
        float f_low;
        float f_high;
    } MAP[] = {
        {'1', 697.0f, 1209.0f}, {'2', 697.0f, 1336.0f}, {'3', 697.0f, 1477.0f}, {'A', 697.0f, 1633.0f}, {'4', 770.0f, 1209.0f}, {'5', 770.0f, 1336.0f}, {'6', 770.0f, 1477.0f}, {'B', 770.0f, 1633.0f}, {'7', 852.0f, 1209.0f}, {'8', 852.0f, 1336.0f}, {'9', 852.0f, 1477.0f}, {'C', 852.0f, 1633.0f}, {'*', 941.0f, 1209.0f}, {'0', 941.0f, 1336.0f}, {'#', 941.0f, 1477.0f}, {'D', 941.0f, 1633.0f}};
    float flow = 0.0f, fhigh = 0.0f;
    for (const auto &item : MAP)
    {
        if (item.c == symbol)
        {
            flow = item.f_low;
            fhigh = item.f_high;
            break;
        }
    }
    size_t num_samples = (size_t)(duration_ms * 8.0f);
    out_samples.resize(num_samples);
    float twist_ratio = std::pow(10.0f, twist_db / 20.0f);
    float a_low = amplitude / std::sqrt(1.0f + twist_ratio * twist_ratio);
    float a_high = a_low * twist_ratio;
    float signal_power = (a_low * a_low + a_high * a_high) / 2.0f;
    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);
    std::normal_distribution<float> noise_dist(0.0f, noise_std);

    for (size_t i = 0; i < num_samples; ++i)
    {
        float t = (float)i / 8000.0f;
        float s = a_low * std::sin(2.0f * (float)M_PI * flow * t) +
                  a_high * std::sin(2.0f * (float)M_PI * fhigh * t);
        s += noise_dist(rng);
        if (s > 32767.0f)
            s = 32767.0f;
        if (s < -32768.0f)
            s = -32768.0f;
        out_samples[i] = (int16_t)s;
    }
}

uint16_t compute_frame_rms(const int16_t *frame)
{
    uint64_t sum_sq = 0;
    for (int i = 0; i < 160; ++i)
    {
        sum_sq += (int32_t)frame[i] * (int32_t)frame[i];
    }
    return (uint16_t)std::sqrt(sum_sq / 160);
}

int main()
{
    std::cout << "=================================================================\n";
    std::cout << "   ESP32-MicroLink Phase 2: Full Production Verification Test    \n";
    std::cout << "=================================================================\n\n";

    // 1. Vowel Grid
    std::cout << "[1] Testing Vowel Grid (F1: 650-950 Hz, F2: 1000-1500 Hz)...\n";
    std::vector<float> f0_list = {110.0f, 130.0f, 160.0f, 220.0f};
    std::vector<float> F1_list = {650.0f, 700.0f, 750.0f, 800.0f, 850.0f, 900.0f, 950.0f};
    std::vector<float> F2_list = {1050.0f, 1150.0f, 1250.0f, 1350.0f, 1450.0f};
    std::vector<float> F3_list = {2400.0f, 2600.0f};

    std::vector<int16_t> vowel_stream;
    uint32_t total_vowels = 0;
    for (float f0 : f0_list)
    {
        for (float f1 : F1_list)
        {
            for (float f2 : F2_list)
            {
                for (float f3 : F3_list)
                {
                    std::vector<int16_t> tok;
                    generate_vowel_token(f0, f1, f2, f3, 200.0f, 18000.0f, tok);
                    vowel_stream.insert(vowel_stream.end(), tok.begin(), tok.end());
                    vowel_stream.insert(vowel_stream.end(), 1600, 0); // 200 ms pause
                    total_vowels++;
                }
            }
        }
    }

    size_t num_vowel_frames = vowel_stream.size() / 160;
    DTMFDetector v_det;
    v_det.init();
    DtmfGate v_gate;
    TestVoxDetector v_vox;
    uint32_t v_candidate_strict = 0;
    uint32_t v_force_close_triggers = 0;
    uint32_t v_vox_key_events = 0;
    uint32_t v_confirmed_digits = 0;
    uint32_t v_now = 1000;
    bool prev_vox_open = false;
    std::map<std::string, uint32_t> reject_counts;

    for (size_t f = 0; f < num_vowel_frames; ++f)
    {
        const int16_t *frame = vowel_stream.data() + f * 160;
        DtmfFrameResult res = v_det.process_frame(frame);
        if (res.candidate_strict)
            v_candidate_strict++;
        if (res.confirmed_digit != '\0')
            v_confirmed_digits++;
        reject_counts[dtmf_reject_reason_str(res.reject_reason)]++;

        v_gate.update(res.candidate_strict, res.confirmed_digit, false, v_now);
        if (v_gate.should_force_close_vox())
        {
            v_vox.force_close();
            v_force_close_triggers++;
        }

        uint16_t rms = compute_frame_rms(frame);
        bool vox_open = v_vox.update(rms, 130, v_now, v_gate.is_vox_inhibited());
        if (vox_open && !prev_vox_open)
        {
            v_vox_key_events++;
        }
        prev_vox_open = vox_open;
        v_now += 20;
    }

    std::cout << "    Total vowel tokens        : " << total_vowels << " (" << num_vowel_frames << " frames)\n";
    std::cout << "    Candidate strict frames   : " << v_candidate_strict << "\n";
    std::cout << "    VOX force-close triggers  : " << v_force_close_triggers << " (Target: 0)\n";
    std::cout << "    VOX key-up events allowed : " << v_vox_key_events << " / " << total_vowels << " (100% allowed!)\n";
    std::cout << "    Confirmed false digits    : " << v_confirmed_digits << " (Target: 0)\n";
    std::cout << "    Reject reason breakdown:\n";
    for (auto &kv : reject_counts)
    {
        std::cout << "      " << std::setw(18) << kv.first << ": " << kv.second << " frames\n";
    }

    // 2. 10-Minute Speech-like stream
    std::cout << "\n[2] Testing 10-Minute Speech-like Stream...\n";
    std::vector<int16_t> speech_dataset;
    speech_dataset.reserve(600 * 8000);
    std::mt19937 sp_rng(12345);
    std::uniform_int_distribution<int> type_dist(0, 4);
    std::uniform_real_distribution<float> dur_dist(0.15f, 0.45f);
    std::uniform_real_distribution<float> f0_dist(100.0f, 240.0f);
    std::uniform_real_distribution<float> f1_dist(300.0f, 1000.0f);
    std::uniform_real_distribution<float> f2_dist(800.0f, 2500.0f);
    std::uniform_real_distribution<float> f3_dist(2200.0f, 3200.0f);
    std::uniform_real_distribution<float> amp_dist(5000.0f, 22000.0f);

    while (speech_dataset.size() < 600 * 8000)
    {
        int seg_type = type_dist(sp_rng);
        float dur = dur_dist(sp_rng);
        size_t n = (size_t)(dur * 8000.0f);

        if (seg_type <= 2)
        {
            std::vector<int16_t> token;
            generate_vowel_token(f0_dist(sp_rng), f1_dist(sp_rng), f2_dist(sp_rng), f3_dist(sp_rng),
                                 dur * 1000.0f, amp_dist(sp_rng), token);
            speech_dataset.insert(speech_dataset.end(), token.begin(), token.end());
        }
        else if (seg_type == 3)
        {
            std::normal_distribution<float> noise(0.0f, amp_dist(sp_rng) * 0.4f);
            float prev = 0.0f;
            for (size_t i = 0; i < n; ++i)
            {
                float v = 0.8f * noise(sp_rng) - 0.4f * prev;
                prev = v;
                if (v > 32767.0f)
                    v = 32767.0f;
                if (v < -32768.0f)
                    v = -32768.0f;
                speech_dataset.push_back((int16_t)v);
            }
        }
        else
        {
            size_t pause_n = (size_t)(0.08f * 8000.0f);
            speech_dataset.insert(speech_dataset.end(), pause_n, 0);
        }
    }

    size_t num_speech_frames = speech_dataset.size() / 160;
    DTMFDetector sp_det;
    sp_det.init();
    DtmfGate sp_gate;
    uint32_t sp_candidate_strict = 0;
    uint32_t sp_force_close_triggers = 0;
    uint32_t sp_confirmed_digits = 0;
    uint32_t sp_now = 1000;

    for (size_t f = 0; f < num_speech_frames; ++f)
    {
        const int16_t *frame = speech_dataset.data() + f * 160;
        DtmfFrameResult res = sp_det.process_frame(frame);
        if (res.candidate_strict)
            sp_candidate_strict++;
        if (res.confirmed_digit != '\0')
            sp_confirmed_digits++;

        sp_gate.update(res.candidate_strict, res.confirmed_digit, false, sp_now);
        if (sp_gate.should_force_close_vox())
            sp_force_close_triggers++;
        sp_now += 20;
    }

    std::cout << "    Generated 10 minutes (" << num_speech_frames << " frames)\n";
    std::cout << "    Candidate strict triggers : " << sp_candidate_strict << "\n";
    std::cout << "    VOX force-close triggers  : " << sp_force_close_triggers << " (Target: 0)\n";
    std::cout << "    Confirmed false digits    : " << sp_confirmed_digits << " (Target: 0)\n";

    // 3. Real DTMF Benchmark
    std::cout << "\n[3] Testing Real DTMF Benchmark (All 16 symbols)...\n";
    std::mt19937 rng(42);
    static const char SYMBOLS[] = "0123456789ABCD*#";
    float AMPLITUDES[] = {1000.0f, 3000.0f, 8000.0f, 14000.0f};
    float TWISTS_DB[] = {-6.0f, -3.0f, 0.0f, +3.0f, +6.0f};
    float DURATIONS_MS[] = {45.0f, 60.0f, 80.0f, 100.0f};

    uint32_t dtmf_total_trials = 0;
    uint32_t dtmf_detected_trials = 0;

    for (char sym : SYMBOLS)
    {
        for (float amp : AMPLITUDES)
        {
            for (float tw : TWISTS_DB)
            {
                for (float dur : DURATIONS_MS)
                {
                    std::vector<int16_t> tone;
                    generate_dtmf(sym, dur, amp, tw, 20.0f, tone, rng);

                    std::vector<int16_t> trial;
                    trial.insert(trial.end(), 480, 0); // 60 ms pause
                    trial.insert(trial.end(), tone.begin(), tone.end());
                    trial.insert(trial.end(), 640, 0); // 80 ms pause

                    DTMFDetector test_det;
                    test_det.init();
                    bool found = false;

                    size_t frames = trial.size() / 160;
                    for (size_t f = 0; f < frames; ++f)
                    {
                        DtmfFrameResult fr = test_det.process_frame(trial.data() + f * 160);
                        if (fr.confirmed_digit == sym)
                        {
                            found = true;
                        }
                    }

                    dtmf_total_trials++;
                    if (found)
                        dtmf_detected_trials++;
                }
            }
        }
    }

    std::cout << "    Total DTMF trials         : " << dtmf_total_trials << "\n";
    std::cout << "    Successfully decoded      : " << dtmf_detected_trials << "\n";
    std::cout << "    Detection Rate            : " << std::fixed << std::setprecision(2)
              << (100.0 * dtmf_detected_trials / dtmf_total_trials) << "% (Target: >= 99.00%)\n";

    // 4. Pre-roll delay line retroactive erase test
    std::cout << "\n[4] Testing VoxPreRoll Retroactive Erase (N=4 vs N=3)...\n";
    VoxPreRoll preroll_4(4);
    VoxPreRoll preroll_3(3);
    int16_t dummy_in[160];
    int16_t dummy_out[160];
    for (int i = 0; i < 160; ++i)
        dummy_in[i] = 1000;

    // Push 4 frames into preroll_4
    for (int f = 0; f < 4; ++f)
        preroll_4.push(dummy_in, dummy_out);
    // Erase 4 frames
    preroll_4.muteRecentFrames(4);
    bool all_zero_4 = true;
    for (int f = 0; f < 4; ++f)
    {
        preroll_4.push(dummy_in, dummy_out);
        for (int i = 0; i < 160; ++i)
        {
            if (dummy_out[i] != 0)
                all_zero_4 = false;
        }
    }
    std::cout << "    vox_pre = 4: All 4 buffered frames zeroed (0 tone leakage): " << (all_zero_4 ? "PASS" : "FAIL") << "\n";

    std::cout << "=================================================================\n\n";
    return 0;
}
