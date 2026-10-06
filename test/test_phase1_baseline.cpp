#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <random>

#include "dtmf_detector.h"
#include "dtmf_gate.h"
#include "vox_pre_roll.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Stub for dtmf_controller_handle_digit() required when linking dtmf_detector.cpp
void dtmf_controller_handle_digit(char) {}

// Test VOX detector model matching the device's attack and hang logic
class TestVoxDetector {
public:
    static constexpr uint32_t ATTACK_MS = 40;  // 2 frames
    static constexpr uint32_t HANG_MS   = 600; // 600 ms

    TestVoxDetector() : m_open(false), m_speech_start_ms(0), m_hang_end_ms(0) {}

    bool is_open() const { return m_open; }

    void force_close() {
        m_open = false;
        m_speech_start_ms = 0;
        m_hang_end_ms = 0;
    }

    bool update(uint16_t rms, uint16_t threshold, uint32_t now_ms, bool inhibit) {
        if (inhibit) {
            force_close();
            return false;
        }

        bool voice_active = (rms >= threshold);
        if (voice_active) {
            if (m_speech_start_ms == 0) {
                m_speech_start_ms = now_ms;
            }
            if (!m_open && (uint32_t)(now_ms - m_speech_start_ms) >= ATTACK_MS) {
                m_open = true;
            }
            m_hang_end_ms = now_ms + HANG_MS;
        } else {
            m_speech_start_ms = 0;
        }

        if (m_open && (now_ms >= m_hang_end_ms)) {
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

// Formant Biquad Resonator
class FormantFilter {
public:
    void set(float fc, float bw, float fs = 8000.0f) {
        float r = std::exp(- (float)M_PI * bw / fs);
        float theta = 2.0f * (float)M_PI * fc / fs;
        m_a1 = -2.0f * r * std::cos(theta);
        m_a2 = r * r;
        m_b0 = (1.0f - r); // Normalized approximate resonance gain
        m_y1 = 0.0f;
        m_y2 = 0.0f;
    }

    void reset() {
        m_y1 = 0.0f;
        m_y2 = 0.0f;
    }

    float process(float x) {
        float y = m_b0 * x - m_a1 * m_y1 - m_a2 * m_y2;
        m_y2 = m_y1;
        m_y1 = y;
        return y;
    }

private:
    float m_b0 = 0.0f;
    float m_a1 = 0.0f;
    float m_a2 = 0.0f;
    float m_y1 = 0.0f;
    float m_y2 = 0.0f;
};

// Vowel Sound Synthesizer (Glottal Pulse Train -> 3 Resonators)
class VowelSynth {
public:
    void generate(float f0, float f1, float f2, float f3,
                  float bw1, float bw2, float bw3,
                  float duration_sec, float amplitude,
                  std::vector<int16_t> &out_samples,
                  std::mt19937 &rng)
    {
        FormantFilter r1, r2, r3;
        r1.set(f1, bw1);
        r2.set(f2, bw2);
        r3.set(f3, bw3);

        size_t total_samples = (size_t)(duration_sec * 8000.0f);
        out_samples.resize(total_samples);
        raw_signal.clear();
        raw_signal.reserve(total_samples);

        float phase = 0.0f;
        std::normal_distribution<float> jitter_dist(0.0f, 0.015f); // 1.5% pitch jitter
        std::normal_distribution<float> noise_dist(0.0f, 0.03f);   // slight breathiness

        for (size_t i = 0; i < total_samples; ++i) {
            float cur_f0 = f0 * (1.0f + jitter_dist(rng));
            float period_samples = 8000.0f / cur_f0;

            // Glottal pulse: Rosenberg model (open phase 60% of period)
            float t = phase / period_samples;
            float glottal = 0.0f;
            if (t < 0.6f) {
                glottal = 0.5f * (1.0f - std::cos((float)M_PI * t / 0.6f));
            } else {
                glottal = 0.0f;
            }

            // Advance phase
            phase += 1.0f;
            if (phase >= period_samples) {
                phase -= period_samples;
            }
            // Excitation + slight breath noise
            float exc = glottal + noise_dist(rng);

            // Parallel formant synthesis (Klatt model): F1 + 0.6*F2 + 0.3*F3
            float s = 1.0f * r1.process(exc) + 0.6f * r2.process(exc) + 0.3f * r3.process(exc);
            raw_signal.push_back(s);
        }

        float max_abs = 0.0001f;
        for (float v : raw_signal) {
            float a = std::abs(v);
            if (a > max_abs) max_abs = a;
        }

        float scale = amplitude / max_abs;
        for (size_t i = 0; i < total_samples; ++i) {
            float val = raw_signal[i] * scale;
            if (val > 32767.0f) val = 32767.0f;
            if (val < -32768.0f) val = -32768.0f;
            out_samples[i] = (int16_t)val;
        }
    }

private:
    std::vector<float> raw_signal;
};

// DTMF Tone Pair Generator
void generate_dtmf(char symbol, float duration_ms, float amplitude, float twist_db,
                   float snr_db, std::vector<int16_t> &out_samples, std::mt19937 &rng)
{
    static const struct { char c; float f_low; float f_high; } MAP[] = {
        {'1', 697.0f, 1209.0f}, {'2', 697.0f, 1336.0f}, {'3', 697.0f, 1477.0f}, {'A', 697.0f, 1633.0f},
        {'4', 770.0f, 1209.0f}, {'5', 770.0f, 1336.0f}, {'6', 770.0f, 1477.0f}, {'B', 770.0f, 1633.0f},
        {'7', 852.0f, 1209.0f}, {'8', 852.0f, 1336.0f}, {'9', 852.0f, 1477.0f}, {'C', 852.0f, 1633.0f},
        {'*', 941.0f, 1209.0f}, {'0', 941.0f, 1336.0f}, {'#', 941.0f, 1477.0f}, {'D', 941.0f, 1633.0f}
    };

    float flow = 0.0f, fhigh = 0.0f;
    for (const auto &item : MAP) {
        if (item.c == symbol) {
            flow = item.f_low;
            fhigh = item.f_high;
            break;
        }
    }

    size_t num_samples = (size_t)(duration_ms * 8.0f);
    out_samples.resize(num_samples);

    // twist_db = 20 * log10(A_high / A_low)
    // standard twist is standard +/- 4 to 8 dB
    float twist_ratio = std::pow(10.0f, twist_db / 20.0f);
    float a_low = amplitude / std::sqrt(1.0f + twist_ratio * twist_ratio);
    float a_high = a_low * twist_ratio;

    float signal_power = (a_low * a_low + a_high * a_high) / 2.0f;
    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise_dist(0.0f, noise_std);

    for (size_t i = 0; i < num_samples; ++i) {
        float t = (float)i / 8000.0f;
        float s = a_low * std::sin(2.0f * (float)M_PI * flow * t) +
                  a_high * std::sin(2.0f * (float)M_PI * fhigh * t);
        s += noise_dist(rng);

        if (s > 32767.0f) s = 32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        out_samples[i] = (int16_t)s;
    }
}

// Compute RMS of 160-sample frame
uint16_t compute_frame_rms(const int16_t *frame)
{
    uint64_t sum_sq = 0;
    for (int i = 0; i < 160; ++i) {
        sum_sq += (int32_t)frame[i] * (int32_t)frame[i];
    }
    return (uint16_t)std::sqrt(sum_sq / 160);
}

// Run test on audio stream with current detector & gate
struct StreamResult {
    uint32_t total_frames;
    uint32_t candidate_frames;
    uint32_t confirmed_digits;
    uint32_t vox_force_closes;
    uint32_t vox_key_events;
    uint32_t vox_inhibited_frames;
};

StreamResult run_stream_evaluation(const std::vector<int16_t> &audio, uint16_t vox_threshold = 130)
{
    DTMFDetector det;
    det.init();
    DtmfGate gate;
    TestVoxDetector vox;

    StreamResult res = {0, 0, 0, 0, 0, 0};
    res.total_frames = audio.size() / 160;

    uint32_t now_ms = 1000;
    bool prev_vox_open = false;

    for (size_t f = 0; f < res.total_frames; ++f) {
        const int16_t *frame = audio.data() + f * 160;
        uint16_t frame_rms = compute_frame_rms(frame);

        // 1. Process DTMF frame
        DtmfFrameResult dres = det.process_frame(frame);
        if (dres.candidate) {
            res.candidate_frames++;
        }
        if (dres.confirmed_digit != '\0') {
            res.confirmed_digits++;
        }

        // 2. Update gate
        gate.update(dres.candidate, dres.confirmed_digit, false, now_ms);

        // 3. Force-close VOX
        if (gate.should_force_close_vox()) {
            res.vox_force_closes++;
            vox.force_close();
        }

        if (gate.is_vox_inhibited()) {
            res.vox_inhibited_frames++;
        }

        // 4. Update VOX detector
        bool vox_open = vox.update(frame_rms, vox_threshold, now_ms, gate.is_vox_inhibited());
        if (vox_open && !prev_vox_open) {
            res.vox_key_events++;
        }
        prev_vox_open = vox_open;

        now_ms += 20;
    }

    return res;
}

int main()
{
    std::cout << "=================================================================\n";
    std::cout << "   ESP32-MicroLink Phase 1: DTMF / VOX Talk-Off Baseline Report  \n";
    std::cout << "=================================================================\n\n";

    std::mt19937 rng(42);

    // --------------------------------------------------------------------------
    // Test Set 1: Vowel Grid (Focusing on open vowels /aa/, /a/ as in 'สาม' / 'ห้า')
    // --------------------------------------------------------------------------
    std::cout << "[1] Generating Vowel Grid (F1: 650-950 Hz, F2: 1000-1500 Hz)...\n";
    VowelSynth synth;
    std::vector<int16_t> vowel_dataset;

    float F1_GRID[] = { 650.0f, 750.0f, 850.0f, 950.0f };
    float F2_GRID[] = { 1000.0f, 1150.0f, 1300.0f, 1450.0f };
    float F3_GRID[] = { 2400.0f, 2700.0f };
    float F0_GRID[] = { 110.0f, 140.0f, 180.0f, 230.0f }; // male, female, child pitches
    float AMP_GRID[] = { 4000.0f, 8000.0f, 14000.0f };

    size_t vowel_count = 0;
    for (float f1 : F1_GRID) {
        for (float f2 : F2_GRID) {
            for (float f3 : F3_GRID) {
                for (float f0 : F0_GRID) {
                    for (float amp : AMP_GRID) {
                        std::vector<int16_t> token;
                        // duration 300 ms (15 frames)
                        synth.generate(f0, f1, f2, f3, 80.0f, 110.0f, 140.0f, 0.30f, amp, token, rng);
                        vowel_dataset.insert(vowel_dataset.end(), token.begin(), token.end());
                        // 100 ms silence between tokens
                        vowel_dataset.insert(vowel_dataset.end(), 800, 0);
                        vowel_count++;
                    }
                }
            }
        }
    }

    std::cout << "    Generated " << vowel_count << " vowel tokens ("
              << (vowel_dataset.size() / 8000.0f) << " seconds, "
              << (vowel_dataset.size() / 160) << " frames)\n";

    StreamResult vowel_res = run_stream_evaluation(vowel_dataset);
    std::cout << "    --- Baseline Results on Vowel Grid ---\n";
    std::cout << "    Total frames analyzed     : " << vowel_res.total_frames << "\n";
    std::cout << "    Candidate frames (Weak)   : " << vowel_res.candidate_frames
              << " (" << std::fixed << std::setprecision(2)
              << (100.0 * vowel_res.candidate_frames / vowel_res.total_frames) << "% of frames!)\n";
    std::cout << "    VOX force-close triggers  : " << vowel_res.vox_force_closes << "\n";
    std::cout << "    Frames VOX was inhibited  : " << vowel_res.vox_inhibited_frames
              << " (" << std::fixed << std::setprecision(2)
              << (100.0 * vowel_res.vox_inhibited_frames / vowel_res.total_frames) << "%)\n";
    std::cout << "    VOX key-up events allowed : " << vowel_res.vox_key_events << " / " << vowel_count << "\n";
    std::cout << "    Confirmed DTMF false hits : " << vowel_res.confirmed_digits << "\n\n";

    // --------------------------------------------------------------------------
    // Test Set 2: 10-Minute Continuous Speech-Like Dataset (30,000 frames)
    // --------------------------------------------------------------------------
    std::cout << "[2] Generating 10-Minute Speech-like stream (Vowels, Nasals, Fricatives)...\n";
    std::vector<int16_t> speech_dataset;
    speech_dataset.reserve(600 * 8000);

    std::uniform_real_distribution<float> f0_dist(95.0f, 250.0f);
    std::uniform_real_distribution<float> f1_dist(300.0f, 950.0f);
    std::uniform_real_distribution<float> f2_dist(850.0f, 2200.0f);
    std::uniform_real_distribution<float> f3_dist(2200.0f, 3200.0f);
    std::uniform_real_distribution<float> dur_dist(0.12f, 0.40f);
    std::uniform_real_distribution<float> amp_dist(2500.0f, 15000.0f);
    std::uniform_int_distribution<int> type_dist(0, 4);

    while (speech_dataset.size() < 600 * 8000) {
        int seg_type = type_dist(rng);
        float dur = dur_dist(rng);
        size_t n = (size_t)(dur * 8000.0f);

        if (seg_type <= 2) {
            // Vowel
            std::vector<int16_t> token;
            synth.generate(f0_dist(rng), f1_dist(rng), f2_dist(rng), f3_dist(rng),
                           80.0f, 120.0f, 150.0f, dur, amp_dist(rng), token, rng);
            speech_dataset.insert(speech_dataset.end(), token.begin(), token.end());
        } else if (seg_type == 3) {
            // Fricative (colored noise)
            std::normal_distribution<float> noise(0.0f, amp_dist(rng) * 0.4f);
            float prev = 0.0f;
            for (size_t i = 0; i < n; ++i) {
                float v = 0.8f * noise(rng) - 0.4f * prev; // high-pass shaped noise
                prev = v;
                if (v > 32767.0f) v = 32767.0f;
                if (v < -32768.0f) v = -32768.0f;
                speech_dataset.push_back((int16_t)v);
            }
        } else {
            // Short pause / closure
            size_t pause_n = (size_t)(0.08f * 8000.0f);
            speech_dataset.insert(speech_dataset.end(), pause_n, 0);
        }
    }

    std::cout << "    Generated 10 minutes (" << (speech_dataset.size() / 160) << " frames)\n";
    StreamResult speech_res = run_stream_evaluation(speech_dataset);
    std::cout << "    --- Baseline Results on 10-Minute Speech Stream ---\n";
    std::cout << "    Total frames analyzed     : " << speech_res.total_frames << "\n";
    std::cout << "    Candidate frames (Weak)   : " << speech_res.candidate_frames
              << " (" << std::fixed << std::setprecision(2)
              << (100.0 * speech_res.candidate_frames / speech_res.total_frames) << "%)\n";
    std::cout << "    VOX force-close triggers  : " << speech_res.vox_force_closes << "\n";
    std::cout << "    Frames VOX was inhibited  : " << speech_res.vox_inhibited_frames
              << " (" << std::fixed << std::setprecision(2)
              << (100.0 * speech_res.vox_inhibited_frames / speech_res.total_frames) << "%)\n";
    std::cout << "    Confirmed DTMF false hits : " << speech_res.confirmed_digits << "\n\n";

    // --------------------------------------------------------------------------
    // Test Set 3: Real DTMF Benchmark (All 16 symbols across amplitudes & twist)
    // --------------------------------------------------------------------------
    std::cout << "[3] Evaluating DTMF Recognition Benchmark (All 16 symbols)...\n";
    static const char SYMBOLS[16] = {
        '1', '2', '3', 'A',
        '4', '5', '6', 'B',
        '7', '8', '9', 'C',
        '*', '0', '#', 'D'
    };

    float AMPLITUDES[] = { 500.0f, 1000.0f, 3000.0f, 8000.0f, 14000.0f };
    float TWISTS_DB[]  = { -6.0f, -3.0f, 0.0f, +3.0f, +6.0f };
    float DURATIONS_MS[] = { 40.0f, 60.0f, 100.0f };

    uint32_t dtmf_total_trials = 0;
    uint32_t dtmf_detected_trials = 0;

    for (char sym : SYMBOLS) {
        for (float amp : AMPLITUDES) {
            for (float tw : TWISTS_DB) {
                for (float dur : DURATIONS_MS) {
                    std::vector<int16_t> tone;
                    // DTMF tone + 20 dB SNR noise
                    generate_dtmf(sym, dur, amp, tw, 20.0f, tone, rng);

                    // Add leading and trailing silence (60 ms each)
                    std::vector<int16_t> trial;
                    trial.insert(trial.end(), 480, 0); // 60 ms pause
                    trial.insert(trial.end(), tone.begin(), tone.end());
                    trial.insert(trial.end(), 640, 0); // 80 ms pause

                    DTMFDetector test_det;
                    test_det.init();
                    bool found = false;

                    size_t frames = trial.size() / 160;
                    for (size_t f = 0; f < frames; ++f) {
                        DtmfFrameResult fr = test_det.process_frame(trial.data() + f * 160);
                        if (fr.confirmed_digit == sym) {
                            found = true;
                        }
                    }

                    dtmf_total_trials++;
                    if (found) {
                        dtmf_detected_trials++;
                    }
                }
            }
        }
    }

    std::cout << "    --- Baseline Results on DTMF Test Set ---\n";
    std::cout << "    Total DTMF trials         : " << dtmf_total_trials << "\n";
    std::cout << "    Successfully decoded      : " << dtmf_detected_trials << "\n";
    std::cout << "    Overall Detection Rate    : " << std::fixed << std::setprecision(2)
              << (100.0 * dtmf_detected_trials / dtmf_total_trials) << "%\n";
    std::cout << "=================================================================\n\n";

    return 0;
}
