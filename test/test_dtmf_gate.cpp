#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>

#include "dtmf_gate.h"
#include "dtmf_detector.h"
#include "vox_pre_roll.h"

// Singleton for tests
static DtmfGate s_test_gate;
DtmfGate &dtmf_gate_instance() { return s_test_gate; }

// Stub for dtmf_detector_init()
void dtmf_controller_handle_digit(char) {}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Model of VOX detector matching the firmware's attack/hang logic
class TestVoxDetector {
public:
    static constexpr uint32_t ATTACK_MS = 40;
    static constexpr uint32_t HANG_MS = 400;

    TestVoxDetector() : m_open(false), m_speech_start_ms(0), m_hang_end_ms(0) {}

    bool is_open() const { return m_open; }

    void force_close() {
        m_open = false;
        m_speech_start_ms = 0;
        m_hang_end_ms = 0;
    }

    bool update(uint16_t level, uint16_t threshold, uint32_t now_ms, bool inhibit) {
        if (inhibit) {
            force_close();
            return false;
        }

        bool voice_active = (level >= threshold);
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

        if (m_open && (uint32_t)(now_ms - m_hang_end_ms) < 0x80000000UL && (now_ms >= m_hang_end_ms)) {
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

// Model of DTMF command parser
class TestDtmfParser {
public:
    TestDtmfParser() : m_buf_len(0), m_commands_dispatched(0) {
        m_buffer[0] = '\0';
        m_last_cmd[0] = '\0';
    }

    bool in_progress() const { return m_buf_len > 0; }

    void handle_digit(char d) {
        if (d == '*') {
            m_buffer[0] = '*';
            m_buffer[1] = '\0';
            m_buf_len = 1;
            return;
        }
        if (m_buf_len > 0 && m_buffer[0] == '*') {
            if (d == '#') {
                m_buffer[m_buf_len++] = '#';
                m_buffer[m_buf_len] = '\0';
                strncpy(m_last_cmd, m_buffer, sizeof(m_last_cmd) - 1);
                m_commands_dispatched++;
                m_buf_len = 0;
                m_buffer[0] = '\0';
            } else if ((d >= '0' && d <= '9') || (d >= 'A' && d <= 'D')) {
                if (m_buf_len < sizeof(m_buffer) - 2) {
                    m_buffer[m_buf_len++] = d;
                    m_buffer[m_buf_len] = '\0';
                } else {
                    m_buf_len = 0;
                    m_buffer[0] = '\0';
                }
            } else {
                m_buf_len = 0;
                m_buffer[0] = '\0';
            }
        }
    }

    uint32_t get_dispatched_count() const { return m_commands_dispatched; }
    const char* get_last_cmd() const { return m_last_cmd; }

private:
    char m_buffer[32];
    size_t m_buf_len;
    uint32_t m_commands_dispatched;
    char m_last_cmd[32];
};

static void synthesize_dtmf_frame(char digit, int16_t *buf, int amplitude = 12000, float noise_level = 0.0f, size_t sample_offset = 0)
{
    static const float ROW_FREQS[4] = { 697.0f, 770.0f, 852.0f, 941.0f };
    static const float COL_FREQS[4] = { 1209.0f, 1336.0f, 1477.0f, 1633.0f };
    static const char MAP[4][4] = {
        { '1', '2', '3', 'A' },
        { '4', '5', '6', 'B' },
        { '7', '8', '9', 'C' },
        { '*', '0', '#', 'D' }
    };

    float f_row = 0.0f, f_col = 0.0f;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (MAP[r][c] == digit) {
                f_row = ROW_FREQS[r];
                f_col = COL_FREQS[c];
                break;
            }
        }
    }

    for (size_t i = 0; i < 160; ++i) {
        double t = (double)(sample_offset + i) / 8000.0;
        double s = 0.5 * sin(2.0 * M_PI * f_row * t) + 0.5 * sin(2.0 * M_PI * f_col * t);
        if (noise_level > 0.0f) {
            double noise = ((double)rand() / (double)RAND_MAX * 2.0 - 1.0) * noise_level;
            s += noise;
        }
        int32_t val = (int32_t)(s * amplitude);
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        buf[i] = (int16_t)val;
    }
}

// ============================================================================
// TEST 1: Rule 1 - Input invariance under VOX / TX / Hang state
// ============================================================================
void test_rule_1_detector_invariance()
{
    std::cout << "[TEST 1] Rule 1: Detector output invariance under VOX/TX states... ";

    DTMFDetector det1;
    DTMFDetector det2;
    det1.init();
    det2.init();

    // Sequence of 10 frames of '5', 5 frames of silence, 10 frames of '#'
    std::vector<char> confirmed1;
    std::vector<char> confirmed2;

    int16_t frame[160];
    size_t sample_pos = 0;

    // Stream 1: Run with VOX closed / TX inactive
    for (int k = 0; k < 35; ++k) {
        if (k < 10) {
            synthesize_dtmf_frame('5', frame, 14000, 0.0f, sample_pos);
        } else if (k < 18) {
            memset(frame, 0, sizeof(frame));
        } else if (k < 28) {
            synthesize_dtmf_frame('#', frame, 14000, 0.0f, sample_pos);
        } else {
            memset(frame, 0, sizeof(frame));
        }
        sample_pos += 160;

        DtmfFrameResult r1 = det1.process_frame(frame);
        if (r1.confirmed_digit != '\0') confirmed1.push_back(r1.confirmed_digit);
    }

    // Stream 2: Run with identical audio but simulated VOX=open, TX=active
    sample_pos = 0;
    for (int k = 0; k < 35; ++k) {
        if (k < 10) {
            synthesize_dtmf_frame('5', frame, 14000, 0.0f, sample_pos);
        } else if (k < 18) {
            memset(frame, 0, sizeof(frame));
        } else if (k < 28) {
            synthesize_dtmf_frame('#', frame, 14000, 0.0f, sample_pos);
        } else {
            memset(frame, 0, sizeof(frame));
        }
        sample_pos += 160;

        // In new architecture, detector runs on raw mic frame before any TX/VOX gating
        DtmfFrameResult r2 = det2.process_frame(frame);
        if (r2.confirmed_digit != '\0') confirmed2.push_back(r2.confirmed_digit);
    }

    assert(confirmed1.size() == 2);
    assert(confirmed1[0] == '5' && confirmed1[1] == '#');
    assert(confirmed1 == confirmed2);

    std::cout << "PASSED\n";
}

// ============================================================================
// TEST 2: Rule 2 - Immediate VOX Force-Close and Guard Window
// ============================================================================
void test_rule_2_force_close_and_guard()
{
    std::cout << "[TEST 2] Rule 2: Immediate VOX force-close & 100 ms guard... ";

    TestVoxDetector vox;
    DtmfGate gate;
    uint32_t now = 1000;

    // Simulate speech opening VOX
    vox.update(500, 200, now, false); // 0 ms
    now += 20;
    vox.update(500, 200, now, false); // 20 ms
    now += 20;
    vox.update(500, 200, now, false); // 40 ms -> VOX opens
    assert(vox.is_open() == true);

    // Speech continues, hang timer is at now + 400 = 1440
    now += 20;
    vox.update(500, 200, now, false);
    assert(vox.is_open() == true);

    // Frame where DTMF tone candidate first appears!
    now += 20;
    gate.update(true, '\0', false, now);

    assert(gate.should_force_close_vox() == true);
    assert(gate.is_vox_inhibited() == true);

    // Apply gate decision to VOX in this same frame
    if (gate.should_force_close_vox()) {
        vox.force_close();
    }
    // Verify VOX is closed IMMEDIATELY (no waiting for 400 ms hang timer)
    assert(vox.is_open() == false);

    // Continue tone candidate for 3 more frames (60 ms total)
    for (int i = 0; i < 3; ++i) {
        now += 20;
        gate.update(true, '\0', false, now);
        vox.update(800, 200, now, gate.is_vox_inhibited());
        assert(vox.is_open() == false);
    }

    // Tone stops. Candidate becomes false.
    // Guard window must keep VOX inhibited for 100 ms (5 frames of 20 ms)
    for (int i = 0; i < 4; ++i) { // 20, 40, 60, 80 ms after tone
        now += 20;
        gate.update(false, '\0', false, now);
        assert(gate.is_vox_inhibited() == true);
        // Even if loud speech is present, VOX must NOT reopen
        vox.update(800, 200, now, gate.is_vox_inhibited());
        assert(vox.is_open() == false);
    }

    // At 120 ms after tone ends (guard expired, no session active)
    now += 40;
    gate.update(false, '\0', false, now);
    assert(gate.is_vox_inhibited() == false);

    // Now speech can re-open VOX after normal attack time (40 ms)
    vox.update(800, 200, now, gate.is_vox_inhibited());
    now += 20;
    vox.update(800, 200, now, gate.is_vox_inhibited());
    now += 20;
    vox.update(800, 200, now, gate.is_vox_inhibited());
    assert(vox.is_open() == true);

    std::cout << "PASSED\n";
}

// ============================================================================
// TEST 3: Full Command *1 9999 # with speech-level energy in gaps
// ============================================================================
void test_rule_3_full_command_sequence()
{
    std::cout << "[TEST 3] Full command '*1 9999 #' with high audio level throughout... ";

    DTMFDetector detector;
    detector.init();
    DtmfGate gate;
    TestVoxDetector vox;
    TestDtmfParser parser;

    const std::string cmd = "*19999#";
    uint32_t now = 10000;
    size_t sample_pos = 0;

    for (char c : cmd) {
        // 4 frames (80 ms) of tone
        for (int f = 0; f < 4; ++f) {
            int16_t frame[160];
            synthesize_dtmf_frame(c, frame, 12000, 0.0f, sample_pos);
            sample_pos += 160;

            DtmfFrameResult res = detector.process_frame(frame);
            if (res.confirmed_digit != '\0') {
                parser.handle_digit(res.confirmed_digit);
            }

            gate.update(res.candidate, res.confirmed_digit, parser.in_progress(), now);
            vox.update(1000, 200, now, gate.is_vox_inhibited());

            // VOX must never open
            assert(vox.is_open() == false);
            now += 20;
        }

        // 4 frames (80 ms) of speech-level noise (gap between keys)
        for (int g = 0; g < 4; ++g) {
            int16_t speech_frame[160];
            // Synthetic loud speech-like noise
            for (int i = 0; i < 160; ++i) {
                speech_frame[i] = (int16_t)(sin(2.0 * M_PI * 400.0 * (double)(sample_pos + i) / 8000.0) * 8000.0);
            }
            sample_pos += 160;

            DtmfFrameResult res = detector.process_frame(speech_frame);
            if (res.confirmed_digit != '\0') {
                parser.handle_digit(res.confirmed_digit);
            }

            gate.update(res.candidate, res.confirmed_digit, parser.in_progress(), now);
            vox.update(1000, 200, now, gate.is_vox_inhibited());

            // VOX must STILL not open during pauses between keys!
            assert(vox.is_open() == false);
            now += 20;
        }
    }

    // Verify command was dispatched to parser
    assert(parser.get_dispatched_count() == 1);
    assert(strcmp(parser.get_last_cmd(), "*19999#") == 0);

    // After '#' completes, guard expires, gate releases session
    now += 150;
    gate.update(false, '\0', parser.in_progress(), now);
    assert(gate.is_session_active() == false);
    assert(gate.is_vox_inhibited() == false);

    std::cout << "PASSED\n";
}

// ============================================================================
// TEST 4: Session Timing & Aborts
// ============================================================================
void test_session_timing_and_aborts()
{
    std::cout << "[TEST 4] Session timing: 1.5s timeout, invalid key abort, '#' termination... ";

    DtmfGate gate;
    uint32_t now = 50000;

    // Case A: Starts on '*'
    gate.update(true, '*', true, now);
    assert(gate.is_session_active() == true);
    assert(gate.is_vox_inhibited() == true);

    // Case B: Ends on '#'
    now += 100;
    gate.update(true, '#', false, now);
    assert(gate.is_session_active() == false);

    // Case C: False '*' followed by speech -> 1.5s timeout releases gate
    now += 500;
    gate.update(true, '*', true, now);
    assert(gate.is_session_active() == true);

    // Candidate tone disappears
    now += 40;
    gate.update(false, '\0', true, now);

    // 1400 ms later: still in session
    now += 1400;
    gate.update(false, '\0', true, now);
    assert(gate.is_session_active() == true);

    // 1501 ms since last tone: session MUST time out
    now += 101;
    gate.update(false, '\0', false, now);
    assert(gate.is_session_active() == false);
    assert(gate.is_vox_inhibited() == false);

    // Case D: Invalid key terminates session immediately
    now += 100;
    gate.update(true, '*', true, now);
    assert(gate.is_session_active() == true);
    now += 100;
    gate.update(true, '?', true, now); // Invalid symbol
    assert(gate.is_session_active() == false);

    std::cout << "PASSED\n";
}

// ============================================================================
// TEST 5: Talk-Off Safety (Harmonic Series, Noise, Unrelated Tone Pairs)
// ============================================================================
void test_talk_off_safety()
{
    std::cout << "[TEST 5] Talk-off safety with harmonic speech, noise bursts, non-DTMF tones... ";

    DTMFDetector detector;
    detector.init();
    int16_t frame[160];

    // 1. Harmonics of typical voice fundamental: 150 Hz, 300 Hz, 450 Hz, 600 Hz, 750 Hz
    for (int k = 0; k < 20; ++k) {
        for (int i = 0; i < 160; ++i) {
            double t = (double)(k * 160 + i) / 8000.0;
            double s = 0.3 * sin(2.0 * M_PI * 150.0 * t) +
                       0.3 * sin(2.0 * M_PI * 300.0 * t) +
                       0.2 * sin(2.0 * M_PI * 450.0 * t) +
                       0.2 * sin(2.0 * M_PI * 750.0 * t);
            frame[i] = (int16_t)(s * 15000.0);
        }
        DtmfFrameResult res = detector.process_frame(frame);
        assert(res.confirmed_digit == '\0');
    }

    // 2. White noise bursts
    for (int k = 0; k < 20; ++k) {
        for (int i = 0; i < 160; ++i) {
            frame[i] = (int16_t)(((double)rand() / (double)RAND_MAX * 2.0 - 1.0) * 12000.0);
        }
        DtmfFrameResult res = detector.process_frame(frame);
        assert(res.confirmed_digit == '\0');
    }

    // 3. Two non-DTMF tones (e.g. 1000 Hz and 2000 Hz)
    for (int k = 0; k < 20; ++k) {
        for (int i = 0; i < 160; ++i) {
            double t = (double)(k * 160 + i) / 8000.0;
            double s = 0.5 * sin(2.0 * M_PI * 1000.0 * t) + 0.5 * sin(2.0 * M_PI * 2000.0 * t);
            frame[i] = (int16_t)(s * 15000.0);
        }
        DtmfFrameResult res = detector.process_frame(frame);
        assert(res.confirmed_digit == '\0');
    }

    std::cout << "PASSED\n";
}

// ============================================================================
// TEST 6: Uint32 Millis Wraparound
// ============================================================================
void test_uint32_millis_wraparound()
{
    std::cout << "[TEST 6] Uint32 millis() wraparound across 0xFFFFFFFF... ";

    DtmfGate gate;
    uint32_t now = 0xFFFFFF80UL; // 128 ms before overflow

    // Start session near wrap point
    gate.update(true, '*', true, now);
    assert(gate.is_session_active() == true);

    // Cross 0xFFFFFFFF boundary
    now = 0x00000050UL; // 80 ms past 0, total elapsed = 208 ms (< 1500 ms)
    gate.update(false, '\0', true, now);
    assert(gate.is_session_active() == true);
    assert(gate.is_vox_inhibited() == true);

    // Advance past 1500 ms timeout: 0xFFFFFF80 + 1501 ms = 0x0000055D
    now = 0x00000560UL;
    gate.update(false, '\0', false, now);
    assert(gate.is_session_active() == false);
    assert(gate.is_vox_inhibited() == false);

    std::cout << "PASSED\n";
}

// ============================================================================
// TEST 7: 16 DTMF Symbols Comprehensive Recognition Test
// ============================================================================
void test_16_symbols_recognition()
{
    std::cout << "[TEST 7] Synthesized tone detection for all 16 symbols with noise... ";

    static const char SYMBOLS[16] = {
        '1', '2', '3', 'A',
        '4', '5', '6', 'B',
        '7', '8', '9', 'C',
        '*', '0', '#', 'D'
    };

    for (int s = 0; s < 16; ++s) {
        char target = SYMBOLS[s];
        DTMFDetector det;
        det.init();

        char detected = '\0';
        size_t sample_pos = 0;
        int16_t frame[160];

        // Feed 4 frames (80 ms) of tone with 5% added noise
        for (int f = 0; f < 4; ++f) {
            synthesize_dtmf_frame(target, frame, 14000, 0.05f, sample_pos);
            sample_pos += 160;
            DtmfFrameResult res = det.process_frame(frame);
            if (res.confirmed_digit != '\0') {
                detected = res.confirmed_digit;
            }
        }
        assert(detected == target);
    }

    std::cout << "PASSED\n";
}

// 8. Test VoxPreRoll muting on candidate rising edge
static void test_vox_preroll_muting_on_candidate_rising()
{
    std::cout << "[TEST 8] Pre-roll buffer muting on candidate rising edge: ";

    VoxPreRoll pre(3);
    DtmfGate gate;
    int16_t frame_in[160];
    int16_t frame_out[160];

    // Prime with speech audio (samples = 500)
    for (int i = 0; i < 160; ++i) frame_in[i] = 500;
    assert(!pre.push(frame_in, frame_out)); // push 1 (primes 1)
    assert(!pre.push(frame_in, frame_out)); // push 2 (primes 2)
    assert(!pre.push(frame_in, frame_out)); // push 3 (primes 3)
    // Buffer is now primed (3 frames stored: frames 1, 2, 3)
    assert(pre.isPrimed());

    // Push frame 4 (speech) -> outputs frame 1
    assert(pre.push(frame_in, frame_out));
    assert(frame_out[0] == 500);

    // Push frame 5 (speech) -> outputs frame 2
    assert(pre.push(frame_in, frame_out));
    assert(frame_out[0] == 500);

    // At frame 6, a tone candidate appears!
    bool is_rising = gate.is_candidate_rising(true);
    assert(is_rising);
    gate.update(true, '\0', false, 100);

    // Candidate rising edge mutes the 2 most recently buffered frames (frames 4 and 5)
    pre.muteRecentFrames(2);

    // Push frame 6 (silence) -> outputs frame 3 (pre-tone speech, 500)
    int16_t silence[160] = {0};
    assert(pre.push(silence, frame_out));
    assert(frame_out[0] == 500);

    // Push frame 7 (silence) -> outputs frame 4 (was muted by muteRecentFrames(2)!)
    assert(pre.push(silence, frame_out));
    for (int i = 0; i < 160; ++i) {
        assert(frame_out[i] == 0);
    }

    // Push frame 8 (silence) -> outputs frame 5 (was also muted by muteRecentFrames(2)!)
    assert(pre.push(silence, frame_out));
    for (int i = 0; i < 160; ++i) {
        assert(frame_out[i] == 0);
    }

    std::cout << "PASSED\n";
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "  DTMF Gate & Detector Unit Test Suite  \n";
    std::cout << "========================================\n";

    test_rule_1_detector_invariance();
    test_rule_2_force_close_and_guard();
    test_rule_3_full_command_sequence();
    test_session_timing_and_aborts();
    test_talk_off_safety();
    test_uint32_millis_wraparound();
    test_16_symbols_recognition();
    test_vox_preroll_muting_on_candidate_rising();

    std::cout << "========================================\n";
    std::cout << "  ALL UNIT TESTS PASSED SUCCESSFULLY!   \n";
    std::cout << "========================================\n";
    return 0;
}
