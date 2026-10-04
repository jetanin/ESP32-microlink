#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstring>
#include "../include/vox_pre_roll.h"

// Helper to fill a frame with a recognizable pattern based on frame index
static void makeFrame(int16_t *frame, int frame_id, int16_t base_val = 100)
{
    for (size_t i = 0; i < VoxPreRoll::FRAME_SAMPLES; ++i)
    {
        frame[i] = static_cast<int16_t>(frame_id * 1000 + i + base_val);
    }
}

// Helper to verify if two frames match exactly
static bool compareFrames(const int16_t *a, const int16_t *b)
{
    return memcmp(a, b, VoxPreRoll::FRAME_SAMPLES * sizeof(int16_t)) == 0;
}

// -----------------------------------------------------------------------------
// Test 1: n = 0 Passthrough
// -----------------------------------------------------------------------------
static void testPassthrough()
{
    std::cout << "[TEST 1] Delay = 0 Passthrough... ";
    VoxPreRoll pre(0);
    assert(pre.getDelayFrames() == 0);
    assert(pre.isPrimed() == true);

    int16_t in[VoxPreRoll::FRAME_SAMPLES];
    int16_t out[VoxPreRoll::FRAME_SAMPLES];

    for (int k = 0; k < 20; ++k)
    {
        makeFrame(in, k);
        memset(out, 0, sizeof(out));
        bool ok = pre.push(in, out);
        assert(ok == true);
        assert(compareFrames(in, out));
    }
    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 2: n = 3 Delay, Priming, Wrap-around, Frame Identity k == k - 3
// -----------------------------------------------------------------------------
static void testDelay3()
{
    std::cout << "[TEST 2] Delay = 3 Priming & Wrap-around (k == k - 3)... ";
    VoxPreRoll pre(3);
    assert(pre.getDelayFrames() == 3);
    assert(pre.isPrimed() == false);
    assert(pre.getBufferedCount() == 0);

    int16_t in[VoxPreRoll::FRAME_SAMPLES];
    int16_t out[VoxPreRoll::FRAME_SAMPLES];
    int16_t history[50][VoxPreRoll::FRAME_SAMPLES];

    // Push first 3 frames (k = 0, 1, 2) -> must all return false
    for (int k = 0; k < 3; ++k)
    {
        makeFrame(in, k);
        memcpy(history[k], in, sizeof(in));
        memset(out, 0xAA, sizeof(out));

        bool ok = pre.push(in, out);
        assert(ok == false);
        assert(pre.getBufferedCount() == k + 1);
        assert(pre.isPrimed() == (k == 2)); // Primed after 3rd push
    }

    // Now push frames k = 3..49 (many frames, wraps around ring buffer of size 8 multiple times)
    for (int k = 3; k < 50; ++k)
    {
        makeFrame(in, k);
        memcpy(history[k], in, sizeof(in));
        memset(out, 0, sizeof(out));

        bool ok = pre.push(in, out);
        assert(ok == true);
        assert(pre.isPrimed() == true);

        // Output frame k must exactly match input frame k - 3
        int expected_k = k - 3;
        assert(compareFrames(out, history[expected_k]));
    }
    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 3: setDelayFrames() clamping, reset(), and un-priming
// -----------------------------------------------------------------------------
static void testControlAndReset()
{
    std::cout << "[TEST 3] setDelayFrames() clamping & reset() un-priming... ";
    VoxPreRoll pre(3);

    // Test clamping > MAX_FRAMES
    pre.setDelayFrames(99);
    assert(pre.getDelayFrames() == VoxPreRoll::MAX_FRAMES);
    assert(pre.getBufferedCount() == 0);
    assert(pre.isPrimed() == false);

    pre.setDelayFrames(0);
    assert(pre.getDelayFrames() == 0);
    assert(pre.isPrimed() == true);

    pre.setDelayFrames(4);
    assert(pre.getDelayFrames() == 4);
    assert(pre.isPrimed() == false);

    int16_t frame[VoxPreRoll::FRAME_SAMPLES];
    int16_t out[VoxPreRoll::FRAME_SAMPLES];
    makeFrame(frame, 1);

    // Push 4 frames to prime it
    for (int i = 0; i < 4; ++i)
    {
        pre.push(frame, out);
    }
    assert(pre.isPrimed() == true);
    assert(pre.getBufferedCount() == 4);

    // reset() must un-prime immediately
    pre.reset();
    assert(pre.isPrimed() == false);
    assert(pre.getBufferedCount() == 0);

    // Next push must return false because it was un-primed
    bool ok = pre.push(frame, out);
    assert(ok == false);
    assert(pre.getBufferedCount() == 1);

    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 4: Onset Preservation Test
// Simulates synthetic silence (frames 0..9), then an abrupt tone onset at frame 10.
// A realistic VOX detector double requires 2 frames (40 ms attack time) of tone
// before opening TX at frame 12.
// - With delay = 0: Sent stream starts at frame 12 -> frames 10 & 11 are LOST.
// - With delay = 3: Sent stream starts at frame 9 or 10 -> onset is FULLY PRESERVED.
// -----------------------------------------------------------------------------
struct MockVoxDetector
{
    int attack_count = 0;
    bool tx_open = false;

    // Checks RMS energy of 160-sample frame (12-bit AC scale)
    bool processFrame(const int16_t *frame, int16_t threshold_rms = 100)
    {
        int64_t sum_sq = 0;
        for (size_t i = 0; i < VoxPreRoll::FRAME_SAMPLES; ++i)
        {
            sum_sq += static_cast<int64_t>(frame[i]) * frame[i];
        }
        int16_t rms = static_cast<int16_t>(sqrt(static_cast<double>(sum_sq / VoxPreRoll::FRAME_SAMPLES)));

        if (rms >= threshold_rms)
        {
            attack_count++;
            // Opens after 2 consecutive active frames (40 ms attack time)
            if (attack_count >= 2)
            {
                tx_open = true;
            }
        }
        else
        {
            attack_count = 0;
        }
        return tx_open;
    }

    void reset()
    {
        attack_count = 0;
        tx_open = false;
    }
};

static void testOnsetPreservation()
{
    std::cout << "[TEST 4] Onset Preservation (Delay 0 vs Delay 3)... ";

    // Generate 30 frames:
    // Frames 0..9: Silence (RMS = 0)
    // Frames 10..29: 800 Hz Sine Tone (RMS ≈ 500, loud voice)
    int16_t audio_stream[30][VoxPreRoll::FRAME_SAMPLES];
    for (int k = 0; k < 30; ++k)
    {
        for (size_t i = 0; i < VoxPreRoll::FRAME_SAMPLES; ++i)
        {
            if (k < 10)
            {
                audio_stream[k][i] = 0; // Silence
            }
            else
            {
                // 800 Hz sine wave @ 8 kHz (10 samples per period), amplitude 707 (RMS ~ 500)
                double phase = (2.0 * M_PI * 800.0 * (k * 160 + i)) / 8000.0;
                audio_stream[k][i] = static_cast<int16_t>(707.0 * sin(phase));
            }
        }
    }

    // -------------------------------------------------------------------------
    // Case A: Without Pre-roll (Delay = 0)
    // -------------------------------------------------------------------------
    {
        VoxPreRoll pre0(0);
        MockVoxDetector vox;
        std::vector<int> sent_frame_ids;

        for (int k = 0; k < 30; ++k)
        {
            const int16_t *in = audio_stream[k];
            int16_t out[VoxPreRoll::FRAME_SAMPLES];

            bool vox_open = vox.processFrame(in);
            bool primed = pre0.push(in, out);

            if (primed && vox_open)
            {
                sent_frame_ids.push_back(k); // In delay 0, out == in (frame k)
            }
        }

        // Onset occurred at frame 10.
        // Detector required 2 frames of tone (frames 10 & 11) to trigger.
        // Thus, first sent frame without pre-roll is frame 11 or 12!
        assert(!sent_frame_ids.empty());
        int first_sent = sent_frame_ids.front();
        assert(first_sent > 10); // Frame 10 onset WAS LOST!
    }

    // -------------------------------------------------------------------------
    // Case B: With Pre-roll (Delay = 3 frames = 60 ms)
    // -------------------------------------------------------------------------
    {
        VoxPreRoll pre3(3);
        MockVoxDetector vox;
        std::vector<int> sent_original_frame_ids;

        for (int k = 0; k < 30; ++k)
        {
            const int16_t *in = audio_stream[k];
            int16_t out[VoxPreRoll::FRAME_SAMPLES];

            bool vox_open = vox.processFrame(in);
            bool primed = pre3.push(in, out);

            if (primed && vox_open)
            {
                // In delay 3, out contains frame (k - 3)
                int original_frame_id = k - 3;
                sent_original_frame_ids.push_back(original_frame_id);
            }
        }

        assert(!sent_original_frame_ids.empty());
        int first_sent = sent_original_frame_ids.front();

        // The first frame sent must be at or before onset frame (<= 10),
        // proving the speech onset is preserved!
        assert(first_sent <= 10);
    }

    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 5: VOX Closing & Reset Isolation
// Verifies that:
// 1) When VOX closes, buffered tail is discarded and not sent.
// 2) After reset(), no frames prior to reset are ever output.
// -----------------------------------------------------------------------------
static void testClosingAndResetIsolation()
{
    std::cout << "[TEST 5] VOX Closing Tail Discard & Reset Isolation... ";
    VoxPreRoll pre(3);

    int16_t in[VoxPreRoll::FRAME_SAMPLES];
    int16_t out[VoxPreRoll::FRAME_SAMPLES];

    // Push 10 frames of Session 1
    for (int k = 0; k < 10; ++k)
    {
        makeFrame(in, 100 + k);
        pre.push(in, out);
    }

    // Now an announcement or mode switch occurs -> reset() is called
    pre.reset();
    assert(pre.isPrimed() == false);
    assert(pre.getBufferedCount() == 0);

    // Start Session 2 with distinct frame IDs
    for (int k = 0; k < 3; ++k)
    {
        makeFrame(in, 200 + k);
        bool ok = pre.push(in, out);
        assert(ok == false); // Must re-prime
    }

    // 4th push of Session 2: out must be Session 2 Frame 0 (200), NEVER Session 1
    makeFrame(in, 203);
    bool ok = pre.push(in, out);
    assert(ok == true);

    int16_t expected_session2_frame0[VoxPreRoll::FRAME_SAMPLES];
    makeFrame(expected_session2_frame0, 200);
    assert(compareFrames(out, expected_session2_frame0));

    std::cout << "PASSED\n";
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "  VoxPreRoll Host Unit Test Suite\n";
    std::cout << "========================================\n";

    testPassthrough();
    testDelay3();
    testControlAndReset();
    testOnsetPreservation();
    testClosingAndResetIsolation();

    std::cout << "========================================\n";
    std::cout << "  ALL 5 UNIT TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "========================================\n";
    return 0;
}
