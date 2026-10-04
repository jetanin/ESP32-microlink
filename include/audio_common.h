#pragma once

#include <Arduino.h>

/**
 * @file audio_common.h
 * @brief Common Audio Definitions for MicroLink (8 kHz GSM-FR / EchoLink standard)
 */

// EchoLink & GSM 06.10 Full-Rate audio specifications:
// - Sample rate: 8000 Hz
// - Frame duration: 20 ms
// - Samples per 20 ms frame: 160 samples
// - Resolution: 16-bit signed PCM (int16_t)
// - Frame size: 320 bytes per frame
constexpr uint32_t AUDIO_SAMPLE_RATE_HZ   = 8000;
constexpr size_t   AUDIO_FRAME_SAMPLES    = 160;
constexpr size_t   AUDIO_FRAME_BYTES      = AUDIO_FRAME_SAMPLES * sizeof(int16_t);

// 2-second loopback buffer size
constexpr size_t   LOOPBACK_RECORD_SECS   = 2;
constexpr size_t   LOOPBACK_TOTAL_SAMPLES = AUDIO_SAMPLE_RATE_HZ * LOOPBACK_RECORD_SECS; // 16,000 samples
constexpr size_t   LOOPBACK_TOTAL_BYTES   = LOOPBACK_TOTAL_SAMPLES * sizeof(int16_t);   // 32,000 bytes

// VOX Sensitivity mapping: step 1-20 -> Moving Energy / RMS threshold (12-bit AC scale)
// Ambient noise floor: ~10-20 RMS. Speaking voice: ~70-200 RMS. Shouting: ~300+ RMS.
// Higher step = more sensitive (triggers on quieter signals)
constexpr uint16_t VOX_THRESHOLDS[21] = {
    0,   // unused (index 0)
    420, // step 1 - least sensitive (loud shout)
    370, // step 2
    325, // step 3
    285, // step 4
    250, // step 5
    220, // step 6
    195, // step 7
    170, // step 8
    150, // step 9
    130, // step 10 - medium (default, normal speech)
    112, // step 11
    96,  // step 12
    82,  // step 13
    70,  // step 14
    58,  // step 15
    48,  // step 16
    39,  // step 17
    31,  // step 18
    24,  // step 19
    18,  // step 20 - most sensitive (quiet whisper)
};

constexpr uint32_t VOX_HANG_MS   = 600; // Hold TX open after speech stops (600 ms)
constexpr uint32_t VOX_ATTACK_MS = 40;  // Speech must persist >= 40 ms (2 frames) to open VOX

