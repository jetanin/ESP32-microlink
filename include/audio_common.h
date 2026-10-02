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
