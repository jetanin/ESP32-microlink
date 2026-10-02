#pragma once

#include <cstdint>
#include <cstddef>
#include "audio_common.h"

// GSM 06.10 constants
#define GSM_FRAME_SAMPLES       160       // 20 ms @ 8 kHz
#define GSM_FRAME_BYTES         33        // 33 bytes per GSM-FR frame (RFC 3551)
#define GSM_BUNDLE_FRAMES       4         // EchoLink groups 4 frames per RTP packet
#define GSM_BUNDLE_SAMPLES      (GSM_FRAME_SAMPLES * GSM_BUNDLE_FRAMES) // 640 samples (80 ms)
#define GSM_BUNDLE_BYTES        (GSM_FRAME_BYTES * GSM_BUNDLE_FRAMES)   // 132 bytes

bool gsm_codec_init();
void gsm_codec_reset();
void gsm_encode_frame(const int16_t* pcm160, uint8_t* gsm33);
void gsm_decode_frame(const uint8_t* gsm33, int16_t* pcm160);
void gsm_encode_4frames(const int16_t* pcm640, uint8_t* gsm132);
void gsm_decode_4frames(const uint8_t* gsm132, int16_t* pcm640);
