/**
 * @file gsm_codec.cpp
 * @brief High-level wrapper for ETSI GSM 06.10 Full-Rate audio codec.
 *
 * MicroLink ESP32-C6 EchoLink Node
 * Adapted from Bruce MacKinnon (KC1FSZ) gsm-0610-codec
 * GPL-3.0 License
 */

#include "gsm_codec.h"
#include "gsm-0610-codec/Encoder.h"
#include "gsm-0610-codec/Decoder.h"
#include "gsm-0610-codec/Parameters.h"
#include <Arduino.h>

static kc1fsz::Encoder* s_encoder = nullptr;
static kc1fsz::Decoder* s_decoder = nullptr;
static SemaphoreHandle_t s_codec_mutex = nullptr;

bool gsm_codec_init()
{
    if (s_codec_mutex == nullptr)
    {
        s_codec_mutex = xSemaphoreCreateMutex();
    }

    if (s_encoder == nullptr)
    {
        s_encoder = new kc1fsz::Encoder(true);
    }
    if (s_decoder == nullptr)
    {
        s_decoder = new kc1fsz::Decoder();
    }

    gsm_codec_reset();
    return (s_encoder != nullptr && s_decoder != nullptr && s_codec_mutex != nullptr);
}

void gsm_codec_reset()
{
    if (s_codec_mutex && xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        if (s_encoder) s_encoder->reset();
        if (s_decoder) s_decoder->reset();
        xSemaphoreGive(s_codec_mutex);
    }
}

void gsm_encode_frame(const int16_t* pcm160, uint8_t* gsm33)
{
    if (!s_encoder || !s_codec_mutex) return;

    if (xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        kc1fsz::Parameters params;
        s_encoder->encode(pcm160, &params);
        params.pack(gsm33);
        xSemaphoreGive(s_codec_mutex);
    }
}

void gsm_decode_frame(const uint8_t* gsm33, int16_t* pcm160)
{
    if (!s_decoder || !s_codec_mutex) return;

    if (xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        kc1fsz::Parameters params;
        params.unpack(gsm33);
        s_decoder->decode(&params, pcm160);
        xSemaphoreGive(s_codec_mutex);
    }
}

void gsm_encode_4frames(const int16_t* pcm640, uint8_t* gsm132)
{
    if (!s_encoder || !s_codec_mutex) return;

    if (xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        const int16_t* pcm_ptr = pcm640;
        uint8_t* gsm_ptr = gsm132;
        kc1fsz::Parameters params;

        for (int i = 0; i < GSM_BUNDLE_FRAMES; i++)
        {
            s_encoder->encode(pcm_ptr, &params);
            params.pack(gsm_ptr);
            pcm_ptr += GSM_FRAME_SAMPLES;
            gsm_ptr += GSM_FRAME_BYTES;
        }
        xSemaphoreGive(s_codec_mutex);
    }
}

void gsm_decode_4frames(const uint8_t* gsm132, int16_t* pcm640)
{
    if (!s_decoder || !s_codec_mutex) return;

    if (xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        const uint8_t* gsm_ptr = gsm132;
        int16_t* pcm_ptr = pcm640;
        kc1fsz::Parameters params;

        for (int i = 0; i < GSM_BUNDLE_FRAMES; i++)
        {
            params.unpack(gsm_ptr);
            s_decoder->decode(&params, pcm_ptr);
            gsm_ptr += GSM_FRAME_BYTES;
            pcm_ptr += GSM_FRAME_SAMPLES;
        }
        xSemaphoreGive(s_codec_mutex);
    }
}
