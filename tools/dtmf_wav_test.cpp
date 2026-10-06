#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <cmath>

#include "dtmf_detector.h"
#include "dtmf_gate.h"

// Stub for dtmf_controller_handle_digit()
void dtmf_controller_handle_digit(char) {}

#pragma pack(push, 1)
struct WavHeader {
    char riff_tag[4];      // "RIFF"
    uint32_t riff_size;
    char wave_tag[4];      // "WAVE"
    char fmt_tag[4];       // "fmt "
    uint32_t fmt_size;
    uint16_t audio_format; // 1 = PCM
    uint16_t num_channels; // 1 = Mono
    uint32_t sample_rate;  // 8000
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample; // 16
};
#pragma pack(pop)

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: dtmf_wav_test <file.wav>\n\n";
        std::cerr << "Format required: 8 kHz, 16-bit, mono PCM WAV.\n";
        std::cerr << "To convert any audio file with ffmpeg:\n";
        std::cerr << "  ffmpeg -i input.m4a -ar 8000 -ac 1 -c:a pcm_s16le output_8k.wav\n";
        return 1;
    }

    const char *wav_path = argv[1];
    std::ifstream file(wav_path, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file: " << wav_path << "\n";
        return 1;
    }

    WavHeader hdr;
    file.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!file || std::strncmp(hdr.riff_tag, "RIFF", 4) != 0 || std::strncmp(hdr.wave_tag, "WAVE", 4) != 0) {
        std::cerr << "Error: Not a valid RIFF/WAVE file.\n";
        return 1;
    }

    if (hdr.audio_format != 1 || hdr.num_channels != 1 || hdr.sample_rate != 8000 || hdr.bits_per_sample != 16) {
        std::cerr << "Error: Unsupported WAV format!\n";
        std::cerr << "  Detected: " << hdr.sample_rate << " Hz, " << hdr.num_channels
                  << " channel(s), " << hdr.bits_per_sample << " bits/sample (format=" << hdr.audio_format << ")\n";
        std::cerr << "  Expected: 8000 Hz, 1 channel (mono), 16 bits/sample, uncompressed PCM.\n";
        std::cerr << "Convert with:\n";
        std::cerr << "  ffmpeg -i " << wav_path << " -ar 8000 -ac 1 -c:a pcm_s16le converted_8k.wav\n";
        return 1;
    }

    // Find "data" chunk
    char chunk_id[4];
    uint32_t chunk_size = 0;
    while (file.read(chunk_id, 4) && file.read(reinterpret_cast<char*>(&chunk_size), 4)) {
        if (std::strncmp(chunk_id, "data", 4) == 0) {
            break;
        }
        file.seekg(chunk_size, std::ios::cur);
    }

    if (std::strncmp(chunk_id, "data", 4) != 0) {
        std::cerr << "Error: Could not locate 'data' chunk in WAV.\n";
        return 1;
    }

    std::vector<int16_t> samples(chunk_size / sizeof(int16_t));
    file.read(reinterpret_cast<char*>(samples.data()), chunk_size);
    size_t actual_samples = file.gcount() / sizeof(int16_t);
    samples.resize(actual_samples);

    std::cout << "Loaded WAV: " << wav_path << " (" << actual_samples << " samples, "
              << (actual_samples / 8000.0f) << " s, " << (actual_samples / 160) << " frames)\n\n";
    std::cout << std::left
              << std::setw(8)  << "Frame"
              << std::setw(10) << "Time(ms)"
              << std::setw(8)  << "Peak"
              << std::setw(8)  << "RMS"
              << std::setw(8)  << "Cand"
              << std::setw(8)  << "Char"
              << std::setw(8)  << "Conf"
              << std::setw(10) << "GateVOX"
              << "\n";
    std::cout << std::string(68, '-') << "\n";

    DTMFDetector det;
    det.init();
    DtmfGate gate;

    size_t total_frames = actual_samples / 160;
    uint32_t candidate_count = 0;
    uint32_t confirmed_count = 0;
    uint32_t force_close_count = 0;

    for (size_t f = 0; f < total_frames; ++f) {
        const int16_t *frame = samples.data() + f * 160;
        uint32_t time_ms = (uint32_t)(f * 20);

        int16_t peak = 0;
        uint64_t sum_sq = 0;
        for (int i = 0; i < 160; ++i) {
            int16_t a = std::abs(frame[i]);
            if (a > peak) peak = a;
            sum_sq += (int32_t)frame[i] * (int32_t)frame[i];
        }
        uint16_t rms = (uint16_t)std::sqrt(sum_sq / 160);

        DtmfFrameResult res = det.process_frame(frame);
        gate.update(res.candidate, res.confirmed_digit, false, time_ms);

        if (res.candidate) candidate_count++;
        if (res.confirmed_digit != '\0') confirmed_count++;
        if (gate.should_force_close_vox()) force_close_count++;

        // Only print frames that have candidate, confirmed digit, gate activity, or significant audio
        if (res.candidate || res.confirmed_digit != '\0' || gate.is_vox_inhibited()) {
            std::cout << std::left
                      << std::setw(8)  << f
                      << std::setw(10) << time_ms
                      << std::setw(8)  << peak
                      << std::setw(8)  << rms
                      << std::setw(8)  << (res.candidate ? "YES" : ".")
                      << std::setw(8)  << (res.candidate_char != '\0' ? res.candidate_char : '-')
                      << std::setw(8)  << (res.confirmed_digit != '\0' ? res.confirmed_digit : '-')
                      << std::setw(10) << (gate.should_force_close_vox() ? "CLOSE" : (gate.is_vox_inhibited() ? "INHIBIT" : "OK"))
                      << "\n";
        }
    }

    std::cout << std::string(68, '-') << "\n";
    std::cout << "Summary for " << wav_path << ":\n";
    std::cout << "  Total frames:         " << total_frames << "\n";
    std::cout << "  Candidate frames:     " << candidate_count << "\n";
    std::cout << "  Confirmed digits:     " << confirmed_count << "\n";
    std::cout << "  VOX force-close/inh:  " << force_close_count << "\n";

    return 0;
}
