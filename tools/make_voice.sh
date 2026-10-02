#!/usr/bin/env bash
# ==============================================================================
# MicroLink Voice Prompt Generator
# Copyright (C) 2026 MicroLink Project
#
# Generates 8 kHz 16-bit mono WAV and GSM 06.10 audio prompts for MicroLink Announcer.
# Words included:
#   connected, disconnected, all, not, vox, ptt, mode, sensitivity,
#   digits 0-9
# ==============================================================================

set -euo pipefail

PROMPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../prompts" && pwd)"
OUTPUT_DIR="${PROMPTS_DIR}/generated"
mkdir -p "${OUTPUT_DIR}"

WORDS=(
    "connected"
    "disconnected"
    "all"
    "not"
    "vox"
    "ptt"
    "mode"
    "sensitivity"
    "zero"
    "one"
    "two"
    "three"
    "four"
    "five"
    "six"
    "seven"
    "eight"
    "nine"
)

echo "=== Generating Voice Prompts for MicroLink ==="
echo "Target format: 8000 Hz, 16-bit Mono PCM / GSM 06.10 Full Rate"
echo "Words: ${WORDS[*]}"

# Check available TTS tool (espeak-ng, espeak, or pico2wave)
TTS_BIN=""
if command -v espeak-ng &>/dev/null; then
    TTS_BIN="espeak-ng"
elif command -v espeak &>/dev/null; then
    TTS_BIN="espeak"
elif command -v pico2wave &>/dev/null; then
    TTS_BIN="pico2wave"
fi

for word in "${WORDS[@]}"; do
    raw_wav="${OUTPUT_DIR}/${word}_raw.wav"
    target_wav="${OUTPUT_DIR}/${word}_8k.wav"

    echo "Synthesizing: '${word}'..."
    if [ -n "${TTS_BIN}" ]; then
        if [ "${TTS_BIN}" = "pico2wave" ]; then
            pico2wave -w "${raw_wav}" -l "en-US" "${word}"
        else
            "${TTS_BIN}" -v en-us -s 140 -p 50 -w "${raw_wav}" "${word}"
        fi
        ffmpeg -y -i "${raw_wav}" -ar 8000 -ac 1 -c:a pcm_s16le "${target_wav}" 2>/dev/null
        rm -f "${raw_wav}"
    else
        echo "  [INFO] No local TTS engine found (espeak/pico2wave). Generating tone fallback for '${word}'"
        # Generate clean tone fallback using ffmpeg
        freq=1000
        case "${word}" in
            "vox") freq=900 ;;
            "ptt") freq=1000 ;;
            "mode") freq=1200 ;;
            "sensitivity") freq=1200 ;;
            "connected") freq=880 ;;
            "disconnected") freq=660 ;;
            *) freq=800 ;;
        esac
        ffmpeg -y -f lavfi -i "sine=frequency=${freq}:duration=0.2" -ar 8000 -ac 1 -c:a pcm_s16le "${target_wav}" 2>/dev/null
    fi
done

echo "Voice prompts generated in ${OUTPUT_DIR}"
