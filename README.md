# ESP32-MicroLink

**EchoLink Node firmware for ESP32-C6** — a self-contained VoIP repeater / link node that connects to the EchoLink amateur radio network over WiFi, using a KY-038 electret microphone for TX audio and a PCM5102A I2S DAC for RX audio playback.

[![License: GPL-3.0](https://img.shields.io/badge/License-GPL%203.0-blue.svg)](LICENSE)
[![Platform: ESP32-C6](https://img.shields.io/badge/Platform-ESP32--C6-red.svg)](https://www.espressif.com/en/products/socs/esp32-c6)
[![Framework: Arduino + ESP-IDF](https://img.shields.io/badge/Framework-Arduino%20%2B%20ESP--IDF-green.svg)](https://platformio.org)

---

## Features

- 🔗 **Full EchoLink VoIP** — RTP/RTCP audio streaming with GSM 06.10 Full-Rate codec
- 🌐 **EchoLink Proxy support** — connects through public proxy servers (no open UDP ports required)
- 🎙️ **PTT and VOX modes** — push-button PTT or voice-activated transmit, selected with a hardware switch (GPIO 23); VOX sensitivity (20 levels) is set with a potentiometer (GPIO 3)
- ⏳ **VOX Pre-Roll Buffer** — static audio delay line (0..6 frames / 0..120 ms) that prevents initial syllable cutoff during VOX attack time
- 📡 **DTMF control** — connect/disconnect stations by sending tones from a radio
- 📢 **Spoken announcements** — beep-tone chimes for connect/disconnect/mode events (routed to both local speaker and EchoLink TX)
- 🌍 **Web UI** — responsive mobile-friendly interface for status monitoring, connect/disconnect, settings and favorites management
- 🔊 **Audio pipeline** — jitter buffer, underflow protection, real-time loopback diagnostics
- 💡 **LED indicators** — Link status patterns on Green LED (GPIO 10), TX active on Red LED (GPIO 11), RX audio on GPIO 22
- 📶 **Captive portal** — SoftAP with DNS for first-time WiFi setup

---

## Hardware

| Component               | Purpose                                                  |
| ----------------------- | -------------------------------------------------------- |
| **ESP32-C6 DevKitC-1**  | Main controller (RISC-V 160 MHz, 320 KB RAM, 8 MB Flash) |
| **PCM5102A DAC module** | I2S audio output to speaker / headphone                  |
| **KY-038 Microphone**   | Analog electret mic for TX audio capture                 |
| **PTT push button**     | Active-LOW push-to-talk                                  |
| **10 kΩ potentiometer** | Linear (B10K), sets VOX sensitivity                      |
| **SPDT switch**         | Selects VOX / PTT mode                                   |
| **LEDs × 3**            | Status indicators (green / red / blue-RX)                |

### Wiring / GPIO Pinout

| GPIO   | Function    | Notes                               |
| ------ | ----------- | ----------------------------------- |
| **18** | I2S BCK     | PCM5102A Bit Clock                  |
| **19** | I2S WS      | PCM5102A Word Select (LCK)          |
| **20** | I2S DOUT    | PCM5102A DIN                        |
| **1**  | Mic ADC     | KY-038 Analog Output → ADC1_CH1     |
| **2**  | PTT Button  | INPUT_PULLUP, active LOW            |
| **3**  | VOX Pot     | Wiper → ADC1_CH3 (VOX sensitivity)  |
| **10** | LED Green   | Link status pattern (Setup / WiFi / Reg / Linked / Idle) |
| **11** | LED Red     | TX active                           |
| **22** | LED RX      | Receiving audio from remote station |
| **23** | Mode Switch | INPUT_PULLUP: LOW = VOX, HIGH = PTT |

> **PCM5102A strap pins:** SCK→GND, FLT→GND, DMP→GND, FMT→GND, XSMT→3V3

> **VOX potentiometer (GPIO 3):** one outer leg → 3V3, other outer leg → GND, wiper → GPIO 3. Add a 100 nF capacitor from the wiper to GND, close to the pin, to reduce ADC noise. Never connect it to 5 V.

> **Mode switch (GPIO 23):** SPDT switch, common → GPIO 23, one side → GND, the other side left unconnected (internal pull-up is used). Switch to GND = **VOX**, open (HIGH) = **PTT**. This way a broken wire or failed switch falls back to PTT, so the node never keys up from noise by accident.

> **Pins to avoid on ESP32-C6:** GPIO 4, 5, 8, 9, 15 (strapping), GPIO 12/13 (USB), GPIO 16/17 (UART0), GPIO 24–30 (internal flash). GPIO 0 and 21 are kept free for a second pot (squelch/COS) and an optional I2S microphone (INMP441 SD).

### Link Status LED Patterns (Green — GPIO 10)

The green LED provides visual diagnostic feedback on connectivity and registration state. Patterns are evaluated top-to-bottom:

| Priority | State | Pattern (ON / OFF ms) | Visual Appearance | Meaning |
| :---: | :--- | :--- | :--- | :--- |
| **1** | `SetupAp` | 80, 120, 80, 720 | Double flash | Waiting for WiFi setup (captive portal active) |
| **2** | `WifiConnecting` | 100, 100 | Fast blink | Trying to join WiFi network |
| **3** | `RegFailed` | 120, 180, 120, 180, 120, 1200 | Triple flash | Registration failed (latches until registration succeeds) |
| **4** | `Registering` | 500, 500 | Slow blink | Waiting for EchoLink directory server login |
| **5** | `Linked` | Steady ON | Solid ON | Connected to remote EchoLink station |
| **6** | `Idle` | 80, 1920 | Heartbeat flash | Ready / registered, no station linked (never fully dark) |

- **Pattern Restart:** On every state change, the pattern restarts immediately with the LED ON and the first step timer reset.
- **Latch Behavior:** `RegFailed` remains active across background retries until registration succeeds.
- **Alive Indication:** `Idle` uses a periodic 80 ms heartbeat so a dark LED never causes ambiguity between "idle" and "crashed/powered off".

---

## Architecture

```
                     EchoLink Network
                          │  RTP/RTCP (UDP)
                    ┌─────▼────────────────┐
                    │   echolink_client    │  FreeRTOS task (prio 3)
                    │  GSM 06.10 codec     │
                    └──────────┬───────────┘
               RX              │         TX
    ┌──────────▼──────────┐    │    ┌────▼─────────────────┐
    │   JitterBuffer      │    │    │   service_tx_audio   │ (prio 3)
    │ (640 ms, 32 frames) │    │    └────▲─────────────────┘
    └──────────┬──────────┘    │         │ read_samples
               │               │    ┌────┴─────────────────┐
    ┌──────────▼──────────┐    │    │     TX PCM FIFO      │ (1280 samples)
    │  audio_playout_task │    │    └────▲─────────────────┘
    │  (prio 5, I2S DMA)  │    │         │
    └──────────┬──────────┘    │    ┌────┴─────────────────┐
               │               │    │  audio_capture_task  │ (prio 4, 50 Hz)
    ┌──────────▼──────────┐    │    │  ADC DMA capture     │
    │   PCM5102A DAC      │    │    │  VoxPreRoll (0-6 fr) │
    │   I2S output        │    │    └────▲─────────────────┘
    └─────────────────────┘    │         │
                               │    ┌────┴──────┐
                               │    │  KY-038   │
                               │    │  Mic ADC  │
                               │    └───────────┘
                               │
                               │    ┌────────────┐
                               └────│  Announcer │  beep chimes
                                    │  (LOCAL+TX)│
                                    └────────────┘
```

**Task priorities (high → low):**  
`audio_playout` (5) > `audio_capture` (4) > `echolink_task` (3) > `status_button / sys_status` (2) > `web / loop` (1)

---

## Audio Specification

| Parameter       | Value                                                         |
| --------------- | ------------------------------------------------------------- |
| Sample rate     | 8,000 Hz                                                      |
| Codec           | GSM 06.10 Full-Rate                                           |
| Frame size      | 160 samples / 20 ms                                           |
| RTP packet      | 4 frames / 80 ms (640 samples)                                |
| Jitter buffer   | 32 frames (640 ms capacity), 4-frame watermark                |
| VOX Pre-roll    | Static ring buffer (up to 8 frames), default 3 frames (60 ms) |
| I2S output      | 32-bit stereo (PCM5102A PLL-mode compatible)                  |

---

## DTMF Control

Send DTMF tones from your transceiver to control the node:

| Sequence          | Action                       |
| ----------------- | ---------------------------- |
| `*1` + node + `#` | Connect to EchoLink node     |
| `*2` + node + `#` | Disconnect one station       |
| `*0#`             | Disconnect all               |
| `*9#`             | Announce status (beep tones) |

---

## PTT / VOX Mode

The transmit mode is selected with the mode switch on GPIO 23:

| Switch position | Mode    | Transmit is triggered by                         |
| --------------- | ------- | ------------------------------------------------ |
| To GND (LOW)    | **VOX** | Microphone level above the sensitivity threshold |
| Open (HIGH)     | **PTT** | PTT button (GPIO 2) held down                    |

- **PTT mode:** Only the PTT button keys the transmitter. The microphone level and the potentiometer are ignored. Audio bypasses the pre-roll buffer completely with **0 ms added latency**.
- **VOX mode:** The PTT button is ignored. Voice detection uses a **4-frame (80 ms) moving RMS energy window** to reject short acoustic clicks, pops, and spikes.
  - **Sensitivity adjustment:** Managed via the potentiometer on GPIO 3 across **20 discrete logarithmic sensitivity steps** (`VOX_THRESHOLDS[1..20]`). Turning clockwise makes VOX more sensitive.
  - **Attack & Hang timers:** Requires **40 ms attack confirmation** (signal sustained above threshold) to open TX, and maintains a **400 ms hang timer** to bridge pauses between words without chopping.
- **VOX Pre-Roll Buffer (Audio Delay Line):**
  - **The Problem:** Because VOX requires ~40 ms of speech to confirm voice activity and open TX, the first syllable of speech (e.g. saying *"one two three"*) would otherwise be cut off (*"...wo three"*).
  - **The Solution:** A fixed static ring buffer (`VoxPreRoll`, 8-frame capacity) delays the transmitted audio by `N` frames (default 3 frames = 60 ms) while VOX detection decisions are evaluated on the live incoming frames. When VOX opens, the delayed stream begins from audio recorded *before* the trigger decision, preserving the speech onset completely.
  - **PTT Bypass:** In PTT mode, audio bypasses the pre-roll delay line completely for zero latency.
  - **Dynamic Configuration:** Configurable from **0 to 6 frames** (`0..120 ms` added TX latency in VOX mode) via serial command `vox preroll <0-6>` and persisted in NVS key `vox_pre`.
  - **Automatic Resets:** The pre-roll buffer is automatically flushed and reset on:
    1. Mode switches (PTT $\leftrightarrow$ VOX in either direction).
    2. Completion of a `ROUTE_TX` announcement plus its 300 ms post-announcement VOX inhibit period (ensures no speaker bleed is transmitted).
    3. WiFi or EchoLink session disconnects.
    4. Any runtime delay setting change.
- **Switching modes** releases TX immediately and resets the VOX state.
- **Announcements** inhibit VOX while playing and for 300 ms afterwards.
- The mode and sensitivity can also be set from the serial console (`mode ptt`, `mode vox`, `sensitivity <1-20>`, `vox preroll <0-6>`).

**Tuning tip:** The KY-038 has no onboard preamp. Use serial command `2` to read the mic level in silence and while speaking, then adjust potentiometer sensitivity to achieve reliable triggering.

---

## Spoken Announcements

The firmware generates real-time beep-tone chimes (no audio files needed):

| Event                | Chime                               |
| -------------------- | ----------------------------------- |
| Connected            | Ascending chime (880 Hz → 1320 Hz)  |
| Disconnected         | Descending chime (1320 Hz → 880 Hz) |
| Disconnect all       | Low + high beep                     |
| PTT mode             | 1000 Hz long beep                   |
| VOX mode             | 900 Hz × 2 short beeps              |
| Sensitivity          | 1200 Hz blip                        |
| Digits (node number) | Stepped tones 600–1320 Hz           |

Announcements are routed to **local speaker** (ROUTE_LOCAL) and/or **EchoLink TX** (ROUTE_TX). While a TX announcement is active, the microphone path is muted to prevent feedback. PTT press immediately aborts the announcement and hands TX to the operator.

---

## Web UI

Access via `http://microlink.local` (mDNS) or the device's IP address.

| Page         | Description                                                   |
| ------------ | ------------------------------------------------------------- |
| **Status**   | Live RX/TX indicators, VU meters, jitter stats, system uptime |
| **Connect**  | Connect/disconnect EchoLink stations, manage favorites        |
| **Settings** | WiFi, EchoLink callsign/password, proxy, web credentials      |

Default login: `admin` / `admin` (change in Settings).

### REST API

| Endpoint          | Method                                      | Description                |
| ----------------- | ------------------------------------------- | -------------------------- |
| `/api/status`     | GET                                         | JSON system status (includes `link_led`: `SetupAp` \| `WifiConnecting` \| `Registering` \| `RegFailed` \| `Idle` \| `Linked`) |
| `/api/connect`    | POST `{"target":"*ECHOTEST*"}`              | Connect to station         |
| `/api/disconnect` | POST                                        | Disconnect current station |
| `/api/dtmf`       | POST `{"command":"*19999#"}`                | Send DTMF command          |
| `/api/announce`   | POST `{"text":"connected 12345","route":3}` | Trigger announcement       |
| `/api/config`     | GET / POST                                  | Read/write configuration   |
| `/api/favorites`  | GET / POST                                  | Read/write favorites list  |
| `/api/loopback`   | POST `{"active":true}`                      | Toggle mic loopback        |

`route`: `1`=local only, `2`=TX only, `3`=local+TX (default)

---

## Building & Flashing

**Requirements:** [PlatformIO](https://platformio.org/) (VSCode extension or CLI)

```bash
# Build firmware
pio run

# Flash firmware
pio run --target upload

# Upload web UI files to LittleFS
pio run --target uploadfs

# Monitor serial output
pio device monitor --baud 115200
```

---

## First-Time Setup

1. Flash the firmware. On first boot with no WiFi configured, the device starts in **SoftAP mode**.
2. Connect your phone/PC to the WiFi network **`MicroLink-Setup`** (no password).
3. A captive portal opens automatically — enter your home WiFi SSID and password.
4. The device reboots and connects to your WiFi. Find its IP from your router or use `microlink.local`.
5. Open the Web UI → **Settings** tab and configure:
   - EchoLink callsign (e.g. `HS1ABC-L`)
   - EchoLink password
   - Proxy settings (if required by your network)
6. Click **Save** and the node registers with the EchoLink directory server.

---

## Serial Commands

Connect to the device at **115200 baud** for diagnostic commands:

```
c                       - Connect to *ECHOTEST* (node 9999)
connect <station>       - Connect to callsign or node number
d                       - Disconnect
mode ptt                - Switch to PTT mode
mode vox                - Switch to VOX mode
sensitivity <1-20>      - Set VOX sensitivity level
vox preroll <0-6>       - Set VOX pre-roll delay (0..120 ms added TX latency in VOX mode)
say <words>             - Test announcer
abort                   - Abort current announcement
set callsign <CALL>     - Set EchoLink callsign
set password <PASS>     - Set EchoLink password
set wifi <SSID> <PASS>  - Set WiFi credentials
set name <NAME>         - Set station name
set location <QTH>      - Set location/frequency
register                - Force EchoLink registration
config                  - Print current configuration
led                     - Print link status LED state and raw inputs
1                       - Play 1 kHz test tone (PCM5102A)
2                       - Measure mic level
3                       - Record (2 s) then play back
4                       - Toggle real-time mic loopback
j                       - Jitter buffer statistics
s                       - System status summary
h                       - Show help
```

---

## Project Structure

```
ESP32-microlink/
├── src/
│   ├── main.cpp               # Setup, loop, serial commands, PTT button task
│   ├── echolink_client.cpp    # EchoLink VoIP engine (RTP/RTCP, TX arbitration)
│   ├── audio_pipeline.cpp     # I2S playout + ADC capture tasks, jitter buffer
│   ├── audio_in.cpp           # ADC DMA input (KY-038)
│   ├── audio_out.cpp          # I2S DMA output (PCM5102A)
│   ├── gsm_codec.cpp          # GSM 06.10 4-frame encode/decode wrappers
│   ├── announcer.cpp          # Beep-tone announcement engine
│   ├── dtmf_controller.cpp    # DTMF command dispatcher
│   ├── dtmf_detector.cpp      # Goertzel DTMF tone detector
│   ├── jitter_buffer.cpp      # Audio jitter buffer with underflow protection
│   ├── web_ui.cpp             # HTTP REST API + embedded fallback UI
│   ├── wifi_manager.cpp       # WiFi STA/AP management + mDNS
│   ├── config_manager.cpp     # NVS Preferences persistence
│   ├── system_state.cpp       # Shared state (mutex-protected)
│   ├── echolink_proxy.cpp     # EchoLink proxy protocol
├── include/                   # Header files (including vox_pre_roll.h, link_led.h)
├── test/                      # Unit test suite (test_vox_pre_roll.cpp, test_link_led.cpp)
├── data/                      # LittleFS web UI files (HTML/CSS/JS)
├── lib/gsm0610/               # GSM 06.10 codec library
├── tools/
│   └── make_voice.sh          # Script to generate TTS voice prompts (optional)
└── platformio.ini
```

---

## Dependencies

| Library                                 | Version   | Purpose                                      |
| --------------------------------------- | --------- | -------------------------------------------- |
| [ArduinoJson](https://arduinojson.org/) | ^7.3.0    | JSON for REST API and config                 |
| ESP-IDF built-ins                       | (bundled) | I2S, ADC DMA, NVS, LittleFS, WiFi, WebServer |

The GSM 06.10 codec is bundled as a local library under `lib/gsm0610/`.

---

## Configuration Reference

All settings stored in ESP32 NVS (non-volatile storage):

| Key          | Default             | Description                         |
| ------------ | ------------------- | ----------------------------------- |
| `wifi_ssid`  | _(empty)_           | WiFi network name                   |
| `wifi_pass`  | _(empty)_           | WiFi password                       |
| `callsign`   | `N0CALL`            | EchoLink callsign (e.g. `HS1ABC-L`) |
| `el_pass`    | _(empty)_           | EchoLink account password           |
| `st_name`    | `MicroLink Node`    | Station name shown in directory     |
| `location`   | `Bangkok, Thailand` | QTH / frequency                     |
| `web_user`   | `admin`             | Web UI username                     |
| `web_pass`   | `admin`             | Web UI password                     |
| `proxy_en`   | `false`             | Use EchoLink proxy server           |
| `proxy_host` | _(empty)_           | Proxy hostname/IP                   |
| `proxy_port` | `8100`              | Proxy TCP port                      |
| `proxy_pass` | `PUBLIC`            | Proxy password                      |
| `vox_pre`    | `3`                 | VOX pre-roll buffer delay in frames (0..6 frames, 20 ms/frame = 0..120 ms added TX latency in VOX mode) |

---

## Security Notes

- **Change the default web login (`admin` / `admin`) right after the first setup.** Anyone on your network who knows it can change your EchoLink settings.
- **Use the web UI on your local network only.** Do not forward its port to the Internet.
- The setup access point **`MicroLink-Setup`** has no password, so only use it during first-time setup, in a place you trust.
- Passwords (WiFi, EchoLink, proxy) are stored in NVS and should never be shown back in the web UI or the API (`/api/config` should return them masked).

---

## License

This project is licensed under the **GNU General Public License v3.0**.  
See [LICENSE](LICENSE) for details.

Inspired by and adapted from [Bruce MacKinnon (KC1FSZ) MicroLink](https://github.com/brucemack/microlink) (GPL-3.0).

---

## Contributing

Pull requests and issues welcome. Please test on hardware before submitting audio pipeline changes — timing is critical at 8 kHz / 20 ms frame intervals on a single-core RISC-V MCU.

---

_73 de E26BFR — Built for the Thai amateur radio community 🇹🇭_
