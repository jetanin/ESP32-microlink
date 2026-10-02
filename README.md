# ESP32-MicroLink

**EchoLink Node firmware for ESP32-C6** — a self-contained VoIP repeater / link node that connects to the EchoLink amateur radio network over WiFi, using a KY-038 electret microphone for TX audio and a PCM5102A I2S DAC for RX audio playback.

[![License: GPL-3.0](https://img.shields.io/badge/License-GPL%203.0-blue.svg)](LICENSE)
[![Platform: ESP32-C6](https://img.shields.io/badge/Platform-ESP32--C6-red.svg)](https://www.espressif.com/en/products/socs/esp32-c6)
[![Framework: Arduino + ESP-IDF](https://img.shields.io/badge/Framework-Arduino%20%2B%20ESP--IDF-green.svg)](https://platformio.org)

---

## Features

- 🔗 **Full EchoLink VoIP** — RTP/RTCP audio streaming with GSM 06.10 Full-Rate codec
- 🌐 **EchoLink Proxy support** — connects through public proxy servers (no open UDP ports required)
- 🎙️ **PTT and VOX modes** — push-button PTT or voice-activated transmit, selected with a hardware switch (GPIO 23); VOX sensitivity is set with a potentiometer (GPIO 3)
- 📡 **DTMF control** — connect/disconnect stations by sending tones from a radio
- 📢 **Spoken announcements** — beep-tone chimes for connect/disconnect/mode events (routed to both local speaker and EchoLink TX)
- 🌍 **Web UI** — responsive mobile-friendly interface for status monitoring, connect/disconnect, settings and favorites management
- 🔊 **Audio pipeline** — jitter buffer, underflow protection, real-time loopback diagnostics
- 💡 **LED indicators** — WiFi/EchoLink status (green), TX active (red), RX audio (GPIO 22)
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
| **10** | LED Green   | WiFi + EchoLink registered          |
| **11** | LED Red     | TX active                           |
| **22** | LED RX      | Receiving audio from remote station |
| **23** | Mode Switch | INPUT_PULLUP: LOW = VOX, HIGH = PTT |

> **PCM5102A strap pins:** SCK→GND, FLT→GND, DMP→GND, FMT→GND, XSMT→3V3

> **VOX potentiometer (GPIO 3):** one outer leg → 3V3, other outer leg → GND, wiper → GPIO 3. Add a 100 nF capacitor from the wiper to GND, close to the pin, to reduce ADC noise. Never connect it to 5 V.

> **Mode switch (GPIO 23):** SPDT switch, common → GPIO 23, one side → GND, the other side left unconnected (internal pull-up is used). Switch to GND = **VOX**, open (HIGH) = **PTT**. This way a broken wire or failed switch falls back to PTT, so the node never keys up from noise by accident.

> **Pins to avoid on ESP32-C6:** GPIO 4, 5, 8, 9, 15 (strapping), GPIO 12/13 (USB), GPIO 16/17 (UART0), GPIO 24–30 (internal flash). GPIO 0 and 21 are kept free for a second pot (squelch/COS) and an optional I2S microphone (INMP441 SD).

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
    ┌──────────▼──────────┐    │    ┌────▼────────────┐
    │   JitterBuffer      │    │    │ service_tx_audio│
    │ (640 ms, 32 frames) │    │    │ ADC DMA capture │
    └──────────┬──────────┘    │    └────▲────────────┘
               │               │         │
    ┌──────────▼──────────┐    │    ┌────┴──────┐
    │  audio_playout_task │    │    │  KY-038   │
    │  (prio 5, I2S DMA)  │    │    │  Mic ADC  │
    └──────────┬──────────┘    │    └───────────┘
               │               │
    ┌──────────▼──────────┐    │    ┌────────────┐
    │   PCM5102A DAC      │    └────│  Announcer │  beep chimes
    │   I2S output        │         │  (LOCAL+TX)│
    └─────────────────────┘         └────────────┘
```

**Task priorities (high → low):**  
`audio_playout` (5) > `audio_capture` (4) > `echolink_task` (3) > `status_button / sys_status` (2) > `web / loop` (1)

---

## Audio Specification

| Parameter     | Value                                          |
| ------------- | ---------------------------------------------- |
| Sample rate   | 8,000 Hz                                       |
| Codec         | GSM 06.10 Full-Rate                            |
| Frame size    | 160 samples / 20 ms                            |
| RTP packet    | 4 frames / 80 ms (640 samples)                 |
| Jitter buffer | 32 frames (640 ms capacity), 4-frame watermark |
| I2S output    | 32-bit stereo (PCM5102A PLL-mode compatible)   |

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

- **PTT mode:** only the PTT button keys the transmitter. The microphone level and the potentiometer are ignored.
- **VOX mode:** the PTT button is ignored. The threshold comes from the potentiometer on GPIO 3: turning it clockwise makes VOX **more sensitive** (lower threshold). VOX uses an attack time, hysteresis and a hang time so that clicks do not key the transmitter and speech is not chopped between words. The timing constants are defined at the top of the VOX source file.
- **Switching modes** releases TX immediately and resets the VOX state, so nothing is left keyed after the switch is moved.
- **Announcements** never key VOX: the VOX input is inhibited while an announcement plays and for a short time afterwards.
- The mode and sensitivity can also be set from the serial console (`mode ptt`, `mode vox`, `sensitivity <1-9>`). The node announces mode changes, and in VOX mode announces the sensitivity step after the potentiometer has stopped moving.

**Tuning tip:** the KY-038 has no preamp and is noisy. Use serial command `2` to read the mic level in silence and while speaking, then adjust the threshold range in the source so the full potentiometer travel covers both values.

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
| `/api/status`     | GET                                         | JSON system status         |
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
sensitivity <1-9>       - Set VOX sensitivity level
say <words>             - Test announcer
abort                   - Abort current announcement
set callsign <CALL>     - Set EchoLink callsign
set password <PASS>     - Set EchoLink password
set wifi <SSID> <PASS>  - Set WiFi credentials
set name <NAME>         - Set station name
set location <QTH>      - Set location/frequency
register                - Force EchoLink registration
config                  - Print current configuration
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
│   └── echolink_protocol.cpp  # EchoLink RTP/RTCP packet formatting
├── include/                   # Header files
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
