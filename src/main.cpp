/**
 * @file main.cpp
 * @brief MicroLink ESP32-C6 EchoLink Node - Phase 5 EchoLink Protocol & GSM Codec
 *
 * Hardware target: ESP32-C6 (single-core RISC-V 160MHz)
 * Phase 5 Features:
 * - Real EchoLink Addressing Server login (TCP 5200) & periodic directory registration
 * - Station directory lookup & remote connection (Callsign / Node #)
 * - VoIP Audio Engine (RTP on UDP 5198, RTCP on UDP 5199)
 * - GSM 06.10 Full-Rate RPE-LTP fixed-point encoder & decoder
 * - Audio Pipeline clocked by I2S DMA with Jitter Buffer (32 frames, 4-frame watermark)
 * - PTT keying with Red LED TX indicator and Green LED connection status
 */

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include "pins.h"
#include "system_state.h"
#include "config_manager.h"
#include "wifi_manager.h"
#include "web_ui.h"
#include "audio_common.h"
#include "audio_out.h"
#include "audio_in.h"
#include "audio_pipeline.h"
#include "gsm_codec.h"
#include "echolink_client.h"
#include "echolink_protocol.h"
#include "dtmf_controller.h"
#include "dtmf_detector.h"
#include "dtmf_gate.h"
#include "announcer.h"
#include "link_led.h"

// Task Priorities (Audio > Network/EchoLink > System/Status > Web UI)
static constexpr UBaseType_t TASK_PRIO_SYSTEM = 2;

// Timing constants
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;

// Audio Subsystems
static AudioOut s_audio_out;
static AdcAudioIn s_audio_in(PIN_MIC_AO, PIN_POT_VOX_SENS); // GPIO1=Mic, GPIO3=VOX Pot
static AudioPipeline s_pipeline(s_audio_in, s_audio_out);

// Announcer local sink: writes directly to I2S DAC (bypasses jitter buffer).
// This avoids the 4-frame watermark delay and lets the announcement play immediately
// whether we are in IDLE, LOOPBACK, or NETWORK mode.
static void announcer_local_sink(const int16_t *samples, size_t count)
{
    // Write directly to I2S output — same path as play_tone()
    // Chunk into AUDIO_FRAME_SAMPLES (160) pieces like write_samples normally does
    const int16_t *ptr = samples;
    size_t remaining = count;
    while (remaining > 0)
    {
        size_t chunk = (remaining > AUDIO_FRAME_SAMPLES) ? AUDIO_FRAME_SAMPLES : remaining;
        s_audio_out.write_samples(ptr, chunk, pdMS_TO_TICKS(50));
        ptr += chunk;
        remaining -= chunk;
    }
}

static void announcer_tx_sink(const int16_t *samples, size_t count)
{
    echolink_client_feed_announcement_pcm(samples, count);
}

static bool announcer_channel_busy()
{
    SystemState st = system_state_get();
    return st.rx_active || st.tx_active;
}

// Operating Modes (PTT vs VOX)
enum class OperatingMode : uint8_t
{
    PTT = 0,
    VOX
};

static constexpr uint32_t TOT_MAX_MS = 60000; // 60 s TX limit

static OperatingMode s_op_mode = OperatingMode::PTT;
static uint8_t s_current_sensitivity = 10;
static uint8_t s_last_announced_sensitivity = 0;
static uint32_t s_pot_stable_start = 0;
static uint32_t s_last_sensitivity_announce_time = 0;
static uint8_t s_last_raw_pot_step = 10;

// TOT state (shared between PTT and VOX modes)
static bool s_tot_triggered = false; // TRUE when TOT has fired and TX was cut
static uint32_t s_tx_started_at = 0; // Wall-clock ms when TX keyed up
static bool s_tx_was_active = false; // Previous TX state (edge detection)

static void switch_mode(OperatingMode new_mode)
{
    if (s_op_mode == new_mode)
        return;

    // Rule: Mode switch during an announcement: abort it and apply normal force TX off
    if (Announcer::instance().busy())
    {
        Announcer::instance().abort();
    }
    echolink_client_tx_force_off();

    // Reset VOX state on mode change
    s_pipeline.reset_vox();
    s_tot_triggered = false;
    s_tx_was_active = false;

    s_op_mode = new_mode;
    audio_pipeline_request_vox_reset();
    Serial.printf("[MODE] Switched to %s mode\n", (s_op_mode == OperatingMode::PTT) ? "PTT" : "VOX");
    system_state_set_mode((uint8_t)s_op_mode, s_current_sensitivity);
    system_state_set_tot(false);

    if (s_op_mode == OperatingMode::PTT)
    {
        Announcer::instance().announcePttMode();
    }
    else
    {
        Announcer::instance().announceVoxMode();
        s_pot_stable_start = millis();
        s_last_sensitivity_announce_time = millis();
    }
}

static void set_vox_sensitivity(uint8_t step)
{
    if (step < 1)
        step = 1;
    if (step > 20)
        step = 20;
    s_current_sensitivity = step;
    // Also update pot tracker so the pot read doesn't immediately override this serial command
    // (the pot will only override again when it's physically moved to a different position)
    s_last_raw_pot_step = step;
    s_pot_stable_start = 0; // Reset stability so sensitivity won't re-announce old value
    system_state_set_mode((uint8_t)s_op_mode, step);
    Serial.printf("[VOX] Sensitivity set to %u / 20 (threshold RMS=%u)\n", step, VOX_THRESHOLDS[step]);
}

// 2-second single-shot loopback PCM buffer (for diagnostic hardware test)
static int16_t s_loopback_buffer[LOOPBACK_TOTAL_SAMPLES];
static volatile size_t s_loopback_recorded_samples = 0;
static volatile bool s_is_recording = false;
static volatile bool s_is_playing = false;

// Forward Declarations
static void run_tone_test(float freq_hz = 1000.0f, uint32_t duration_ms = 2000);
static void run_mic_level_test(uint32_t duration_ms = 3000);
static void run_2sec_loopback_test();
static void toggle_realtime_loopback();
static void print_jitter_stats();
static void print_banner();
static void print_status_summary();
static void status_button_task(void *pvParameters);

void setup()
{
    Serial.begin(115200);
    delay(1000); // Allow USB CDC connection to stabilize

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_RX, OUTPUT);
    pinMode(PIN_BUTTON_PTT, INPUT_PULLUP);
    pinMode(PIN_SWITCH_VOX, INPUT_PULLUP); // LOW = VOX mode, HIGH = PTT mode

    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_RX, LOW);

    // Read VOX switch on boot to set initial operating mode
    s_op_mode = (digitalRead(PIN_SWITCH_VOX) == LOW) ? OperatingMode::VOX : OperatingMode::PTT;
    s_current_sensitivity = 10;
    s_last_raw_pot_step = 10;

    // 1. Initialize State Machine & NVS Config
    system_state_init();
    config_manager_init();
    ConfigData cfg = config_manager_get();
    s_pipeline.set_vox_preroll_delay(cfg.vox_pre);
    Serial.printf("VOX pre-roll: %u frames (%u ms)\n", cfg.vox_pre, (unsigned int)(cfg.vox_pre * 20));
    if (cfg.vox_pre < DtmfGate::PRE_ROLL_N_FRAMES)
    {
        Serial.printf("[VOX] WARNING: vox_pre (%u frames) < %u frames (80 ms). Tone leakage may occur during DTMF detection onset!\n",
                      cfg.vox_pre, DtmfGate::PRE_ROLL_N_FRAMES);
    }
    // Sync initial mode to system state (must be after system_state_init)
    system_state_set_mode((uint8_t)s_op_mode, s_current_sensitivity);

    // Spawn status task immediately so LED patterns run during WiFi and EchoLink connection
    BaseType_t res = xTaskCreate(
        status_button_task,
        "sys_status_task",
        3072,
        nullptr,
        TASK_PRIO_SYSTEM,
        nullptr);
    if (res != pdPASS)
    {
        Serial.println(F("[ERROR] Failed to create sys_status_task!"));
    }

    print_banner();

    // 2. Initialize Audio Pipeline & Jitter Buffer
    Serial.print(F("[PIPELINE] Initializing Audio Pipeline (I2S DMA + Continuous ADC1 + Jitter Buffer)... "));
    if (s_pipeline.init())
    {
        s_pipeline.start();
        Serial.println(F("OK"));
    }
    else
    {
        Serial.println(F("FAILED!"));
    }

    // 3. Initialize GSM 06.10 Codec
    Serial.print(F("[GSM] Initializing ETSI GSM 06.10 Codec Engine... "));
    if (gsm_codec_init())
    {
        Serial.println(F("OK"));
    }
    else
    {
        Serial.println(F("FAILED!"));
    }

    // 4. Initialize WiFi & Fallback AP
    wifi_manager_init();
    wifi_manager_start();

    // 5. Initialize Web UI & REST APIs
    web_ui_init();

    // 6. Initialize and Start EchoLink Client Background Task
    Serial.print(F("[ECHOLINK] Initializing EchoLink Client Engine... "));
    if (echolink_client_init())
    {
        if (echolink_client_start())
        {
            Serial.println(F("OK (Task running at Priority 3)"));
        }
        else
        {
            Serial.println(F("FAILED to start task!"));
        }
    }
    else
    {
        Serial.println(F("FAILED to init client!"));
    }

    // 7. Initialize DTMF Tone Detector & Command Controller
    dtmf_controller_init();
    dtmf_detector_init();

    // 8. Initialize Spoken Voice & Beep Announcer Engine
    announcer_init(announcer_local_sink, announcer_tx_sink, announcer_channel_busy);

    // Boot announcement: "ptt mode" or "vox mode"
    if (s_op_mode == OperatingMode::PTT)
    {
        Announcer::instance().announcePttMode();
    }
    else
    {
        Announcer::instance().announceVoxMode();
    }

    Serial.println(F("============================================================"));
    Serial.println(F("MicroLink Phase 5 EchoLink Node Ready!"));
    Serial.printf("Web Interface URL : http://%s/\n", wifi_manager_get_ip().toString().c_str());
    if (wifi_manager_is_connected())
    {
        Serial.println(F("mDNS Hostname     : http://microlink.local/"));
    }
    else
    {
        Serial.println(F("SoftAP Mode       : Connect to 'MicroLink-Setup' -> 192.168.4.1"));
    }
    Serial.println(F("Commands: 'c'=Connect *ECHOTEST* | 'd'=Disconnect | '4'=Local Loopback | 'j'=Jitter | 's'=Status"));
    Serial.println(F("DTMF    : *1<node>#=Connect | *2<node>#=Disconnect | *0#=Drop All | *9#=Status"));
    Serial.println(F("============================================================\n"));
}

void loop()
{
    static uint32_t last_heartbeat = 0;
    static uint32_t last_stats_sync = 0;
    static bool last_vox_switch = true; // Previous state of GPIO23 (HIGH = PTT)
    uint32_t now = millis();

    // =========================================================================
    // 1. Process WiFi, Web Server and Announcer
    // =========================================================================
    wifi_manager_process();
    web_ui_process();
    announcer_update();

    // Reset VOX pre-roll buffer at the end of ROUTE_TX announcement + VOX inhibit window
    static bool s_was_vox_inhibited = false;
    bool is_vox_inhibited = Announcer::instance().is_vox_inhibited();
    if (s_was_vox_inhibited && !is_vox_inhibited)
    {
        audio_pipeline_request_vox_reset();
    }
    s_was_vox_inhibited = is_vox_inhibited;

    // Reset VOX pre-roll on WiFi disconnect
    static bool s_was_wifi_connected = false;
    bool is_wifi_connected = wifi_manager_is_connected();
    if (s_was_wifi_connected && !is_wifi_connected)
    {
        audio_pipeline_request_vox_reset();
    }
    s_was_wifi_connected = is_wifi_connected;

    // =========================================================================
    // 2. VOX Mode Switch Polling (GPIO 23, debounced)
    // =========================================================================
    {
        static uint32_t switch_debounce_ts = 0;
        static bool switch_raw_last = true;
        bool switch_raw = (digitalRead(PIN_SWITCH_VOX) == LOW); // LOW = VOX mode
        if (switch_raw != switch_raw_last)
        {
            switch_raw_last = switch_raw;
            switch_debounce_ts = now;
        }
        if (now - switch_debounce_ts >= 50)
        {
            OperatingMode wanted = switch_raw ? OperatingMode::VOX : OperatingMode::PTT;
            if (wanted != s_op_mode)
            {
                switch_mode(wanted);
                last_vox_switch = switch_raw;
            }
        }
    }

    // =========================================================================
    // 3. Sensitivity Stability Announce (pot moved logic in section 5's stats sync)
    // =========================================================================
    if (s_op_mode == OperatingMode::VOX &&
        s_last_raw_pot_step == s_current_sensitivity &&
        s_pot_stable_start > 0 &&
        (now - s_pot_stable_start >= 1500) &&
        (s_current_sensitivity != s_last_announced_sensitivity) &&
        (now - s_last_sensitivity_announce_time >= 3000))
    {
        s_last_announced_sensitivity = s_current_sensitivity;
        s_last_sensitivity_announce_time = now;
        Announcer::instance().announceSensitivity(s_current_sensitivity);
    }

    // =========================================================================
    // 4. TOT (Time-Out Timer) — monitors any TX source, 60 s hard limit
    // =========================================================================
    {
        bool tx_now = echolink_client_is_tx_active();

        // Rising edge: TX just opened
        if (tx_now && !s_tx_was_active)
        {
            s_tx_started_at = now;
            s_tot_triggered = false;
            system_state_set_tot(false);
        }

        // TX is active: check elapsed time
        if (tx_now && !s_tot_triggered)
        {
            uint32_t elapsed = now - s_tx_started_at;
            if (elapsed >= TOT_MAX_MS)
            {
                // Time-Out: force release all TX
                Serial.println(F("[TOT] *** 60-second Time-Out Timer fired! TX released. ***"));
                if (Announcer::instance().busy())
                    Announcer::instance().abort();
                echolink_client_tx_force_off();
                s_pipeline.force_close_vox();
                digitalWrite(PIN_LED_RED, LOW);
                s_tot_triggered = true;
                system_state_set_tot(true);
            }
        }

        // Falling edge: TX released
        if (!tx_now && s_tx_was_active)
        {
            s_tot_triggered = false; // Ready for next TX
            system_state_set_tot(false);
        }

        s_tx_was_active = tx_now;
    }

    // =========================================================================
    // 5. Mic metrics sync (every 100 ms) — must run BEFORE VOX VAD
    // =========================================================================
    if (now - last_stats_sync >= 100)
    {
        last_stats_sync = now;

        JitterBufferStats jstats = s_pipeline.get_stats();
        system_state_update_jitter(jstats.current_depth_frames, jstats.underflow_count, jstats.overflow_count);

        uint16_t r_min, r_max, r_avg, vpp;
        s_audio_in.get_metrics(r_min, r_max, r_avg, vpp);

        // Map vpp to 0-100 percentage for Web UI
        int16_t pct = (vpp * 100) / 1200;
        if (pct > 100)
            pct = 100;
        system_state_update_mic(r_avg, pct);

        // Also read pot sensitivity every 100 ms
        uint8_t pot_step = s_audio_in.get_pot_step();
        if (pot_step != s_last_raw_pot_step)
        {
            Serial.printf("[POT] VOX Sensitivity changed: step %u -> %u (raw=%u)\n",
                          s_last_raw_pot_step, pot_step, s_audio_in.get_pot_raw());
            s_last_raw_pot_step = pot_step;
            s_current_sensitivity = pot_step;
            s_pot_stable_start = now;
        }
        // Keep system state in sync with current mode/sensitivity (Web UI reads this)
        system_state_set_mode((uint8_t)s_op_mode, s_current_sensitivity);
    }

    // =========================================================================
    // 6. VOX Periodic Status (VOX detector runs synchronously in audio_capture_task)
    // =========================================================================
    if (s_op_mode == OperatingMode::VOX)
    {
        static uint32_t vox_debug_ts = 0;
        if (now - vox_debug_ts >= 2000)
        {
            vox_debug_ts = now;
            uint16_t moving_rms = s_audio_in.get_moving_rms();
            uint16_t threshold = VOX_THRESHOLDS[s_current_sensitivity];
            Serial.printf("[VOX] RMS=%u | Threshold=%u (sens=%u/20) | TX=%s | Pot raw=%u step=%u\n",
                          moving_rms, threshold, s_current_sensitivity,
                          s_pipeline.is_vox_tx_open() ? "OPEN" : "idle",
                          s_audio_in.get_pot_raw(), s_audio_in.get_pot_step());
        }
    }

    // =========================================================================
    // 7. Sync Web UI loopback toggle with AudioPipeline
    // =========================================================================
    bool web_loopback_req = system_state_get_loopback();
    if (web_loopback_req != s_pipeline.is_loopback_active())
    {
        if (web_loopback_req)
        {
            s_pipeline.start_loopback();
            digitalWrite(PIN_LED_RX, HIGH);
        }
        else
        {
            s_pipeline.stop_loopback();
            digitalWrite(PIN_LED_RX, LOW);
        }
    }

    // =========================================================================
    // 8. Periodic Heartbeat Log
    // =========================================================================
    if (!s_is_recording && !s_is_playing && (now - last_heartbeat >= HEARTBEAT_INTERVAL_MS))
    {
        last_heartbeat = now;
        size_t free_heap = esp_get_free_heap_size();
        uint32_t uptime_s = now / 1000;
        system_state_update_metrics(free_heap, uptime_s);
        SystemState st = system_state_get();
        JitterBufferStats js = s_pipeline.get_stats();

        Serial.printf("[HEARTBEAT] Up: %02lu:%02lu:%02lu | Heap: %u B | WiFi: %s | EL: %s | Station: %s (%s) | JB: %u/32 | Under/Drop: %u/%u | Mode: %s | VOX-Sen: %u (pot: raw=%u step=%u)\n",
                      (unsigned long)(uptime_s / 3600),
                      (unsigned long)((uptime_s % 3600) / 60),
                      (unsigned long)(uptime_s % 60),
                      (unsigned int)free_heap,
                      wifi_state_str(st.wifi_state),
                      echolink_state_str(st.echolink_state),
                      station_state_str(st.station_state),
                      strlen(st.connected_callsign) > 0 ? st.connected_callsign : "NONE",
                      (unsigned int)js.current_depth_frames,
                      (unsigned int)js.underflow_count,
                      (unsigned int)js.overflow_count,
                      (s_op_mode == OperatingMode::VOX) ? "VOX" : "PTT",
                      (unsigned int)s_current_sensitivity,
                      (unsigned int)s_audio_in.get_pot_raw(),
                      (unsigned int)s_audio_in.get_pot_step());
    }

    // =========================================================================
    // 9. Serial Command Interpreter
    // =========================================================================
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.equalsIgnoreCase("1") || line.equalsIgnoreCase("tone"))
        {
            run_tone_test(1000.0f, 2000);
        }
        else if (line.equalsIgnoreCase("2") || line.equalsIgnoreCase("mic"))
        {
            run_mic_level_test(3000);
        }
        else if (line.equalsIgnoreCase("3"))
        {
            run_2sec_loopback_test();
        }
        else if (line.equalsIgnoreCase("4") || line.equalsIgnoreCase("loopback"))
        {
            toggle_realtime_loopback();
        }
        else if (line.equalsIgnoreCase("j") || line.equalsIgnoreCase("jitter"))
        {
            print_jitter_stats();
        }
        else if (line.equalsIgnoreCase("c") || line.equalsIgnoreCase("echotest"))
        {
            Serial.println(F("[ECHOLINK] Initiating connection to *ECHOTEST* (#9999)..."));
            echolink_client_connect("*ECHOTEST*");
        }
        else if (line.startsWith("connect ") || line.startsWith("call "))
        {
            String target = line.substring(line.indexOf(' ') + 1);
            target.trim();
            target.toUpperCase();
            Serial.printf("[ECHOLINK] Connecting to '%s'...\n", target.c_str());
            echolink_client_connect(target.c_str());
        }
        else if (line.equalsIgnoreCase("d") || line.equalsIgnoreCase("disconnect") || line.equalsIgnoreCase("drop") || line.equalsIgnoreCase("dropall"))
        {
            dtmf_controller_handle_command("*0#");
        }
        else if (line.startsWith("*") && line.endsWith("#"))
        {
            dtmf_controller_handle_command(line.c_str());
        }
        else if (line.startsWith("dtmf ") || line.equalsIgnoreCase("dtmf"))
        {
            String val = "";
            if (line.length() > 5)
            {
                val = line.substring(5);
                val.trim();
            }

            if (val.equalsIgnoreCase("debug on"))
            {
                dtmf_gate_instance().set_debug(true);
                Serial.println(F("[DTMF] Debug logging ENABLED"));
            }
            else if (val.equalsIgnoreCase("debug off"))
            {
                dtmf_gate_instance().set_debug(false);
                Serial.println(F("[DTMF] Debug logging DISABLED"));
            }
            else if (val.equalsIgnoreCase("stats") || val.equalsIgnoreCase("stat"))
            {
                const DtmfStats &st = dtmf_gate_instance().get_stats();
                Serial.println(F("--- DTMF & VOX Gate Statistics ---"));
                Serial.printf("Frames analyzed:     %lu\n", (unsigned long)st.frames_analyzed);
                Serial.printf("Candidate frames:    %lu\n", (unsigned long)st.candidate_frames);
                Serial.printf("Confirmed digits:    %lu\n", (unsigned long)st.confirmed_digits);
                Serial.printf("VOX force-closes:    %lu\n", (unsigned long)st.vox_force_closes);
                Serial.printf("Commands dispatched: %lu\n", (unsigned long)st.commands_dispatched);
                Serial.printf("Commands rejected:   %lu\n", (unsigned long)st.commands_rejected);
                Serial.printf("Last reject reason:  %s\n", strlen(st.last_reject_reason) > 0 ? st.last_reject_reason : "none");
                Serial.println(F("----------------------------------"));
            }
            else if (val.equalsIgnoreCase("stats reset") || val.equalsIgnoreCase("reset stats"))
            {
                dtmf_gate_instance().reset_stats();
                Serial.println(F("[DTMF] Statistics reset to zero"));
            }
            else
            {
                dtmf_controller_handle_command(val.c_str());
            }
        }
        else if (line.startsWith("add "))
        {
            String val = line.substring(4);
            val.trim();
            char dtmf_cmd[32];
            snprintf(dtmf_cmd, sizeof(dtmf_cmd), "*1%s#", val.c_str());
            dtmf_controller_handle_command(dtmf_cmd);
        }
        else if (line.startsWith("drop "))
        {
            String val = line.substring(5);
            val.trim();
            char dtmf_cmd[32];
            snprintf(dtmf_cmd, sizeof(dtmf_cmd), "*2%s#", val.c_str());
            dtmf_controller_handle_command(dtmf_cmd);
        }
        else if (line.equalsIgnoreCase("status") || line.equalsIgnoreCase("s"))
        {
            dtmf_controller_handle_command("*9#");
        }
        else if (line.equalsIgnoreCase("mode ptt"))
        {
            switch_mode(OperatingMode::PTT);
        }
        else if (line.equalsIgnoreCase("mode vox"))
        {
            switch_mode(OperatingMode::VOX);
        }
        else if (line.startsWith("sensitivity ") || line.startsWith("sens "))
        {
            int step = line.substring(line.indexOf(' ') + 1).toInt();
            set_vox_sensitivity((uint8_t)step);
        }
        else if (line.startsWith("vox preroll") || line.startsWith("vox pre"))
        {
            int sp = line.indexOf(' ', 4);
            if (sp > 0)
            {
                String arg = line.substring(sp + 1);
                arg.trim();
                if (arg.length() > 0 && isDigit(arg.charAt(0)))
                {
                    int val = arg.toInt();
                    if (val >= 0 && val <= 6)
                    {
                        uint8_t frames = (uint8_t)val;
                        config_manager_set_vox_preroll(frames);
                        s_pipeline.set_vox_preroll_delay(frames);
                        Serial.printf("VOX pre-roll: %u frames (%u ms)\n", frames, (unsigned int)(frames * 20));
                    }
                    else
                    {
                        Serial.println(F("[VOX] Invalid pre-roll frames: must be 0..6 (0..120 ms latency)"));
                    }
                }
                else
                {
                    uint8_t cur = s_pipeline.get_vox_preroll_delay();
                    Serial.printf("VOX pre-roll: %u frames (%u ms)\n", cur, (unsigned int)(cur * 20));
                }
            }
            else
            {
                uint8_t cur = s_pipeline.get_vox_preroll_delay();
                Serial.printf("VOX pre-roll: %u frames (%u ms)\n", cur, (unsigned int)(cur * 20));
            }
        }
        else if (line.startsWith("say "))
        {
            String val = line.substring(4);
            val.trim();
            Announcer::instance().say(val.c_str(), ROUTE_LOCAL | ROUTE_TX);
        }
        else if (line.equalsIgnoreCase("abort"))
        {
            Announcer::instance().abort();
        }
        else if (line.equalsIgnoreCase("register") || line.equalsIgnoreCase("logon"))
        {
            Serial.println(F("[ECHOLINK] Triggering immediate addressing server registration..."));
            echolink_client_trigger_registration();
        }
        else if (line.startsWith("set callsign ") || line.startsWith("set call "))
        {
            String val = line.substring(line.lastIndexOf(' ') + 1);
            val.trim();
            val.toUpperCase();
            if (val.length() > 0)
            {
                ConfigData cfg = config_manager_get();
                strncpy(cfg.callsign, val.c_str(), sizeof(cfg.callsign) - 1);
                config_manager_save(cfg);
                Serial.printf("[CONFIG] Callsign set to: %s\n", cfg.callsign);
                echolink_client_trigger_registration();
            }
        }
        else if (line.startsWith("set password ") || line.startsWith("set elpass ") || line.startsWith("set pass "))
        {
            String val = line.substring(line.lastIndexOf(' ') + 1);
            val.trim();
            if (val.length() > 0)
            {
                ConfigData cfg = config_manager_get();
                strncpy(cfg.el_password, val.c_str(), sizeof(cfg.el_password) - 1);
                config_manager_save(cfg);
                Serial.println(F("[CONFIG] EchoLink password updated successfully."));
                echolink_client_trigger_registration();
            }
        }
        else if (line.startsWith("set wifi "))
        {
            String rest = line.substring(9);
            rest.trim();
            int sp = rest.indexOf(' ');
            String ssid = (sp > 0) ? rest.substring(0, sp) : rest;
            String pass = (sp > 0) ? rest.substring(sp + 1) : "";
            ssid.trim();
            pass.trim();
            if (ssid.length() > 0)
            {
                ConfigData cfg = config_manager_get();
                strncpy(cfg.wifi_ssid, ssid.c_str(), sizeof(cfg.wifi_ssid) - 1);
                strncpy(cfg.wifi_pass, pass.c_str(), sizeof(cfg.wifi_pass) - 1);
                config_manager_save(cfg);
                Serial.printf("[CONFIG] WiFi SSID set to: %s. Reconnecting...\n", cfg.wifi_ssid);
                wifi_manager_reconnect();
            }
        }
        else if (line.startsWith("set name "))
        {
            String val = line.substring(9);
            val.trim();
            ConfigData cfg = config_manager_get();
            strncpy(cfg.station_name, val.c_str(), sizeof(cfg.station_name) - 1);
            config_manager_save(cfg);
            Serial.printf("[CONFIG] Station name set to: %s\n", cfg.station_name);
        }
        else if (line.startsWith("set location ") || line.startsWith("set qth "))
        {
            String val = line.substring(line.indexOf(' ') + 1);
            val = val.substring(val.indexOf(' ') + 1);
            val.trim();
            ConfigData cfg = config_manager_get();
            strncpy(cfg.location, val.c_str(), sizeof(cfg.location) - 1);
            config_manager_save(cfg);
            Serial.printf("[CONFIG] Location set to: %s\n", cfg.location);
        }
        else if (line.equalsIgnoreCase("config"))
        {
            ConfigData cfg = config_manager_get();
            Serial.println(F("\n--- Current Stored Configuration ---"));
            Serial.printf("WiFi SSID     : %s\n", cfg.wifi_ssid);
            Serial.printf("WiFi Password : %s\n", mask_secret(cfg.wifi_pass).c_str());
            Serial.printf("Station Call  : %s\n", cfg.callsign);
            Serial.printf("EchoLink Pass : %s\n", mask_secret(cfg.el_password).c_str());
            Serial.printf("Station Name  : %s\n", cfg.station_name);
            Serial.printf("Location/QTH  : %s\n", cfg.location);
            Serial.printf("Web Admin User: %s\n", cfg.web_user);
            Serial.printf("VOX Pre-roll  : %u frames (%u ms)\n", cfg.vox_pre, (unsigned int)(cfg.vox_pre * 20));
            Serial.println(F("------------------------------------\n"));
        }
        else if (line.equalsIgnoreCase("s") || line.equalsIgnoreCase("status"))
        {
            print_status_summary();
        }
        else if (line.equalsIgnoreCase("led"))
        {
            SystemState st = system_state_get();
            LinkInputs in;
            in.apMode = (st.wifi_state == WifiState::AP_MODE);
            in.wifiUp = (st.wifi_state == WifiState::CONNECTED);
            in.regFailed = st.reg_failed;
            in.registered = (st.echolink_state == EchoLinkState::LOGGED_IN);
            in.linked = (st.station_state == StationState::CONNECTED);
            LinkLed l = pickLinkLed(in);
            Serial.printf("LED state: %s (apMode=%d, wifiUp=%d, regFailed=%d, registered=%d, linked=%d)\n",
                          linkLedName(l), (int)in.apMode, (int)in.wifiUp, (int)in.regFailed, (int)in.registered, (int)in.linked);
        }
        else if (line.equalsIgnoreCase("h") || line.equalsIgnoreCase("?") || line.equalsIgnoreCase("help"))
        {
            Serial.println(F("\n--- MicroLink EchoLink Commands ---"));
            Serial.println(F(" 'c'                      - Connect to *ECHOTEST* (Node 9999)"));
            Serial.println(F(" 'connect <station>'      - Connect to callsign or node number"));
            Serial.println(F(" 'd'                      - Disconnect from current station"));
            Serial.println(F(" 'mode ptt'               - Switch to PTT mode"));
            Serial.println(F(" 'mode vox'               - Switch to VOX mode"));
            Serial.println(F(" 'sensitivity <1-20>'     - Set VOX sensitivity level"));
            Serial.println(F(" 'vox preroll <0-6>'      - Set VOX pre-roll frames (0..120 ms latency)"));
            Serial.println(F(" 'say <words>'            - Test announcer (local + TX)"));
            Serial.println(F(" 'abort'                  - Abort current announcement"));
            Serial.println(F(" 'set callsign <CALL>'    - Set EchoLink callsign (e.g. HS1ABC-L)"));
            Serial.println(F(" 'set password <PASS>'    - Set EchoLink password"));
            Serial.println(F(" 'set wifi <SSID> <PASS>' - Set WiFi credentials"));
            Serial.println(F(" 'set name <NAME>'        - Set Sysop / Station name"));
            Serial.println(F(" 'set location <QTH>'     - Set Location / Frequency"));
            Serial.println(F(" 'register'               - Force immediate EchoLink registration"));
            Serial.println(F(" 'config'                 - Print current stored configuration"));
            Serial.println(F(" 'led'                    - Print link status LED state and inputs"));
            Serial.println(F(" '1'                      - Play 1 kHz tone on PCM5102A DAC"));
            Serial.println(F(" '2'                      - Measure KY-038 mic level"));
            Serial.println(F(" '3'                      - 2-second record and playback"));
            Serial.println(F(" '4'                      - Toggle Real-time local loopback"));
            Serial.println(F(" 'dtmf <seq>'             - Execute DTMF command (e.g. *19999#, *0#)"));
            Serial.println(F(" 'dtmf debug on|off'      - Toggle live DTMF detector debug prints"));
            Serial.println(F(" 'dtmf stats'             - Print DTMF & VOX gate counters"));
            Serial.println(F(" 'j'                      - Print Jitter Buffer statistics"));
            Serial.println(F(" 's'                      - Print system status summary"));
            Serial.println(F(" 'h'                      - Show help\n"));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}

/**
 * @brief Background FreeRTOS task handling PTT button with LED feedback
 */
static void status_button_task(void *pvParameters)
{
    (void)pvParameters;

    bool last_raw_button = digitalRead(PIN_BUTTON_PTT);
    bool stable_button_state = last_raw_button;
    uint32_t last_debounce_time = millis();
    uint32_t ptt_down_time = 0;

    for (;;)
    {
        uint32_t now = millis();
        bool current_raw_button = digitalRead(PIN_BUTTON_PTT);

        if (current_raw_button != last_raw_button)
        {
            last_debounce_time = now;
            last_raw_button = current_raw_button;
        }

        if ((now - last_debounce_time) >= BUTTON_DEBOUNCE_MS)
        {
            if (current_raw_button != stable_button_state)
            {
                stable_button_state = current_raw_button;

                if (stable_button_state == LOW)
                {
                    // PTT Pressed -> Transmit active
                    ptt_down_time = now;
                    system_state_set_ptt(true);
                    // Note: PIN_LED_RED now centrally driven by st.tx_active in the LED section below
                    Serial.println(F("\n[PTT BUTTON] >>> PRESSED (TX ACTIVE)"));

                    // Notify EchoLink client
                    echolink_client_set_ptt(true);

                    // If not in continuous loopback and not connected, record for diagnostic test
                    if (!s_pipeline.is_loopback_active() && !echolink_client_is_connected())
                    {
                        s_loopback_recorded_samples = 0;
                        s_is_recording = true;
                    }
                }
                else
                {
                    // PTT Released -> Transmit inactive
                    system_state_set_ptt(false);
                    // Note: PIN_LED_RED now centrally driven by st.tx_active in the LED section below
                    s_is_recording = false;

                    // Notify EchoLink client
                    echolink_client_set_ptt(false);

                    uint32_t held_duration = now - ptt_down_time;
                    Serial.printf("[PTT BUTTON] <<< RELEASED (TX INACTIVE, %lu ms)\n", (unsigned long)held_duration);

                    // If diagnostic test recording was made, play back through DAC
                    if (!s_pipeline.is_loopback_active() && !echolink_client_is_connected() && s_loopback_recorded_samples > 0)
                    {
                        s_is_playing = true;
                        digitalWrite(PIN_LED_RX, HIGH);
                        s_audio_out.start();

                        size_t played = 0;
                        while (played < s_loopback_recorded_samples)
                        {
                            size_t chunk = s_loopback_recorded_samples - played;
                            if (chunk > AUDIO_FRAME_SAMPLES)
                                chunk = AUDIO_FRAME_SAMPLES;
                            size_t written = s_audio_out.write_samples(&s_loopback_buffer[played], chunk, pdMS_TO_TICKS(100));
                            if (written == 0)
                                break;
                            played += written;
                        }

                        digitalWrite(PIN_LED_RX, LOW);
                        s_is_playing = false;
                        Serial.println(F("[PTT BUTTON] Diagnostic playback finished.\n"));
                    }
                }
            }
        }

        // PTT diagnostic recording accumulation (when not in EchoLink session)
        if (s_is_recording && stable_button_state == LOW && !echolink_client_is_connected())
        {
            if (s_loopback_recorded_samples < LOOPBACK_TOTAL_SAMPLES)
            {
                size_t needed = LOOPBACK_TOTAL_SAMPLES - s_loopback_recorded_samples;
                if (needed > AUDIO_FRAME_SAMPLES)
                    needed = AUDIO_FRAME_SAMPLES;
                size_t got = s_audio_in.read_samples(&s_loopback_buffer[s_loopback_recorded_samples], needed, pdMS_TO_TICKS(20));
                s_loopback_recorded_samples += got;
            }
        }

        SystemState st = system_state_get();

        // RED LED: TX indicator — follows tx_active from ANY source (PTT, VOX, Announcement, Web UI)
        // This ensures VOX TX also lights up the Red LED, not just the hardware PTT button
        digitalWrite(PIN_LED_RED, st.tx_active ? HIGH : LOW);

        // RX active LED indicator (GPIO 22): ON when receiving audio, in loopback, or playing tones/recordings
        bool rx_led_active = st.rx_active || s_pipeline.is_loopback_active() || s_is_playing;
        digitalWrite(PIN_LED_RX, rx_led_active ? HIGH : LOW);

        // Link Status Green LED (PIN_LED_GREEN)
        static LedPattern s_link_pattern;
        static LinkLed s_last_link_led = LinkLed::Idle;
        static bool s_link_led_inited = false;
        static int s_last_pin_level = -1;

        bool wifi_connected = (st.wifi_state == WifiState::CONNECTED) && wifi_manager_is_connected();
        if (!wifi_connected && st.wifi_state == WifiState::CONNECTED)
        {
            system_state_set_wifi(WifiState::CONNECTING);
        }

        LinkInputs link_in;
        link_in.apMode = (st.wifi_state == WifiState::AP_MODE);
        link_in.wifiUp = wifi_connected;
        link_in.regFailed = st.reg_failed;
        link_in.registered = (st.echolink_state == EchoLinkState::LOGGED_IN) && wifi_connected;
        link_in.linked = (st.station_state == StationState::CONNECTED) && wifi_connected;

        LinkLed cur_led = pickLinkLed(link_in);
        if (!s_link_led_inited || cur_led != s_last_link_led)
        {
            Serial.printf("LED state: %s\n", linkLedName(cur_led));
            s_last_link_led = cur_led;
            s_link_led_inited = true;
        }

        bool pin_on = s_link_pattern.update(cur_led, now);
        int pin_level = pin_on ? HIGH : LOW;
        if (pin_level != s_last_pin_level)
        {
            digitalWrite(PIN_LED_GREEN, pin_level);
            s_last_pin_level = pin_level;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void run_tone_test(float freq_hz, uint32_t duration_ms)
{
    if (s_is_recording || s_is_playing)
        return;
    s_is_playing = true;
    digitalWrite(PIN_LED_RX, HIGH);

    Serial.printf("\n[TEST 1: TONE] Generating %.0f Hz sine wave via AudioPipeline (%u ms)...\n", freq_hz, (unsigned int)duration_ms);
    digitalWrite(PIN_LED_RX, HIGH);
    s_pipeline.play_tone(freq_hz, duration_ms);
    digitalWrite(PIN_LED_RX, LOW);

    s_is_playing = false;
    Serial.println(F("[TEST 1: TONE] Playback finished.\n"));
}

static void run_mic_level_test(uint32_t duration_ms)
{
    if (s_is_recording || s_is_playing)
        return;
    s_is_recording = true;

    Serial.printf("\n[TEST 2: MIC LEVEL] Measuring KY-038 for %u ms...\n", (unsigned int)duration_ms);
    uint32_t start_ms = millis();

    while (millis() - start_ms < duration_ms)
    {
        uint16_t r_min, r_max, r_avg, vpp;
        s_audio_in.get_metrics(r_min, r_max, r_avg, vpp);

        int bars = (vpp * 25) / 1000;
        if (bars > 25)
            bars = 25;
        char meter[28];
        for (int i = 0; i < 25; ++i)
            meter[i] = (i < bars) ? '=' : ' ';
        meter[25] = '\0';

        Serial.printf("[MIC METER] Raw:[Min:%4u Max:%4u Avg:%4u Vpp:%4u] |%s|\n",
                      r_min, r_max, r_avg, vpp, meter);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    s_is_recording = false;
    Serial.println(F("[TEST 2: MIC LEVEL] Measurement completed.\n"));
}

static void run_2sec_loopback_test()
{
    if (s_is_recording || s_is_playing)
        return;

    Serial.println(F("\n============================================================"));
    Serial.println(F("[TEST 3: LOOPBACK] 2.0-Second Record & Playback..."));
    Serial.println(F("============================================================"));

    s_is_recording = true;
    digitalWrite(PIN_LED_RED, HIGH);
    Serial.println(F("[LOOPBACK] >>> RECORDING 2.0s. Speak now!"));

    size_t samples_recorded = 0;
    uint32_t rec_start_ms = millis();

    while (samples_recorded < LOOPBACK_TOTAL_SAMPLES)
    {
        size_t needed = LOOPBACK_TOTAL_SAMPLES - samples_recorded;
        if (needed > AUDIO_FRAME_SAMPLES)
            needed = AUDIO_FRAME_SAMPLES;

        size_t got = s_audio_in.read_samples(&s_loopback_buffer[samples_recorded], needed, pdMS_TO_TICKS(100));
        if (got == 0)
            break;
        samples_recorded += got;
    }

    digitalWrite(PIN_LED_RED, LOW);
    s_is_recording = false;
    uint32_t rec_duration = millis() - rec_start_ms;

    Serial.printf("[LOOPBACK] >>> Recorded %u samples in %lu ms\n", (unsigned int)samples_recorded, (unsigned long)rec_duration);
    vTaskDelay(pdMS_TO_TICKS(300));

    s_is_playing = true;
    digitalWrite(PIN_LED_RX, HIGH);
    Serial.println(F("[LOOPBACK] <<< PLAYING BACK through PCM5102A DAC..."));

    s_audio_out.start();
    size_t samples_played = 0;
    uint32_t play_start_ms = millis();

    while (samples_played < samples_recorded)
    {
        size_t chunk = samples_recorded - samples_played;
        if (chunk > AUDIO_FRAME_SAMPLES)
            chunk = AUDIO_FRAME_SAMPLES;

        size_t written = s_audio_out.write_samples(&s_loopback_buffer[samples_played], chunk, pdMS_TO_TICKS(100));
        if (written == 0)
            break;
        samples_played += written;
    }

    digitalWrite(PIN_LED_RX, LOW);
    s_is_playing = false;
    uint32_t play_duration = millis() - play_start_ms;

    Serial.printf("[LOOPBACK] <<< Playback finished in %lu ms\n\n", (unsigned long)play_duration);
}

static void toggle_realtime_loopback()
{
    bool current = s_pipeline.is_loopback_active();
    if (!current)
    {
        s_pipeline.start_loopback();
        system_state_set_loopback(true);
        digitalWrite(PIN_LED_RX, HIGH);
        Serial.println(F("\n[PIPELINE] Real-time Loopback ACTIVE. Speak into mic to hear on DAC!"));
        Serial.println(F("[PIPELINE] Audio path: ADC DMA -> JitterBuffer (4-frame watermark) -> I2S DMA -> DAC."));
    }
    else
    {
        s_pipeline.stop_loopback();
        system_state_set_loopback(false);
        digitalWrite(PIN_LED_RX, LOW);
        Serial.println(F("\n[PIPELINE] Real-time Loopback STOPPED."));
        print_jitter_stats();
    }
}

static void print_jitter_stats()
{
    JitterBufferStats js = s_pipeline.get_stats();
    Serial.println(F("\n--- Jitter Buffer Diagnostics ---"));
    Serial.printf("Buffer Capacity   : %u frames (%u ms)\n", JitterBuffer::CAPACITY_FRAMES, JitterBuffer::CAPACITY_FRAMES * 20);
    Serial.printf("Playout Watermark : %u frames (%u ms)\n", js.watermark_frames, js.watermark_frames * 20);
    Serial.printf("Current Depth     : %u frames (%u ms)\n", js.current_depth_frames, js.current_depth_frames * 20);
    Serial.printf("Buffering Status  : %s\n", js.is_buffering ? "BUFFERING (Waiting for watermark)" : "PLAYING");
    Serial.printf("Frames Pushed     : %lu (%lu ms)\n", (unsigned long)js.frames_pushed, (unsigned long)js.frames_pushed * 20);
    Serial.printf("Frames Popped     : %lu (%lu ms)\n", (unsigned long)js.frames_popped, (unsigned long)js.frames_popped * 20);
    Serial.printf("Underflow Count   : %lu (Silence concealment frames inserted)\n", (unsigned long)js.underflow_count);
    Serial.printf("Overflow Count    : %lu (Frames dropped due to buffer full)\n", (unsigned long)js.overflow_count);
    Serial.println(F("---------------------------------\n"));
}

static void print_banner()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    esp_flash_get_size(nullptr, &flash_size);

    ConfigData cfg = config_manager_get();

    Serial.println(F("\n============================================================"));
    Serial.println(F("       MicroLink EchoLink Node Firmware (ESP32-C6)"));
    Serial.println(F("    Phase 5: EchoLink Protocol & GSM Codec Integration"));
    Serial.println(F("============================================================"));
    Serial.printf("ESP-IDF Version   : %s\n", esp_get_idf_version());
    Serial.printf("CPU Cores / Clock : %d Core (RISC-V) @ %lu MHz\n", chip_info.cores, (unsigned long)getCpuFrequencyMhz());
    Serial.printf("Flash Size        : %lu MB\n", (unsigned long)(flash_size / (1024 * 1024)));
    Serial.printf("Audio Rate/Codec  : 8000 Hz Mono, GSM 06.10 Full-Rate (160 samples / 33 bytes)\n");
    Serial.printf("VoIP Framing      : RTP (UDP 5198, 4 frames/144B) & RTCP (UDP 5199)\n");
    Serial.printf("Addressing Server : %s:%d\n", ECHOLINK_DEFAULT_ADDR_SERVER, ECHOLINK_DEFAULT_ADDR_PORT);
    Serial.printf("Station Identity  : %s (%s - %s)\n", cfg.callsign, cfg.station_name, cfg.location);
    Serial.printf("Configured WiFi   : %s\n", strlen(cfg.wifi_ssid) > 0 ? cfg.wifi_ssid : "(SoftAP Mode)");
    Serial.println(F("------------------------------------------------------------"));
}

static void print_status_summary()
{
    SystemState st = system_state_get();
    ConfigData cfg = config_manager_get();
    JitterBufferStats js = s_pipeline.get_stats();

    Serial.println(F("\n--- MicroLink Node Status Report ---"));
    Serial.printf("Station Identity  : %s (%s - %s)\n", cfg.callsign, cfg.station_name, cfg.location);
    Serial.printf("Uptime            : %lu seconds\n", (unsigned long)st.uptime_sec);
    Serial.printf("Free Heap         : %u Bytes (Min: %u Bytes)\n", (unsigned int)st.free_heap, (unsigned int)esp_get_minimum_free_heap_size());
    Serial.printf("WiFi State        : %s (IP: %s, RSSI: %d dBm)\n",
                  wifi_state_str(st.wifi_state), wifi_manager_get_ip().toString().c_str(), wifi_manager_get_rssi());
    Serial.printf("EchoLink Reg      : %s\n", echolink_state_str(st.echolink_state));
    Serial.printf("Connected Station : %s (Callsign: %s, Node: %lu)\n",
                  station_state_str(st.station_state),
                  strlen(st.connected_callsign) > 0 ? st.connected_callsign : "NONE",
                  (unsigned long)st.connected_node);
    Serial.printf("PTT / TX Active   : %s\n", st.tx_active ? "YES (Transmitting)" : "NO");
    Serial.printf("RX Active         : %s\n", st.rx_active ? "YES (Receiving Audio)" : "NO");
    Serial.printf("Jitter Depth      : %u frames (%u ms)\n", js.current_depth_frames, js.current_depth_frames * 20);
    Serial.printf("Under / Overflows : %u / %u\n", js.underflow_count, js.overflow_count);
    Serial.printf("Mic Level         : Raw %u, %d%%\n", st.mic_raw_level, st.mic_level_pct);
    Serial.println(F("------------------------------------\n"));
}
