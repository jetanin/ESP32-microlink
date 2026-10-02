#include "dtmf_controller.h"
#include "dtmf_detector.h"
#include "echolink_client.h"
#include "system_state.h"
#include "audio_pipeline.h"
#include "wifi_manager.h"
#include "announcer.h"
#include <cstring>
#include <cctype>

static DTMFController s_controller;

DTMFController::DTMFController()
    : m_buf_len(0),
      m_last_action(DTMFActionType::NONE),
      m_playing_feedback(false)
{
    m_buffer[0] = '\0';
    m_last_command[0] = '\0';
}

void DTMFController::init()
{
    clear_buffer();
    m_last_action = DTMFActionType::NONE;
    m_playing_feedback = false;
    Serial.println(F("[DTMF] DTMF Command Controller initialized."));
}

void DTMFController::clear_buffer()
{
    m_buffer[0] = '\0';
    m_buf_len = 0;
}

void DTMFController::handle_digit(char digit)
{
    if (m_playing_feedback) return;

    Serial.printf("[DTMF] >>> Digit Detected: '%c'\n", digit);

    if (digit == '*')
    {
        // Start of new DTMF command sequence
        m_buffer[0] = '*';
        m_buffer[1] = '\0';
        m_buf_len = 1;
        Serial.println(F("[DTMF] Command started: '*'"));
        return;
    }

    if (m_buf_len > 0 && m_buffer[0] == '*')
    {
        if (digit == '#')
        {
            // End of DTMF command sequence
            m_buffer[m_buf_len] = '#';
            m_buffer[m_buf_len + 1] = '\0';
            m_buf_len++;

            Serial.printf("[DTMF] Complete sequence received: %s\n", m_buffer);
            handle_command(m_buffer);
            clear_buffer();
        }
        else if (isdigit((unsigned char)digit) || (digit >= 'A' && digit <= 'D'))
        {
            if (m_buf_len < sizeof(m_buffer) - 2)
            {
                m_buffer[m_buf_len] = digit;
                m_buffer[m_buf_len + 1] = '\0';
                m_buf_len++;
                Serial.printf("[DTMF] Sequence accumulating: %s\n", m_buffer);
            }
            else
            {
                Serial.println(F("[DTMF] Buffer overflow, sequence reset."));
                clear_buffer();
            }
        }
    }
}

bool DTMFController::handle_command(const char *command_str)
{
    if (!command_str || strlen(command_str) == 0) return false;

    char cmd[32];
    strncpy(cmd, command_str, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';

    // Trim whitespace
    char *p = cmd;
    while (*p == ' ') p++;
    char *end = p + strlen(p) - 1;
    while (end > p && (*end == ' ' || *end == '\r' || *end == '\n')) { *end = '\0'; end--; }

    strncpy(m_last_command, p, sizeof(m_last_command) - 1);
    m_last_command[sizeof(m_last_command) - 1] = '\0';

    DTMFCommand parsed{};
    parsed.action = DTMFActionType::NONE;
    strncpy(parsed.raw_code, p, sizeof(parsed.raw_code) - 1);

    // 1. Check *0# or "dropall" (Disconnect all)
    if (strcmp(p, "*0#") == 0 || strcasecmp(p, "dropall") == 0 || strcasecmp(p, "disconnect all") == 0)
    {
        parsed.action = DTMFActionType::DISCONNECT_ALL;
    }
    // 2. Check *9# or "status" (Announce status)
    else if (strcmp(p, "*9#") == 0 || strcasecmp(p, "status") == 0)
    {
        parsed.action = DTMFActionType::STATUS;
    }
    // 3. Check *1<id># or "add <id>" or "connect <id>" (Connect)
    else if ((strncmp(p, "*1", 2) == 0 && p[strlen(p) - 1] == '#') ||
             strncasecmp(p, "add ", 4) == 0 || strncasecmp(p, "connect ", 8) == 0)
    {
        parsed.action = DTMFActionType::CONNECT;
        if (strncmp(p, "*1", 2) == 0)
        {
            size_t id_len = strlen(p) - 3; // strip '*1' and '#'
            if (id_len > 0 && id_len < sizeof(parsed.target_id))
            {
                strncpy(parsed.target_id, p + 2, id_len);
                parsed.target_id[id_len] = '\0';
            }
        }
        else
        {
            const char *arg = strchr(p, ' ') + 1;
            while (*arg == ' ') arg++;
            strncpy(parsed.target_id, arg, sizeof(parsed.target_id) - 1);
        }
    }
    // 4. Check *2<id># or "drop <id>" or "disconnect <id>" (Disconnect one)
    else if ((strncmp(p, "*2", 2) == 0 && p[strlen(p) - 1] == '#') ||
             strncasecmp(p, "drop ", 5) == 0 || strncasecmp(p, "disconnect ", 11) == 0)
    {
        parsed.action = DTMFActionType::DISCONNECT_ONE;
        if (strncmp(p, "*2", 2) == 0)
        {
            size_t id_len = strlen(p) - 3; // strip '*2' and '#'
            if (id_len > 0 && id_len < sizeof(parsed.target_id))
            {
                strncpy(parsed.target_id, p + 2, id_len);
                parsed.target_id[id_len] = '\0';
            }
        }
        else
        {
            const char *arg = strchr(p, ' ') + 1;
            while (*arg == ' ') arg++;
            strncpy(parsed.target_id, arg, sizeof(parsed.target_id) - 1);
        }
    }

    if (parsed.action != DTMFActionType::NONE)
    {
        m_last_action = parsed.action;
        execute_action(parsed);
        return true;
    }
    else
    {
        Serial.printf("[DTMF] Unknown command format: '%s'\n", p);
        play_feedback(DTMFActionType::NONE, false);
        return false;
    }
}

void DTMFController::execute_action(const DTMFCommand &cmd)
{
    switch (cmd.action)
    {
        case DTMFActionType::CONNECT:
        {
            Serial.printf("[DTMF] >>> ACTION: Connect to node '%s' (MicroLink: add)\n", cmd.target_id);
            bool ok = echolink_client_connect(cmd.target_id);
            play_feedback(DTMFActionType::CONNECT, ok);
            break;
        }

        case DTMFActionType::DISCONNECT_ONE:
        {
            Serial.printf("[DTMF] >>> ACTION: Disconnect node '%s' (MicroLink: drop)\n", cmd.target_id);
            echolink_client_disconnect();
            play_feedback(DTMFActionType::DISCONNECT_ONE, true);
            break;
        }

        case DTMFActionType::DISCONNECT_ALL:
        {
            Serial.println(F("[DTMF] >>> ACTION: Disconnect all stations (MicroLink: dropall)"));
            echolink_client_disconnect();
            Announcer::instance().announceDisconnectedAll();
            play_feedback(DTMFActionType::DISCONNECT_ALL, true);
            break;
        }

        case DTMFActionType::STATUS:
        {
            Serial.println(F("[DTMF] >>> ACTION: Announce status (MicroLink: status)"));
            announce_status();
            break;
        }

        default:
            break;
    }
}

void DTMFController::play_feedback(DTMFActionType action, bool success)
{
    AudioPipeline *pipeline = AudioPipeline::instance();
    if (!pipeline) return;

    m_playing_feedback = true;
    dtmf_detector_set_enabled(false); // Inhibit detector to prevent acoustic self-triggering

    if (!success || action == DTMFActionType::NONE)
    {
        // Low error buzz: 300 Hz for 250 ms
        pipeline->play_tone(300.0f, 250);
    }
    else
    {
        switch (action)
        {
            case DTMFActionType::CONNECT:
                // Ascending chime: 880 Hz (100 ms) -> 1320 Hz (150 ms)
                pipeline->play_tone(880.0f, 100);
                vTaskDelay(pdMS_TO_TICKS(40));
                pipeline->play_tone(1320.0f, 150);
                break;

            case DTMFActionType::DISCONNECT_ONE:
                // Descending chime: 1320 Hz (100 ms) -> 880 Hz (150 ms)
                pipeline->play_tone(1320.0f, 100);
                vTaskDelay(pdMS_TO_TICKS(40));
                pipeline->play_tone(880.0f, 150);
                break;

            case DTMFActionType::DISCONNECT_ALL:
                // Double descending beep: 1000 Hz (80 ms) -> 600 Hz (150 ms)
                pipeline->play_tone(1000.0f, 80);
                vTaskDelay(pdMS_TO_TICKS(40));
                pipeline->play_tone(600.0f, 150);
                break;

            default:
                break;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    dtmf_detector_reset();
    dtmf_detector_set_enabled(true);
    m_playing_feedback = false;
}

void DTMFController::announce_status()
{
    SystemState st = system_state_get();

    // 1. Print formatted status announcement to console
    Serial.println(F("\n================== MICROLINK STATUS =================="));
    Serial.printf("Station State    : %s\n", station_state_str(st.station_state));
    Serial.printf("Connected Station: %s (#%lu)\n",
                  strlen(st.connected_callsign) > 0 ? st.connected_callsign : "NONE",
                  (unsigned long)st.connected_node);
    Serial.printf("WiFi State       : %s (IP: %s)\n",
                  wifi_state_str(st.wifi_state),
                  wifi_manager_get_ip().toString().c_str());
    Serial.printf("EchoLink State   : %s\n", echolink_state_str(st.echolink_state));
    Serial.printf("Transmitter (TX) : %s\n", st.tx_active ? "ACTIVE" : "INACTIVE");
    Serial.printf("Receiver (RX)    : %s\n", st.rx_active ? "ACTIVE" : "INACTIVE");
    Serial.printf("Free Heap        : %u bytes\n", (unsigned int)st.free_heap);
    Serial.printf("Uptime           : %lu sec\n", (unsigned long)st.uptime_sec);
    Serial.println(F("======================================================\n"));

    // 2. Play CW Morse Code Telemetry over DAC
    AudioPipeline *pipeline = AudioPipeline::instance();
    if (!pipeline) return;

    m_playing_feedback = true;
    dtmf_detector_set_enabled(false);

    if (st.station_state == StationState::CONNECTED)
    {
        // Morse Code 'C' (- . - .) for "Connected" @ 800 Hz
        pipeline->play_tone(800.0f, 180); // Dah
        vTaskDelay(pdMS_TO_TICKS(60));
        pipeline->play_tone(800.0f, 60);  // Dit
        vTaskDelay(pdMS_TO_TICKS(60));
        pipeline->play_tone(800.0f, 180); // Dah
        vTaskDelay(pdMS_TO_TICKS(60));
        pipeline->play_tone(800.0f, 60);  // Dit
    }
    else
    {
        // Morse Code 'I' (. .) for "Idle" @ 800 Hz
        pipeline->play_tone(800.0f, 60);  // Dit
        vTaskDelay(pdMS_TO_TICKS(60));
        pipeline->play_tone(800.0f, 60);  // Dit
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    dtmf_detector_reset();
    dtmf_detector_set_enabled(true);
    m_playing_feedback = false;
}

// Global wrappers
void dtmf_controller_init()
{
    s_controller.init();
}

void dtmf_controller_handle_digit(char digit)
{
    s_controller.handle_digit(digit);
}

bool dtmf_controller_handle_command(const char *cmd)
{
    return s_controller.handle_command(cmd);
}

bool dtmf_controller_is_playing_feedback()
{
    return s_controller.is_playing_feedback();
}

const char* dtmf_controller_get_buffer()
{
    return s_controller.get_current_buffer();
}

const char* dtmf_controller_get_last_command()
{
    return s_controller.get_last_command();
}
