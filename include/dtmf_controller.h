#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

/**
 * @file dtmf_controller.h
 * @brief High-Level DTMF Command Controller for EchoLink
 *
 * Supported Commands:
 * DTMF Code    | Action          | MicroLink Equivalent
 * -------------+-----------------+---------------------
 * *1 + id + #  | Connect         | add <id>
 * *2 + id + #  | Disconnect one  | drop <id>
 * *0#          | Disconnect all  | dropall
 * *9#          | Announce status | status
 */

enum class DTMFActionType {
    NONE,
    CONNECT,
    DISCONNECT_ONE,
    DISCONNECT_ALL,
    STATUS
};

struct DTMFCommand {
    DTMFActionType action;
    char target_id[16];
    char raw_code[32];
    bool success;
};

class DTMFController {
public:
    DTMFController();

    void init();
    
    // Process single incoming digit (from audio detector, serial, or web)
    void handle_digit(char digit);

    // Process complete DTMF or text command (e.g. "*19999#", "add 9999", "*0#", "dropall", "*9#", "status")
    bool handle_command(const char *command_str);

    // Reset current command buffer
    void clear_buffer();

    // Check if feedback audio tone is currently playing
    bool is_playing_feedback() const { return m_playing_feedback; }

    const char* get_current_buffer() const { return m_buffer; }
    const char* get_last_command() const { return m_last_command; }
    DTMFActionType get_last_action() const { return m_last_action; }

private:
    void execute_action(const DTMFCommand &cmd);
    void play_feedback(DTMFActionType action, bool success);
    void announce_status();

    char m_buffer[32];
    size_t m_buf_len;
    char m_last_command[32];
    DTMFActionType m_last_action;
    volatile bool m_playing_feedback;
};

// Global instance functions
void dtmf_controller_init();
void dtmf_controller_handle_digit(char digit);
bool dtmf_controller_handle_command(const char *cmd);
bool dtmf_controller_is_playing_feedback();
const char* dtmf_controller_get_buffer();
const char* dtmf_controller_get_last_command();
