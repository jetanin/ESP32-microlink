#ifndef DTMF_GATE_H
#define DTMF_GATE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void dtmf_controller_clear_buffer();

/**
 * @struct DtmfStats
 * @brief Runtime diagnostics counters since boot for DTMF detection and VOX gating.
 */
struct DtmfStats
{
    uint32_t frames_analyzed;
    uint32_t candidate_frames;
    uint32_t confirmed_digits;
    uint32_t vox_force_closes;
    uint32_t commands_dispatched;
    uint32_t commands_rejected;
    char last_reject_reason[32];
};

/**
 * @enum DtmfGateState
 * @brief 4-State DTMF Gate state machine coordinating VOX inhibition and speech talk-off immunity.
 */
enum class DtmfGateState : uint8_t
{
    None = 0,        // Normal speech/quiescent state (VOX allowed)
    CandidateStrict, // Frame 1 strict candidate (purely internal detector state, VOX unaffected!)
    TonePresent,     // >= 2 consecutive strict frames (~40 ms; VOX force-closed, inhibited, pre-roll erased)
    ConfirmedDigit   // Debounced confirmed digit delivered
};

/**
 * @class DtmfGate
 * @brief Pure C++ gate logic coordinating DTMF detection, command sessions, and VOX inhibition.
 *
 * Rules:
 * 1. Single-frame candidate (CandidateStrict) does NOT affect VOX or TX in any way.
 * 2. Sustained tone (TonePresent, >= 2 frames) force-closes VOX, asserts vox_inhibit,
 *    mutes TX audio, and retroactively erases N=4 frames in the pre-roll delay line.
 * 3. Only '*' starts a command session. Unconfirmed/isolated digits outside session are dropped.
 * 4. Constant-time execution, integer-only time arithmetic, host testable.
 */
class DtmfGate
{
public:
    static constexpr uint8_t PRE_ROLL_N_FRAMES = 4;      // Window (1) + Confirm (2) + Margin (1) = 4 frames (80 ms)
    static constexpr uint32_t SESSION_TIMEOUT_MS = 1500; // Command session timeout after last tone
    static constexpr uint32_t GUARD_TIME_MS = 40;        // Guard time after tone ends (2 frames = 40 ms)

    DtmfGate()
        : m_state(DtmfGateState::None),
          m_session_active(false),
          m_consecutive_strict(0),
          m_consecutive_absent(0),
          m_tone_end_time_ms(0),
          m_last_tone_time_ms(0),
          m_force_close_vox(false),
          m_vox_inhibit(false),
          m_mute_tx(false),
          m_erase_preroll(false),
          m_debug_enabled(false),
          m_stats{0, 0, 0, 0, 0, 0, {0}}
    {
    }

    /**
     * @brief Reset gate state.
     */
    void reset()
    {
        m_state = DtmfGateState::None;
        m_session_active = false;
        m_consecutive_strict = 0;
        m_consecutive_absent = 0;
        m_tone_end_time_ms = 0;
        m_last_tone_time_ms = 0;
        m_force_close_vox = false;
        m_vox_inhibit = false;
        m_mute_tx = false;
        m_erase_preroll = false;
    }

    /**
     * @brief Explicitly terminate active command session.
     */
    void end_session()
    {
        m_session_active = false;
    }

    /**
     * @brief Update gate logic on each captured 20 ms frame.
     *
     * @param candidate_strict   True if frame passed all 7 strict DTMF criteria.
     * @param confirmed_digit    Debounced confirmed digit character, or '\0' if none.
     * @param parser_in_progress True if the command parser currently has buffered digits.
     * @param now_ms             Current timestamp in milliseconds.
     */
    void update(bool candidate_strict, char confirmed_digit, bool parser_in_progress, uint32_t now_ms)
    {
        m_stats.frames_analyzed++;

            if (candidate_strict)
            {
                m_stats.candidate_frames++;
                m_consecutive_strict++;
                m_consecutive_absent = 0;
                m_last_tone_time_ms = now_ms;
            }
            else
            {
                m_consecutive_absent++;
                m_consecutive_strict = 0;
            }

        // State Machine transitions:
        // State 0 -> State 1: candidateStrict (Frame 1)
        // Rule 1: A single-frame candidate must NOT affect VOX or transmit audio in any way!
        if (m_consecutive_strict == 1)
        {
            m_state = DtmfGateState::CandidateStrict;
        }
        // State 1 -> State 2: TonePresent (>= 2 consecutive strict frames)
        // Rule 2: Force-close VOX, assert vox_inhibit, mute TX, and retroactively erase pre-roll
        else if (m_consecutive_strict >= 2)
        {
            m_state = DtmfGateState::TonePresent;
        }
        // Tone absent for >= 2 consecutive frames: exit TonePresent / ConfirmedDigit
        else if (m_consecutive_absent >= 2)
        {
            if (m_state != DtmfGateState::None)
            {
                m_tone_end_time_ms = now_ms;
                m_state = DtmfGateState::None;
            }
        }

        // Pulse force_close_vox and erase pre-roll on TonePresent rising edge (frame 2)
        bool rising_tone_present = (m_consecutive_strict == 2);
        m_force_close_vox = rising_tone_present;
        if (rising_tone_present)
        {
            m_erase_preroll = true;
            m_stats.vox_force_closes++;
        }

        // Confirmed digit handling
        if (confirmed_digit != '\0')
        {
            m_stats.confirmed_digits++;
            m_last_tone_time_ms = now_ms;
            m_state = DtmfGateState::ConfirmedDigit;

            // Rule 3: Only '*' starts a session. Outside session, non-'*' are dropped!
            if (confirmed_digit == '*')
            {
                m_session_active = true;
            }
            else if (m_session_active)
            {
                if (confirmed_digit == '#')
                {
                    m_session_active = false;
                }
                else if (!is_valid_dtmf_char(confirmed_digit))
                {
                    m_session_active = false;
                    record_command_rejected("invalid_key");
                }
            }
        }

        // Session timeout (1.5 s after last tone)
        if (m_session_active)
        {
            if ((uint32_t)(now_ms - m_last_tone_time_ms) >= SESSION_TIMEOUT_MS)
            {
                m_session_active = false;
                dtmf_controller_clear_buffer();
            }
        }

        // Guard time (40 ms = 2 frames) after tone ends
        bool in_guard = (m_tone_end_time_ms > 0) &&
                        (m_state == DtmfGateState::None) &&
                        ((uint32_t)(now_ms - m_tone_end_time_ms) < GUARD_TIME_MS);

        bool tone_active = (m_state == DtmfGateState::TonePresent || m_state == DtmfGateState::ConfirmedDigit);
        m_vox_inhibit = tone_active || m_session_active || parser_in_progress || in_guard;
        m_mute_tx = tone_active || in_guard;
    }

    /**
     * @brief Determine if confirmed digit should be delivered to the command parser.
     * Rule 3: Only '*' starts a session. Digits outside session that are not '*' are dropped.
     */
    char get_digit_for_parser(char confirmed_digit) const
    {
        if (confirmed_digit == '\0')
            return '\0';
        if (confirmed_digit == '*')
            return '*';
        if (confirmed_digit == '#')
            return '#';
        if (m_session_active)
            return confirmed_digit;
        return '\0'; // Dropped outside session
    }

    bool should_force_close_vox() const { return m_force_close_vox; }
    void clear_force_close_vox() { m_force_close_vox = false; }
    bool is_vox_inhibited() const { return m_vox_inhibit; }
    bool should_mute_tx() const { return m_mute_tx; }
    bool is_session_active() const { return m_session_active; }

    bool should_erase_preroll() const { return m_erase_preroll; }
    void clear_erase_preroll() { m_erase_preroll = false; }

    DtmfGateState get_state() const { return m_state; }

    // Diagnostics & stats
    const DtmfStats &get_stats() const { return m_stats; }
    void reset_stats() { memset(&m_stats, 0, sizeof(m_stats)); }

    void record_command_dispatched()
    {
        m_stats.commands_dispatched++;
    }

    void record_command_rejected(const char *reason)
    {
        m_stats.commands_rejected++;
        if (reason)
        {
            strncpy(m_stats.last_reject_reason, reason, sizeof(m_stats.last_reject_reason) - 1);
            m_stats.last_reject_reason[sizeof(m_stats.last_reject_reason) - 1] = '\0';
        }
    }

    void set_debug(bool enable) { m_debug_enabled = enable; }
    bool is_debug() const { return m_debug_enabled; }

private:
    static bool is_valid_dtmf_char(char c)
    {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'D') || c == '*' || c == '#';
    }

    DtmfGateState m_state;
    bool m_session_active;
    uint8_t m_consecutive_strict;
    uint8_t m_consecutive_absent;
    uint32_t m_tone_end_time_ms;
    uint32_t m_last_tone_time_ms;

    bool m_force_close_vox;
    bool m_vox_inhibit;
    bool m_mute_tx;
    bool m_erase_preroll;

    bool m_debug_enabled;
    DtmfStats m_stats;
};

// Global singleton accessor
DtmfGate &dtmf_gate_instance();

#endif // DTMF_GATE_H
