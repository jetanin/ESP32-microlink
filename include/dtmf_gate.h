#ifndef DTMF_GATE_H
#define DTMF_GATE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
 * @class DtmfGate
 * @brief Pure C++ gate logic coordinating DTMF detection, command sessions, and VOX inhibition.
 *
 * Requirements:
 * - Runs every 20 ms frame in the audio capture task.
 * - Integer-only arithmetic, constant-time execution, no dynamic allocation, host testable.
 * - Wraparound-safe time arithmetic: (uint32_t)(now - t0) >= duration.
 * - Closes VOX immediately when a DTMF tone candidate is detected (no waiting for hang or attack time).
 * - Manages a 1.5 s command session so inter-digit pauses do not reopen VOX.
 */
class DtmfGate
{
public:
    static constexpr uint32_t SESSION_TIMEOUT_MS = 1500; // Command session timeout after last tone
    static constexpr uint32_t CANDIDATE_GUARD_MS = 100;  // Guard time after tone candidate ends

    DtmfGate()
        : m_session_active(false),
          m_last_tone_time_ms(0),
          m_candidate_end_time_ms(0),
          m_prev_candidate(false),
          m_force_close_vox(false),
          m_vox_inhibit(false),
          m_mute_tx(false),
          m_debug_enabled(false),
          m_stats{0, 0, 0, 0, 0, 0, {0}}
    {
    }

    /**
     * @brief Reset gate state.
     */
    void reset()
    {
        m_session_active = false;
        m_last_tone_time_ms = 0;
        m_candidate_end_time_ms = 0;
        m_prev_candidate = false;
        m_force_close_vox = false;
        m_vox_inhibit = false;
        m_mute_tx = false;
    }

    /**
     * @brief Explicitly terminate active command session (e.g. on '#', invalid key, or command execution).
     */
    void end_session()
    {
        m_session_active = false;
    }

    /**
     * @brief Update gate logic on each captured 20 ms frame.
     *
     * @param candidate          True if a 1-frame DTMF tone pair is present.
     * @param confirmed_digit    Confirmed debounced digit character, or '\0' if none.
     * @param parser_in_progress True if the command parser currently has buffered digits.
     * @param now_ms             Current timestamp in milliseconds.
     */
    void update(bool candidate, char confirmed_digit, bool parser_in_progress, uint32_t now_ms)
    {
        m_stats.frames_analyzed++;

        if (candidate)
        {
            m_stats.candidate_frames++;
            m_last_tone_time_ms = now_ms;
            m_candidate_end_time_ms = now_ms;
        }

        if (confirmed_digit != '\0')
        {
            m_stats.confirmed_digits++;
            m_last_tone_time_ms = now_ms;

            if (confirmed_digit == '*')
            {
                // Command session begins on confirmed '*'
                m_session_active = true;
            }
            else if (m_session_active)
            {
                if (confirmed_digit == '#')
                {
                    // Command sequence complete
                    m_session_active = false;
                }
                else if (is_valid_dtmf_char(confirmed_digit))
                {
                    // Valid alphanumeric DTMF continuation ('0'-'9', 'A'-'D')
                }
                else
                {
                    // Invalid/unexpected digit terminates session
                    m_session_active = false;
                    record_command_rejected("invalid_key");
                }
            }
        }

        // Session timeout: 1.5 s with no new tone
        if (m_session_active)
        {
            if ((uint32_t)(now_ms - m_last_tone_time_ms) >= SESSION_TIMEOUT_MS)
            {
                m_session_active = false;
            }
            // If parser has already been cleared or reset externally and candidate has cleared
            if (!parser_in_progress && (confirmed_digit == '\0') && !candidate &&
                ((uint32_t)(now_ms - m_candidate_end_time_ms) >= CANDIDATE_GUARD_MS))
            {
                m_session_active = false;
            }
        }

        // Guard window: 100 ms after tone ends to prevent VOX re-opening on speech echo / edge
        bool candidate_guard_active = !candidate &&
                                      ((uint32_t)(now_ms - m_candidate_end_time_ms) < CANDIDATE_GUARD_MS);

        bool tone_active_or_guarded = candidate || candidate_guard_active;

        // VOX is inhibited and TX is muted whenever a tone is present, within guard window, or during session
        m_vox_inhibit = tone_active_or_guarded || m_session_active || parser_in_progress;
        m_mute_tx = m_vox_inhibit;

        // Force close VOX if tone is active or session is active
        m_force_close_vox = m_vox_inhibit;

        if (m_force_close_vox && (candidate && !m_prev_candidate))
        {
            m_stats.vox_force_closes++;
        }

        m_prev_candidate = candidate;
    }

    bool should_force_close_vox() const { return m_force_close_vox; }
    bool is_vox_inhibited() const { return m_vox_inhibit; }
    bool should_mute_tx() const { return m_mute_tx; }
    bool is_session_active() const { return m_session_active; }

    /**
     * @brief Check whether candidate has a rising edge this frame.
     * Used to mute the pre-roll delay line so the onset of the tone is not transmitted.
     */
    bool is_candidate_rising(bool candidate) const
    {
        return candidate && !m_prev_candidate;
    }

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

    bool m_session_active;
    uint32_t m_last_tone_time_ms;
    uint32_t m_candidate_end_time_ms;
    bool m_prev_candidate;

    bool m_force_close_vox;
    bool m_vox_inhibit;
    bool m_mute_tx;

    bool m_debug_enabled;
    DtmfStats m_stats;
};

// Global singleton accessor
DtmfGate &dtmf_gate_instance();

#endif // DTMF_GATE_H
