#ifndef LINK_LED_H
#define LINK_LED_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief States for the green link status LED.
 */
enum class LinkLed : uint8_t
{
    SetupAp,
    WifiConnecting,
    Registering,
    RegFailed,
    Idle,
    Linked
};

/**
 * @brief Inputs for determining the active LinkLed state.
 */
struct LinkInputs
{
    bool apMode;
    bool wifiUp;
    bool regFailed;
    bool registered;
    bool linked;
};

/**
 * @brief Evaluates link inputs strictly in priority order (top to bottom).
 *
 * 1. in SoftAP / setup mode           -> SetupAp
 * 2. WiFi not connected               -> WifiConnecting
 * 3. registration failed              -> RegFailed
 * 4. not yet registered (in progress) -> Registering
 * 5. connected to a remote station    -> Linked
 * 6. otherwise                        -> Idle
 */
inline LinkLed pickLinkLed(const LinkInputs &in)
{
    if (in.apMode)
    {
        return LinkLed::SetupAp;
    }
    if (!in.wifiUp)
    {
        return LinkLed::WifiConnecting;
    }
    if (in.regFailed)
    {
        return LinkLed::RegFailed;
    }
    if (!in.registered)
    {
        return LinkLed::Registering;
    }
    if (in.linked)
    {
        return LinkLed::Linked;
    }
    return LinkLed::Idle;
}

/**
 * @brief String representation of LinkLed state for logging and API.
 */
inline const char *linkLedName(LinkLed state)
{
    switch (state)
    {
    case LinkLed::SetupAp:
        return "SetupAp";
    case LinkLed::WifiConnecting:
        return "WifiConnecting";
    case LinkLed::Registering:
        return "Registering";
    case LinkLed::RegFailed:
        return "RegFailed";
    case LinkLed::Idle:
        return "Idle";
    case LinkLed::Linked:
        return "Linked";
    default:
        return "Unknown";
    }
}

/**
 * @brief Pattern engine for driving LED blink sequences.
 *
 * Pure C++ class with no external dependencies (no Arduino, no FreeRTOS).
 * Constant-time updates, wraparound-safe time arithmetic, and zero heap.
 */
class LedPattern
{
public:
    LedPattern()
        : m_cur_state(LinkLed::Idle),
          m_has_state(false),
          m_step_start_ms(0),
          m_step_idx(0)
    {
    }

    /**
     * @brief Reset internal pattern state.
     */
    void reset()
    {
        m_has_state = false;
        m_step_start_ms = 0;
        m_step_idx = 0;
    }

    /**
     * @brief Update pattern engine with current state and timestamp.
     *
     * @param state  Target LinkLed state.
     * @param nowMs  Current timestamp in milliseconds.
     * @return true if LED should be ON, false if OFF.
     */
    bool update(LinkLed state, uint32_t nowMs)
    {
        // On state change, restart pattern immediately with LED ON
        if (!m_has_state || state != m_cur_state)
        {
            m_cur_state = state;
            m_has_state = true;
            m_step_start_ms = nowMs;
            m_step_idx = 0;
            return true; // All patterns start with ON
        }

        // Linked is steady ON
        if (state == LinkLed::Linked)
        {
            return true;
        }

        const uint16_t *steps = nullptr;
        size_t count = 0;
        getPattern(state, steps, count);

        if (!steps || count == 0)
        {
            return false;
        }

        // Advance steps using wraparound-safe time comparison
        // Limit iterations to prevent loop lock in case of large time skips
        size_t iter_guard = 0;
        while ((uint32_t)(nowMs - m_step_start_ms) >= steps[m_step_idx])
        {
            m_step_start_ms += steps[m_step_idx];
            m_step_idx = (m_step_idx + 1) % count;
            if (++iter_guard > count * 2)
            {
                // Clock jumped significantly ahead: resync to start of current step
                m_step_start_ms = nowMs;
                break;
            }
        }

        // Patterns start with ON at step 0, then alternate: even = ON, odd = OFF
        return (m_step_idx % 2 == 0);
    }

    /**
     * @brief Current active state in the pattern engine.
     */
    LinkLed getState() const
    {
        return m_cur_state;
    }

private:
    static void getPattern(LinkLed state, const uint16_t *&steps, size_t &count)
    {
        // Pattern durations in ms: ON, OFF, ON, OFF...
        static const uint16_t PATTERN_SETUP_AP[] = {80, 120, 80, 720};
        static const uint16_t PATTERN_WIFI_CONNECTING[] = {100, 100};
        static const uint16_t PATTERN_REGISTERING[] = {500, 500};
        static const uint16_t PATTERN_REG_FAILED[] = {120, 180, 120, 180, 120, 1200};
        static const uint16_t PATTERN_IDLE[] = {80, 1920};

        switch (state)
        {
        case LinkLed::SetupAp:
            steps = PATTERN_SETUP_AP;
            count = sizeof(PATTERN_SETUP_AP) / sizeof(PATTERN_SETUP_AP[0]);
            break;
        case LinkLed::WifiConnecting:
            steps = PATTERN_WIFI_CONNECTING;
            count = sizeof(PATTERN_WIFI_CONNECTING) / sizeof(PATTERN_WIFI_CONNECTING[0]);
            break;
        case LinkLed::Registering:
            steps = PATTERN_REGISTERING;
            count = sizeof(PATTERN_REGISTERING) / sizeof(PATTERN_REGISTERING[0]);
            break;
        case LinkLed::RegFailed:
            steps = PATTERN_REG_FAILED;
            count = sizeof(PATTERN_REG_FAILED) / sizeof(PATTERN_REG_FAILED[0]);
            break;
        case LinkLed::Idle:
            steps = PATTERN_IDLE;
            count = sizeof(PATTERN_IDLE) / sizeof(PATTERN_IDLE[0]);
            break;
        case LinkLed::Linked:
        default:
            steps = nullptr;
            count = 0;
            break;
        }
    }

    LinkLed m_cur_state;
    bool m_has_state;
    uint32_t m_step_start_ms;
    size_t m_step_idx;
};

#endif // LINK_LED_H
