#include <iostream>
#include <cassert>
#include <vector>
#include "../include/link_led.h"

// -----------------------------------------------------------------------------
// Test 1: pickLinkLed Priority & Conflict Matrix
// -----------------------------------------------------------------------------
static void testPickLinkLed()
{
    std::cout << "[TEST 1] pickLinkLed Priority and Conflict Matrix... ";

    // Row 1: SoftAP mode overrides everything
    {
        LinkInputs in = {.apMode = true, .wifiUp = true, .regFailed = true, .registered = true, .linked = true};
        assert(pickLinkLed(in) == LinkLed::SetupAp);
    }
    {
        LinkInputs in = {.apMode = true, .wifiUp = false, .regFailed = false, .registered = false, .linked = false};
        assert(pickLinkLed(in) == LinkLed::SetupAp);
    }

    // Row 2: WiFi not connected (and not AP mode) -> WifiConnecting
    {
        LinkInputs in = {.apMode = false, .wifiUp = false, .regFailed = true, .registered = true, .linked = true};
        assert(pickLinkLed(in) == LinkLed::WifiConnecting);
    }
    {
        LinkInputs in = {.apMode = false, .wifiUp = false, .regFailed = false, .registered = false, .linked = false};
        assert(pickLinkLed(in) == LinkLed::WifiConnecting);
    }

    // Row 3: RegFailed overrides registered/linked
    {
        // Conflict: regFailed together with linked -> RegFailed
        LinkInputs in = {.apMode = false, .wifiUp = true, .regFailed = true, .registered = true, .linked = true};
        assert(pickLinkLed(in) == LinkLed::RegFailed);
    }
    {
        LinkInputs in = {.apMode = false, .wifiUp = true, .regFailed = true, .registered = false, .linked = false};
        assert(pickLinkLed(in) == LinkLed::RegFailed);
    }

    // Row 4: In progress registration (wifiUp, not regFailed, not registered)
    {
        LinkInputs in = {.apMode = false, .wifiUp = true, .regFailed = false, .registered = false, .linked = false};
        assert(pickLinkLed(in) == LinkLed::Registering);
    }
    {
        // linked is impossible when not registered, but priority dictates Registering
        LinkInputs in = {.apMode = false, .wifiUp = true, .regFailed = false, .registered = false, .linked = true};
        assert(pickLinkLed(in) == LinkLed::Registering);
    }

    // Row 5: Linked (registered && linked)
    {
        LinkInputs in = {.apMode = false, .wifiUp = true, .regFailed = false, .registered = true, .linked = true};
        assert(pickLinkLed(in) == LinkLed::Linked);
    }

    // Row 6: Idle (registered && !linked)
    {
        LinkInputs in = {.apMode = false, .wifiUp = true, .regFailed = false, .registered = true, .linked = false};
        assert(pickLinkLed(in) == LinkLed::Idle);
    }

    // Verify string names
    assert(std::string(linkLedName(LinkLed::SetupAp)) == "SetupAp");
    assert(std::string(linkLedName(LinkLed::WifiConnecting)) == "WifiConnecting");
    assert(std::string(linkLedName(LinkLed::Registering)) == "Registering");
    assert(std::string(linkLedName(LinkLed::RegFailed)) == "RegFailed");
    assert(std::string(linkLedName(LinkLed::Idle)) == "Idle");
    assert(std::string(linkLedName(LinkLed::Linked)) == "Linked");

    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 2: Pattern Engine Step Boundaries and Periods
// -----------------------------------------------------------------------------
static void testPatternBoundaries()
{
    std::cout << "[TEST 2] Pattern Engine Boundaries & Durations... ";
    LedPattern engine;
    uint32_t t = 1000;

    // --- Linked: Steady ON ---
    assert(engine.update(LinkLed::Linked, t) == true);
    assert(engine.update(LinkLed::Linked, t + 10) == true);
    assert(engine.update(LinkLed::Linked, t + 5000) == true);
    assert(engine.update(LinkLed::Linked, t + 100000) == true);

    // --- SetupAp: 80 ON, 120 OFF, 80 ON, 720 OFF (Period: 1000 ms) ---
    t = 2000;
    assert(engine.update(LinkLed::SetupAp, t) == true);        // t=0: ON
    assert(engine.update(LinkLed::SetupAp, t + 79) == true);   // t=79: ON
    assert(engine.update(LinkLed::SetupAp, t + 80) == false);  // t=80: OFF
    assert(engine.update(LinkLed::SetupAp, t + 199) == false); // t=199: OFF
    assert(engine.update(LinkLed::SetupAp, t + 200) == true);  // t=200: ON (2nd flash)
    assert(engine.update(LinkLed::SetupAp, t + 279) == true);  // t=279: ON
    assert(engine.update(LinkLed::SetupAp, t + 280) == false); // t=280: OFF (long pause)
    assert(engine.update(LinkLed::SetupAp, t + 999) == false); // t=999: OFF
    assert(engine.update(LinkLed::SetupAp, t + 1000) == true); // t=1000: Next cycle starts with ON
    assert(engine.update(LinkLed::SetupAp, t + 1079) == true);
    assert(engine.update(LinkLed::SetupAp, t + 1080) == false);
    // Across 5 full cycles
    assert(engine.update(LinkLed::SetupAp, t + 5000) == true);
    assert(engine.update(LinkLed::SetupAp, t + 5080) == false);

    // --- WifiConnecting: 100 ON, 100 OFF (Period: 200 ms) ---
    t = 10000;
    assert(engine.update(LinkLed::WifiConnecting, t) == true);        // t=0: ON
    assert(engine.update(LinkLed::WifiConnecting, t + 99) == true);   // t=99: ON
    assert(engine.update(LinkLed::WifiConnecting, t + 100) == false); // t=100: OFF
    assert(engine.update(LinkLed::WifiConnecting, t + 199) == false); // t=199: OFF
    assert(engine.update(LinkLed::WifiConnecting, t + 200) == true);  // t=200: Next cycle ON
    assert(engine.update(LinkLed::WifiConnecting, t + 299) == true);
    assert(engine.update(LinkLed::WifiConnecting, t + 300) == false);

    // --- Registering: 500 ON, 500 OFF (Period: 1000 ms) ---
    t = 20000;
    assert(engine.update(LinkLed::Registering, t) == true);        // t=0: ON
    assert(engine.update(LinkLed::Registering, t + 499) == true);  // t=499: ON
    assert(engine.update(LinkLed::Registering, t + 500) == false); // t=500: OFF
    assert(engine.update(LinkLed::Registering, t + 999) == false); // t=999: OFF
    assert(engine.update(LinkLed::Registering, t + 1000) == true); // t=1000: Next cycle ON

    // --- RegFailed: 120 ON, 180 OFF, 120 ON, 180 OFF, 120 ON, 1200 OFF (Period: 1920 ms) ---
    t = 30000;
    assert(engine.update(LinkLed::RegFailed, t) == true); // Flash 1: 0..119 ON
    assert(engine.update(LinkLed::RegFailed, t + 119) == true);
    assert(engine.update(LinkLed::RegFailed, t + 120) == false); // Pause 1: 120..299 OFF
    assert(engine.update(LinkLed::RegFailed, t + 299) == false);
    assert(engine.update(LinkLed::RegFailed, t + 300) == true); // Flash 2: 300..419 ON
    assert(engine.update(LinkLed::RegFailed, t + 419) == true);
    assert(engine.update(LinkLed::RegFailed, t + 420) == false); // Pause 2: 420..599 OFF
    assert(engine.update(LinkLed::RegFailed, t + 599) == false);
    assert(engine.update(LinkLed::RegFailed, t + 600) == true); // Flash 3: 600..719 ON
    assert(engine.update(LinkLed::RegFailed, t + 719) == true);
    assert(engine.update(LinkLed::RegFailed, t + 720) == false); // Long pause: 720..1919 OFF
    assert(engine.update(LinkLed::RegFailed, t + 1919) == false);
    assert(engine.update(LinkLed::RegFailed, t + 1920) == true); // Cycle 2 starts ON

    // --- Idle: 80 ON, 1920 OFF (Period: 2000 ms) ---
    t = 40000;
    assert(engine.update(LinkLed::Idle, t) == true); // Heartbeat ON
    assert(engine.update(LinkLed::Idle, t + 79) == true);
    assert(engine.update(LinkLed::Idle, t + 80) == false); // Heartbeat OFF
    assert(engine.update(LinkLed::Idle, t + 1999) == false);
    assert(engine.update(LinkLed::Idle, t + 2000) == true); // Next heartbeat ON
    assert(engine.update(LinkLed::Idle, t + 2079) == true);
    assert(engine.update(LinkLed::Idle, t + 2080) == false);

    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 3: State Change Mid-Pattern Immediately Restarts at ON
// -----------------------------------------------------------------------------
static void testMidPatternStateChange()
{
    std::cout << "[TEST 3] State Change Mid-Pattern Restarts at ON... ";
    LedPattern engine;
    uint32_t t = 50000;

    // Start in Registering (500 ON, 500 OFF)
    assert(engine.update(LinkLed::Registering, t) == true);
    // At t + 600, it is currently in OFF phase
    assert(engine.update(LinkLed::Registering, t + 600) == false);

    // Mid-pattern switch to Idle at t + 650 (while previous state was OFF)
    // Must immediately restart with ON for Idle's 80 ms flash!
    assert(engine.update(LinkLed::Idle, t + 650) == true);
    assert(engine.update(LinkLed::Idle, t + 650 + 79) == true);
    assert(engine.update(LinkLed::Idle, t + 650 + 80) == false);

    // Mid-pattern switch to WifiConnecting at t + 1000 (while Idle is in its 1920 ms OFF phase)
    assert(engine.update(LinkLed::Idle, t + 1000) == false);
    assert(engine.update(LinkLed::WifiConnecting, t + 1001) == true); // Immediately ON!
    assert(engine.update(LinkLed::WifiConnecting, t + 1001 + 99) == true);
    assert(engine.update(LinkLed::WifiConnecting, t + 1001 + 100) == false);

    std::cout << "PASSED\n";
}

// -----------------------------------------------------------------------------
// Test 4: Wraparound Arithmetic Across 0xFFFFFFFF
// -----------------------------------------------------------------------------
static void testWraparound()
{
    std::cout << "[TEST 4] uint32 Millis Wraparound... ";
    LedPattern engine;

    // Start near 0xFFFFFFFF: e.g. 0xFFFFFF00
    uint32_t t = 0xFFFFFF00;

    // Idle: 80 ms ON, 1920 ms OFF (total period 2000 ms)
    assert(engine.update(LinkLed::Idle, t) == true);
    assert(engine.update(LinkLed::Idle, t + 79) == true);
    assert(engine.update(LinkLed::Idle, t + 80) == false);

    // Cross the 0xFFFFFFFF boundary:
    // 0xFFFFFF00 + 0x0100 wraps to 0x00000000 (after 256 ms)
    // At t + 256 ms (which is 0x00000000), it should still be in OFF phase (80..1999 ms)
    uint32_t t_wrapped = t + 256;
    assert(t_wrapped == 0x00000000);
    assert(engine.update(LinkLed::Idle, t_wrapped) == false);

    // Advance to period end: t + 2000 wraps to 0xFFFFFF00 + 2000 = 0x000006D0
    uint32_t t_next_period = t + 2000;
    assert(engine.update(LinkLed::Idle, t_next_period) == true); // New heartbeat ON
    assert(engine.update(LinkLed::Idle, t_next_period + 79) == true);
    assert(engine.update(LinkLed::Idle, t_next_period + 80) == false);

    // Test SetupAp across wrap
    t = 0xFFFFFF80;                                     // 128 ms before wrap
    assert(engine.update(LinkLed::SetupAp, t) == true); // 0..79 ON
    assert(engine.update(LinkLed::SetupAp, t + 79) == true);
    assert(engine.update(LinkLed::SetupAp, t + 80) == false); // 80..199 OFF
    assert(engine.update(LinkLed::SetupAp, t + 199) == false);
    assert(engine.update(LinkLed::SetupAp, t + 200) == true); // 200..279 ON (wraps across 0)
    assert(engine.update(LinkLed::SetupAp, t + 279) == true);
    assert(engine.update(LinkLed::SetupAp, t + 280) == false);

    std::cout << "PASSED\n";
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "  LinkLed Host Unit Test Suite\n";
    std::cout << "========================================\n";

    testPickLinkLed();
    testPatternBoundaries();
    testMidPatternStateChange();
    testWraparound();

    std::cout << "========================================\n";
    std::cout << "  ALL UNIT TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "========================================\n";
    return 0;
}
