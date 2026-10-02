/**
 * @file echolink_client.h
 * @brief EchoLink client for ESP32-C6: Addressing server, directory lookup, RTP/RTCP VoIP.
 *
 * MicroLink ESP32-C6 EchoLink Node
 * Adapted from Bruce MacKinnon (KC1FSZ) MicroLink
 * GPL-3.0 License
 */

#pragma once

#include <cstdint>
#include <cstdbool>
#include <cstddef>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize the EchoLink client module.
     * @return true if initialized successfully.
     */
    bool echolink_client_init();

    /**
     * @brief Start the FreeRTOS background task (Priority 3).
     * @return true if task started.
     */
    bool echolink_client_start();

    /**
     * @brief Stop the EchoLink background task and disconnect if active.
     */
    void echolink_client_stop();

    /**
     * @brief Initiate a connection to a remote EchoLink station.
     * @param callsign_or_node Station callsign (e.g. "*ECHOTEST*") or node number (e.g. "9999").
     * @return true if lookup and connection request initiated.
     */
    bool echolink_client_connect(const char *callsign_or_node);

    /**
     * @brief Disconnect from currently connected station.
     */
    void echolink_client_disconnect();

    /**
     * @brief Trigger an immediate registration attempt with the addressing server.
     */
    void echolink_client_trigger_registration();

    /**
     * @brief Check if registered with EchoLink addressing server.
     */
    bool echolink_client_is_registered();

    /**
     * @brief Check if connected to a remote station.
     */
    bool echolink_client_is_connected();

    /**
     * @brief Get currently connected station callsign.
     */
    const char *echolink_client_get_connected_callsign();

    /**
     * @brief Get currently connected station node number.
     */
    uint32_t echolink_client_get_connected_node();

    /**
     * @brief Set PTT state (called by hardware button or web UI).
     * @param active true to begin transmitting, false to stop.
     */
    void echolink_client_set_ptt(bool active);

    /**
     * @brief Check if RF / VoIP transmitter is currently active
     */
    bool echolink_client_is_tx_active();

#ifdef __cplusplus
}

#include "announcer.h"

// C++ TX Arbitration API
bool echolink_client_tx_request(TxSource source);
void echolink_client_tx_release(TxSource source);
void echolink_client_tx_switch_to(TxSource new_source);
void echolink_client_tx_force_off();
TxSource echolink_client_get_tx_source();
void echolink_client_feed_announcement_pcm(const int16_t *samples, size_t count);

#endif
