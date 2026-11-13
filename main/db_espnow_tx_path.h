/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2026 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

/*
 * Decision logic of the ESP-NOW transmit path, kept free of ESP-IDF so the host tests can
 * pin it: which MAC a packet goes to, and the single-flight state that decides when the next
 * packet may be handed to the radio. db_esp_now.c owns the radio calls and feeds this module
 * the facts it needs (is the peer registered, what time is it).
 */

#ifndef DB_ESP32_DB_ESPNOW_TX_PATH_H
#define DB_ESP32_DB_ESPNOW_TX_PATH_H

#include <stdbool.h>
#include <stdint.h>

#include "db_espnow_limits.h"
#include "db_espnow_link_stats.h"

/**
 * How the ESP-NOW task picks the destination of one queued packet.
 *
 * Stored in a uint8_t inside db_espnow_queue_event_t; values are internal to one unit and
 * never go on air.
 */
typedef enum {
    /** Today's behaviour, and what every pre-existing caller gets: the role's single known
     *  peer when ESPNOW_UNICAST asks for addressing (db_espnow_unicast_target()), else
     *  broadcast. */
    DB_ESPNOW_DEST_AUTO = 0,
    /** Broadcast, whatever ESPNOW_UNICAST says. For measuring instruments that must not sit
     *  behind MAC retries (decision doc, Trap 9) - the link-stats report uses it. */
    DB_ESPNOW_DEST_BROADCAST = 1,
    /** Addressed to the MAC carried in the event - used only if addressing is enabled
     *  (ESPNOW_UNICAST != 0) and that MAC is a registered ESP-NOW peer; otherwise exactly
     *  DB_ESPNOW_DEST_AUTO. Nothing sets this yet; phase B's per-message classification will. */
    DB_ESPNOW_DEST_PEER = 2,
} db_espnow_dest_mode_t;

/**
 * The address today's DB_ESPNOW_DEST_AUTO rule would pick, before the "is it a registered
 * peer" check (which needs ESP-NOW and stays with the caller).
 *
 * AIR: the learnt GND MAC. GND: the one AIR peer, only when exactly one is known - with
 * several, a single-flight path cannot fan out and a broadcast reaches them all in one slot.
 *
 * @param unicast_enabled ESPNOW_UNICAST != DB_ESPNOW_UNICAST_OFF
 * @param is_air true in the ESP-NOW AIR role (beacon included), false on the GND
 * @param gnd_mac_known true once the AIR has learnt its GND's MAC
 * @param gnd_mac The learnt GND MAC (may be NULL when unknown)
 * @param peer_count Number of AIR peers the GND knows
 * @param first_peer_mac MAC of the GND's first peer (may be NULL when peer_count is 0)
 * @return The candidate MAC, or NULL meaning broadcast
 */
const uint8_t *db_espnow_auto_target_candidate(bool unicast_enabled, bool is_air, bool gnd_mac_known,
                                               const uint8_t *gnd_mac, uint8_t peer_count,
                                               const uint8_t *first_peer_mac);

/**
 * Final destination of one packet.
 *
 * Never returns an address in mode 0 (ESPNOW_UNICAST off) - not even for an explicit
 * DB_ESPNOW_DEST_PEER - so "mode 0 broadcasts everything" stays true by construction rather
 * than by every future caller remembering it.
 *
 * @param mode How the packet asked to be routed
 * @param dest_mac MAC carried by the event (only read for DB_ESPNOW_DEST_PEER; may be NULL)
 * @param dest_registered Whether dest_mac is currently a registered ESP-NOW peer
 * @param unicast_enabled ESPNOW_UNICAST != DB_ESPNOW_UNICAST_OFF
 * @param auto_target What DB_ESPNOW_DEST_AUTO resolves to right now (already checked to be a
 *                    registered peer by the caller), or NULL for broadcast
 * @return The destination MAC, or NULL meaning broadcast
 */
const uint8_t *db_espnow_resolve_destination(db_espnow_dest_mode_t mode, const uint8_t *dest_mac,
                                             bool dest_registered, bool unicast_enabled,
                                             const uint8_t *auto_target);

/**
 * Single-flight state of the send path.
 *
 * Packets are handed to the radio one at a time and the next only after the previous one's
 * send callback, because ESP-NOW does not guarantee callback order for packets sent in quick
 * succession (Espressif's esp_now.rst) and MAVLink frames split across packets must arrive in
 * order. Upstream held this in a bare `ready_to_send` flag that only a send-callback event
 * could set again, so one lost event wedged transmission for good. This adds a deadline.
 */
typedef struct {
    bool in_flight;         // a packet was handed to the radio and its callback has not come
    bool addressed;         // that packet went to a peer MAC (true) or broadcast (false)
    uint32_t sent_at_ms;    // when it was handed over, for the deadline
} db_espnow_tx_tracker_t;

/**
 * Reset to "nothing in flight".
 *
 * @param tracker Tracker to initialise
 */
void db_espnow_tx_tracker_init(db_espnow_tx_tracker_t *tracker);

/**
 * Whether the next packet may be handed to the radio.
 *
 * @param tracker Tracker
 * @return true when nothing is in flight
 */
bool db_espnow_tx_ready(const db_espnow_tx_tracker_t *tracker);

/**
 * Record that esp_now_send() accepted a packet, and count it as sent.
 *
 * Called at the same point the ESP-NOW sequence number is advanced, which is what makes
 * sent_broadcast + sent_addressed the peer's sequence-gap denominator.
 *
 * @param tracker Tracker
 * @param addressed true if the packet went to a peer MAC, false if broadcast
 * @param now_ms Current time in ms (any monotonic base; wrap-safe)
 * @param counters Counters to update
 */
void db_espnow_tx_on_sent(db_espnow_tx_tracker_t *tracker, bool addressed, uint32_t now_ms,
                          db_link_stats_counters_t *counters);

/**
 * Record a send callback.
 *
 * A callback with nothing in flight belongs to a packet already written off by
 * db_espnow_tx_check_timeout(); it is counted as late and otherwise ignored, so it can never
 * release a second packet while one is genuinely in flight from the caller's point of view.
 *
 * @param tracker Tracker
 * @param success status == ESP_NOW_SEND_SUCCESS
 * @param counters Counters to update
 * @return true if this completed the in-flight packet (the caller may send the next one),
 *         false if it was a late callback
 */
bool db_espnow_tx_on_send_cb(db_espnow_tx_tracker_t *tracker, bool success, db_link_stats_counters_t *counters);

/**
 * Write off an in-flight packet whose callback is overdue.
 *
 * @param tracker Tracker
 * @param now_ms Current time in ms, same base as db_espnow_tx_on_sent()
 * @param timeout_ms Deadline after the send
 * @param counters Counters to update
 * @return true if the packet was written off (sending may resume), false otherwise
 */
bool db_espnow_tx_check_timeout(db_espnow_tx_tracker_t *tracker, uint32_t now_ms, uint32_t timeout_ms,
                                db_link_stats_counters_t *counters);

#endif // DB_ESP32_DB_ESPNOW_TX_PATH_H
