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

#include <stddef.h>

#include "db_espnow_tx_path.h"

/**
 * Whether a MAC is the all-ones broadcast address.
 *
 * @param mac MAC to test
 * @return true for FF:FF:FF:FF:FF:FF
 */
static bool db_espnow_mac_is_broadcast(const uint8_t *mac) {
    for (uint8_t i = 0; i < DB_ESPNOW_MAC_ADDR_LEN; i++) {
        if (mac[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param unicast_enabled Addressing enabled by ESPNOW_UNICAST
 * @param is_air AIR role
 * @param gnd_mac_known AIR knows its GND
 * @param gnd_mac The GND MAC
 * @param peer_count GND's number of known peers
 * @param first_peer_mac GND's first peer
 * @return Candidate MAC or NULL
 */
const uint8_t *db_espnow_auto_target_candidate(const bool unicast_enabled, const bool is_air,
                                               const bool gnd_mac_known, const uint8_t *gnd_mac,
                                               const uint8_t peer_count, const uint8_t *first_peer_mac) {
    if (!unicast_enabled) {
        return NULL;
    }
    if (is_air) {
        return gnd_mac_known ? gnd_mac : NULL;
    }
    return peer_count == 1U ? first_peer_mac : NULL;
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param mode Requested routing
 * @param dest_mac Event's MAC
 * @param dest_registered dest_mac is a registered peer
 * @param unicast_enabled Addressing enabled by ESPNOW_UNICAST
 * @param auto_target DB_ESPNOW_DEST_AUTO's answer
 * @return Destination MAC or NULL for broadcast
 */
const uint8_t *db_espnow_resolve_destination(const db_espnow_dest_mode_t mode, const uint8_t *dest_mac,
                                             const bool dest_registered, const bool unicast_enabled,
                                             const uint8_t *auto_target) {
    switch (mode) {
        case DB_ESPNOW_DEST_BROADCAST:
            return NULL;
        case DB_ESPNOW_DEST_PEER:
            /* A MAC that is not registered cannot be addressed (esp_now_send() would fail
             * with ESP_ERR_ESPNOW_NOT_FOUND); falling back is "no peer" fallback, which the
             * design allows - it is not a fallback on send failure, which it forbids. */
            if (unicast_enabled && dest_mac != NULL && dest_registered && !db_espnow_mac_is_broadcast(dest_mac)) {
                return dest_mac;
            }
            break;
        case DB_ESPNOW_DEST_AUTO:
        default:
            break;
    }
    /* auto_target is computed with the same unicast_enabled, so it is NULL in mode 0; the
     * explicit check keeps that true even if a caller passes something else. */
    return unicast_enabled ? auto_target : NULL;
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param unicast_mode ESPNOW_UNICAST
 * @param mode Requested routing
 * @param dest_mac Event's MAC
 * @param dest_registered dest_mac is a registered peer
 * @param single_peer The role's single known registered peer, or NULL
 * @return Destination MAC or NULL for broadcast
 */
const uint8_t *db_espnow_resolve_destination_for_mode(const uint8_t unicast_mode, const db_espnow_dest_mode_t mode,
                                                      const uint8_t *dest_mac, const bool dest_registered,
                                                      const uint8_t *single_peer) {
    switch (unicast_mode) {
        case DB_ESPNOW_ADDRESSING_ALL:
            /* Phase A's rule, untouched. CONTROL is not a value it knows, and its default branch
             * routes unknown requests like AUTO - which is exactly "single peer, else broadcast". */
            return db_espnow_resolve_destination(mode, dest_mac, dest_registered, true, single_peer);
        case DB_ESPNOW_ADDRESSING_CONTROL:
            switch (mode) {
                case DB_ESPNOW_DEST_CONTROL:
                    return single_peer;
                case DB_ESPNOW_DEST_PEER:
                    if (dest_mac != NULL && dest_registered && !db_espnow_mac_is_broadcast(dest_mac)) {
                        return dest_mac;
                    }
                    /* No-peer fallback (allowed), not send-failure fallback (forbidden). Not the
                     * single known peer: the frame names a different unit. */
                    return NULL;
                case DB_ESPNOW_DEST_AUTO:
                case DB_ESPNOW_DEST_BROADCAST:
                default:
                    /* Telemetry, heartbeats, RADIO_STATUS, the peer-list frame, link stats:
                     * one broadcast slot serves the fleet, and an instrument must not sit
                     * behind retries (Traps 1 and 9). */
                    return NULL;
            }
        case DB_ESPNOW_ADDRESSING_OFF:
        default:
            return NULL;
    }
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param tracker Tracker to initialise
 */
void db_espnow_tx_tracker_init(db_espnow_tx_tracker_t *tracker) {
    if (tracker == NULL) {
        return;
    }
    tracker->in_flight = false;
    tracker->addressed = false;
    tracker->sent_at_ms = 0;
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param tracker Tracker
 * @return true when nothing is in flight
 */
bool db_espnow_tx_ready(const db_espnow_tx_tracker_t *tracker) {
    return tracker != NULL && !tracker->in_flight;
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param tracker Tracker
 * @param addressed Sent to a peer MAC
 * @param now_ms Current time
 * @param counters Counters to update
 */
void db_espnow_tx_on_sent(db_espnow_tx_tracker_t *tracker, const bool addressed, const uint32_t now_ms,
                          db_link_stats_counters_t *counters) {
    if (tracker == NULL || counters == NULL) {
        return;
    }
    tracker->in_flight = true;
    tracker->addressed = addressed;
    tracker->sent_at_ms = now_ms;
    db_link_stats_inc(addressed ? &counters->sent_addressed : &counters->sent_broadcast);
}

/**
 * See db_espnow_tx_path.h.
 *
 * @param tracker Tracker
 * @param success Callback reported ESP_NOW_SEND_SUCCESS
 * @param counters Counters to update
 * @return true if the in-flight packet completed
 */
bool db_espnow_tx_on_send_cb(db_espnow_tx_tracker_t *tracker, const bool success, db_link_stats_counters_t *counters) {
    if (tracker == NULL || counters == NULL) {
        return false;
    }
    if (!tracker->in_flight) {
        db_link_stats_inc(&counters->send_cb_late);
        return false;
    }
    tracker->in_flight = false;
    if (tracker->addressed) {
        db_link_stats_inc(success ? &counters->addressed_delivered : &counters->addressed_failed);
    } else if (!success) {
        db_link_stats_inc(&counters->broadcast_failed);
    }
    return true;
}

/**
 * See db_espnow_tx_path.h. The elapsed time is computed in unsigned arithmetic so a wrap of
 * the millisecond clock (every ~49.7 days) cannot produce a negative age.
 *
 * @param tracker Tracker
 * @param now_ms Current time
 * @param timeout_ms Deadline
 * @param counters Counters to update
 * @return true if the in-flight packet was written off
 */
bool db_espnow_tx_check_timeout(db_espnow_tx_tracker_t *tracker, const uint32_t now_ms, const uint32_t timeout_ms,
                                db_link_stats_counters_t *counters) {
    if (tracker == NULL || counters == NULL || !tracker->in_flight) {
        return false;
    }
    if ((uint32_t) (now_ms - tracker->sent_at_ms) < timeout_ms) {
        return false;
    }
    tracker->in_flight = false;
    db_link_stats_inc(&counters->send_cb_timeout);
    return true;
}
