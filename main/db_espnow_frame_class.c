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
#include <string.h>

#include "db_espnow_frame_class.h"

/**
 * Read the payload_type of a TUNNEL payload, little-endian as MAVLink sends it.
 *
 * A MAVLink v2 sender strips trailing zero bytes, so a short payload means "the rest is zero";
 * reading only what is present and treating the remainder as zero restores that.
 *
 * @param payload TUNNEL payload
 * @param payload_len Valid bytes in payload
 * @return payload_type
 */
static uint16_t db_espnow_tunnel_payload_type(const uint8_t *payload, const uint16_t payload_len) {
    if (payload == NULL) {
        return 0U;
    }
    const uint16_t lo = payload_len > DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_TYPE_OFS ?
                        payload[DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_TYPE_OFS] : 0U;
    const uint16_t hi = payload_len > DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_TYPE_OFS + 1U ?
                        payload[DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_TYPE_OFS + 1U] : 0U;
    return (uint16_t) (lo | (uint16_t) (hi << 8));
}

/**
 * See db_espnow_frame_class.h. Written as two switch statements rather than a table so each
 * list reads exactly like the decision document it comes from.
 *
 * @param dir Direction of travel
 * @param msgid MAVLink message id
 * @param tunnel_payload_type TUNNEL payload_type
 * @return true for a control frame
 */
bool db_espnow_frame_is_control(const db_espnow_link_dir_t dir, const uint32_t msgid,
                                const uint16_t tunnel_payload_type) {
    if (dir == DB_ESPNOW_LINK_GND_TO_AIR) {
        switch (msgid) {
            case DB_MAVLINK_MSG_ID_COMMAND_LONG:
            case DB_MAVLINK_MSG_ID_COMMAND_INT:
            case DB_MAVLINK_MSG_ID_MISSION_COUNT:
            case DB_MAVLINK_MSG_ID_MISSION_ITEM_INT:
            case DB_MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            case DB_MAVLINK_MSG_ID_MISSION_ACK:
            case DB_MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            case DB_MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            case DB_MAVLINK_MSG_ID_PARAM_SET:
            case DB_MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            case DB_MAVLINK_MSG_ID_SET_MODE:
            /* Beyond the decision doc (2026-11-14): starts the GCS's mission pull/verify. */
            case DB_MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                return true;
            case DB_MAVLINK_MSG_ID_TUNNEL:
                /* Only the addressed wrapper. FLEET_LIST is GND->GCS over serial and the link-stats
                 * report is an instrument (Trap 9); neither is a control frame. */
                return tunnel_payload_type == DB_ESPNOW_CLASS_TUNNEL_ADDRESSED_DATA;
            default:
                return false;
        }
    }
    if (dir == DB_ESPNOW_LINK_AIR_TO_GND) {
        switch (msgid) {
            case DB_MAVLINK_MSG_ID_COMMAND_ACK:
            case DB_MAVLINK_MSG_ID_MISSION_ACK:
            case DB_MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            case DB_MAVLINK_MSG_ID_MISSION_ITEM_INT:
            case DB_MAVLINK_MSG_ID_MISSION_COUNT:
            case DB_MAVLINK_MSG_ID_PARAM_VALUE:
            case DB_MAVLINK_MSG_ID_STATUSTEXT:
            /* Beyond the decision doc (2026-11-14): the item request this ArduCopter actually
             * sends during an upload - never MISSION_REQUEST_INT in the 2026-11-12 traces. */
            case DB_MAVLINK_MSG_ID_MISSION_REQUEST:
                return true;
            default:
                return false;
        }
    }
    return false;
}

/**
 * Whether a MAC can be the address of one AIR unit: not a group address (I/G bit of the first
 * octet, which also covers FF:FF:FF:FF:FF:FF) and not all zero.
 *
 * @param mac MAC to test
 * @return true for a plausible individual address
 */
static bool db_espnow_mac_is_individual(const uint8_t *mac) {
    if ((mac[0] & 0x01U) != 0U) {
        return false;
    }
    for (uint8_t i = 0; i < DB_ESPNOW_MAC_ADDR_LEN; i++) {
        if (mac[i] != 0U) {
            return true;
        }
    }
    return false;
}

/**
 * See db_espnow_frame_class.h.
 *
 * @param tunnel_payload TUNNEL payload
 * @param payload_len Valid bytes in tunnel_payload
 * @param mac_out Receives the MAC on success
 * @return true if a well-formed destination was found
 */
bool db_espnow_addressed_data_dest(const uint8_t *tunnel_payload, const uint16_t payload_len,
                                   uint8_t mac_out[DB_ESPNOW_MAC_ADDR_LEN]) {
    if (tunnel_payload == NULL || mac_out == NULL) {
        return false;
    }
    /* The MAC itself must be on the wire. Trailing-zero truncation can only remove it if no
     * inner frame follows, and a frame with no inner frame is malformed anyway. */
    if (payload_len < DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_OFS + DB_ESPNOW_MAC_ADDR_LEN) {
        return false;
    }
    if (db_espnow_tunnel_payload_type(tunnel_payload, payload_len) != DB_ESPNOW_CLASS_TUNNEL_ADDRESSED_DATA) {
        return false;
    }
    /* Same acceptance rule as the AIR side: a frame it would drop is not worth addressing. */
    const uint8_t inner_len = tunnel_payload[DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_LENGTH_OFS];
    if (inner_len <= DB_ESPNOW_MAC_ADDR_LEN || inner_len > DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_MAX) {
        return false;
    }
    const uint8_t *mac = &tunnel_payload[DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_OFS];
    if (!db_espnow_mac_is_individual(mac)) {
        return false;
    }
    memcpy(mac_out, mac, DB_ESPNOW_MAC_ADDR_LEN);
    return true;
}

/**
 * See db_espnow_frame_class.h.
 *
 * @param unicast_mode ESPNOW_UNICAST
 * @param dir Direction of travel
 * @param msgid MAVLink message id
 * @param payload Message payload
 * @param payload_len Valid bytes in payload
 * @param frame_len Whole frame length
 * @param max_alone_len ESP-NOW payload limit
 * @return The routing decision
 */
db_espnow_frame_route_t db_espnow_route_frame(const uint8_t unicast_mode, const db_espnow_link_dir_t dir,
                                              const uint32_t msgid, const uint8_t *payload,
                                              const uint16_t payload_len, const uint16_t frame_len,
                                              const uint16_t max_alone_len) {
    db_espnow_frame_route_t route;
    memset(&route, 0, sizeof(route));
    route.travel_alone = false;
    route.dest_mode = DB_ESPNOW_DEST_AUTO;

    if (unicast_mode != DB_ESPNOW_ADDRESSING_CONTROL) {
        return route;   // modes 0 and 2: classification does not exist
    }
    const uint16_t tunnel_type = msgid == DB_MAVLINK_MSG_ID_TUNNEL ?
                                 db_espnow_tunnel_payload_type(payload, payload_len) : 0U;
    if (!db_espnow_frame_is_control(dir, msgid, tunnel_type)) {
        return route;
    }
    if (frame_len == 0U || frame_len > max_alone_len) {
        return route;
    }
    route.travel_alone = true;
    if (msgid == DB_MAVLINK_MSG_ID_TUNNEL) {
        /* Only ADDRESSED_DATA reaches here (it is the only control TUNNEL, GND->AIR only). The
         * GCS named the drone, so either honour that name or reach every drone - never pick
         * "the single known peer", which would deliver a frame meant for one unit to another
         * whose MAC filter then throws it away. */
        if (db_espnow_addressed_data_dest(payload, payload_len, route.dest_mac)) {
            route.dest_mode = DB_ESPNOW_DEST_PEER;
        } else {
            route.dest_mode = DB_ESPNOW_DEST_BROADCAST;
        }
        return route;
    }
    route.dest_mode = DB_ESPNOW_DEST_CONTROL;
    return route;
}
