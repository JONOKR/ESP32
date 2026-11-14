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
 * Which MAVLink frames are "control frames" - the low-rate frames whose loss changes what the
 * aircraft or the operator does - and how one such frame is routed in ESPNOW_UNICAST mode 1.
 *
 * Specification: docs/decisions/2026-11-10-link-reliability-architecture.md, Layer 2 and
 * implementation Steps 4 and 5. The allowlists below are that document's lists plus two
 * additions; an addition costs an airtime slot the whole fleet shares, so the bar for joining is
 * "does losing exactly one of these change behaviour?".
 *
 * ADDITIONS BEYOND THE DECISION DOC (2026-11-14), so the divergence stays visible here until the
 * document is amended:
 *  - MISSION_REQUEST (40), AIR->GND. In the 2026-11-12 mission-upload traces this ArduCopter
 *    answered MISSION_COUNT with MISSION_REQUEST - 25 per upload - and never with
 *    MISSION_REQUEST_INT (51), the only item request the doc lists. Without it, every item
 *    request of a mission upload would still travel by broadcast in mode 1.
 *  - MISSION_REQUEST_LIST (43), GND->AIR. The GCS sends it to pull or verify a mission
 *    (GroundControl mission_upload.py, _send_mission_request_list); it starts that handshake and
 *    the doc's list omitted it.
 * MISSION_ITEM (39) is deliberately NOT added: nothing in this system sends it.
 *
 * Pure C, free of ESP-IDF and fastmavlink, so the host tests can pin every list entry. The
 * caller (db_serial.c) does the MAVLink parsing and hands over the message id and, for TUNNEL,
 * the raw message payload.
 */

#ifndef DB_ESP32_DB_ESPNOW_FRAME_CLASS_H
#define DB_ESP32_DB_ESPNOW_FRAME_CLASS_H

#include <stdbool.h>
#include <stdint.h>

#include "db_espnow_limits.h"
#include "db_espnow_tx_path.h"

/*
 * MAVLink message ids the classifier names. Literals rather than fastmavlink's
 * FASTMAVLINK_MSG_ID_* so this module stays free of the generated library; the host test
 * asserts each one against fastmavlink so a typo cannot survive.
 */
#define DB_MAVLINK_MSG_ID_HEARTBEAT            0U
#define DB_MAVLINK_MSG_ID_SYS_STATUS           1U
#define DB_MAVLINK_MSG_ID_SET_MODE             11U
#define DB_MAVLINK_MSG_ID_PARAM_REQUEST_READ   20U
#define DB_MAVLINK_MSG_ID_PARAM_VALUE          22U
#define DB_MAVLINK_MSG_ID_PARAM_SET            23U
#define DB_MAVLINK_MSG_ID_GPS_RAW_INT          24U
#define DB_MAVLINK_MSG_ID_SCALED_IMU           26U
#define DB_MAVLINK_MSG_ID_ATTITUDE             30U
#define DB_MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33U
#define DB_MAVLINK_MSG_ID_MISSION_REQUEST      40U
#define DB_MAVLINK_MSG_ID_MISSION_SET_CURRENT  41U
#define DB_MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43U
#define DB_MAVLINK_MSG_ID_MISSION_COUNT        44U
#define DB_MAVLINK_MSG_ID_MISSION_CLEAR_ALL    45U
#define DB_MAVLINK_MSG_ID_MISSION_ACK          47U
#define DB_MAVLINK_MSG_ID_MISSION_REQUEST_INT  51U
#define DB_MAVLINK_MSG_ID_MISSION_ITEM_INT     73U
#define DB_MAVLINK_MSG_ID_VFR_HUD              74U
#define DB_MAVLINK_MSG_ID_COMMAND_INT          75U
#define DB_MAVLINK_MSG_ID_COMMAND_LONG         76U
#define DB_MAVLINK_MSG_ID_COMMAND_ACK          77U
#define DB_MAVLINK_MSG_ID_RADIO_STATUS         109U
#define DB_MAVLINK_MSG_ID_SCALED_IMU2          116U
#define DB_MAVLINK_MSG_ID_SCALED_IMU3          129U
#define DB_MAVLINK_MSG_ID_BATTERY_STATUS       147U
#define DB_MAVLINK_MSG_ID_STATUSTEXT           253U
#define DB_MAVLINK_MSG_ID_TUNNEL               385U

/*
 * DroneBridge TUNNEL payload types. Mirrors of DB_ESPNOW_TUNNEL_* in db_mavlink_msgs.h, which
 * cannot be included here (it pulls in fastmavlink); db_serial.c asserts they are equal at
 * compile time.
 */
#define DB_ESPNOW_CLASS_TUNNEL_FLEET_LIST      0xDB01U
#define DB_ESPNOW_CLASS_TUNNEL_ADDRESSED_DATA  0xDB02U
#define DB_ESPNOW_CLASS_TUNNEL_LINK_STATS      0xDB03U

/*
 * TUNNEL payload layout on the wire (fastmavlink FASTMAVLINK_MSG_TUNNEL_FIELD_*_OFS), asserted
 * by the host test: payload_type u16 LE @0, target_system @2, target_component @3,
 * payload_length @4, payload[128] @5.
 */
#define DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_TYPE_OFS    0U
#define DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_LENGTH_OFS  4U
#define DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_OFS         5U
#define DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_MAX         128U

/** Direction a frame travels over the radio, i.e. which allowlist applies. */
typedef enum {
    /** GCS -> GND serial -> radio -> AIR -> flight controller. */
    DB_ESPNOW_LINK_GND_TO_AIR = 0,
    /** Flight controller -> AIR serial -> radio -> GND -> GCS. */
    DB_ESPNOW_LINK_AIR_TO_GND = 1,
} db_espnow_link_dir_t;

/**
 * How one complete MAVLink frame read from the serial port is to be handed to the radio.
 */
typedef struct {
    /** true: flush whatever is accumulated, then queue this frame on its own (Trap 2). false:
     *  the existing aggregation path, untouched. */
    bool travel_alone;
    /** Routing request for the queued packet. Only meaningful when travel_alone is true;
     *  the aggregated path always queues DB_ESPNOW_DEST_AUTO, as before. */
    db_espnow_dest_mode_t dest_mode;
    /** Destination for DB_ESPNOW_DEST_PEER, all zero otherwise. */
    uint8_t dest_mac[DB_ESPNOW_MAC_ADDR_LEN];
} db_espnow_frame_route_t;

/**
 * Whether a MAVLink message is a control frame for the given direction.
 *
 * GND->AIR: COMMAND_LONG, COMMAND_INT, MISSION_COUNT, MISSION_ITEM_INT, MISSION_REQUEST_INT,
 * MISSION_ACK, MISSION_CLEAR_ALL, MISSION_SET_CURRENT, PARAM_SET, PARAM_REQUEST_READ, SET_MODE,
 * TUNNEL carrying DB_ESPNOW_TUNNEL_ADDRESSED_DATA, and MISSION_REQUEST_LIST (added 2026-11-14,
 * beyond the doc - see the top of this file).
 * AIR->GND: COMMAND_ACK, MISSION_ACK, MISSION_REQUEST_INT, MISSION_ITEM_INT, MISSION_COUNT,
 * PARAM_VALUE, STATUSTEXT, and MISSION_REQUEST (added 2026-11-14, beyond the doc - see the top
 * of this file).
 * Everything else is telemetry - explicitly HEARTBEAT, RADIO_STATUS (the instrument, Trap 9),
 * every other TUNNEL including FLEET_LIST and the 0xDB03 link-stats report, and every id this
 * firmware does not know.
 *
 * @param dir Direction of travel
 * @param msgid MAVLink message id
 * @param tunnel_payload_type TUNNEL payload_type; read only when msgid is TUNNEL
 * @return true for a control frame
 */
bool db_espnow_frame_is_control(db_espnow_link_dir_t dir, uint32_t msgid, uint16_t tunnel_payload_type);

/**
 * Read the destination MAC out of a DB_ESPNOW_TUNNEL_ADDRESSED_DATA TUNNEL payload.
 *
 * Accepts exactly what the AIR-side filter in db_parse_mavlink_from_radio() would deliver: an
 * inner payload_length greater than the 6-byte MAC and no larger than the 128-byte TUNNEL
 * payload array. Additionally refuses a group (multicast/broadcast) or all-zero MAC, which no
 * AIR unit has. Never reads at or beyond payload_len, so a short (MAVLink v2 truncated) or
 * corrupt payload cannot cause an over-read.
 *
 * @param tunnel_payload The TUNNEL message payload as received (msg.payload)
 * @param payload_len Number of valid bytes in tunnel_payload (msg.len, the wire length)
 * @param mac_out Receives the MAC on success; untouched on failure. May not be NULL.
 * @return true if a well-formed destination was found
 */
bool db_espnow_addressed_data_dest(const uint8_t *tunnel_payload, uint16_t payload_len,
                                   uint8_t mac_out[DB_ESPNOW_MAC_ADDR_LEN]);

/**
 * Routing of one complete MAVLink frame read from the serial port.
 *
 * Only ESPNOW_UNICAST mode 1 ever returns travel_alone. Mode 0 and mode 2 get exactly the
 * aggregation path they had before classification existed - mode 2 deliberately so, because it
 * is the diagnostic every earlier measurement was taken in. Within mode 1:
 *  - a telemetry frame is aggregated as before (and goes out broadcast, see
 *    db_espnow_resolve_destination_for_mode());
 *  - a control frame travels alone, to: the MAC inside a well-formed ADDRESSED_DATA tunnel;
 *    DB_ESPNOW_DEST_BROADCAST for a malformed or short ADDRESSED_DATA; DB_ESPNOW_DEST_CONTROL
 *    (the single known peer, else broadcast) for every other control frame;
 *  - a control frame longer than max_alone_len cannot be one packet and is left to the
 *    existing split-and-aggregate path. No allowlisted message gets near the ESP-NOW payload
 *    limit (the largest, an ADDRESSED_DATA TUNNEL, is at most 158 B signed against 215 B), so
 *    this is a guard, not a code path.
 *
 * @param unicast_mode ESPNOW_UNICAST (DB_ESPNOW_ADDRESSING_*)
 * @param dir Direction of travel
 * @param msgid MAVLink message id of a frame that parsed with a valid CRC
 * @param payload Message payload (msg.payload); read only for TUNNEL. May be NULL otherwise.
 * @param payload_len Valid bytes in payload (msg.len)
 * @param frame_len Length of the whole MAVLink frame on the wire
 * @param max_alone_len Largest payload one ESP-NOW packet carries (DB_ESPNOW_PAYLOAD_MAXSIZE)
 * @return The routing decision
 */
db_espnow_frame_route_t db_espnow_route_frame(uint8_t unicast_mode, db_espnow_link_dir_t dir, uint32_t msgid,
                                              const uint8_t *payload, uint16_t payload_len, uint16_t frame_len,
                                              uint16_t max_alone_len);

#endif // DB_ESP32_DB_ESPNOW_FRAME_CLASS_H
