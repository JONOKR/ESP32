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
 * Host tests for ESPNOW_UNICAST mode 1 (decision doc 2026-11-10, Steps 4 and 5): the control-frame
 * allowlists per direction, destination extraction from ADDRESSED_DATA, the per-frame routing
 * decision, the per-mode destination table, and the sequence-number spaces that keep a frame
 * addressed to one AIR from reading as loss on another.
 *
 * Frames are built and parsed with the real fastmavlink library where it matters, so MAVLink v2
 * trailing-zero truncation is exercised exactly as the firmware sees it.
 */

#undef NDEBUG
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "db_espnow_frame_class.h"
#include "db_espnow_seq_space.h"
#include "db_espnow_tx_path.h"
#include "common/common.h"

#define MAX_ALONE 215U   // DB_ESPNOW_PAYLOAD_MAXSIZE on the firmware

static const uint8_t MAC_A[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
static const uint8_t MAC_B[6] = {0xA0, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};
static const uint8_t MAC_BC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/** GND->AIR allowlist, verbatim from the decision doc (TUNNEL/ADDRESSED_DATA tested separately). */
static const uint32_t UPLINK_CONTROL[] = {
        FASTMAVLINK_MSG_ID_COMMAND_LONG, FASTMAVLINK_MSG_ID_COMMAND_INT, FASTMAVLINK_MSG_ID_MISSION_COUNT,
        FASTMAVLINK_MSG_ID_MISSION_ITEM_INT, FASTMAVLINK_MSG_ID_MISSION_REQUEST_INT, FASTMAVLINK_MSG_ID_MISSION_ACK,
        FASTMAVLINK_MSG_ID_MISSION_CLEAR_ALL, FASTMAVLINK_MSG_ID_MISSION_SET_CURRENT, FASTMAVLINK_MSG_ID_PARAM_SET,
        FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ, FASTMAVLINK_MSG_ID_SET_MODE,
};

/** AIR->GND allowlist, verbatim from the decision doc. */
static const uint32_t DOWNLINK_CONTROL[] = {
        FASTMAVLINK_MSG_ID_COMMAND_ACK, FASTMAVLINK_MSG_ID_MISSION_ACK, FASTMAVLINK_MSG_ID_MISSION_REQUEST_INT,
        FASTMAVLINK_MSG_ID_MISSION_ITEM_INT, FASTMAVLINK_MSG_ID_MISSION_COUNT, FASTMAVLINK_MSG_ID_PARAM_VALUE,
        FASTMAVLINK_MSG_ID_STATUSTEXT,
};

/** "Explicitly not on the list", plus frames that must never be mistaken for control. */
static const uint32_t TELEMETRY[] = {
        FASTMAVLINK_MSG_ID_HEARTBEAT, FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT, FASTMAVLINK_MSG_ID_ATTITUDE,
        FASTMAVLINK_MSG_ID_VFR_HUD, FASTMAVLINK_MSG_ID_SCALED_IMU, FASTMAVLINK_MSG_ID_SCALED_IMU2,
        FASTMAVLINK_MSG_ID_SCALED_IMU3, FASTMAVLINK_MSG_ID_GPS_RAW_INT, FASTMAVLINK_MSG_ID_SYS_STATUS,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS, FASTMAVLINK_MSG_ID_RADIO_STATUS, FASTMAVLINK_MSG_ID_MISSION_CURRENT,
        FASTMAVLINK_MSG_ID_SYSTEM_TIME, 99999U /* unknown id */,
};

/**
 * Whether an id is in a list.
 *
 * @param list List
 * @param n Entries
 * @param id Id to find
 * @return true if present
 */
static bool in_list(const uint32_t *list, size_t n, uint32_t id) {
    for (size_t i = 0; i < n; i++) {
        if (list[i] == id) {
            return true;
        }
    }
    return false;
}

/**
 * The classifier's literal ids and TUNNEL offsets must be fastmavlink's.
 */
static void test_constants_match_fastmavlink(void) {
    assert(DB_MAVLINK_MSG_ID_HEARTBEAT == FASTMAVLINK_MSG_ID_HEARTBEAT);
    assert(DB_MAVLINK_MSG_ID_SYS_STATUS == FASTMAVLINK_MSG_ID_SYS_STATUS);
    assert(DB_MAVLINK_MSG_ID_SET_MODE == FASTMAVLINK_MSG_ID_SET_MODE);
    assert(DB_MAVLINK_MSG_ID_PARAM_REQUEST_READ == FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ);
    assert(DB_MAVLINK_MSG_ID_PARAM_VALUE == FASTMAVLINK_MSG_ID_PARAM_VALUE);
    assert(DB_MAVLINK_MSG_ID_PARAM_SET == FASTMAVLINK_MSG_ID_PARAM_SET);
    assert(DB_MAVLINK_MSG_ID_GPS_RAW_INT == FASTMAVLINK_MSG_ID_GPS_RAW_INT);
    assert(DB_MAVLINK_MSG_ID_SCALED_IMU == FASTMAVLINK_MSG_ID_SCALED_IMU);
    assert(DB_MAVLINK_MSG_ID_ATTITUDE == FASTMAVLINK_MSG_ID_ATTITUDE);
    assert(DB_MAVLINK_MSG_ID_GLOBAL_POSITION_INT == FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    assert(DB_MAVLINK_MSG_ID_MISSION_SET_CURRENT == FASTMAVLINK_MSG_ID_MISSION_SET_CURRENT);
    assert(DB_MAVLINK_MSG_ID_MISSION_COUNT == FASTMAVLINK_MSG_ID_MISSION_COUNT);
    assert(DB_MAVLINK_MSG_ID_MISSION_CLEAR_ALL == FASTMAVLINK_MSG_ID_MISSION_CLEAR_ALL);
    assert(DB_MAVLINK_MSG_ID_MISSION_ACK == FASTMAVLINK_MSG_ID_MISSION_ACK);
    assert(DB_MAVLINK_MSG_ID_MISSION_REQUEST_INT == FASTMAVLINK_MSG_ID_MISSION_REQUEST_INT);
    assert(DB_MAVLINK_MSG_ID_MISSION_ITEM_INT == FASTMAVLINK_MSG_ID_MISSION_ITEM_INT);
    assert(DB_MAVLINK_MSG_ID_VFR_HUD == FASTMAVLINK_MSG_ID_VFR_HUD);
    assert(DB_MAVLINK_MSG_ID_COMMAND_INT == FASTMAVLINK_MSG_ID_COMMAND_INT);
    assert(DB_MAVLINK_MSG_ID_COMMAND_LONG == FASTMAVLINK_MSG_ID_COMMAND_LONG);
    assert(DB_MAVLINK_MSG_ID_COMMAND_ACK == FASTMAVLINK_MSG_ID_COMMAND_ACK);
    assert(DB_MAVLINK_MSG_ID_RADIO_STATUS == FASTMAVLINK_MSG_ID_RADIO_STATUS);
    assert(DB_MAVLINK_MSG_ID_SCALED_IMU2 == FASTMAVLINK_MSG_ID_SCALED_IMU2);
    assert(DB_MAVLINK_MSG_ID_SCALED_IMU3 == FASTMAVLINK_MSG_ID_SCALED_IMU3);
    assert(DB_MAVLINK_MSG_ID_BATTERY_STATUS == FASTMAVLINK_MSG_ID_BATTERY_STATUS);
    assert(DB_MAVLINK_MSG_ID_STATUSTEXT == FASTMAVLINK_MSG_ID_STATUSTEXT);
    assert(DB_MAVLINK_MSG_ID_TUNNEL == FASTMAVLINK_MSG_ID_TUNNEL);
    assert(DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_TYPE_OFS == FASTMAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_TYPE_OFS);
    assert(DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_LENGTH_OFS == FASTMAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LENGTH_OFS);
    assert(DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_OFS == FASTMAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_OFS);
    assert(DB_ESPNOW_CLASS_TUNNEL_PAYLOAD_MAX == FASTMAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_NUM);
}

/**
 * Every allowlisted id is control in its direction, and only there; everything else is telemetry.
 */
static void test_allowlists(void) {
    for (size_t i = 0; i < sizeof(UPLINK_CONTROL) / sizeof(UPLINK_CONTROL[0]); i++) {
        assert(db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, UPLINK_CONTROL[i], 0));
        // in the other direction only if it is on that list too (the mission sub-protocol ids are)
        assert(db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, UPLINK_CONTROL[i], 0) ==
               in_list(DOWNLINK_CONTROL, sizeof(DOWNLINK_CONTROL) / sizeof(DOWNLINK_CONTROL[0]), UPLINK_CONTROL[i]));
    }
    for (size_t i = 0; i < sizeof(DOWNLINK_CONTROL) / sizeof(DOWNLINK_CONTROL[0]); i++) {
        assert(db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, DOWNLINK_CONTROL[i], 0));
        assert(db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, DOWNLINK_CONTROL[i], 0) ==
               in_list(UPLINK_CONTROL, sizeof(UPLINK_CONTROL) / sizeof(UPLINK_CONTROL[0]), DOWNLINK_CONTROL[i]));
    }
    // a GCS never sends COMMAND_ACK/PARAM_VALUE/STATUSTEXT up; an FC never sends commands down
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_COMMAND_ACK, 0));
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, FASTMAVLINK_MSG_ID_COMMAND_LONG, 0));
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, FASTMAVLINK_MSG_ID_SET_MODE, 0));

    for (size_t i = 0; i < sizeof(TELEMETRY) / sizeof(TELEMETRY[0]); i++) {
        assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, TELEMETRY[i], 0));
        assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, TELEMETRY[i], 0));
    }
    // Trap 9: RADIO_STATUS is the instrument, never control - whatever junk is in the tunnel arg
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_RADIO_STATUS, 0xDB02));
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, FASTMAVLINK_MSG_ID_RADIO_STATUS, 0xDB02));

    // TUNNEL: only ADDRESSED_DATA, only uplink
    assert(db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_TUNNEL, 0xDB02));
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, FASTMAVLINK_MSG_ID_TUNNEL, 0xDB02));
    const uint16_t other_tunnels[] = {0xDB01 /* FLEET_LIST */, 0xDB03 /* link stats */, 0x0000, 0xFFFF};
    for (size_t i = 0; i < sizeof(other_tunnels) / sizeof(other_tunnels[0]); i++) {
        assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_TUNNEL, other_tunnels[i]));
        assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_AIR_TO_GND, FASTMAVLINK_MSG_ID_TUNNEL, other_tunnels[i]));
    }
    // the payload type is only read for TUNNEL
    assert(!db_espnow_frame_is_control(DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_HEARTBEAT, 0xDB02));
    // an out-of-range direction is never control
    assert(!db_espnow_frame_is_control((db_espnow_link_dir_t) 7, FASTMAVLINK_MSG_ID_COMMAND_LONG, 0));
}

/**
 * A complete, real MAVLink frame as parsed by fastmavlink - what db_serial.c hands the classifier.
 */
typedef struct {
    uint8_t frame[FASTMAVLINK_FRAME_LEN_MAX];
    fmav_result_t result;
    fmav_message_t msg;
} parsed_frame_t;

/**
 * Parse a frame buffer byte by byte, exactly as the serial path does.
 *
 * @param bytes Frame bytes
 * @param len Length
 * @param out Parsed frame
 */
static void parse_frame(const uint8_t *bytes, uint16_t len, parsed_frame_t *out) {
    fmav_status_t status;
    memset(&status, 0, sizeof(status));
    memset(out, 0, sizeof(*out));
    bool complete = false;
    for (uint16_t i = 0; i < len; i++) {
        fmav_result_t r = {0};
        if (fmav_parse_and_check_to_frame_buf(&r, out->frame, &status, bytes[i])) {
            out->result = r;
            complete = true;
        }
    }
    assert(complete);
    assert(out->result.res == FASTMAVLINK_PARSE_RESULT_OK);
    fmav_frame_buf_to_msg(&out->msg, &out->result, out->frame);
}

/**
 * Build and parse an ADDRESSED_DATA TUNNEL as the GCS builds it (fleet.py build_addressed_payload).
 *
 * @param mac Destination MAC
 * @param inner_len Length of the inner frame (bytes are a recognisable non-zero pattern)
 * @param payload_length_field Value to put in the payload_length field
 * @param out Parsed frame
 */
static void make_addressed(const uint8_t *mac, uint8_t inner_len, uint8_t payload_length_field, parsed_frame_t *out) {
    fmav_tunnel_t t;
    memset(&t, 0, sizeof(t));
    t.payload_type = 0xDB02;
    t.payload_length = payload_length_field;
    memcpy(t.payload, mac, 6);
    for (uint8_t i = 0; i < inner_len && 6U + i < sizeof(t.payload); i++) {
        t.payload[6 + i] = (uint8_t) (0xFD - i);    // starts like a MAVLink v2 frame, non-zero
    }
    uint8_t buf[FASTMAVLINK_FRAME_LEN_MAX];
    fmav_status_t tx;
    memset(&tx, 0, sizeof(tx));
    const uint16_t len = fmav_msg_tunnel_encode_to_frame_buf(buf, 255, 190, &t, &tx);
    parse_frame(buf, len, out);
}

/**
 * ADDRESSED_DATA yields the embedded MAC; anything malformed or short yields nothing, without
 * reading past the bytes it was given.
 */
static void test_addressed_data_dest(void) {
    uint8_t mac[6];
    parsed_frame_t f;

    make_addressed(MAC_A, 20, 26, &f);
    memset(mac, 0, sizeof(mac));
    assert(db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));
    assert(memcmp(mac, MAC_A, 6) == 0);

    // the largest the GCS builds: 6 + 122 = 128
    make_addressed(MAC_B, 122, 128, &f);
    assert(db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));
    assert(memcmp(mac, MAC_B, 6) == 0);

    // payload_length that the AIR filter would reject: <= 6 (no inner frame) or > 128
    make_addressed(MAC_A, 20, 6, &f);
    assert(!db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));
    make_addressed(MAC_A, 20, 129, &f);
    assert(!db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));
    make_addressed(MAC_A, 20, 255, &f);
    assert(!db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));

    // group and null MACs name no AIR unit
    make_addressed(MAC_BC, 20, 26, &f);
    assert(!db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));
    const uint8_t multicast[6] = {0x01, 0x00, 0x5E, 0x00, 0x00, 0x01};
    make_addressed(multicast, 20, 26, &f);
    assert(!db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));
    const uint8_t zero[6] = {0};
    make_addressed(zero, 20, 26, &f);
    assert(!db_espnow_addressed_data_dest(f.msg.payload, f.msg.len, mac));

    // wrong payload type
    uint8_t raw[16] = {0x01, 0xDB, 0, 0, 20, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0xFD, 1, 2, 3, 4};
    assert(!db_espnow_addressed_data_dest(raw, sizeof(raw), mac));
    raw[0] = 0x02;
    assert(db_espnow_addressed_data_dest(raw, sizeof(raw), mac));

    /* Short payloads. The buffer is poisoned past the stated length with a valid-looking
     * continuation: if the function read past payload_len it would "find" a MAC there. */
    uint8_t poisoned[64];
    memset(poisoned, 0, sizeof(poisoned));
    poisoned[0] = 0x02;
    poisoned[1] = 0xDB;
    poisoned[4] = 26;
    memcpy(&poisoned[5], MAC_A, 6);
    poisoned[11] = 0xFD;
    for (uint16_t short_len = 0; short_len < 11U; short_len++) {
        memset(mac, 0xEE, sizeof(mac));
        assert(!db_espnow_addressed_data_dest(poisoned, short_len, mac));
        assert(mac[0] == 0xEE && mac[5] == 0xEE);       // untouched on failure
    }
    assert(db_espnow_addressed_data_dest(poisoned, 11U, mac));    // exactly type + header + MAC
    assert(!db_espnow_addressed_data_dest(NULL, 40, mac));
    assert(!db_espnow_addressed_data_dest(poisoned, 40, NULL));
}

/**
 * The per-frame decision: only mode 1 ever sends a frame alone; destinations follow the doc.
 */
static void test_route_frame(void) {
    parsed_frame_t f;
    db_espnow_frame_route_t r;

    // a real COMMAND_LONG, GND->AIR
    fmav_command_long_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.target_system = 2;
    cmd.target_component = 1;
    cmd.command = 400;      // MAV_CMD_COMPONENT_ARM_DISARM
    cmd.param1 = 1.0f;
    uint8_t buf[FASTMAVLINK_FRAME_LEN_MAX];
    fmav_status_t tx;
    memset(&tx, 0, sizeof(tx));
    uint16_t len = fmav_msg_command_long_encode_to_frame_buf(buf, 255, 190, &cmd, &tx);
    parse_frame(buf, len, &f);

    // modes 0 and 2 (and anything unknown): classification does not exist
    const uint8_t not_mode_1[] = {DB_ESPNOW_ADDRESSING_OFF, DB_ESPNOW_ADDRESSING_ALL, 3, 255};
    for (size_t i = 0; i < sizeof(not_mode_1); i++) {
        r = db_espnow_route_frame(not_mode_1[i], DB_ESPNOW_LINK_GND_TO_AIR, f.result.msgid, f.msg.payload,
                                  f.msg.len, f.result.frame_len, MAX_ALONE);
        assert(!r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_AUTO);
    }
    // mode 1: alone, to the single known peer (CONTROL)
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_CONTROL);
    // but a COMMAND_LONG coming from an FC is not on the downlink list
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_AIR_TO_GND, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(!r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_AUTO);
    // a frame too long for one packet is left to the split path (a guard; nothing allowlisted is that long)
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, (uint16_t) (f.result.frame_len - 1U));
    assert(!r.travel_alone);

    // COMMAND_ACK, AIR->GND: the original failure
    fmav_command_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.command = 400;
    memset(&tx, 0, sizeof(tx));
    len = fmav_msg_command_ack_encode_to_frame_buf(buf, 1, 1, &ack, &tx);
    parse_frame(buf, len, &f);
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_AIR_TO_GND, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_CONTROL);

    // RADIO_STATUS from an FC/SiK: never alone, never addressed (Trap 9)
    fmav_radio_status_t rs;
    memset(&rs, 0, sizeof(rs));
    rs.rssi = 100;
    memset(&tx, 0, sizeof(tx));
    len = fmav_msg_radio_status_encode_to_frame_buf(buf, 1, 68, &rs, &tx);
    parse_frame(buf, len, &f);
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_AIR_TO_GND, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(!r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_AUTO);

    // ADDRESSED_DATA: the embedded MAC
    make_addressed(MAC_B, 40, 46, &f);
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_PEER && memcmp(r.dest_mac, MAC_B, 6) == 0);
    // ... and with the largest inner frame the GCS builds, still one packet
    make_addressed(MAC_B, 122, 128, &f);
    assert(f.result.frame_len <= MAX_ALONE);
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_PEER);
    // malformed ADDRESSED_DATA: still alone (it is a control frame), but broadcast - never a guessed peer
    make_addressed(MAC_B, 40, 200, &f);
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_BROADCAST);
    const uint8_t zero[6] = {0};
    assert(memcmp(r.dest_mac, zero, 6) == 0);
    // a short ADDRESSED_DATA payload (type present, MAC cut off) - broadcast, no over-read
    const uint8_t short_tunnel[7] = {0x02, 0xDB, 0, 0, 20, 0x10, 0x20};
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_TUNNEL,
                              short_tunnel, sizeof(short_tunnel), 30, MAX_ALONE);
    assert(r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_BROADCAST);
    // a TUNNEL whose type byte itself is truncated reads as type 0: telemetry
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_GND_TO_AIR, FASTMAVLINK_MSG_ID_TUNNEL,
                              short_tunnel, 1, 13, MAX_ALONE);
    assert(!r.travel_alone);
    // ADDRESSED_DATA from an FC is not control
    make_addressed(MAC_B, 40, 46, &f);
    r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, DB_ESPNOW_LINK_AIR_TO_GND, f.result.msgid, f.msg.payload,
                              f.msg.len, f.result.frame_len, MAX_ALONE);
    assert(!r.travel_alone);

    // FLEET_LIST and link stats tunnels: never alone, never addressed
    fmav_tunnel_t t;
    memset(&t, 0, sizeof(t));
    t.payload_type = 0xDB01;
    t.payload_length = 13;
    t.payload[0] = 2;
    memcpy(&t.payload[1], MAC_A, 6);
    memcpy(&t.payload[7], MAC_B, 6);
    memset(&tx, 0, sizeof(tx));
    len = fmav_msg_tunnel_encode_to_frame_buf(buf, 1, 68, &t, &tx);
    parse_frame(buf, len, &f);
    for (int dir = 0; dir < 2; dir++) {
        r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, (db_espnow_link_dir_t) dir, f.result.msgid,
                                  f.msg.payload, f.msg.len, f.result.frame_len, MAX_ALONE);
        assert(!r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_AUTO);
    }
    t.payload_type = 0xDB03;
    memset(&tx, 0, sizeof(tx));
    len = fmav_msg_tunnel_encode_to_frame_buf(buf, 1, 68, &t, &tx);
    parse_frame(buf, len, &f);
    for (int dir = 0; dir < 2; dir++) {
        r = db_espnow_route_frame(DB_ESPNOW_ADDRESSING_CONTROL, (db_espnow_link_dir_t) dir, f.result.msgid,
                                  f.msg.payload, f.msg.len, f.result.frame_len, MAX_ALONE);
        assert(!r.travel_alone && r.dest_mode == DB_ESPNOW_DEST_AUTO);
    }
}

/**
 * The per-mode destination table on db_espnow_resolve_destination_for_mode().
 */
static void test_resolve_for_mode(void) {
    const db_espnow_dest_mode_t all[] = {DB_ESPNOW_DEST_AUTO, DB_ESPNOW_DEST_BROADCAST, DB_ESPNOW_DEST_PEER,
                                         DB_ESPNOW_DEST_CONTROL};
    // mode 0, and any unknown mode: broadcast, whatever is asked and whatever is known
    const uint8_t broadcast_modes[] = {DB_ESPNOW_ADDRESSING_OFF, 3, 200};
    for (size_t m = 0; m < sizeof(broadcast_modes); m++) {
        for (size_t i = 0; i < 4; i++) {
            assert(db_espnow_resolve_destination_for_mode(broadcast_modes[m], all[i], MAC_B, true, MAC_A) == NULL);
        }
    }

    // mode 1
    const uint8_t m1 = DB_ESPNOW_ADDRESSING_CONTROL;
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_AUTO, NULL, false, MAC_A) == NULL);   // telemetry
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_BROADCAST, MAC_B, true, MAC_A) == NULL);
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_CONTROL, NULL, false, MAC_A) == MAC_A);
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_CONTROL, NULL, false, NULL) == NULL);  // no / several peers
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_PEER, MAC_B, true, MAC_A) == MAC_B);
    // unregistered named peer: broadcast, NOT the single known peer (that would be another drone)
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_PEER, MAC_B, false, MAC_A) == NULL);
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_PEER, NULL, true, MAC_A) == NULL);
    assert(db_espnow_resolve_destination_for_mode(m1, DB_ESPNOW_DEST_PEER, MAC_BC, true, MAC_A) == NULL);
    assert(db_espnow_resolve_destination_for_mode(m1, (db_espnow_dest_mode_t) 77, MAC_B, true, MAC_A) == NULL);

    // mode 2: phase A's rule, unchanged
    const uint8_t m2 = DB_ESPNOW_ADDRESSING_ALL;
    assert(db_espnow_resolve_destination_for_mode(m2, DB_ESPNOW_DEST_AUTO, NULL, false, MAC_A) == MAC_A);
    assert(db_espnow_resolve_destination_for_mode(m2, DB_ESPNOW_DEST_AUTO, NULL, false, NULL) == NULL);
    assert(db_espnow_resolve_destination_for_mode(m2, DB_ESPNOW_DEST_BROADCAST, MAC_B, true, MAC_A) == NULL);
    assert(db_espnow_resolve_destination_for_mode(m2, DB_ESPNOW_DEST_CONTROL, NULL, false, MAC_A) == MAC_A);
    assert(db_espnow_resolve_destination_for_mode(m2, DB_ESPNOW_DEST_PEER, MAC_B, true, MAC_A) == MAC_B);
    assert(db_espnow_resolve_destination_for_mode(m2, DB_ESPNOW_DEST_PEER, MAC_B, false, MAC_A) == MAC_A);
    for (size_t i = 0; i < 4; i++) {
        assert(db_espnow_resolve_destination_for_mode(m2, all[i], MAC_B, true, MAC_A) ==
               db_espnow_resolve_destination(all[i], MAC_B, true, true, MAC_A));
        assert(db_espnow_resolve_destination_for_mode(m2, all[i], MAC_B, false, NULL) ==
               db_espnow_resolve_destination(all[i], MAC_B, false, true, NULL));
    }
}

/**
 * Sender spaces: the AIR has one, the GND one for broadcast and one per destination; the
 * broadcast space alone reproduces the single counter of mode 0.
 */
static void test_seq_tx(void) {
    db_espnow_seq_tx_t air;
    db_espnow_seq_tx_init(&air, false);
    assert(db_espnow_seq_tx_counter(&air, NULL) == db_espnow_seq_tx_counter(&air, MAC_A));
    uint32_t *c = db_espnow_seq_tx_counter(&air, NULL);
    for (int i = 0; i < 5; i++) {
        *c = db_espnow_seq_next(*c);
    }
    assert(*db_espnow_seq_tx_counter(&air, MAC_B) == 5U);

    db_espnow_seq_tx_t gnd;
    db_espnow_seq_tx_init(&gnd, true);
    uint32_t *b = db_espnow_seq_tx_counter(&gnd, NULL);
    uint32_t *a = db_espnow_seq_tx_counter(&gnd, MAC_A);
    uint32_t *bb = db_espnow_seq_tx_counter(&gnd, MAC_B);
    assert(b != a && a != bb && b != bb);
    assert(*a == 0U && *bb == 0U);
    *b = 41U;
    *a = db_espnow_seq_next(*a);
    assert(db_espnow_seq_tx_counter(&gnd, MAC_A) == a && *a == 1U);   // found again, not re-created
    assert(*db_espnow_seq_tx_counter(&gnd, NULL) == 41U);             // untouched by addressed sends

    // wrap rule is the firmware's: UINT32_MAX -> 0
    assert(db_espnow_seq_next(UINT32_MAX) == 0U);
    assert(db_espnow_seq_next(7U) == 8U);

    // a full table recycles one slot (a sender reset from that receiver's point of view)
    db_espnow_seq_tx_init(&gnd, true);
    uint8_t mac[6] = {0x02, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < DB_ESPNOW_SEQ_ADDRESSED_SPACES; i++) {
        mac[5] = i;
        *db_espnow_seq_tx_counter(&gnd, mac) = 100U + i;
    }
    mac[5] = 0xEE;
    uint32_t *fresh = db_espnow_seq_tx_counter(&gnd, mac);
    assert(fresh != NULL && *fresh == 0U);
    mac[5] = 1;
    assert(*db_espnow_seq_tx_counter(&gnd, mac) == 101U);             // others intact
    assert(db_espnow_seq_tx_counter(NULL, NULL) == NULL);
}

/**
 * Receiver rule per space, and the scenario that motivated spaces: a GND talking to two AIR
 * units, addressing control frames to one of them.
 */
static void test_seq_rx(void) {
    db_espnow_seq_rx_t rx = {0};
    uint16_t lost = 0;
    assert(db_espnow_seq_rx_update(&rx, 5000U, &lost) == DB_ESPNOW_SEQ_RX_FIRST && lost == 0U);  // adopt, no history
    assert(db_espnow_seq_rx_update(&rx, 5001U, &lost) == DB_ESPNOW_SEQ_RX_FORWARD && lost == 0U);
    assert(db_espnow_seq_rx_update(&rx, 5004U, &lost) == DB_ESPNOW_SEQ_RX_FORWARD && lost == 2U);
    assert(db_espnow_seq_rx_update(&rx, 5004U, &lost) == DB_ESPNOW_SEQ_RX_BACKWARD && lost == 0U);  // equal = backwards
    assert(db_espnow_seq_rx_update(&rx, 3U, &lost) == DB_ESPNOW_SEQ_RX_BACKWARD && lost == 0U);     // sender reboot
    assert(db_espnow_seq_rx_update(&rx, 200000U, &lost) == DB_ESPNOW_SEQ_RX_FORWARD && lost == UINT16_MAX);  // saturates
    assert(db_espnow_seq_rx_update(&rx, 200001U, &lost) == DB_ESPNOW_SEQ_RX_FORWARD && lost == UINT16_MAX);

    /* GND -> two AIRs. Per packet: is it broadcast, and if addressed, to whom. The GND stamps
     * each from its space; each AIR sees broadcasts plus what is addressed to it. Nothing is lost
     * on air, so both AIRs must count zero. With a single counter AIR B would count every frame
     * addressed to A. */
    db_espnow_seq_tx_t gnd;
    db_espnow_seq_tx_init(&gnd, true);
    db_espnow_seq_rx_t a_bc = {0}, a_ad = {0}, b_bc = {0}, b_ad = {0};
    uint16_t a_lost = 0, b_lost = 0;
    uint32_t single_counter = 0;
    db_espnow_seq_rx_t b_single = {0};
    uint16_t b_single_lost = 0;
    const char pattern[] = "bbAbbAAbBbAbbbAAAb";    // b broadcast, A/B addressed to that AIR
    for (size_t i = 0; i < sizeof(pattern) - 1U; i++) {
        const uint8_t *dest = pattern[i] == 'A' ? MAC_A : (pattern[i] == 'B' ? MAC_B : NULL);
        uint32_t *ctr = db_espnow_seq_tx_counter(&gnd, dest);
        const uint32_t seq = *ctr;
        *ctr = db_espnow_seq_next(*ctr);
        if (dest == NULL || dest == MAC_A) {
            (void) db_espnow_seq_rx_update(dest == NULL ? &a_bc : &a_ad, seq, &a_lost);
        }
        if (dest == NULL || dest == MAC_B) {
            (void) db_espnow_seq_rx_update(dest == NULL ? &b_bc : &b_ad, seq, &b_lost);
            (void) db_espnow_seq_rx_update(&b_single, single_counter, &b_single_lost);
        }
        single_counter++;
    }
    assert(a_lost == 0U && b_lost == 0U);
    assert(b_single_lost == 7U);    // what one shared counter would have reported: A's frames after B's first packet

    // a genuinely lost packet is still counted, in whichever space it was
    db_espnow_seq_tx_init(&gnd, true);
    memset(&a_bc, 0, sizeof(a_bc));
    memset(&a_ad, 0, sizeof(a_ad));
    a_lost = 0;
    for (int i = 0; i < 10; i++) {
        const uint8_t *dest = (i % 3 == 0) ? MAC_A : NULL;
        uint32_t *ctr = db_espnow_seq_tx_counter(&gnd, dest);
        const uint32_t seq = *ctr;
        *ctr = db_espnow_seq_next(*ctr);
        if (i == 4 || i == 6) {
            continue;   // lost on air (i == 6 is addressed: 6 % 3 == 0)
        }
        (void) db_espnow_seq_rx_update(dest == NULL ? &a_bc : &a_ad, seq, &a_lost);
    }
    assert(a_lost == 2U);
    assert(db_espnow_seq_rx_update(NULL, 1U, &a_lost) == DB_ESPNOW_SEQ_RX_FIRST);
}

/**
 * Runs all classification, routing and sequence-space tests.
 *
 * @return 0 on success (assert aborts otherwise)
 */
int main(void) {
    test_constants_match_fastmavlink();
    test_allowlists();
    test_addressed_data_dest();
    test_route_frame();
    test_resolve_for_mode();
    test_seq_tx();
    test_seq_rx();
    return 0;
}
