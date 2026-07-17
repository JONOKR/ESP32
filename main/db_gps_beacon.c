/*
 *   This file is part of DroneBridge JONOKR (fork of DroneBridge: https://github.com/DroneBridge/ESP32)
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
 */

#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <nvs.h>
#include "db_gps_beacon.h"
#include "db_serial.h"
#include "db_mavlink_msgs.h"
#include "db_parameters.h"
#include "globals.h"

#define TAG "DB_GPS_BEACON"

// UBX framing
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07
#define UBX_NAV_PVT_LEN 92
#define UBX_MAX_PAYLOAD 512      // parser skips anything longer (we only need NAV-PVT)

// Emission cadence. Position rides at the GPS fix rate (CFG-RATE below);
// heartbeat + raw fix info at 1 Hz keep the GCS registry + fix display alive.
// 1 Hz navigation rate matches the hardware-verified GPS_beacon_example.cpp
// (setNavigationFrequency(1)); raise CFG-RATE-MEAS here if the trajectory
// features later need denser beacon tracks.
#define BEACON_GPS_MEAS_RATE_MS 1000     // 1 Hz NAV-PVT
#define BEACON_HEARTBEAT_US     1000000  // 1 Hz
#define BEACON_GPS_RAW_US       1000000  // 1 Hz
#define BEACON_GPS_CFG_RETRY_US 5000000  // (re)send GPS config every 5 s until PVT flows

// MAVLink identity: beacons take part in the GCS fleet system exactly like
// drones. Factory-fresh they heartbeat sysid 1 ("unnumbered", same semantics
// as a stock ArduPilot). The GCS assigns a unique id by MAC via the addressed
// TUNNEL (PARAM_SET SYSID_THISMAV + reboot) - the beacon applies it to ITSELF,
// persists it in NVS, reboots, and comes back with the assigned id, which the
// GCS's normal fleet verification confirms.
#define BEACON_SYSID_UNNUMBERED 1
#define BEACON_COMP_ID    1     // MAV_COMP_ID_AUTOPILOT1 - keyed like a vehicle at the GCS
#define BEACON_MAC_LEN    6
#define BEACON_NVS_NAMESPACE "db_beacon"
#define BEACON_NVS_KEY_SYSID "sys_id"

typedef struct {
    bool valid;          // gnssFixOK && >= 2D
    uint8_t fix_type;    // UBX fixType
    uint8_t num_sv;
    int32_t lon_1e7;
    int32_t lat_1e7;
    int32_t height_mm;   // ellipsoid
    int32_t hmsl_mm;     // mean sea level
    uint32_t h_acc_mm;
    uint32_t v_acc_mm;
    int32_t vel_n_mms;
    int32_t vel_e_mms;
    int32_t vel_d_mms;
    int32_t gspeed_mms;
    int32_t head_motion_1e5; // deg * 1e-5
    uint32_t s_acc_mms;
    uint32_t head_acc_1e5;
    uint16_t pdop_001;   // * 0.01
} db_beacon_fix_t;

// UBX parser state machine
typedef enum {
    UBX_S_SYNC1, UBX_S_SYNC2, UBX_S_CLASS, UBX_S_ID, UBX_S_LEN1, UBX_S_LEN2,
    UBX_S_PAYLOAD, UBX_S_CK_A, UBX_S_CK_B
} ubx_state_t;

static struct {
    ubx_state_t state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t len;
    uint16_t pos;
    uint8_t ck_a, ck_b;
    uint8_t payload[UBX_MAX_PAYLOAD];
} ubx;

static db_beacon_fix_t last_fix;
static bool fix_ever_received = false;
static bool home_set = false;
static int32_t home_hmsl_mm = 0;
static uint8_t beacon_sysid = 0;
static int64_t last_heartbeat_us = 0;
static int64_t last_gps_raw_us = 0;
static int64_t last_gps_cfg_us = 0;
static bool position_pending = false;   // a fresh PVT arrived, not yet transmitted
static fmav_status_t fmav_status_beacon;

// ---------------------------------------------------------------------------
// Fleet identity (NVS-persisted, GCS-assigned)
// ---------------------------------------------------------------------------

/** Load the fleet-assigned system id from NVS. 0 = never assigned. */
static uint8_t db_beacon_load_assigned_sysid(void) {
    nvs_handle_t handle;
    uint8_t value = 0;
    if (nvs_open(BEACON_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_u8(handle, BEACON_NVS_KEY_SYSID, &value) != ESP_OK) {
            value = 0;
        }
        nvs_close(handle);
    }
    return value;
}

/** Persist the fleet-assigned system id so it survives reboots. */
static void db_beacon_store_assigned_sysid(uint8_t sysid) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(BEACON_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed storing sysid: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(handle, BEACON_NVS_KEY_SYSID, sysid);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write failed storing sysid: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Fleet-assigned sysid %i persisted to NVS", sysid);
    }
}

/**
 * Handle one complete inner MAVLink message from a MAC-addressed tunnel.
 * The beacon has no flight controller, so it acts on the two fleet-management
 * messages ITSELF (a drone's AIR unit would forward them to the FC instead):
 *   PARAM_SET SYSID_THISMAV  -> persist as our own system id
 *   PREFLIGHT_REBOOT_SHUTDOWN -> restart so the new id takes effect
 * Everything else is ignored (never written to the GPS serial port).
 */
static void db_beacon_handle_inner_msg(fmav_message_t *msg) {
    if (msg->msgid == FASTMAVLINK_MSG_ID_PARAM_SET) {
        fmav_param_set_t param_set;
        fmav_msg_param_set_decode(&param_set, msg);
        if (strncmp(param_set.param_id, "SYSID_THISMAV", sizeof(param_set.param_id)) == 0) {
            long v = lroundf(param_set.param_value);
            if (v >= 2 && v <= 254) {
                if ((uint8_t) v != db_beacon_load_assigned_sysid()) {
                    db_beacon_store_assigned_sysid((uint8_t) v);
                } // GCS retries the PARAM_SET for reliability - repeat writes are skipped
            } else {
                ESP_LOGW(TAG, "Ignoring SYSID_THISMAV=%ld (valid range 2..254)", v);
            }
        }
    } else if (msg->msgid == FASTMAVLINK_MSG_ID_COMMAND_LONG) {
        fmav_command_long_t cmd;
        fmav_msg_command_long_decode(&cmd, msg);
        if (cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && cmd.param1 == 1.0f) {
            ESP_LOGI(TAG, "Fleet reboot command received - restarting");
            esp_restart();
        }
    }
}

void db_gps_beacon_handle_radio(uint8_t *buffer, int bytes_read) {
    // Parse the radio stream just enough to spot DroneBridge ADDRESSED_DATA
    // tunnels for our MAC; all other radio traffic is irrelevant to a beacon
    // (and must never reach the GPS on the serial port).
    static uint8_t radio_parse_buf[296];
    static fmav_status_t radio_status;
    static fmav_message_t msg;
    for (int i = 0; i < bytes_read; ++i) {
        fmav_result_t result = {0};
        if (!fmav_parse_and_check_to_frame_buf(&result, radio_parse_buf, &radio_status, buffer[i])) {
            continue;
        }
        fmav_frame_buf_to_msg(&msg, &result, radio_parse_buf);
        if (result.res != FASTMAVLINK_PARSE_RESULT_OK || msg.msgid != FASTMAVLINK_MSG_ID_TUNNEL) {
            continue;
        }
        if (fmav_msg_tunnel_get_field_payload_type(&msg) != DB_ESPNOW_TUNNEL_ADDRESSED_DATA) {
            continue;
        }
        // payload = [dest_mac(6)][inner MAVLink frame...]; zero-filled copy
        // restores MAVLink v2 trailing-zero truncation (same as db_serial.c).
        fmav_tunnel_t tunnel;
        memset(&tunnel, 0, sizeof(tunnel));
        uint16_t copy_len = (msg.len < (uint16_t) sizeof(tunnel)) ? msg.len : (uint16_t) sizeof(tunnel);
        memcpy(&tunnel, msg.payload, copy_len);
        if (tunnel.payload_length <= BEACON_MAC_LEN ||
            memcmp(tunnel.payload, LOCAL_MAC_ADDRESS, BEACON_MAC_LEN) != 0) {
            continue;   // addressed to a different unit
        }
        // Parse the inner frame (a complete MAVLink message) and act on it.
        static uint8_t inner_parse_buf[296];
        static fmav_status_t inner_status;
        static fmav_message_t inner_msg;
        for (uint16_t j = BEACON_MAC_LEN; j < tunnel.payload_length; ++j) {
            fmav_result_t inner_result = {0};
            if (!fmav_parse_and_check_to_frame_buf(&inner_result, inner_parse_buf, &inner_status,
                                                   tunnel.payload[j])) {
                continue;
            }
            fmav_frame_buf_to_msg(&inner_msg, &inner_result, inner_parse_buf);
            if (inner_result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                db_beacon_handle_inner_msg(&inner_msg);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// u-blox helpers
// ---------------------------------------------------------------------------

static void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/**
 * Configure the GPS via UBX-CFG-VALSET (M9/M10 — an older M8 NAKs it,
 * harmless). Mirrors the hardware-verified GPS_beacon_example.cpp setup:
 *   setUART1Output(COM_TYPE_UBX)  -> UBX out on, NMEA out off (UART1)
 *   setAutoPVT(true)              -> NAV-PVT pushed on UART1
 *   setNavigationFrequency(...)   -> CFG-RATE-MEAS
 *   saveConfiguration()           -> persisted (layers RAM+BBR+Flash here)
 * Sent blind (no ACK handling) and re-sent every BEACON_GPS_CFG_RETRY_US
 * until PVT frames actually flow — so a GPS that boots slower than the ESP32
 * still gets configured, while an already-persisted GPS streams immediately
 * and never triggers a config (no repeated flash writes).
 */
static void db_beacon_send_gps_config(void) {
    // VALSET payload: version(1)=0, layers(1)=7 (RAM+BBR+Flash), reserved(2), cfgData...
    //   CFG-UART1OUTPROT-UBX  (0x10740001, L)  = 1
    //   CFG-UART1OUTPROT-NMEA (0x10740002, L)  = 0
    //   CFG-MSGOUT-UBX_NAV_PVT_UART1 (0x20910007, U1) = 1
    //   CFG-RATE-MEAS         (0x30210001, U2) = BEACON_GPS_MEAS_RATE_MS
    uint8_t payload[4 + 5 + 5 + 5 + 6] = {
            0x00, 0x07, 0x00, 0x00,
            0x01, 0x00, 0x74, 0x10, 0x01,
            0x02, 0x00, 0x74, 0x10, 0x00,
            0x07, 0x00, 0x91, 0x20, 0x01,
            0x01, 0x00, 0x21, 0x30,
            (uint8_t) (BEACON_GPS_MEAS_RATE_MS & 0xFF), (uint8_t) (BEACON_GPS_MEAS_RATE_MS >> 8),
    };
    uint8_t frame[8 + sizeof(payload)];
    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = 0x06;    // CFG
    frame[3] = 0x8A;    // VALSET
    frame[4] = (uint8_t) (sizeof(payload) & 0xFF);
    frame[5] = (uint8_t) (sizeof(payload) >> 8);
    memcpy(&frame[6], payload, sizeof(payload));
    ubx_checksum(&frame[2], 4 + sizeof(payload), &frame[6 + sizeof(payload)], &frame[7 + sizeof(payload)]);
    write_to_serial(frame, sizeof(frame));
}

static uint16_t rd_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static int32_t rd_i32(const uint8_t *p) { int32_t v; memcpy(&v, p, 4); return v; }

/** Decode a validated UBX-NAV-PVT payload into last_fix. */
static void db_beacon_handle_nav_pvt(const uint8_t *p, uint16_t len) {
    if (len < UBX_NAV_PVT_LEN) {
        return; // truncated / unknown variant
    }
    last_fix.fix_type = p[20];
    const bool gnss_fix_ok = (p[21] & 0x01) != 0;
    last_fix.num_sv = p[23];
    last_fix.lon_1e7 = rd_i32(&p[24]);
    last_fix.lat_1e7 = rd_i32(&p[28]);
    last_fix.height_mm = rd_i32(&p[32]);
    last_fix.hmsl_mm = rd_i32(&p[36]);
    last_fix.h_acc_mm = rd_u32(&p[40]);
    last_fix.v_acc_mm = rd_u32(&p[44]);
    last_fix.vel_n_mms = rd_i32(&p[48]);
    last_fix.vel_e_mms = rd_i32(&p[52]);
    last_fix.vel_d_mms = rd_i32(&p[56]);
    last_fix.gspeed_mms = rd_i32(&p[60]);
    last_fix.head_motion_1e5 = rd_i32(&p[64]);
    last_fix.s_acc_mms = rd_u32(&p[68]);
    last_fix.head_acc_1e5 = rd_u32(&p[72]);
    last_fix.pdop_001 = rd_u16(&p[76]);
    last_fix.valid = gnss_fix_ok && last_fix.fix_type >= 2;
    fix_ever_received = true;
    if (last_fix.valid && last_fix.fix_type >= 3 && !home_set) {
        home_set = true;    // first 3D fix anchors relative altitude
        home_hmsl_mm = last_fix.hmsl_mm;
        ESP_LOGI(TAG, "Home altitude locked at %.1f m MSL", last_fix.hmsl_mm / 1000.0);
    }
    position_pending = last_fix.valid;
}

/** Feed one byte to the UBX state machine; dispatches complete NAV-PVT frames. */
static void db_beacon_parse_ubx_byte(uint8_t b) {
    switch (ubx.state) {
        case UBX_S_SYNC1:
            if (b == UBX_SYNC1) ubx.state = UBX_S_SYNC2;
            break;
        case UBX_S_SYNC2:
            ubx.state = (b == UBX_SYNC2) ? UBX_S_CLASS : UBX_S_SYNC1;
            break;
        case UBX_S_CLASS:
            ubx.msg_class = b;
            ubx.ck_a = b;
            ubx.ck_b = b;
            ubx.state = UBX_S_ID;
            break;
        case UBX_S_ID:
            ubx.msg_id = b;
            ubx.ck_a += b;
            ubx.ck_b += ubx.ck_a;
            ubx.state = UBX_S_LEN1;
            break;
        case UBX_S_LEN1:
            ubx.len = b;
            ubx.ck_a += b;
            ubx.ck_b += ubx.ck_a;
            ubx.state = UBX_S_LEN2;
            break;
        case UBX_S_LEN2:
            ubx.len |= (uint16_t) b << 8;
            ubx.ck_a += b;
            ubx.ck_b += ubx.ck_a;
            ubx.pos = 0;
            if (ubx.len > UBX_MAX_PAYLOAD) {
                ubx.state = UBX_S_SYNC1;    // not a frame we care about — resync
            } else {
                ubx.state = (ubx.len == 0) ? UBX_S_CK_A : UBX_S_PAYLOAD;
            }
            break;
        case UBX_S_PAYLOAD:
            ubx.payload[ubx.pos++] = b;
            ubx.ck_a += b;
            ubx.ck_b += ubx.ck_a;
            if (ubx.pos >= ubx.len) ubx.state = UBX_S_CK_A;
            break;
        case UBX_S_CK_A:
            ubx.state = (b == ubx.ck_a) ? UBX_S_CK_B : UBX_S_SYNC1;
            break;
        case UBX_S_CK_B:
            if (b == ubx.ck_b && ubx.msg_class == UBX_CLASS_NAV && ubx.msg_id == UBX_ID_NAV_PVT) {
                db_beacon_handle_nav_pvt(ubx.payload, ubx.len);
            }
            ubx.state = UBX_S_SYNC1;
            break;
        default:
            ubx.state = UBX_S_SYNC1;
            break;
    }
}

// ---------------------------------------------------------------------------
// MAVLink synthesis
// ---------------------------------------------------------------------------

/** UBX fixType -> MAVLink GPS_FIX_TYPE. */
static uint8_t db_beacon_mav_fix_type(void) {
    switch (last_fix.fix_type) {
        case 2: return GPS_FIX_TYPE_2D_FIX;
        case 3: return GPS_FIX_TYPE_3D_FIX;
        case 4: return GPS_FIX_TYPE_3D_FIX;   // GNSS + dead reckoning
        default: return GPS_FIX_TYPE_NO_FIX;
    }
}

/** UBX headMot (deg*1e-5, may be negative) -> MAVLink centidegrees 0..35999. */
static uint16_t db_beacon_heading_cdeg(void) {
    int32_t cdeg = last_fix.head_motion_1e5 / 1000;
    while (cdeg < 0) cdeg += 36000;
    return (uint16_t) (cdeg % 36000);
}

/**
 * Append the MAVLink frames due this tick into one buffer and send it as a
 * single radio packet (one ESP-NOW frame = one airtime slot, so batching the
 * 1 Hz heartbeat/raw-fix with a position sample is essentially free).
 */
static void db_beacon_emit_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns) {
    static uint8_t buf[512];    // worst case HEARTBEAT+GPS_RAW_INT+GLOBAL_POSITION_INT ≈ 130 B
    uint16_t pos = 0;
    const int64_t now_us = esp_timer_get_time();

    if (now_us - last_heartbeat_us >= BEACON_HEARTBEAT_US) {
        last_heartbeat_us = now_us;
        // GENERIC + INVALID identifies the beacon at the GCS (FCs report a real
        // autopilot, the DroneBridge bridge reports ONBOARD_CONTROLLER).
        pos += fmav_msg_heartbeat_pack_to_frame_buf(
                &buf[pos], beacon_sysid, BEACON_COMP_ID,
                MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE,
                &fmav_status_beacon);
    }
    if (fix_ever_received && now_us - last_gps_raw_us >= BEACON_GPS_RAW_US) {
        last_gps_raw_us = now_us;
        pos += fmav_msg_gps_raw_int_pack_to_frame_buf(
                &buf[pos], beacon_sysid, BEACON_COMP_ID,
                (uint64_t) now_us, db_beacon_mav_fix_type(),
                last_fix.lat_1e7, last_fix.lon_1e7, last_fix.hmsl_mm,
                last_fix.pdop_001,          // eph — pDOP*100, same convention as ArduPilot
                UINT16_MAX,                 // epv unknown
                (uint16_t) (last_fix.gspeed_mms / 10),  // cm/s
                db_beacon_heading_cdeg(),
                last_fix.num_sv,
                last_fix.height_mm,         // alt_ellipsoid
                last_fix.h_acc_mm, last_fix.v_acc_mm, last_fix.s_acc_mms,
                last_fix.head_acc_1e5, 0 /* yaw n/a */,
                &fmav_status_beacon);
    }
    if (position_pending) {
        position_pending = false;
        pos += fmav_msg_global_position_int_pack_to_frame_buf(
                &buf[pos], beacon_sysid, BEACON_COMP_ID,
                (uint32_t) (now_us / 1000),
                last_fix.lat_1e7, last_fix.lon_1e7,
                last_fix.hmsl_mm,
                home_set ? (last_fix.hmsl_mm - home_hmsl_mm) : 0,
                (int16_t) (last_fix.vel_n_mms / 10),
                (int16_t) (last_fix.vel_e_mms / 10),
                (int16_t) (last_fix.vel_d_mms / 10),
                db_beacon_heading_cdeg(),
                &fmav_status_beacon);
    }
    if (pos > 0) {
        db_send_to_all_clients(tcp_clients, udp_conns, buf, pos);
    }
}

// ---------------------------------------------------------------------------
// Entry point (called from the control-module loop)
// ---------------------------------------------------------------------------

void db_gps_beacon_process(int *tcp_clients, udp_conn_list_t *udp_conns) {
    const int64_t now_us = esp_timer_get_time();

    if (beacon_sysid == 0) {
        // First call: load the fleet-assigned system id from NVS; a unit the
        // GCS has never numbered heartbeats sysid 1 ("unnumbered" - the same
        // semantics as a factory ArduPilot, so the fleet tab and Assign-IDs
        // flow treat beacons exactly like drones).
        uint8_t assigned = db_beacon_load_assigned_sysid();
        beacon_sysid = (assigned != 0) ? assigned : BEACON_SYSID_UNNUMBERED;
        DB_MAV_SYS_ID = beacon_sysid;
        // Give an already-configured GPS (config persisted to its flash on a
        // previous run) one full retry period to start streaming PVT before we
        // send the config — avoids rewriting the GPS flash on every boot.
        last_gps_cfg_us = now_us;
        ESP_LOGI(TAG, "GPS beacon role active — sysid %i (%s), GPS expected at %li baud",
                 beacon_sysid, (assigned != 0) ? "fleet-assigned" : "unnumbered",
                 (long) DB_PARAM_SERIAL_BAUD);
    }

    // Configure the GPS until it talks to us (a GPS may boot slower than the ESP32).
    if (!fix_ever_received && now_us - last_gps_cfg_us >= BEACON_GPS_CFG_RETRY_US) {
        last_gps_cfg_us = now_us;
        db_beacon_send_gps_config();
    }

    // Drain the serial link through the UBX parser.
    uint8_t read_buf[128];
    int bytes_read = db_read_serial(read_buf, sizeof(read_buf));
    for (int i = 0; i < bytes_read; i++) {
        db_beacon_parse_ubx_byte(read_buf[i]);
    }
    if (bytes_read > 0) {
        serial_total_byte_count += bytes_read;
    }

    db_beacon_emit_mavlink(tcp_clients, udp_conns);
}
