/*
*   This file is part of DroneBridge:https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2025 Wolfgang Christl
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

#ifndef DB_PARAMETERS_H
#define DB_PARAMETERS_H

#include <string.h>
#include <cJSON.h>
#include <nvs.h>
#include <stdint.h>
#include <driver/gpio.h>
#include "common/common.h"

/**
 * Consumes one formatted parameter line emitted by db_param_stream_values().
 *
 * @param line Formatted parameter line, including its trailing newline.
 * @param line_length Number of bytes in line, excluding a string terminator.
 * @param context Caller-owned context passed through unchanged.
 * @return true to continue streaming or false to stop streaming.
 */
typedef bool (*db_param_output_callback_t)(const uint8_t *line, size_t line_length, void *context);

#define DB_BUILD_VERSION 23
#define DB_MAJOR_VERSION 2
#define DB_MINOR_VERSION 4
#define DB_PATCH_VERSION 1
#define DB_MATURITY_VERSION "stable"
#define DB_TYPE_VERSION 255 // FIRMWARE_VERSION_TYPE_OFFICIAL -> https://mavlink.io/en/messages/common.html#FIRMWARE_VERSION_TYPE

#define DB_PARAM_TOTAL_NUM          29  // total number of db parameters
#define DB_PARAM_MAV_CNT            18  // Number of MAVLink parameters returned by ESP32 in the PARAM message. Needed by GCS.

#define DB_PARAM_NAME_MAXLEN        16      // max len of a parameter/key stored in the ESP32 NVM
#define DB_PARAM_MAX_MAV_PARAM_NAME_LEN 16  // max len of the field used to store the mav param name (max len 16 by def.)
#define DB_PARAM_VALUE_MAXLEN       64      // max len of a value stored for a key in the ESP32 NVM (string len)
#define DB_SERIAL_PACK_SIZE_MAX     1023U   // maximum configured serial packet size; keep below UART_BUF_SIZE
#define MAX_LTM_FRAMES_IN_BUFFER    5


#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_5
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_4
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_6
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_7
#define DB_DEFAULT_UART_BAUD_RATE 57600
#elif CONFIG_DB_OFFICIAL_BOARD_1_X_C6
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_21
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_2
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_22
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_23
#define DB_DEFAULT_UART_BAUD_RATE 57600
#elif CONFIG_DB_GENERIC_BOARD
// initially set pins to 0 to allow the start of the system on all boards. User has to set the correct pins
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_BAUD_RATE 57600
#else
// someone fucked up the config - fallback to generic config
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_BAUD_RATE 57600
#endif

/* ---------- Optional Macros for quick access to param values ---------- */
#define DB_PARAM_WIFI_SSID (char *) db_param_ssid.value.db_param_str.value
#define DB_PARAM_PASS (char *) db_param_pass.value.db_param_str.value
#define DB_PARAM_ESPNOW_LINK_SECRET ((char *) db_param_espnow_link_secret.value.db_param_str.value)
#define DB_PARAM_CHANNEL db_param_channel.value.db_param_u8.value
#define DB_PARAM_RADIO_MODE db_param_radio_mode.value.db_param_u8.value
#define DB_PARAM_STA_IP db_param_wifi_sta_ip.value.db_param_str.value
#define DB_PARAM_STA_GW db_param_wifi_sta_gw.value.db_param_str.value
#define DB_PARAM_STA_IP_NETMASK db_param_wifi_sta_netmask.value.db_param_str.value
#define DB_PARAM_AP_IP (char *) db_param_wifi_ap_ip.value.db_param_str.value
#define DB_PARAM_WIFI_EN_GN db_param_wifi_en_gn.value.db_param_u8.value
#define DB_PARAM_DIS_RADIO_ON_ARM db_param_dis_radio_armed.value.db_param_u8.value
#define DB_PARAM_SERIAL_BAUD db_param_baud.value.db_param_i32.value
#define DB_PARAM_SERIAL_PROTO db_param_proto.value.db_param_u8.value
#define DB_PARAM_SERIAL_PACK_SIZE db_param_serial_pack_size.value.db_param_u16.value
#define DB_PARAM_GPIO_TX db_param_gpio_tx.value.db_param_u8.value
#define DB_PARAM_GPIO_RX db_param_gpio_rx.value.db_param_u8.value
#define DB_PARAM_GPIO_RTS db_param_gpio_rts.value.db_param_u8.value
#define DB_PARAM_GPIO_CTS db_param_gpio_cts.value.db_param_u8.value
#define DB_PARAM_SERIAL_RTS_THRESH db_param_gpio_rts_thresh.value.db_param_u8.value
#define DB_PARAM_EN_EXT_ANT db_param_radio_ant_ext.value.db_param_u8.value
#define DB_PARAM_ESPNOW_AIR2AIR db_param_espnow_air2air.value.db_param_u8.value

/**
 * ESP-NOW addressing mode (docs/decisions/2026-11-10-link-reliability-architecture.md):
 *  0 = broadcast everything - the behaviour this firmware has always had.
 *  1 = control frames addressed, everything else broadcast. Implemented: a frame on
 *      the decision doc's allowlist (db_espnow_frame_is_control(), per direction)
 *      flushes the serial aggregation buffer and leaves alone, addressed to the MAC in
 *      an ADDRESSED_DATA tunnel, else to the single known peer, else broadcast.
 *      Telemetry, heartbeats, RADIO_STATUS, the GND's peer-list frame and the
 *      link-stats report stay broadcast. Needs both ends on firmware that has it - see
 *      db_espnow_seq_space.h for what an older peer does with it.
 *  2 = everything addressed to the single known peer - a diagnostic for measuring
 *      whether addressed frames pass at all, not for flying. Unchanged since it was
 *      introduced so its measurements stay comparable.
 * The routing table for all three is on db_espnow_resolve_destination_for_mode().
 *
 * 802.11 never acknowledges a group-addressed frame, so a broadcast ESP-NOW
 * packet is sent once and a lost one is gone. An addressed frame gets a MAC-layer
 * ACK and automatic hardware retries.
 *
 * **The JONOKR role images default to mode 1** (DB_BUILD_DEFAULT_UNICAST below); a build
 * with no -D DB_ROLE keeps mode 0, i.e. stock DroneBridge behaviour. Measured 2026-11-14
 * at -77 to -81 dBm (memory.md 5.7): what mode 1 removes is not frame loss - telemetry
 * stays broadcast - but the GCS's one-second protocol timeouts. In mode 0 a lost mission
 * item is only noticed when a retry timer expires (4-8 items per upload took 1.1-3.1 s);
 * in mode 1 the MAC retries repair it in milliseconds and no item exceeded 70 ms. The
 * command round trip to the flight controller went from 4 impaired samples in 40, worst
 * case 2.3 s, to none in 80, worst case 183 ms.
 *
 * **Mode 1 requires both ends on ESP32 4f60b57 or newer.** An older AIR still receives
 * everything but its RADIO_STATUS.fixed uplink-loss count becomes meaningless, and a unit
 * older than 58e78a0 discards addressed frames outright. Mode 0 interoperates with any
 * firmware, which is why it stays the no-role default.
 */
#define DB_PARAM_ESPNOW_UNICAST db_param_espnow_unicast.value.db_param_u8.value

enum {
    DB_ESPNOW_UNICAST_OFF = 0,      // broadcast everything
    DB_ESPNOW_UNICAST_CONTROL = 1,  // control frames addressed, everything else broadcast (implemented)
    DB_ESPNOW_UNICAST_ALL = 2,      // everything addressed - diagnostic only, not for flying
};

/**
 * PHY rate ESP-NOW transmits at, as a parameter so it can be changed
 * independently of the addressing mode.
 *
 * These two must never move together. On 2026-11-11 the LR 250 kbps rate config
 * was applied inside peer registration, so enabling addressed sending also
 * dropped the PHY rate 4x. GND->AIR then failed outright while AIR->GND kept
 * working, and no measurement could say which of the two changes caused it -
 * the single-flight send path spends up to 7 MAC retries per frame, and the
 * GND carries the busier direction.
 *
 * DRIVER_DEFAULT leaves the rate alone: ESP-NOW's documented default is 1 Mbps
 * (802.11b), the rate every range number measured before this date was taken
 * at. The LR settings are the open question of whether
 * esp_wifi_set_protocol(.. | WIFI_PROTOCOL_LR) alone ever put ESP-NOW data
 * frames on an LR rate - see memory.md section 5. Set it alone, with
 * ESPNOW_UNICAST off, and walk the ladder.
 */
#define DB_PARAM_ESPNOW_RATE db_param_espnow_rate.value.db_param_u8.value

enum {
    DB_ESPNOW_RATE_DRIVER_DEFAULT = 0,  // do not call the rate API at all
    DB_ESPNOW_RATE_LR_250K = 1,         // WIFI_PHY_MODE_LR @ WIFI_PHY_RATE_LORA_250K
    DB_ESPNOW_RATE_LR_500K = 2,         // WIFI_PHY_MODE_LR @ WIFI_PHY_RATE_LORA_500K
};

enum E_DB_WIFI_MODE {
  DB_WIFI_MODE_AP         = 1, // Wi-Fi access point mode with 802.11b mode enabled
  DB_WIFI_MODE_STA        = 2, // Wi-Fi client mode with 802.11b and LR mode enabled
  DB_WIFI_MODE_AP_LR      = 3, // ESP32 WiFi LR Mode 802.11b
  DB_WIFI_MODE_ESPNOW_AIR = 4, // ESP-NOW Mode for broadcasting device
  DB_WIFI_MODE_ESPNOW_GND = 5, // ESP-NOW Mode for GND station
  DB_BLUETOOTH_MODE       = 6, // Bluetooth BLE mode
  DB_WIFI_MODE_END        = 7, // End of enum
};

enum E_DB_SERIAL_PROTOCOL {
    DB_SERIAL_PROTOCOL_MSPLTM = 1,
    DB_SERIAL_PROTOCOL_MAVLINK = 4,
    DB_SERIAL_PROTOCOL_TRANSPARENT = 5,
    // JONOKR beacon/armband role: the UART carries a u-blox GPS instead of a
    // flight controller; the ESP32 itself synthesizes the MAVLink position
    // stream. See db_gps_beacon.c.
    DB_SERIAL_PROTOCOL_UBX_BEACON = 6
};

/*
 * Role-baked build defaults (JONOKR provisioning). The flasher GUI builds one
 * image per unit role (air / ground / beacon) with these overridden on the
 * compiler command line (see main/CMakeLists.txt + build-fw.ps1 -Role), so a
 * freshly-flashed (erased) unit boots straight into its role with zero web-UI
 * configuration. Unset builds keep the stock DroneBridge defaults.
 */
#ifndef DB_BUILD_DEFAULT_RADIO_MODE
#define DB_BUILD_DEFAULT_RADIO_MODE DB_WIFI_MODE_AP
#endif
#ifndef DB_BUILD_DEFAULT_SERIAL_PROTO
#define DB_BUILD_DEFAULT_SERIAL_PROTO DB_SERIAL_PROTOCOL_MAVLINK
#endif
#ifndef DB_BUILD_DEFAULT_BAUD
#define DB_BUILD_DEFAULT_BAUD DB_DEFAULT_UART_BAUD_RATE
#endif
#ifndef DB_BUILD_DEFAULT_GPIO_TX
#define DB_BUILD_DEFAULT_GPIO_TX DB_DEFAULT_UART_TX_PIN
#endif
#ifndef DB_BUILD_DEFAULT_GPIO_RX
#define DB_BUILD_DEFAULT_GPIO_RX DB_DEFAULT_UART_RX_PIN
#endif
// Fallbacks match the pre-existing stock defaults in db_parameters.c (channel
// 6, internal antenna) so a build with no -D DB_ROLE is unchanged.
#ifndef DB_BUILD_DEFAULT_CHANNEL
#define DB_BUILD_DEFAULT_CHANNEL 6
#endif
#ifndef DB_BUILD_DEFAULT_EN_EXT_ANT
#define DB_BUILD_DEFAULT_EN_EXT_ANT false
#endif
/* ESP-NOW addressing mode a freshly-erased unit boots with. The role images set this to
 * DB_ESPNOW_UNICAST_CONTROL; the fallback is OFF so a build without -D DB_ROLE behaves
 * exactly like stock DroneBridge. See DB_PARAM_ESPNOW_UNICAST above for the evidence and
 * for the firmware-version requirement this carries. */
#ifndef DB_BUILD_DEFAULT_UNICAST
#define DB_BUILD_DEFAULT_UNICAST DB_ESPNOW_UNICAST_OFF
#endif

typedef struct db_parameter_str_s {
    uint8_t *value;
    uint8_t *default_value;
    uint8_t min_len;    // valid if >= min_len
    uint8_t max_len;    // allocated capacity including NUL; valid string length is < max_len
} db_param_str_t;

typedef struct db_parameter_u8_s {
    uint8_t value;
    uint8_t default_value;
    uint8_t min;
    uint8_t max;
} db_param_u8_t;

typedef struct db_parameter_u16_s {
    uint16_t value;
    uint16_t default_value;
    uint16_t min;
    uint16_t max;
} db_param_u16_t;

typedef struct db_parameter_i32_s {
    int32_t value;
    int32_t default_value;
    int32_t min;
    int32_t max;
} db_param_i32_t;

enum db_param_type_t {
  STRING,
  UINT8,
  UINT16,
  INT32
};

typedef struct db_parameter_s {
    uint8_t db_name[DB_PARAM_NAME_MAXLEN];  // equals the NVS key, JSON object name and web interface input ID
    enum db_param_type_t type;
    struct {    // no string support for mavlink
      uint8_t param_name[DB_PARAM_MAX_MAV_PARAM_NAME_LEN];
      uint16_t param_index;         // strings will be ignored for mavlink set to -1 for strings
      MAV_PARAM_TYPE param_type;    // set to MAV_PARAM_TYPE_ENUM_END for strings
    } mav_t;
    union {
      db_param_str_t db_param_str;
      db_param_u8_t db_param_u8;
      db_param_u16_t db_param_u16;
      db_param_i32_t db_param_i32;
    } value;
} db_parameter_t;

extern uint8_t DB_RADIO_MODE_DESIGNATED;
extern db_parameter_t *db_params[DB_PARAM_TOTAL_NUM];

extern db_parameter_t db_param_ssid;
extern db_parameter_t db_param_pass;
extern db_parameter_t db_param_espnow_link_secret;
extern db_parameter_t db_param_wifi_ap_ip;
extern db_parameter_t db_param_wifi_sta_ip;
extern db_parameter_t db_param_wifi_sta_gw;
extern db_parameter_t db_param_wifi_sta_netmask;
extern db_parameter_t db_param_udp_client_ip;
extern db_parameter_t db_param_wifi_hostname;
extern db_parameter_t db_param_radio_mode;
extern db_parameter_t db_param_channel;
extern db_parameter_t db_param_wifi_en_gn;
extern db_parameter_t db_param_radio_ant_ext;
extern db_parameter_t db_param_baud;
extern db_parameter_t db_param_gpio_tx;
extern db_parameter_t db_param_gpio_rx;
extern db_parameter_t db_param_gpio_rts;
extern db_parameter_t db_param_gpio_cts;
extern db_parameter_t db_param_gpio_rts_thresh;
extern db_parameter_t db_param_proto;
extern db_parameter_t db_param_serial_pack_size;
extern db_parameter_t db_param_serial_read_timeout;
extern db_parameter_t db_param_ltm_per_packet;
extern db_parameter_t db_param_dis_radio_armed;
extern db_parameter_t db_param_udp_client_port;
extern db_parameter_t db_param_rssi_dbm;
extern db_parameter_t db_param_espnow_air2air;
extern db_parameter_t db_param_espnow_unicast;
extern db_parameter_t db_param_espnow_rate;

void db_param_init_parameters();
void db_param_set_to_default(db_parameter_t *db_parameter);
void db_param_reset_all();
bool db_param_stream_values(db_param_output_callback_t output_callback, void *context);
void db_param_log_values();
void db_param_read_all_params_nvs(const nvs_handle_t *nvs_handle);
void db_param_write_all_params_nvs(const nvs_handle_t *nvs_handle);
void db_param_read_all_params_json(const cJSON *root_obj);
void db_param_write_all_params_json(cJSON *root_obj);

bool db_param_is_valid_str(char *new_string_value, db_parameter_t *target_param);
bool db_param_is_valid_u8(uint8_t new_u8_value, db_parameter_t *target_param);
bool db_param_is_valid_u16(uint16_t new_u16_value, db_parameter_t *target_param);
bool db_param_is_valid_i32(int32_t new_i32_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_str(char *new_string_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_u8(uint8_t new_u8_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_u16(uint16_t new_u16_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_i32(int32_t new_i32_value, db_parameter_t *target_param);
bool is_valid_ip4(const char *ipaddress);

#endif //DB_PARAMETERS_H
