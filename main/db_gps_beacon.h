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

#ifndef DB_ESP32_DB_GPS_BEACON_H
#define DB_ESP32_DB_GPS_BEACON_H

#include "db_esp32_control.h"

/*
 * GPS beacon / armband role (DB_SERIAL_PROTOCOL_UBX_BEACON).
 *
 * The unit's UART carries a u-blox GPS (e.g. MicoAir M10) instead of a flight
 * controller. This module parses UBX NAV-PVT from the GPS and synthesizes the
 * MAVLink stream a tiny "vehicle" would produce - HEARTBEAT (1 Hz) +
 * GPS_RAW_INT (1 Hz) + GLOBAL_POSITION_INT (at the GPS fix rate) - and hands
 * it to the normal radio TX path (db_send_to_all_clients). To the ground
 * station the beacon therefore looks like any other MAVLink node: no GCS
 * transport changes, position lands on the map through the existing pipeline.
 *
 * Identity: HEARTBEAT type MAV_TYPE_GENERIC + MAV_AUTOPILOT_INVALID marks the
 * node as a beacon (flight controllers report a real autopilot; the
 * DroneBridge bridge itself reports MAV_TYPE_ONBOARD_CONTROLLER). The system
 * id comes from the SAME GCS fleet flow as a drone: factory-fresh the beacon
 * heartbeats sysid 1 ("unnumbered"); the GCS's Assign-IDs sends an addressed
 * PARAM_SET SYSID_THISMAV + reboot to the unit's MAC, which the beacon applies
 * to ITSELF (persisted in NVS) instead of forwarding to an FC.
 */

// Called from the control-module loop instead of the FC serial parsers when
// DB_PARAM_SERIAL_PROTO == DB_SERIAL_PROTOCOL_UBX_BEACON. Non-blocking.
void db_gps_beacon_process(int *tcp_clients, udp_conn_list_t *udp_conns);

// Radio->beacon path: scans incoming radio data for DroneBridge ADDRESSED_DATA
// tunnels targeting this unit's MAC and applies the fleet-management messages
// (SYSID_THISMAV assignment, reboot) to the beacon itself. All other radio
// traffic is dropped - it must never reach the GPS on the serial port.
void db_gps_beacon_handle_radio(uint8_t *buffer, int bytes_read);

#endif //DB_ESP32_DB_GPS_BEACON_H
