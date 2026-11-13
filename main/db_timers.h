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

#ifndef DB_ESP32_DB_TIMERS_H
#define DB_ESP32_DB_TIMERS_H

#define DB_TIMER_RSSI_PERIOD_MS 1000
#define DB_TIMER_MAVLINK_HEARTBEAT_MS 1000 // Heartbeat every second
#define DB_TIMER_MAVLINK_RADIOSTATUS_MS 1000 // Radio Status every second
#define DB_TIMER_MAVLINK_FLEET_LIST_MS 1000 // GND fleet-list TUNNEL to GCS every second
// Link-stats TUNNEL (db_espnow_link_stats.h) every 2 s = 0.5 Hz. The counters are cumulative, so the rate only sets
// the time resolution of a ratio, never its accuracy, and a lost report costs nothing but resolution. Cost on air per
// AIR unit: one ESP-NOW packet carrying a <= 79 B MAVLink frame (10 header + 5 TUNNEL fields + 62 + 2 CRC) every 2 s,
// i.e. <= ~40 B/s, ~11 % of the ~344 B/s telemetry budget (GroundControl DEFAULT_RATES_HZ) - kept below RADIO_STATUS's
// 1 Hz because it is a diagnostic, not a live display. The GND's copy goes over USB and costs no airtime.
#define DB_TIMER_LINK_STATS_MS 2000
// Periodic LED state evaluation interval (db_status_led_process()).
#define DB_TIMER_STATUS_LED_MS 1000

void db_timer_start_wifi_rssi_timer();

void db_timer_start_mavlink_heartbeat();

void db_timer_start_mavlink_radio_status();

// Starts the GND-only periodic fleet-list TUNNEL emitter (self-gates on role).
void db_timer_start_mavlink_fleet_list();

// Starts the periodic link-stats TUNNEL emitter (self-gates to the ESP-NOW roles).
void db_timer_start_link_stats();

// Starts periodic processing for the C6 status LED logic.
void db_timer_start_status_led();

#endif // DB_ESP32_DB_TIMERS_H
