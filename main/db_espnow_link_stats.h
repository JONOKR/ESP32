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
 * ESP-NOW link statistics: the counters each unit keeps about its own radio path, and the
 * wire format that carries them to the ground station.
 *
 * Pure C on purpose - no ESP-IDF, no FreeRTOS - so the format is pinned by the host test
 * suite under test/. The firmware-side glue (when to emit, which MAVLink status to encode
 * with) lives in db_mavlink_msgs.c and db_timers.c.
 *
 * WHY THESE COUNTERS EXIST
 * RADIO_STATUS already carries two loss counts, both measured at the *receiver* from gaps
 * in the sender's ESP-NOW sequence number: `rxerrors` (downlink, counted on the GND) and
 * `fixed` (uplink, counted on the AIR). A count without a denominator is not a ratio, and
 * the two directions carry very different packet volumes. `sent_broadcast + sent_addressed`
 * on one unit is exactly the population its peer's sequence-gap counter observes: the
 * sequence number is incremented once per esp_now_send() that returned ESP_OK, for every
 * ESP-NOW packet type (user data, the GND's peer-list frame, and this report itself), and
 * these two counters are incremented at exactly that point. Packets that never reached the
 * radio (queue full, allocation failure, esp_now_send() error) consume no sequence number
 * and are counted separately below, so they can never be mistaken for air loss.
 *
 * ============================================================================================
 * WIRE FORMAT - "link stats" report, version 1
 * ============================================================================================
 * Carrier: MAVLink v2 TUNNEL (msg id 385), payload_type = 0xDB03
 *          (DB_ESPNOW_TUNNEL_LINK_STATS), target_system = 0, target_component = 0,
 *          payload_length = 62 for version 1. MAVLink header sysid = the unit's MAVLink
 *          system id (db_get_mav_sys_id(), may be 0 before it has learnt one), compid = 68
 *          (MAV_COMP_ID_TELEMETRY_RADIO). Do NOT identify the unit by sysid - several AIR units
 *          can share sysid 1 before fleet numbering; use the MAC in the payload.
 *
 * All multi-byte fields little-endian, no padding. Offsets in bytes into TUNNEL.payload:
 *
 *   off size type    field
 *    0   1   u8      version              = 1
 *    1   1   u8      role                 0 = GND, 1 = AIR (flight controller), 2 = AIR (GPS beacon)
 *    2   6   u8[6]   mac                  this unit's own ESP-NOW (WiFi STA) MAC, as the peer sees it
 *    8   1   u8      espnow_unicast       ESPNOW_UNICAST parameter value at emission time
 *    9   1   u8      espnow_rate          ESPNOW_RATE parameter value at emission time
 *   10   4   u32     uptime_ms            ms since boot, wraps after ~49.7 days; a decrease means
 *                                         the unit rebooted and every counter restarted at 0
 *   14   4   u32     sent_broadcast       ESP-NOW packets handed to the radio, broadcast destination
 *   18   4   u32     sent_addressed       ESP-NOW packets handed to the radio, addressed to a peer MAC
 *   22   4   u32     addressed_delivered  addressed sends whose callback said ESP_NOW_SEND_SUCCESS
 *                                         (MAC-layer ACK received)
 *   26   4   u32     addressed_failed     addressed sends whose callback said ESP_NOW_SEND_FAIL
 *                                         (no ACK after the driver's own retries)
 *   30   4   u32     broadcast_failed     broadcast sends whose callback said ESP_NOW_SEND_FAIL
 *                                         (the driver could not transmit it at all)
 *   34   4   u32     send_error           packets taken off the send queue that never reached the
 *                                         radio: oversized, encryption failure, or esp_now_send()
 *                                         returned an error. Consumed no sequence number.
 *   38   4   u32     drop_send_queue_full packets dropped because the ESP-NOW send queue was full
 *   42   4   u32     drop_uart_queue_full received packets dropped because the serial-write queue
 *                                         was full (decrypted fine, never reached the serial port)
 *   46   4   u32     drop_rx_queue_full   received packets dropped in the WiFi receive callback
 *                                         because the receive-event queue was full
 *   50   4   u32     drop_alloc_fail      packets dropped because malloc() failed, anywhere in the
 *                                         ESP-NOW path (either direction)
 *   54   4   u32     send_cb_timeout      send-path recoveries: no send callback arrived within
 *                                         DB_ESPNOW_SEND_CB_TIMEOUT_MS, so the in-flight packet was
 *                                         written off (its fate is unknown) and sending resumed
 *   58   4   u32     send_cb_late         send callbacks that arrived with nothing in flight, i.e.
 *                                         after a timeout recovery; ignored for delivery counting
 *
 * Every counter is free-running since boot and wraps modulo 2^32 - read them as deltas with
 * unsigned 32-bit arithmetic. Each sent packet ends in exactly one of: delivered, failed,
 * broadcast-ok (sent_broadcast - broadcast_failed), or written off by a timeout; so with no
 * timeouts, sent_addressed == addressed_delivered + addressed_failed (+1 while one is in
 * flight).
 *
 * MAVLink v2 truncates trailing zero bytes of a message payload; pymavlink re-pads them, so
 * a reader must slice payload[:payload_length] and treat missing bytes as zero.
 *
 * Compatibility rules: fields are only ever APPENDED, and payload_length grows with them; a
 * reader of version 1 must accept payload_length >= 62 and ignore what follows. `version`
 * changes only if an existing field changes meaning or position - a reader must drop a report
 * whose version it does not know.
 *
 * Emission (see db_timer_link_stats_callback): every DB_TIMER_LINK_STATS_MS (2000 ms), only in
 * the ESP-NOW roles, only when the serial protocol is MAVLink (or the GPS beacon).
 *   - GND: written straight to its serial link (USB to the GCS). Never put on air.
 *   - AIR / beacon: sent over ESP-NOW, ALWAYS broadcast whatever ESPNOW_UNICAST says, in its
 *     own ESP-NOW packet (never aggregated with RADIO_STATUS - a longer frame has a higher
 *     error rate and would bias that instrument). The GND forwards it to the GCS unchanged,
 *     exactly like the AIR's RADIO_STATUS.
 * ============================================================================================
 */

#ifndef DB_ESP32_DB_ESPNOW_LINK_STATS_H
#define DB_ESP32_DB_ESPNOW_LINK_STATS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "db_espnow_limits.h"

/** Version byte of the report layout described above. */
#define DB_LINK_STATS_VERSION 1U
/** Length in bytes of a version-1 report. */
#define DB_LINK_STATS_V1_LEN 62U

/** Values of the report's role byte. GND/AIR match DB_ESPNOW_ORIGIN_GND/_AIR on purpose. */
#define DB_LINK_STATS_ROLE_GND         0U
#define DB_LINK_STATS_ROLE_AIR         1U
#define DB_LINK_STATS_ROLE_AIR_BEACON  2U

/**
 * Free-running counters, one instance per unit (db_link_stats). Written from three tasks - the
 * WiFi driver task (receive/send callbacks), the ESP-NOW task and the tasks that enqueue data
 * (control task, timer task) - so every write goes through db_link_stats_inc().
 */
typedef struct {
    uint32_t sent_broadcast;
    uint32_t sent_addressed;
    uint32_t addressed_delivered;
    uint32_t addressed_failed;
    uint32_t broadcast_failed;
    uint32_t send_error;
    uint32_t drop_send_queue_full;
    uint32_t drop_uart_queue_full;
    uint32_t drop_rx_queue_full;
    uint32_t drop_alloc_fail;
    uint32_t send_cb_timeout;
    uint32_t send_cb_late;
} db_link_stats_counters_t;

/** Everything a report carries: who is reporting, under which settings, and the counters. */
typedef struct {
    uint8_t role;
    uint8_t mac[DB_ESPNOW_MAC_ADDR_LEN];
    uint8_t espnow_unicast;
    uint8_t espnow_rate;
    uint32_t uptime_ms;
    db_link_stats_counters_t counters;
} db_link_stats_report_t;

/** This unit's counters since boot. */
extern db_link_stats_counters_t db_link_stats;

/**
 * Increment one counter, safely against concurrent writers.
 *
 * Atomic rather than a plain ++ because the same counter can be bumped from two tasks (the
 * ESP-NOW enqueue runs in both the control task and the timer task) and the ESP32 classic and
 * S3 run those on two cores at once; a lost read-modify-write would silently undercount
 * exactly the events this exists to expose. Relaxed ordering: the counters are independent
 * statistics, nothing synchronises on them. On cores without atomic instructions (C3, S2) the
 * compiler emits a libatomic call that ESP-IDF implements with a short critical section.
 *
 * @param counter Pointer to a field of a db_link_stats_counters_t
 */
static inline void db_link_stats_inc(uint32_t *counter) {
    (void) __atomic_fetch_add(counter, 1U, __ATOMIC_RELAXED);
}

/**
 * Copy every counter with a single atomic load each, so a report never contains a torn value.
 * The counters are not mutually consistent at one instant (another task may bump one between
 * two loads); that skew is at most a packet or two and irrelevant at the reporting rate.
 *
 * @param src Counters to read (normally &db_link_stats)
 * @param dst Destination snapshot
 */
void db_link_stats_snapshot(const db_link_stats_counters_t *src, db_link_stats_counters_t *dst);

/**
 * Serialise a report into the version-1 wire format documented at the top of this file.
 *
 * Written field by field with explicit little-endian byte order rather than by memcpy of a
 * packed struct, so the wire format does not depend on the compiler or the host's endianness
 * - the host tests run on x86, the firmware on Xtensa and RISC-V.
 *
 * @param report Report to serialise
 * @param out Output buffer
 * @param out_len Size of the output buffer
 * @return Number of bytes written (DB_LINK_STATS_V1_LEN), or 0 if out is too small or an
 *         argument is NULL
 */
size_t db_link_stats_pack(const db_link_stats_report_t *report, uint8_t *out, size_t out_len);

/**
 * Parse a report in the wire format. Reference implementation of the reader side: the host
 * tests use it to pin the format, and it states the acceptance rules a ground-station parser
 * must follow (known version, payload at least DB_LINK_STATS_V1_LEN bytes, extra bytes
 * ignored).
 *
 * @param in Received TUNNEL payload (the first payload_length bytes)
 * @param in_len payload_length
 * @param report Output
 * @return true if the report was accepted, false if it was too short, of an unknown version,
 *         or an argument was NULL
 */
bool db_link_stats_unpack(const uint8_t *in, size_t in_len, db_link_stats_report_t *report);

#endif // DB_ESP32_DB_ESPNOW_LINK_STATS_H
