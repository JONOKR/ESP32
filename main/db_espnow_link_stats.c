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

#include <string.h>

#include "db_espnow_link_stats.h"

db_link_stats_counters_t db_link_stats = {0};

/**
 * Atomically load one counter.
 *
 * @param counter Counter to read
 * @return Its current value
 */
static uint32_t db_link_stats_load(const uint32_t *counter) {
    return __atomic_load_n(counter, __ATOMIC_RELAXED);
}

/**
 * See db_espnow_link_stats.h.
 *
 * @param src Counters to read
 * @param dst Destination snapshot
 */
void db_link_stats_snapshot(const db_link_stats_counters_t *src, db_link_stats_counters_t *dst) {
    if (src == NULL || dst == NULL) {
        return;
    }
    dst->sent_broadcast = db_link_stats_load(&src->sent_broadcast);
    dst->sent_addressed = db_link_stats_load(&src->sent_addressed);
    dst->addressed_delivered = db_link_stats_load(&src->addressed_delivered);
    dst->addressed_failed = db_link_stats_load(&src->addressed_failed);
    dst->broadcast_failed = db_link_stats_load(&src->broadcast_failed);
    dst->send_error = db_link_stats_load(&src->send_error);
    dst->drop_send_queue_full = db_link_stats_load(&src->drop_send_queue_full);
    dst->drop_uart_queue_full = db_link_stats_load(&src->drop_uart_queue_full);
    dst->drop_rx_queue_full = db_link_stats_load(&src->drop_rx_queue_full);
    dst->drop_alloc_fail = db_link_stats_load(&src->drop_alloc_fail);
    dst->send_cb_timeout = db_link_stats_load(&src->send_cb_timeout);
    dst->send_cb_late = db_link_stats_load(&src->send_cb_late);
}

/**
 * Write a 32-bit value little-endian.
 *
 * @param out Destination, at least 4 bytes
 * @param value Value to write
 */
static void db_link_stats_put_u32(uint8_t *out, const uint32_t value) {
    out[0] = (uint8_t) (value & 0xFFU);
    out[1] = (uint8_t) ((value >> 8) & 0xFFU);
    out[2] = (uint8_t) ((value >> 16) & 0xFFU);
    out[3] = (uint8_t) ((value >> 24) & 0xFFU);
}

/**
 * Read a little-endian 32-bit value.
 *
 * @param in Source, at least 4 bytes
 * @return The decoded value
 */
static uint32_t db_link_stats_get_u32(const uint8_t *in) {
    return (uint32_t) in[0] | ((uint32_t) in[1] << 8) | ((uint32_t) in[2] << 16) | ((uint32_t) in[3] << 24);
}

/**
 * See db_espnow_link_stats.h. The order of the put_u32 calls below IS the wire format; keep
 * it in step with the table in the header and with test/db_espnow_link_stats_test.c.
 *
 * @param report Report to serialise
 * @param out Output buffer
 * @param out_len Size of the output buffer
 * @return Bytes written, or 0 on error
 */
size_t db_link_stats_pack(const db_link_stats_report_t *report, uint8_t *out, size_t out_len) {
    if (report == NULL || out == NULL || out_len < DB_LINK_STATS_V1_LEN) {
        return 0;
    }
    const db_link_stats_counters_t *c = &report->counters;
    out[0] = DB_LINK_STATS_VERSION;
    out[1] = report->role;
    memcpy(&out[2], report->mac, DB_ESPNOW_MAC_ADDR_LEN);
    out[8] = report->espnow_unicast;
    out[9] = report->espnow_rate;
    db_link_stats_put_u32(&out[10], report->uptime_ms);
    db_link_stats_put_u32(&out[14], c->sent_broadcast);
    db_link_stats_put_u32(&out[18], c->sent_addressed);
    db_link_stats_put_u32(&out[22], c->addressed_delivered);
    db_link_stats_put_u32(&out[26], c->addressed_failed);
    db_link_stats_put_u32(&out[30], c->broadcast_failed);
    db_link_stats_put_u32(&out[34], c->send_error);
    db_link_stats_put_u32(&out[38], c->drop_send_queue_full);
    db_link_stats_put_u32(&out[42], c->drop_uart_queue_full);
    db_link_stats_put_u32(&out[46], c->drop_rx_queue_full);
    db_link_stats_put_u32(&out[50], c->drop_alloc_fail);
    db_link_stats_put_u32(&out[54], c->send_cb_timeout);
    db_link_stats_put_u32(&out[58], c->send_cb_late);
    return DB_LINK_STATS_V1_LEN;
}

/**
 * See db_espnow_link_stats.h.
 *
 * @param in Received payload
 * @param in_len Its length
 * @param report Output
 * @return true if accepted
 */
bool db_link_stats_unpack(const uint8_t *in, size_t in_len, db_link_stats_report_t *report) {
    if (in == NULL || report == NULL || in_len < DB_LINK_STATS_V1_LEN || in[0] != DB_LINK_STATS_VERSION) {
        return false;
    }
    db_link_stats_counters_t *c = &report->counters;
    report->role = in[1];
    memcpy(report->mac, &in[2], DB_ESPNOW_MAC_ADDR_LEN);
    report->espnow_unicast = in[8];
    report->espnow_rate = in[9];
    report->uptime_ms = db_link_stats_get_u32(&in[10]);
    c->sent_broadcast = db_link_stats_get_u32(&in[14]);
    c->sent_addressed = db_link_stats_get_u32(&in[18]);
    c->addressed_delivered = db_link_stats_get_u32(&in[22]);
    c->addressed_failed = db_link_stats_get_u32(&in[26]);
    c->broadcast_failed = db_link_stats_get_u32(&in[30]);
    c->send_error = db_link_stats_get_u32(&in[34]);
    c->drop_send_queue_full = db_link_stats_get_u32(&in[38]);
    c->drop_uart_queue_full = db_link_stats_get_u32(&in[42]);
    c->drop_rx_queue_full = db_link_stats_get_u32(&in[46]);
    c->drop_alloc_fail = db_link_stats_get_u32(&in[50]);
    c->send_cb_timeout = db_link_stats_get_u32(&in[54]);
    c->send_cb_late = db_link_stats_get_u32(&in[58]);
    return true;
}
