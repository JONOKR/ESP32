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
 * Host tests for the ESP-NOW link-stats wire format and the transmit-path decision logic
 * (destination selection, single-flight tracker with its deadline). The byte offsets below are
 * written out by hand on purpose: they ARE the contract with the ground-station parser, so a
 * change to db_link_stats_pack() that moves a field must fail here.
 */

#undef NDEBUG
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "db_espnow_link_stats.h"
#include "db_espnow_tx_path.h"
#include "common/common.h"

static const uint8_t MAC_A[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
static const uint8_t MAC_B[6] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};
static const uint8_t MAC_BC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/**
 * Reads a little-endian u32 at an offset, independently of the code under test.
 *
 * @param p Buffer
 * @param off Offset
 * @return Decoded value
 */
static uint32_t le32(const uint8_t *p, size_t off) {
    return (uint32_t) p[off] | ((uint32_t) p[off + 1] << 8) | ((uint32_t) p[off + 2] << 16) |
           ((uint32_t) p[off + 3] << 24);
}

/**
 * Builds a report whose every counter has a distinct value, so a swapped field cannot pass.
 *
 * @return The report
 */
static db_link_stats_report_t sample_report(void) {
    db_link_stats_report_t r;
    memset(&r, 0, sizeof(r));
    r.role = DB_LINK_STATS_ROLE_AIR_BEACON;
    memcpy(r.mac, MAC_A, 6);
    r.espnow_unicast = 2;
    r.espnow_rate = 1;
    r.uptime_ms = 0x01020304U;
    r.counters.sent_broadcast = 0x11111111U;
    r.counters.sent_addressed = 0x22222222U;
    r.counters.addressed_delivered = 0x33333333U;
    r.counters.addressed_failed = 0x44444444U;
    r.counters.broadcast_failed = 0x55555555U;
    r.counters.send_error = 0x66666666U;
    r.counters.drop_send_queue_full = 0x77777777U;
    r.counters.drop_uart_queue_full = 0x88888888U;
    r.counters.drop_rx_queue_full = 0x99999999U;
    r.counters.drop_alloc_fail = 0xAAAAAAAAU;
    r.counters.send_cb_timeout = 0xBBBBBBBBU;
    r.counters.send_cb_late = 0xDEADBEEFU;
    return r;
}

/**
 * Pins every byte offset of the version-1 format, and little-endian order.
 */
static void test_pack_layout(void) {
    const db_link_stats_report_t r = sample_report();
    uint8_t out[128];
    memset(out, 0xEE, sizeof(out));
    assert(db_link_stats_pack(&r, out, sizeof(out)) == 62U);
    assert(DB_LINK_STATS_V1_LEN == 62U);
    assert(out[0] == 1U);                       // version
    assert(out[1] == 2U);                       // role: beacon
    assert(memcmp(&out[2], MAC_A, 6) == 0);
    assert(out[8] == 2U && out[9] == 1U);       // unicast mode, rate
    assert(out[10] == 0x04 && out[11] == 0x03 && out[12] == 0x02 && out[13] == 0x01);  // LE
    assert(le32(out, 10) == 0x01020304U);
    assert(le32(out, 14) == 0x11111111U);
    assert(le32(out, 18) == 0x22222222U);
    assert(le32(out, 22) == 0x33333333U);
    assert(le32(out, 26) == 0x44444444U);
    assert(le32(out, 30) == 0x55555555U);
    assert(le32(out, 34) == 0x66666666U);
    assert(le32(out, 38) == 0x77777777U);
    assert(le32(out, 42) == 0x88888888U);
    assert(le32(out, 46) == 0x99999999U);
    assert(le32(out, 50) == 0xAAAAAAAAU);
    assert(le32(out, 54) == 0xBBBBBBBBU);
    assert(le32(out, 58) == 0xDEADBEEFU);
    assert(out[62] == 0xEE);                    // nothing written past the record
}

/**
 * Round trip, plus the reader's acceptance rules: short, unknown version, trailing extension bytes.
 */
static void test_unpack_rules(void) {
    const db_link_stats_report_t r = sample_report();
    uint8_t buf[80];
    memset(buf, 0, sizeof(buf));
    assert(db_link_stats_pack(&r, buf, sizeof(buf)) == 62U);
    db_link_stats_report_t back;
    memset(&back, 0, sizeof(back));
    assert(db_link_stats_unpack(buf, 62U, &back));
    assert(memcmp(&back, &r, sizeof(r)) == 0);

    // A later firmware appends fields: a v1 reader must still accept it and ignore the rest.
    buf[62] = 0x5A;
    assert(db_link_stats_unpack(buf, 70U, &back));
    assert(back.counters.send_cb_late == 0xDEADBEEFU);

    assert(!db_link_stats_unpack(buf, 61U, &back));     // truncated
    buf[0] = 2U;
    assert(!db_link_stats_unpack(buf, 62U, &back));     // unknown version
    buf[0] = 1U;
    assert(db_link_stats_pack(&r, buf, 61U) == 0U);     // output too small
    assert(db_link_stats_pack(NULL, buf, sizeof(buf)) == 0U);
}

/**
 * The record must fit a MAVLink TUNNEL payload, and a TUNNEL frame carrying it must fit one ESP-NOW packet
 * (DB_ESPNOW_PAYLOAD_MAXSIZE = 250 - 12 - 16 - 6 - 1 = 215), so it never gets split across packets.
 */
static void test_fits_tunnel_and_packet(void) {
    fmav_tunnel_t t;
    memset(&t, 0, sizeof(t));
    assert(sizeof(t.payload) >= DB_LINK_STATS_V1_LEN);
    const db_link_stats_report_t r = sample_report();
    t.payload_type = 0xDB03;
    t.payload_length = (uint8_t) db_link_stats_pack(&r, t.payload, sizeof(t.payload));
    uint8_t frame[FASTMAVLINK_FRAME_LEN_MAX];
    fmav_status_t st;
    memset(&st, 0, sizeof(st));
    const uint16_t len = fmav_msg_tunnel_encode_to_frame_buf(frame, 1, 68, &t, &st);
    assert(len <= 79U);     // 10 header + 5 fixed TUNNEL fields + 62 + 2 CRC (trailing zeros may shorten it)
    assert(len <= 215U);
}

/**
 * Snapshot copies every field, and db_link_stats_inc counts.
 */
static void test_snapshot_and_inc(void) {
    db_link_stats_counters_t c;
    memset(&c, 0, sizeof(c));
    db_link_stats_inc(&c.drop_alloc_fail);
    db_link_stats_inc(&c.drop_alloc_fail);
    c.send_cb_late = UINT32_MAX;
    db_link_stats_inc(&c.send_cb_late);         // wraps modulo 2^32 by design
    db_link_stats_counters_t s;
    memset(&s, 0xFF, sizeof(s));
    db_link_stats_snapshot(&c, &s);
    assert(memcmp(&s, &c, sizeof(c)) == 0);
    assert(s.drop_alloc_fail == 2U && s.send_cb_late == 0U);
}

/**
 * Today's AUTO rule, now isolated: AIR -> known GND; GND -> only with exactly one peer; never in mode 0.
 */
static void test_auto_target(void) {
    // mode 0: never an address, in either role
    assert(db_espnow_auto_target_candidate(false, true, true, MAC_A, 0, NULL) == NULL);
    assert(db_espnow_auto_target_candidate(false, false, false, NULL, 1, MAC_B) == NULL);
    // AIR
    assert(db_espnow_auto_target_candidate(true, true, true, MAC_A, 5, MAC_B) == MAC_A);
    assert(db_espnow_auto_target_candidate(true, true, false, MAC_A, 1, MAC_B) == NULL);
    // GND: exactly one peer
    assert(db_espnow_auto_target_candidate(true, false, false, NULL, 1, MAC_B) == MAC_B);
    assert(db_espnow_auto_target_candidate(true, false, false, NULL, 0, MAC_B) == NULL);
    assert(db_espnow_auto_target_candidate(true, false, false, NULL, 2, MAC_B) == NULL);
}

/**
 * Destination resolution for the three routing modes.
 */
static void test_resolve_destination(void) {
    // AUTO: whatever today's rule says, and broadcast in mode 0 even if handed a target
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_AUTO, NULL, false, true, MAC_A) == MAC_A);
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_AUTO, NULL, false, true, NULL) == NULL);
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_AUTO, NULL, false, false, MAC_A) == NULL);
    // BROADCAST: always, in every mode - the Trap 9 guard for the instruments
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_BROADCAST, MAC_B, true, true, MAC_A) == NULL);
    // PEER: used when enabled and registered
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_PEER, MAC_B, true, true, MAC_A) == MAC_B);
    // PEER not registered -> falls back to AUTO, not to "drop"
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_PEER, MAC_B, false, true, MAC_A) == MAC_A);
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_PEER, MAC_B, false, true, NULL) == NULL);
    // PEER in mode 0 -> broadcast, even if registered: mode 0 broadcasts everything by construction
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_PEER, MAC_B, true, false, NULL) == NULL);
    // PEER with no MAC, or the broadcast MAC -> AUTO
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_PEER, NULL, true, true, MAC_A) == MAC_A);
    assert(db_espnow_resolve_destination(DB_ESPNOW_DEST_PEER, MAC_BC, true, true, NULL) == NULL);
    // unknown mode value -> AUTO
    assert(db_espnow_resolve_destination((db_espnow_dest_mode_t) 77, MAC_B, true, true, MAC_A) == MAC_A);
}

/**
 * Single flight, delivery accounting, and the deadline that keeps a lost callback from wedging the path.
 */
static void test_tracker(void) {
    db_espnow_tx_tracker_t t;
    db_link_stats_counters_t c;
    memset(&c, 0, sizeof(c));
    db_espnow_tx_tracker_init(&t);
    assert(db_espnow_tx_ready(&t));

    // broadcast, success
    db_espnow_tx_on_sent(&t, false, 1000U, &c);
    assert(!db_espnow_tx_ready(&t));
    assert(c.sent_broadcast == 1U && c.sent_addressed == 0U);
    assert(db_espnow_tx_on_send_cb(&t, true, &c));
    assert(db_espnow_tx_ready(&t));
    assert(c.broadcast_failed == 0U && c.addressed_delivered == 0U);

    // broadcast, fail
    db_espnow_tx_on_sent(&t, false, 1001U, &c);
    assert(db_espnow_tx_on_send_cb(&t, false, &c));
    assert(c.broadcast_failed == 1U);

    // addressed, delivered then failed
    db_espnow_tx_on_sent(&t, true, 1002U, &c);
    assert(db_espnow_tx_on_send_cb(&t, true, &c));
    db_espnow_tx_on_sent(&t, true, 1003U, &c);
    assert(db_espnow_tx_on_send_cb(&t, false, &c));
    assert(c.sent_addressed == 2U && c.addressed_delivered == 1U && c.addressed_failed == 1U);

    // deadline: not before it, exactly at it
    db_espnow_tx_on_sent(&t, true, 2000U, &c);
    assert(!db_espnow_tx_check_timeout(&t, 2499U, 500U, &c));
    assert(!db_espnow_tx_ready(&t));
    assert(db_espnow_tx_check_timeout(&t, 2500U, 500U, &c));
    assert(db_espnow_tx_ready(&t));
    assert(c.send_cb_timeout == 1U);
    assert(!db_espnow_tx_check_timeout(&t, 9999U, 500U, &c));   // nothing in flight -> nothing to write off
    // its callback arrives late: counted, releases nothing, not counted as delivered
    assert(!db_espnow_tx_on_send_cb(&t, true, &c));
    assert(c.send_cb_late == 1U && c.addressed_delivered == 1U);

    // deadline across a wrap of the millisecond clock
    db_espnow_tx_on_sent(&t, false, UINT32_MAX - 100U, &c);
    assert(!db_espnow_tx_check_timeout(&t, 200U, 500U, &c));     // 301 ms elapsed
    assert(db_espnow_tx_check_timeout(&t, 400U, 500U, &c));      // 501 ms elapsed
    assert(c.send_cb_timeout == 2U);

    // every addressed send ended in exactly one bucket: delivered, failed, or written off by the deadline
    assert(c.sent_addressed == c.addressed_delivered + c.addressed_failed + 1U /* the timed-out one */);
}

/**
 * Runs all link-stats and transmit-path tests.
 *
 * @return 0 on success (assert aborts otherwise)
 */
int main(void) {
    test_pack_layout();
    test_unpack_rules();
    test_fits_tunnel_and_packet();
    test_snapshot_and_inc();
    test_auto_target();
    test_resolve_destination();
    test_tracker();
    return 0;
}
