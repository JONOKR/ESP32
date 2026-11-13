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
 * ESP-NOW sequence-number spaces.
 *
 * THE PROBLEM. Every DroneBridge ESP-NOW packet carries one 32-bit seq_num in its (authenticated,
 * unencrypted) header, and receivers count a jump of more than one as lost packets. Until
 * addressed sending existed, every packet a sender emitted reached every receiver in range, so
 * "a gap" meant "lost". Once the GND addresses a control frame to AIR #1, AIR #2 never receives
 * it - by design, the radio filters it - and would count it as lost: its uplink loss counter
 * (RADIO_STATUS.fixed) would report the GND's traffic to other drones as link loss.
 *
 * THE FIX. A sender that addresses more than one destination keeps a separate counter per
 * space: one for broadcast packets and one per addressed destination MAC. Every receiver hears
 * every packet of the broadcast space and every packet of the space addressed to itself, so
 * each space it tracks is complete again. The receiver tells the spaces apart by the frame's
 * 802.11 destination address, which the receive callback already provides - nothing new goes on
 * air and the header layout is unchanged.
 *
 * WHO SPLITS. Only the GND (split_addressed = true). An AIR addresses only its one GND, and
 * that GND also hears all of the AIR's broadcasts, so for the only receiver that counts the
 * AIR's sequence (the GND, for its loss count and for the MAVLink parser's gap detection) one
 * space is already complete. Splitting it would cost the GND's parser something real: a gap
 * detected per space no longer reveals a loss in the merged byte stream, so a frame split across
 * two packets could be glued onto the next packet's bytes. The AIR therefore keeps one space,
 * bit-identical to phase A in every mode.
 *
 * MODE 0 is untouched by construction: nothing is ever addressed there, so the GND uses only
 * the broadcast space, which increments exactly as the single counter did.
 *
 * Pure C, host-tested.
 */

#ifndef DB_ESP32_DB_ESPNOW_SEQ_SPACE_H
#define DB_ESP32_DB_ESPNOW_SEQ_SPACE_H

#include <stdbool.h>
#include <stdint.h>

#include "db_espnow_limits.h"

/* One addressed space per possible GND peer: the GND only addresses registered peers, and it
 * registers at most DB_ESPNOW_MAX_BROADCAST_PEERS, so the table never has to evict there. */
#define DB_ESPNOW_SEQ_ADDRESSED_SPACES DB_ESPNOW_MAX_BROADCAST_PEERS

/** Next-to-send sequence number of one addressed destination. */
typedef struct {
    bool in_use;
    uint8_t mac[DB_ESPNOW_MAC_ADDR_LEN];
    uint32_t next;
} db_espnow_seq_tx_slot_t;

/** Sender side: the counters a unit stamps into outgoing packet headers. */
typedef struct {
    bool split_addressed;       // false: one space for everything (AIR); true: per destination (GND)
    uint32_t broadcast_next;    // the broadcast space - and the only space when not split
    db_espnow_seq_tx_slot_t addressed[DB_ESPNOW_SEQ_ADDRESSED_SPACES];
    uint8_t evict_next;         // round-robin victim if the table is ever full
} db_espnow_seq_tx_t;

/**
 * Reset all counters to 0.
 *
 * @param tx Sender state
 * @param split_addressed true to keep a space per addressed destination (GND), false for one
 *                        space for everything (AIR, beacon)
 */
void db_espnow_seq_tx_init(db_espnow_seq_tx_t *tx, bool split_addressed);

/**
 * Counter to stamp into a packet going to dest, and to advance once esp_now_send() accepted it.
 *
 * With split_addressed false this is always the single counter. With it true, an addressed
 * destination seen for the first time gets a fresh space starting at 0; if all
 * DB_ESPNOW_SEQ_ADDRESSED_SPACES are taken (impossible on a GND, see above) one is recycled
 * round-robin, which its receiver sees as a sequence reset - the same thing it sees when the
 * sender reboots, and handles the same way.
 *
 * @param tx Sender state
 * @param dest Destination MAC, or NULL for broadcast
 * @return Pointer to the counter (never NULL unless tx is NULL)
 */
uint32_t *db_espnow_seq_tx_counter(db_espnow_seq_tx_t *tx, const uint8_t *dest);

/**
 * The sequence number after seq. Wraps UINT32_MAX to 0, as the firmware always has.
 *
 * @param seq Current sequence number
 * @return Next sequence number
 */
uint32_t db_espnow_seq_next(uint32_t seq);

/** Receiver side: one sequence space being tracked for loss. */
typedef struct {
    bool initialised;       // false until the first packet of this space is seen
    uint32_t last_seq;      // last sequence number seen in this space
} db_espnow_seq_rx_t;

/** What one received sequence number meant. */
typedef enum {
    DB_ESPNOW_SEQ_RX_FIRST = 0,     // first packet of this space: adopted, nothing counted
    DB_ESPNOW_SEQ_RX_FORWARD = 1,   // advanced; any gap was added to the loss total
    DB_ESPNOW_SEQ_RX_BACKWARD = 2,  // not greater than the last: sender reset or replay; total cleared
} db_espnow_seq_rx_result_t;

/**
 * Account one received sequence number against a space and a saturating 16-bit loss total.
 *
 * Exactly the rule the AIR's uplink loss counter has used since it existed: the first packet
 * is adopted (an AIR powered up beside a running GND must not count its history as lost); a
 * forward jump adds the gap, saturating at UINT16_MAX because the GCS reads a delta and a pinned
 * counter still reads as "losing everything"; anything not greater - a sender reboot or a replay
 * - clears the total and resynchronises, keeping the link usable.
 *
 * @param rx Space state
 * @param seq Sequence number just received in this space
 * @param lost_total Loss total to update (shared between spaces by the caller)
 * @return What the number meant
 */
db_espnow_seq_rx_result_t db_espnow_seq_rx_update(db_espnow_seq_rx_t *rx, uint32_t seq, uint16_t *lost_total);

#endif // DB_ESP32_DB_ESPNOW_SEQ_SPACE_H
