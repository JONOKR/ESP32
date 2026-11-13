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

#include <stddef.h>
#include <string.h>

#include "db_espnow_seq_space.h"

/**
 * See db_espnow_seq_space.h.
 *
 * @param tx Sender state
 * @param split_addressed Keep a space per addressed destination
 */
void db_espnow_seq_tx_init(db_espnow_seq_tx_t *tx, const bool split_addressed) {
    if (tx == NULL) {
        return;
    }
    memset(tx, 0, sizeof(*tx));
    tx->split_addressed = split_addressed;
}

/**
 * See db_espnow_seq_space.h.
 *
 * @param tx Sender state
 * @param dest Destination MAC or NULL for broadcast
 * @return Pointer to the counter
 */
uint32_t *db_espnow_seq_tx_counter(db_espnow_seq_tx_t *tx, const uint8_t *dest) {
    if (tx == NULL) {
        return NULL;
    }
    if (dest == NULL || !tx->split_addressed) {
        return &tx->broadcast_next;
    }
    for (uint8_t i = 0; i < DB_ESPNOW_SEQ_ADDRESSED_SPACES; i++) {
        if (tx->addressed[i].in_use && memcmp(tx->addressed[i].mac, dest, DB_ESPNOW_MAC_ADDR_LEN) == 0) {
            return &tx->addressed[i].next;
        }
    }
    db_espnow_seq_tx_slot_t *slot = NULL;
    for (uint8_t i = 0; i < DB_ESPNOW_SEQ_ADDRESSED_SPACES; i++) {
        if (!tx->addressed[i].in_use) {
            slot = &tx->addressed[i];
            break;
        }
    }
    if (slot == NULL) {
        slot = &tx->addressed[tx->evict_next];
        tx->evict_next = (uint8_t) ((tx->evict_next + 1U) % DB_ESPNOW_SEQ_ADDRESSED_SPACES);
    }
    slot->in_use = true;
    memcpy(slot->mac, dest, DB_ESPNOW_MAC_ADDR_LEN);
    slot->next = 0;
    return &slot->next;
}

/**
 * See db_espnow_seq_space.h.
 *
 * @param seq Current sequence number
 * @return Next sequence number
 */
uint32_t db_espnow_seq_next(const uint32_t seq) {
    return seq < UINT32_MAX ? seq + 1U : 0U;
}

/**
 * See db_espnow_seq_space.h.
 *
 * @param rx Space state
 * @param seq Received sequence number
 * @param lost_total Loss total to update
 * @return What the number meant
 */
db_espnow_seq_rx_result_t db_espnow_seq_rx_update(db_espnow_seq_rx_t *rx, const uint32_t seq, uint16_t *lost_total) {
    if (rx == NULL || lost_total == NULL) {
        return DB_ESPNOW_SEQ_RX_FIRST;
    }
    db_espnow_seq_rx_result_t result;
    if (!rx->initialised) {
        rx->initialised = true;
        result = DB_ESPNOW_SEQ_RX_FIRST;
    } else if (seq > rx->last_seq) {
        const uint32_t lost = (uint32_t) *lost_total + (seq - rx->last_seq - 1U);
        *lost_total = (lost > UINT16_MAX) ? UINT16_MAX : (uint16_t) lost;
        result = DB_ESPNOW_SEQ_RX_FORWARD;
    } else {
        *lost_total = 0;
        result = DB_ESPNOW_SEQ_RX_BACKWARD;
    }
    rx->last_seq = seq;
    return result;
}
