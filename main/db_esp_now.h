#include <sys/cdefs.h>
/*
 *   This file is part of DroneBridge:https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2024 Wolfgang Christl
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

#ifndef DB_ESP32_DB_ESP_NOW_H
#define DB_ESP32_DB_ESP_NOW_H

#include <stdint.h>
#include <esp_now.h>
#include "db_espnow_limits.h"   // DB_ESPNOW_MAX_BROADCAST_PEERS (19), upstream since v2.4.0
#include "db_espnow_tx_path.h"  // db_espnow_dest_mode_t
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Queue depth for the send / recv-callback / uart-write queues. 12, up from
// upstream's 6, so a telemetry burst or a mission-upload volley rides out a
// short link fade instead of overflowing and dropping frames (incl.
// heartbeats). Memory cost is small: 3 queues x 12 slots + <=250 B malloc'd
// payload each. Kept through the v2.4.1 merge on purpose - this link has no
// retransmission, so an overflow is an unrecoverable loss (memory.md 5.0).
#define ESPNOW_QUEUE_SIZE   12
// ESPNOW_MAXDELAY (512 ticks = 512 ms) used to be the enqueue budget of both WiFi-driver
// callbacks. It is gone from the radio path: those callbacks run in the WiFi driver's own
// task, and blocking it for half a second stalls every receive and every transmit
// completion on the unit. It was kept that long only because a lost send-callback event
// left `ready_to_send` false forever and wedged transmission (memory.md 5.4). That
// fragility is now removed at its root instead, two ways:
//  - send-callback events have their own queue (DB_ESPNOW_SEND_CB_QUEUE_SIZE), so a burst
//    of received packets can no longer crowd out the one event the send path waits on;
//  - the in-flight packet has a deadline (DB_ESPNOW_SEND_CB_TIMEOUT_MS), so even a lost
//    event delays transmission by a bounded time and is counted (send_cb_timeout).
// Both callbacks therefore enqueue with a zero timeout: a received packet that does not
// fit is dropped and counted (drop_rx_queue_full).
//
// Depth of the send-callback event queue. The send path is single-flight, so at most one
// callback is normally outstanding; a late callback after a timeout recovery makes two.
// 4 leaves headroom so this queue can never be the reason an event is lost.
#define DB_ESPNOW_SEND_CB_QUEUE_SIZE 4
// Deadline for a send callback after esp_now_send() accepted a packet. Espressif do not
// document the unicast retry count (esp_now.rst only says SEND_SUCCESS means "received
// successfully on the MAC layer"); the decision doc's "~7 retries" is the IEEE 802.11
// default short retry limit, assumed rather than verified. Worst case per attempt at the
// slowest rate this firmware uses, LR 250 kbps: a full 250 B ESP-NOW frame plus ~45 B of
// 802.11 action-frame overhead is ~2360 bits = ~9.4 ms, plus the ACK and LR preambles
// (~2-3 ms), plus a maximal 802.11b backoff of 1023 slots x 20 us = ~20.5 ms, i.e. ~33 ms;
// 8 attempts = ~270 ms. 500 ms clears that with margin, so a slow-but-real callback is
// never written off, and equals the old blocking budget, so the worst case introduces no
// stall class that did not already exist. In normal operation the deadline is never hit:
// it exists for the event that is lost, and send_cb_timeout makes any hit visible.
#define DB_ESPNOW_SEND_CB_TIMEOUT_MS 500
// ...but the two *data-path* queues must DROP instead of block. Measured
// 2026-11-12 (memory.md 5.4): with the radio at an LR rate the 12-slot send queue
// stays full, and a 512 ms xQueueSend inside read_process_serial_link stalled the
// whole control task for ~370 ms at a time - during which the GND answered neither
// its own serial nor the downlink queue, because both are served by that one loop.
// A dropped MAVLink frame is retried by the protocol above; a half-second stall
// breaks every timer above it (GroundControl's REQUEST_RETRY_S and read_param
// timeout_s are both 1.0 s). 10 ms absorbs scheduling jitter while staying two
// orders of magnitude clear of those timers.
#define DB_ESPNOW_DATA_QUEUE_DELAY_MS 10
// Peer-list ("internal telemetry") period. This frame is sizeof(db_esp_now_clients_list_t)
// = 211 B - the whole fixed DB_ESPNOW_MAX_BROADCAST_PEERS array, sent whether one peer
// is connected or nineteen - i.e. a near-maximum ESP-NOW frame against a 215 B payload
// limit. It used to be scheduled by a loop-iteration counter rather than a clock, which
// put it on air at roughly 66 Hz: ~113 kbps of RSSI housekeeping on a link whose measured
// goodput is 20-50 kbps (memory.md 5.1). 2 Hz is ample for an RSSI display.
// The frame's *size* is deliberately unchanged: the receiving AIR unit checks the payload
// length against sizeof(db_esp_now_clients_list_t) exactly, so sending only `size` entries
// would need both units reflashed in lockstep and would blind the AIR's RADIO_STATUS
// reporting until that happened. Shrinking it is a follow-up, not part of this change.
#define DB_ESPNOW_INTERNAL_TELEMETRY_PERIOD_US 500000
#define DB_ESPNOW_AES_IV_LEN       12   // 96 bit
#define DB_ESPNOW_AES_TAG_LEN      16
#define DB_ESPNOW_AES_KEY_LEN      256  // in bits 128 & 192 are supported by ESP32
#define DB_ESPNOW_PAYLOAD_MAXSIZE  (ESP_NOW_MAX_DATA_LEN-DB_ESPNOW_AES_IV_LEN-DB_ESPNOW_AES_TAG_LEN-6-1) // (origin, packet_type, seq_num) = 6 bytes, payload_length_decrypted = 1 byte

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0)

extern QueueHandle_t db_espnow_send_queue;    // Queue that contains data to be sent via ESP-NOW
extern QueueHandle_t db_uart_write_queue;    // Queue that contains data to be written to UART (filled by ESP-NOW task)

typedef struct {
    int8_t gnd_rssi;               // RSSI in dBm of that peer when we received the last message
    uint16_t last_seq_num;          // last sequ. number received by the GND station ToDO: Not something we need to send via internal telemetry
    uint16_t gnd_rx_lost_packets;   // number of lost packets on the GND side
    uint8_t broadcast_peer_mac[ESP_NOW_ETH_ALEN];  // mac address of a peer that sent us broadcast messages
} __attribute__((__packed__)) db_esp_now_bpeer_info_t;

// structure used by ESP-NOW (manly GND) to keep infos about RSSI from every device that sent us something
typedef struct {
    uint8_t size;               // number of peers within db_esp_now_bpeer_info
    int8_t gnd_noise_floor;    // noise floor as measured on GND side (not all ESP32s support that feature)
    db_esp_now_bpeer_info_t db_esp_now_bpeer_info[DB_ESPNOW_MAX_BROADCAST_PEERS];
} __attribute__((__packed__)) db_esp_now_clients_list_t;    // structure is sent as ESP-NOW internal telemetry frame to AIR ESP32

// Local list of known ESP-NOW peers (their MAC + last RSSI/seq). Populated on the GND side.
// Used by the fleet-list builder to tell the GCS which AIR units are currently connected.
/**
 * Read-only view of the ESP-NOW peer list, for building the fleet-list TUNNEL.
 *
 * Upstream made the list itself file-local in v2.4.1 and dropped the `extern`
 * that used to be here. JONOKR's fleet addressing still needs to read it
 * (db_mav_create_fleet_list), so it goes through this accessor instead of
 * re-exporting the global - the dependency stays explicit and upstream's
 * encapsulation survives the next merge.
 *
 * Only populated on the GND, which is the side that tracks peers.
 *
 * @return pointer to the peer list, or NULL before ESP-NOW is initialised
 */
const db_esp_now_clients_list_t *db_espnow_peer_list(void);

typedef enum {
    DB_ESPNOW_SEND_CB,
    DB_ESPNOW_RECV_CB,
} db_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} db_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int8_t rssi;
    int8_t noise_floor;
    uint8_t data_len;
    uint8_t *data;
} db_espnow_event_recv_cb_t;

typedef union {
    db_espnow_event_send_cb_t send_cb;
    db_espnow_event_recv_cb_t recv_cb;
} db_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    db_espnow_event_id_t id;
    db_espnow_event_info_t info;
} db_espnow_event_t;

typedef enum {
    DB_ESP_NOW_PACKET_TYPE_DATA = 0,
    DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY = 1,
} db_espnow_queue_event_type_t;

/* Element of db_espnow_send_queue. Internal to one unit - none of this goes on air. */
typedef struct {
    uint8_t data_len;
    uint8_t *data;
    db_espnow_queue_event_type_t packet_type;   // only relevant when reading queue that sends via ESP-NOW
    uint8_t dest_mode;                          // db_espnow_dest_mode_t; DB_ESPNOW_DEST_AUTO = today's routing
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];         // only read when dest_mode == DB_ESPNOW_DEST_PEER
} __attribute__((__packed__)) db_espnow_queue_event_t;

/** Element of db_uart_write_queue, including the source stream identity. */
typedef struct {
    uint8_t data_len;
    uint8_t *data;
    uint8_t source_mac[ESP_NOW_ETH_ALEN];
    uint32_t seq_num;
    uint8_t origin;
} db_espnow_uart_queue_event_t;

enum {
    DB_ESPNOW_ORIGIN_GND = 0,
    DB_ESPNOW_ORIGIN_AIR = 1,
};

typedef struct {
    uint8_t origin;                         // type: db_espnow_data_origin_t: 0=packet sent by GCS - 1=packet sent by drone
    uint8_t packet_type;                    // DB_ESP_NOW_PACKET_TYPE_DATA or DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY or FEC in the future
    uint32_t seq_num;                       // Sequence number of the packet
    uint8_t aes_iv[DB_ESPNOW_AES_IV_LEN];   // Initialization vector for AES
} __attribute__((__packed__)) db_esp_now_packet_header_t;             // authenticated but NOT encrypted by AES-GCM

typedef struct {
    uint8_t payload_length_decrypted;           // length of unencrypted payload
    uint8_t payload[DB_ESPNOW_PAYLOAD_MAXSIZE]; // actual payload data
} __attribute__((__packed__)) db_esp_now_packet_protected_data_t; // encrypted & authenticated by AES-GCM

/* DroneBridge for ESP32 ESP-NOW packet */
typedef struct {
    db_esp_now_packet_header_t db_esp_now_packet_header;
    uint8_t tag[DB_ESPNOW_AES_TAG_LEN];     // AES-GCM Tag
    db_esp_now_packet_protected_data_t db_esp_now_packet_protected_data;    // shall be last in struct, so we can cut off when payload is smaller
} __attribute__((__packed__)) db_esp_now_packet_t;  // total size must be <=250bytes (ESP-NOW requirement)

void db_start_espnow_module();

_Noreturn void process_espnow_data();

#endif //DB_ESP32_DB_ESP_NOW_H
