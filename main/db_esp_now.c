/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
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

#include <sys/cdefs.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <string.h>
#include <esp_crc.h>
#include <esp_timer.h>
#include <mbedtls/gcm.h>
#include <mbedtls/md.h>
#include <mbedtls/pkcs5.h>
#include "db_esp_now.h"

#include <db_parameters.h>

#include "globals.h"
#include "main.h"
#include "espnow.h"
#include "db_espnow_seq_space.h"

#define TAG "DB_ESPNOW"

/* The pure routing module names the ESPNOW_UNICAST modes itself (it cannot include
 * db_parameters.h); these keep the two in step. */
_Static_assert(DB_ESPNOW_UNICAST_OFF == DB_ESPNOW_ADDRESSING_OFF, "ESPNOW_UNICAST mode 0 mismatch");
_Static_assert(DB_ESPNOW_UNICAST_CONTROL == DB_ESPNOW_ADDRESSING_CONTROL, "ESPNOW_UNICAST mode 1 mismatch");
_Static_assert(DB_ESPNOW_UNICAST_ALL == DB_ESPNOW_ADDRESSING_ALL, "ESPNOW_UNICAST mode 2 mismatch");

/* At most one "send failed" console line per this many ms. At the edge of range an addressed
 * frame can fail several times a second, and on a 115200-baud console each line costs ~5 ms of
 * UART time in the ESP-NOW task's own loop. Every failure is still counted in db_link_stats
 * (addressed_failed / broadcast_failed); the log is only a pointer to look there. */
#define DB_ESPNOW_SEND_FAIL_LOG_INTERVAL_MS 1000

static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
/* Receive-callback events only (DB_ESPNOW_RECV_CB). Send callbacks have their own queue below
 * so that received traffic can never crowd out the event the send path depends on. */
static QueueHandle_t db_espnow_send_recv_callback_queue;
/* Send-callback results (db_espnow_event_send_cb_t), filled from the WiFi task with a zero
 * timeout. See DB_ESPNOW_SEND_CB_QUEUE_SIZE. */
static QueueHandle_t db_espnow_send_cb_queue;
/* Single-flight state of the send path; owned by the ESP-NOW task, which is its only user. */
static db_espnow_tx_tracker_t db_espnow_tx_tracker;
QueueHandle_t db_espnow_send_queue;    // Queue that contains data to be sent via ESP-NOW (filled by control task)
QueueHandle_t db_uart_write_queue;    // Queue that contains data to be written to UART (filled by ESP-NOW task)
static db_esp_now_clients_list_t *db_esp_now_clients_list;

/* Forward declarations: the send path below needs these, and they are defined further
 * down beside the peer bookkeeping they read. */
static const uint8_t *db_espnow_unicast_target(void);
static void db_espnow_sync_rate_config(void);

/**
 * Read-only view of the ESP-NOW peer list - see db_esp_now.h for why this
 * exists instead of an `extern`.
 *
 * @return pointer to the peer list, or NULL before ESP-NOW is initialised
 */
const db_esp_now_clients_list_t *db_espnow_peer_list(void) {
    return db_esp_now_clients_list;
} // Local list of known ESP-NOW peers with the last RSSI value and noise floor and sequence number

/* Sequence counters stamped into outgoing headers. Before phase B the header's own seq_num field
 * was the one counter; it is now set from here for every packet, per space - see
 * db_espnow_seq_space.h for why the GND splits by destination and the AIR does not. Initialised
 * in db_espnow_init(); owned by the ESP-NOW task. */
static db_espnow_seq_tx_t db_espnow_seq_tx;

// packet that is filled with payload and sent. Its seq_num is set from db_espnow_seq_tx per send.
static db_esp_now_packet_t db_esp_now_packet_global = {
        .db_esp_now_packet_header.seq_num = 0,
        .db_esp_now_packet_header.packet_type = DB_ESP_NOW_PACKET_TYPE_DATA
};   // make static so it gets instanced only once

mbedtls_gcm_context aes;
static uint8_t const db_esp_now_packet_header_len = sizeof(db_esp_now_packet_header_t);

/**
 * Returns whether the local radio role is configured to accept a packet origin.
 *
 * AIR-to-AIR delivery is deliberately limited to MAVLink mode so MSP/LTM and
 * transparent serial links retain their existing peer isolation.
 *
 * @param origin Authenticated origin value from an ESP-NOW packet header.
 * @return true when packets from the origin may be processed locally.
 */
static bool db_espnow_origin_is_allowed(const uint8_t origin) {
    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        return origin == DB_ESPNOW_ORIGIN_AIR;
    }
    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR) {
        return origin == DB_ESPNOW_ORIGIN_GND ||
               (origin == DB_ESPNOW_ORIGIN_AIR &&
                DB_PARAM_SERIAL_PROTO == DB_SERIAL_PROTOCOL_MAVLINK &&
                DB_PARAM_ESPNOW_AIR2AIR);
    }
    return false;
}

/**
 * Generates a AES key from a password using pkcs5 - pbkdf2 and mbedTLS
 *
 * @param password The password that gets transformed to PKCS5 key used for encryption
 * @param key Output buffer for the key
 * @param keylen Length of the aes key to be generated
 */
void generate_pkcs5_key(const char* password, unsigned char* key, size_t keylen) {
    mbedtls_md_context_t mdctx;
    const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256); // Use SHA-256 for key derivation

    // Initialize the context
    mbedtls_md_init(&mdctx);
    int ret = mbedtls_md_setup(&mdctx, md_info, 0);
    switch (ret) {
        case 0:
            // success
            break;
        case MBEDTLS_ERR_MD_BAD_INPUT_DATA:
            ESP_LOGW(TAG, "mbedtls_md_setup returned MBEDTLS_ERR_MD_BAD_INPUT_DATA");
            break;
        case MBEDTLS_ERR_MD_ALLOC_FAILED:
            ESP_LOGW(TAG, "mbedtls_md_setup returned MBEDTLS_ERR_MD_ALLOC_FAILED");
            break;
        default:
            ESP_LOGW(TAG, "mbedtls_md_setup returned unknown error: %i", ret);
            break;
    }
    static unsigned char generic_salt[] = "GenSalt147894562";    // Generic on purpose. No need for salt in this application
    ret = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,
                                        (const unsigned char *)password, strlen(password),
                                        generic_salt, strlen((char*) generic_salt), 10000, keylen, key);
    switch (ret) {
        case 0:
            // success
            break;
        case MBEDTLS_ERR_PKCS5_BAD_INPUT_DATA:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_BAD_INPUT_DATA");
            break;
        case MBEDTLS_ERR_PKCS5_INVALID_FORMAT:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_INVALID_FORMAT");
            break;
        case MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE");
            break;
        case MBEDTLS_ERR_PKCS5_PASSWORD_MISMATCH:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_PASSWORD_MISMATCH");
            break;
        default:
            ESP_LOGW(TAG, "mbedtls_pkcs5_pbkdf2_hmac returned unknown error: %02x", ret);
            break;
    }
    // Clean up
    mbedtls_md_free(&mdctx);
}

/**
 * Encrypts and authenticates a DroneBridge for ESP32 ESP-NOW packet with its payload using AES-GCM 256
 * Beware we are using the same key for both communication directions.
 * Calls mbedtls_gcm_crypt_and_tag()
 *
 * @param db_esp_now_packet Packet containing payload data and header info. AES IV & TAG will be filled
 * @param encrypt_payload_len Length of the to be encrypted data (db_esp_now_packet_protected_data)
 * @return 0 on success, -1 in case of unknown error of mbedtls_gcm_crypt_and_tag or return value of mbedtls_gcm_crypt_and_tag
 * payload is encrypted and part of the db_esp_now_packet
 */
int db_encrypt_payload(db_esp_now_packet_t* db_esp_now_packet, uint8_t encrypt_payload_len) {
    // Generate random IV - This is risky since (password+IV) shall never be reused!
    // Counter would secure for a single session but super unsecure with multiple sessions where the password is not changed
    esp_fill_random(db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN);
    int ret = mbedtls_gcm_crypt_and_tag(
            &aes,
            MBEDTLS_GCM_ENCRYPT,
            encrypt_payload_len,
            db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_header, db_esp_now_packet_header_len,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            DB_ESPNOW_AES_TAG_LEN, db_esp_now_packet->tag
    );

    if (ret == 0) {
        return 0;
    } else {
        if (ret == MBEDTLS_ERR_GCM_BAD_INPUT) {
            ESP_LOGW(TAG, "mbedtls_gcm_crypt_and_tag returned MBEDTLS_ERR_GCM_BAD_INPUT");
        } else {
            ESP_LOGW(TAG, "mbedtls_gcm_crypt_and_tag returned unknown error");
        }
        return -1;
    }
}

/**
 * Decrypt the DroneBridge ESP-NOW packet payload and authenticate its content using AES-GCM 256
 *
 * @param db_esp_now_packet db_esp_now_packet_protected_data_t structure wth the data de decrypt inside
 * @param decrypt_out_buffer Pointer to the out buffer for the decrypted data
 * @param len_encrypted_data length of the buffer (db_esp_now_packet_protected_data)
 * @return returns result of mbedtls_gcm_auth_decrypt e.g. 0 on success and valid data
 */
int db_decrypt_payload(db_esp_now_packet_t* db_esp_now_packet, uint8_t* decrypt_out_buffer,
                       uint8_t len_encrypted_data) {
    int ret_decrypt = mbedtls_gcm_auth_decrypt(
            &aes,
            len_encrypted_data,
            db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_header, db_esp_now_packet_header_len,
            db_esp_now_packet->tag, DB_ESPNOW_AES_TAG_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            decrypt_out_buffer
    );
    switch (ret_decrypt) {
        case 0:
            // Successfully decrypted and authenticated
            break;
        case MBEDTLS_ERR_GCM_AUTH_FAILED:
            ESP_LOGW(TAG, "MBEDTLS_ERR_GCM_AUTH_FAILED: Authenticated decryption failed."); break;
        case MBEDTLS_ERR_GCM_BAD_INPUT:
            ESP_LOGW(TAG, "MBEDTLS_ERR_GCM_BAD_INPUT: Bad input parameters to function."); break;
        default:
            ESP_LOGW(TAG, "Unknown mbedtls_gcm_auth_decrypt error %i", ret_decrypt); break;
    }
    return ret_decrypt;
}


/**
 * Encrypts, authenticates and sends packet via ESP-NOW
 *
 * The destination is resolved by the caller (db_read_uart_queue_and_send) because it has to
 * know afterwards whether the packet in flight was addressed or broadcast: that decides how
 * its send callback is counted.
 *
 * @param db_esp_now_packet Packet to send
 * @param payload_length Value of payload_length_decrypted of db_esp_now_packet - saves us from creating a var inside the function
 * @param dest Destination MAC, or NULL to broadcast
 * @return false if no packet was sent, true if packet was sent (actual sending will be confirmed by the send-callback)
 */
bool db_espnow_encrypt_auth_send(db_esp_now_packet_t *db_esp_now_packet, const uint8_t payload_length,
                                 const uint8_t *dest) {
    static int ret;
    static esp_err_t err;
    // after encryption the payload_length_decrypted will be unreadable -> keep a copy (payload_length) for sending the packet
    // const uint8_t payload_length = db_esp_now_packet->db_esp_now_packet_protected_data.payload_length_decrypted;
    ret = db_encrypt_payload(db_esp_now_packet, db_esp_now_packet->db_esp_now_packet_protected_data.payload_length_decrypted + 1);
    if (ret == 0) {
        /* Pick up a changed ESPNOW_RATE before sending, so the parameter and the radio
         * cannot disagree (there is no reboot command to fall back on). */
        db_espnow_sync_rate_config();
        if (dest == NULL) {
            dest = BROADCAST_MAC;
        }
        err = esp_now_send(dest, (const uint8_t *) db_esp_now_packet, (db_esp_now_packet_header_len + DB_ESPNOW_AES_TAG_LEN + payload_length + 1));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) sending ESP-NOW data - size %i !", esp_err_to_name(err), (db_esp_now_packet_header_len + DB_ESPNOW_AES_TAG_LEN + payload_length + 1));
            return false;
        } else {
            return true;
        }
    } else {
        ESP_LOGE(TAG, "db_encrypt_payload returned error code: %i, not sending packet", ret);
        return false;
    }
}

/**
 * Destination of one dequeued packet under the current ESPNOW_UNICAST mode. The decision itself
 * is db_espnow_resolve_destination_for_mode(), the single host-tested place the mode semantics
 * live (its header carries the full table); this only gathers the facts that need ESP-NOW.
 *
 * Mode 0 and BROADCAST requests return before touching the peer table, as before.
 *
 * @param evt Dequeued send-queue element
 * @return Destination MAC, or NULL to broadcast
 */
static const uint8_t *db_espnow_event_destination(const db_espnow_queue_event_t *evt) {
    const uint8_t unicast_mode = DB_PARAM_ESPNOW_UNICAST;
    const db_espnow_dest_mode_t mode = (db_espnow_dest_mode_t) evt->dest_mode;
    if (unicast_mode == DB_ESPNOW_UNICAST_OFF || mode == DB_ESPNOW_DEST_BROADCAST) {
        return NULL;
    }
    const bool dest_registered = mode == DB_ESPNOW_DEST_PEER && esp_now_is_peer_exist(evt->dest_mac);
    return db_espnow_resolve_destination_for_mode(unicast_mode, mode, evt->dest_mac, dest_registered,
                                                  db_espnow_unicast_target());
}

/**
 * Tries to read one entry from the ESP-NOW send queue (mainly filled by the UART task) and sends it, addressed or
 * broadcast as db_espnow_event_destination() decides.
 * Only call this function when nothing is in flight (db_espnow_tx_ready())! Otherwise order of packets is not
 * guaranteed.
 *
 * A packet that is dequeued but cannot be put on air (oversized, encryption or esp_now_send() error) is lost - there
 * is no requeue, as before - and is now counted as send_error. It consumes no sequence number, so the peer's loss
 * counter never sees it; this counter is the only place it shows up.
 *
 * @return false if no packet was sent, true if packet was sent (actual sending will be confirmed by the send-callback)
 */
bool db_read_uart_queue_and_send() {
    static db_espnow_queue_event_t evt;
    // Receive data from Queue that was put there by other tasks to be sent via ESP-NOW
    if (db_espnow_send_queue != NULL && xQueueReceive(db_espnow_send_queue, &evt, 0) == pdTRUE) {
        if (evt.data == NULL || evt.data_len > DB_ESPNOW_PAYLOAD_MAXSIZE) {
            /* Every producer checks both already; this guards the fixed-size packet buffer
             * below against a future producer that does not. */
            ESP_LOGE(TAG, "Invalid send-queue element (len %u) - dropped", evt.data_len);
            free(evt.data);
            db_link_stats_inc(&db_link_stats.send_error);
            return false;
        }
        db_esp_now_packet_global.db_esp_now_packet_header.packet_type = evt.packet_type;
        if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
            db_esp_now_packet_global.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_GND;
        } else {
            db_esp_now_packet_global.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_AIR;
        }
        // Create an DroneBridge for ESP32 ESP-NOW packet
        db_esp_now_packet_global.db_esp_now_packet_protected_data.payload_length_decrypted = evt.data_len;
        // ToDo: Potential to optimize: esp_now_send() does not need an instance of buffer after sending, however using evt.data as pointer resulted in errors
        memcpy(db_esp_now_packet_global.db_esp_now_packet_protected_data.payload, evt.data, evt.data_len);
        free(evt.data);
        const uint8_t *dest = db_espnow_event_destination(&evt);
        /* The sequence number comes from the destination's space and must be in the header
         * before encryption: the header is the AES-GCM associated data. */
        uint32_t *seq_counter = db_espnow_seq_tx_counter(&db_espnow_seq_tx, dest);
        db_esp_now_packet_global.db_esp_now_packet_header.seq_num = *seq_counter;
        // Encrypt, authenticate and send packet
        if(db_espnow_encrypt_auth_send(&db_esp_now_packet_global, evt.data_len, dest)) {
            // advance this space's sequence number for its next packet (wraps to 0 after UINT32_MAX)
            *seq_counter = db_espnow_seq_next(*seq_counter);
            /* Counted here, beside the sequence-number increment, and nowhere else: that is
             * what makes sent_broadcast + sent_addressed the exact population the peer's
             * sequence-gap loss counter observes. */
            db_espnow_tx_on_sent(&db_espnow_tx_tracker, dest != NULL,
                                 (uint32_t) (esp_timer_get_time() / 1000), &db_link_stats);
            return true;
        } else {
            db_link_stats_inc(&db_link_stats.send_error);
            return false;
        }
    } else {
        if (db_espnow_send_queue == NULL) {
            ESP_LOGE(TAG, "db_espnow_send_queue is NULL!");
        } else {
            // nothing to do - Queue is empty
        }

    }
    return false;
}

/* MAC of the GND station as learnt from received packets, and whether it is known.
 * The AIR side keeps only this one address: it talks to exactly one ground station,
 * while the GND tracks a whole fleet in db_esp_now_clients_list. Before 2026-11-11 the
 * AIR side learnt nothing, which is half of why the first unicast attempt failed
 * asymmetrically - only the GND ever had an address to send to. */
static uint8_t db_espnow_gnd_mac[ESP_NOW_ETH_ALEN] = {0};
static bool db_espnow_gnd_mac_known = false;

/**
 * Apply DB_PARAM_ESPNOW_RATE to one already-registered peer.
 *
 * Does nothing at all on DRIVER_DEFAULT - deliberately, because "do not call the API"
 * and "call it with the driver's own default" are not the same thing and only the
 * former reproduces the behaviour every range measurement before 2026-11-11 was taken
 * under. The rate therefore never changes unless the parameter asks for it.
 *
 * esp_now_set_peer_rate_config() must be called *after* esp_now_add_peer(), and the
 * interface must already permit the PHY mode (main.c enables WIFI_PROTOCOL_LR) or the
 * driver rejects the rate.
 *
 * Failure is not fatal: the peer stays usable at whatever rate the driver chose, so a
 * rejected rate degrades to the default rather than losing the link.
 *
 * @param peer_mac MAC address of a peer already added to the ESP-NOW peer list
 * @return true if the requested rate was applied, false if it was rejected
 */
/* Value of DB_PARAM_ESPNOW_RATE last pushed to the driver; -1 until the rate API has
 * been called at all this boot. The distinction matters: "never called" is the state
 * every range measurement before 2026-11-11 was taken under, and it cannot be recreated
 * by calling the API with a default-looking argument. */
static int16_t db_espnow_rate_pushed = -1;

static bool db_espnow_apply_rate_config(const uint8_t peer_mac[ESP_NOW_ETH_ALEN]) {
    const uint8_t want = DB_PARAM_ESPNOW_RATE;
    if (want == DB_ESPNOW_RATE_DRIVER_DEFAULT && db_espnow_rate_pushed < 0) {
        return true;    // untouched since boot - leave the driver's own choice alone
    }
    esp_now_rate_config_t rate = {.ersu = false, .dcm = false};
    switch (want) {
        case DB_ESPNOW_RATE_LR_250K:
            rate.phymode = WIFI_PHY_MODE_LR;
            rate.rate = WIFI_PHY_RATE_LORA_250K;
            break;
        case DB_ESPNOW_RATE_LR_500K:
            rate.phymode = WIFI_PHY_MODE_LR;
            rate.rate = WIFI_PHY_RATE_LORA_500K;
            break;
        default:
            /* Going back to the default at runtime cannot "un-call" the API, so it is
             * stated explicitly as ESP-NOW's documented default of 1 Mbps 802.11b. That
             * is the intent of mode 0, but it is not bit-identical to a boot that never
             * set a rate - reboot for that. */
            rate.phymode = WIFI_PHY_MODE_11B;
            rate.rate = WIFI_PHY_RATE_1M_L;
            break;
    }
    const esp_err_t rate_err = esp_now_set_peer_rate_config(peer_mac, &rate);
    if (rate_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_set_peer_rate_config(mode %d) rejected for " MACSTR ": %s",
                 (int) want, MAC2STR(peer_mac), esp_err_to_name(rate_err));
        return false;
    }
    ESP_LOGI(TAG, "ESP-NOW peer " MACSTR " rate set (mode %d)", MAC2STR(peer_mac), (int) want);
    return true;
}

/**
 * Push DB_PARAM_ESPNOW_RATE to every registered peer whenever the parameter has changed.
 *
 * Without this the rate would only ever be applied at peer registration, i.e. at boot,
 * and there is **no MAVLink reboot command in this firmware** - only
 * MAV_CMD_REQUEST_MESSAGE is handled. Changing the rate would then mean power-cycling
 * both units, which at a far test position means walking to the aircraft for every step
 * of a ladder. Worse, the parameter would read back as the new value while the radio kept
 * transmitting at the old one, so a measurement would quietly describe the wrong rate.
 *
 * Cheap enough for the send path: one comparison per packet, and the ESP-NOW calls happen
 * only on the transition.
 */
static void db_espnow_sync_rate_config(void) {
    const uint8_t want = DB_PARAM_ESPNOW_RATE;
    if ((int16_t) want == db_espnow_rate_pushed) {
        return;
    }
    if (want == DB_ESPNOW_RATE_DRIVER_DEFAULT && db_espnow_rate_pushed < 0) {
        return;     // already untouched; nothing to do
    }
    db_espnow_rate_pushed = (int16_t) want;     // set first: apply_rate_config reads it
    db_espnow_apply_rate_config(BROADCAST_MAC);
    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR) {
        if (db_espnow_gnd_mac_known) {
            db_espnow_apply_rate_config(db_espnow_gnd_mac);
        }
    } else if (db_esp_now_clients_list != NULL) {
        for (uint8_t i = 0; i < db_esp_now_clients_list->size &&
                            i < DB_ESPNOW_MAX_BROADCAST_PEERS; i++) {
            db_espnow_apply_rate_config(
                    db_esp_now_clients_list->db_esp_now_bpeer_info[i].broadcast_peer_mac);
        }
    }
    ESP_LOGI(TAG, "ESP-NOW rate mode now %d on all known peers", (int) want);
}

/**
 * Register a peer with ESP-NOW so packets can be addressed to it directly.
 *
 * Two things here are easy to get wrong and both cost a day the first time.
 *
 * **Unencrypted at the ESP-NOW layer, on purpose.** DroneBridge already encrypts and
 * authenticates every payload itself with AES-GCM, and ESP-NOW's limit on *encrypted*
 * peers is far below its 20-peer total, which a fleet would hit.
 *
 * **The peer's PHY mode must match the interface.** ESP-NOW's default bit rate is 1 Mbps
 * (802.11b) while this firmware sets the interface to
 * WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR, so a peer registered without a rate config can
 * transmit on a different PHY mode than the link was set up for.
 * esp_now_set_peer_rate_config() must be called *after* esp_now_add_peer(), so it is
 * applied here, right after registration, as whatever DB_PARAM_ESPNOW_RATE asks for
 * (db_espnow_apply_rate_config() - which leaves the driver's own choice untouched on
 * DRIVER_DEFAULT). It is not pinned to any one rate: the rate is a separate parameter so
 * that it and the addressing mode can never change together.
 *
 * Failure is never fatal: db_espnow_unicast_target() only offers a peer that is actually
 * registered, so anything that fails here falls back to broadcast.
 *
 * @param peer_mac MAC address of the peer to register
 * @return true if the peer is registered (or already was), false on error
 */
static bool db_espnow_register_peer(const uint8_t peer_mac[ESP_NOW_ETH_ALEN]) {
    if (esp_now_is_peer_exist(peer_mac)) {
        return true;
    }
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer.ifidx = WIFI_IF_STA;   // ESP-NOW modes run the radio in STA mode
    peer.encrypt = false;       // payload is already encrypted by DroneBridge
    const esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not add ESP-NOW peer " MACSTR " (%s) - staying on broadcast for it",
                 MAC2STR(peer_mac), esp_err_to_name(err));
        return false;
    }
    db_espnow_apply_rate_config(peer_mac);
    ESP_LOGI(TAG, "ESP-NOW peer " MACSTR " registered for addressed sending", MAC2STR(peer_mac));
    return true;
}

/**
 * MAC address to send the next ESP-NOW packet to, or NULL to broadcast it.
 *
 * Returns an address only when DB_PARAM_ESPNOW_UNICAST asks for one and exactly one peer
 * is known, so a default build broadcasts exactly as it always has.
 *
 * **Why exactly one peer.** The send path is single-flight - it waits for the
 * send callback so packet order is preserved - and one payload must therefore remain one
 * esp_now_send() call. With several AIR units a broadcast also reaches all of them in one
 * airtime slot where N unicasts cost N, on a link whose goodput the whole fleet shares.
 * Per-destination fan-out needs per-peer queues, which is a larger change.
 *
 * The two roles learn their counterpart differently and neither is guaranteed: the GND
 * fills db_esp_now_clients_list from received packets, the AIR side records the GND's MAC
 * above. With nothing learnt yet this returns NULL and discovery happens by broadcast,
 * which is the permanent bootstrap and not an error path.
 *
 * @return pointer to the peer's MAC, or NULL if the packet should be broadcast
 */
static const uint8_t *db_espnow_unicast_target(void) {
    /* The choice itself is db_espnow_auto_target_candidate() (host-tested); unchanged in
     * substance from before it was moved there. */
    const bool have_list = db_esp_now_clients_list != NULL;
    const uint8_t *mac = db_espnow_auto_target_candidate(
            DB_PARAM_ESPNOW_UNICAST != DB_ESPNOW_UNICAST_OFF,
            DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR,
            db_espnow_gnd_mac_known, db_espnow_gnd_mac,
            have_list ? db_esp_now_clients_list->size : 0U,
            have_list ? db_esp_now_clients_list->db_esp_now_bpeer_info[0].broadcast_peer_mac : NULL);
    /* Registration can have failed (peer table full, ESP-NOW not ready when the peer was
     * first seen); broadcast is the honest fallback. */
    if (mac == NULL || !esp_now_is_peer_exist(mac)) {
        return NULL;
    }
    return mac;
}

/**
 * Count ESP-NOW packets lost on the uplink (GND -> AIR) from gaps in the sender's sequence number.
 *
 * The mirror of the GND's own accounting in db_espnow_process_rcv_data(), for the one
 * direction nothing in this stack measured until now. Every packet carries a seq_num the
 * sender increments once per send, so a jump of more than one means the difference never
 * arrived. The running total reaches the GCS in RADIO_STATUS.fixed, beside the downlink's
 * count in .rxerrors - see db_timer_mavlink_radiostatus_callback().
 *
 * **Why this exists.** On 2026-11-07 an ESP-NOW unicast change killed the uplink outright
 * while the downlink stayed perfect, and because no counter watched this direction the
 * symptom was indistinguishable from the dead AIR->FC UART wire found the day before.
 * That cost a day. "The GND is not heard at all" and "the GND is heard fine but the FC
 * ignores its commands" are unrelated faults, and this is what separates them. It also
 * means a walk-away range test measures both directions instead of guessing that the link
 * is symmetric.
 *
 * Only the GND is counted. An AIR unit can hear sibling AIR units, whose sequence numbers
 * come from a different sender and would corrupt the arithmetic; the caller guarantees
 * origin == DB_ESPNOW_ORIGIN_GND.
 *
 * **Two spaces.** A GND that addresses control frames to individual AIR units numbers its
 * broadcasts and its packets for each AIR separately (db_espnow_seq_space.h), so that a frame
 * it addressed to a *different* AIR - which this unit never receives - is not a gap here. The
 * frame's 802.11 destination tells the spaces apart; both feed the one reported total. In mode 0
 * nothing is addressed and only the broadcast space ever moves, i.e. this counts exactly as it
 * did before spaces existed. The rule per space (adopt the first packet, add forward gaps,
 * saturate, clear on a backward step) is db_espnow_seq_rx_update(), unchanged in substance.
 *
 * @param seq_num Sequence number from the header of the packet just received from the GND
 * @param addressed true if the packet was addressed to this unit, false if it was broadcast
 */
static void db_espnow_count_uplink_loss(const uint32_t seq_num, const bool addressed) {
    db_espnow_seq_rx_t *space = addressed ? &db_esp_signal_quality.air_seq_rx_addressed
                                          : &db_esp_signal_quality.air_seq_rx_broadcast;
    if (db_espnow_seq_rx_update(space, seq_num, &db_esp_signal_quality.air_rx_packets_lost) ==
        DB_ESPNOW_SEQ_RX_BACKWARD) {
        /* Sequence went backwards: the GND rebooted and restarted at 0, or this is a
         * replayed frame. Same choice the GND makes for its own direction - resynchronise
         * and keep the link usable rather than discard a packet that decrypted correctly. */
        ESP_LOGW(TAG, "Uplink sequence number lower than expected (%s space). GND did reset or packet may be part of a replay attack.",
                 addressed ? "addressed" : "broadcast");
    }
}

/**
 * Add a new ESP-NOW broadcast peer to the list of known ESP-NOW peers. Checks if peer is already known based on MAC.
 * List is then used to keep track of the RSSI, packet loss and signal quality on GND side.
 *
 * @param esp_now_clients List of broadcast peers (ESP-NOW) with their RSSI
 * @param broadcast_peer_mac MAC address of a potentially new broadcast peer
 * @return index of the peer inside the list or -1 in case of error e.g. if list is full or not initialized
 */
int16_t update_peer_list(db_esp_now_clients_list_t *esp_now_clients, uint8_t broadcast_peer_mac[6]) {
    if (esp_now_clients == NULL) { // Check if the list is NULL
        return -1; // Do nothing
    }
    for (uint8_t i = 0; i < esp_now_clients->size; i++) {
        if (memcmp(esp_now_clients->db_esp_now_bpeer_info[i].broadcast_peer_mac, broadcast_peer_mac, ESP_NOW_ETH_ALEN) == 0) {
            // found the client - he is already part of the known clients list
            return i;
        }
    }
    if (esp_now_clients->size == DB_ESPNOW_MAX_BROADCAST_PEERS) { // Check if the list is full
        return -1; // Do nothing - list is full already
    } else {
        // Add new peer to list - Copy the element data to the end of the array
        db_esp_now_bpeer_info_t db_esp_now_bpeer_info = {.gnd_rssi = -127, .last_seq_num = 0, .gnd_rx_lost_packets = 0};
        memcpy(db_esp_now_bpeer_info.broadcast_peer_mac, broadcast_peer_mac, ESP_NOW_ETH_ALEN);
        esp_now_clients->db_esp_now_bpeer_info[esp_now_clients->size] = db_esp_now_bpeer_info;
        esp_now_clients->size++; // Increment the size of the list
        /* Register with ESP-NOW too, so packets can be addressed to this peer when
         * DB_PARAM_ESPNOW_UNICAST asks for it. Harmless while broadcasting. */
        db_espnow_register_peer(broadcast_peer_mac);
        return (esp_now_clients->size-1) ;
    }
}

/**
 * Steps to process received ESPNOW data. ESP firmware ensures that only correct packets are forwarded to us
 * 1. Decrypt & authenticate
 * 2. Check if we know the client based on MAC - update rssi in case we are GND - AIR side only expects to only ever have on GND peer anyways
 * 3. Check sequence number and if we are GND station then update lost packet count based on seq. numbers
 * 4. Write payload to uart-queue so it can be processed by the control_espnow task
 *
 * @param data Received raw data via ESP-NOW
 * @param data_len  Length of received data
 * @param src_addr Source MAC address of the data
 * @param rssi RSSI in dBm of this packet
 * @param noise_floor Noise floor in dBm measured while receiving this packet
 * @param addressed true if the frame was addressed to this unit, false if it was broadcast. Selects the sequence
 *                  space the AIR's uplink loss counter uses (db_espnow_count_uplink_loss()).
 */
void db_espnow_process_rcv_data(uint8_t *data, uint16_t data_len, uint8_t *src_addr, int8_t rssi, int8_t noise_floor,
                                bool addressed) {
    const uint16_t minimum_packet_length = db_esp_now_packet_header_len + DB_ESPNOW_AES_TAG_LEN + 1U;
    if (data == NULL || src_addr == NULL || data_len < minimum_packet_length) {
        ESP_LOGW(TAG, "Ignoring malformed ESP-NOW packet of length %u", data_len);
        return;
    }
    db_esp_now_packet_t *db_esp_now_packet = (db_esp_now_packet_t *) data;
    const uint16_t protected_data_length = data_len - DB_ESPNOW_AES_TAG_LEN - db_esp_now_packet_header_len;
    if (protected_data_length > DB_ESPNOW_PAYLOAD_MAXSIZE + 1U) {
        ESP_LOGW(TAG, "Ignoring oversized ESP-NOW protected payload of length %u", protected_data_length);
        return;
    }
    const uint8_t len_payload = (uint8_t) protected_data_length;
    uint8_t db_decrypted_data[len_payload];

    /* Decrypt and authenticate packet - only then process its contents */
    if (db_decrypt_payload(db_esp_now_packet, db_decrypted_data, len_payload) == 0) {
        const uint8_t origin = db_esp_now_packet->db_esp_now_packet_header.origin;
        if (!db_espnow_origin_is_allowed(origin)) {
            ESP_LOGW(TAG, "Ignoring authenticated ESP-NOW packet with disallowed origin %u", origin);
            return;
        }
        if (db_decrypted_data[0] != len_payload - 1U) {
            ESP_LOGW(TAG, "Ignoring ESP-NOW packet with inconsistent decrypted payload length");
            return;
        }

        /* Check if we know that peer already */
        if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND && origin == DB_ESPNOW_ORIGIN_AIR) {
            db_esp_now_clients_list->gnd_noise_floor = noise_floor;
            int16_t peer_index = update_peer_list(db_esp_now_clients_list, src_addr);
            if (peer_index != -1) {
                // Update the list with RSSI and packet loss only on the GND side.
                db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].gnd_rssi = rssi;
            // check packet sequence number
                if (db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].last_seq_num < db_esp_now_packet->db_esp_now_packet_header.seq_num) {
                // Count the lost packets per peer since the last received packet
                    db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].gnd_rx_lost_packets +=
                        (db_esp_now_packet->db_esp_now_packet_header.seq_num -
                        db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].last_seq_num - 1);
                } else {
                    ESP_LOGW(TAG, "Sequence number lower than expected. Sender did reset or packet may be part of a replay attack.");
                // accept packet anyway for now to make for a more robust link
                    db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].gnd_rx_lost_packets = 0;
                }

                db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].last_seq_num = db_esp_now_packet->db_esp_now_packet_header.seq_num;
            }
        } else if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR && origin == DB_ESPNOW_ORIGIN_GND) {
            // AIR link quality must describe the GND link, never a neighbouring AIR peer.
            db_esp_signal_quality.air_rssi = rssi;
            db_esp_signal_quality.air_noise_floor = noise_floor;
            db_espnow_count_uplink_loss(db_esp_now_packet->db_esp_now_packet_header.seq_num, addressed);
            /* Learn the ground station's address so replies can be addressed to it. The
             * AIR side learnt nothing before 2026-11-11, which is why the first unicast
             * attempt went one way only. Re-register whenever the MAC changes - a
             * different GND, or the very first packet. */
            if (!db_espnow_gnd_mac_known ||
                memcmp(db_espnow_gnd_mac, src_addr, ESP_NOW_ETH_ALEN) != 0) {
                memcpy(db_espnow_gnd_mac, src_addr, ESP_NOW_ETH_ALEN);
                db_espnow_gnd_mac_known = db_espnow_register_peer(db_espnow_gnd_mac);
            }
        }

        /* Process packet depending on packet type */
        if (db_esp_now_packet->db_esp_now_packet_header.packet_type == DB_ESP_NOW_PACKET_TYPE_DATA) {
            // Pass data to UART queue
            db_espnow_uart_queue_event_t db_uart_evt = {0};
            db_uart_evt.data_len = db_decrypted_data[0];    // should be equal to len_payload-1 if everything worked out
            db_uart_evt.data = malloc(db_uart_evt.data_len);
            if (db_uart_evt.data == NULL) {
                /* Out of heap: drop this packet rather than memcpy into NULL and reset the
                 * unit mid-flight. The link is lossy anyway; a reboot is not. */
                db_link_stats_inc(&db_link_stats.drop_alloc_fail);
                ESP_LOGE(TAG, "malloc(%u) failed - dropping received packet", db_uart_evt.data_len);
                return;
            }
            // For some reason it seems we cannot directly decrypt to db_espnow_uart_event_t -> Queues get set to NULL
            memcpy(db_uart_evt.data, &db_decrypted_data[1], db_uart_evt.data_len);
            memcpy(db_uart_evt.source_mac, src_addr, ESP_NOW_ETH_ALEN);
            db_uart_evt.seq_num = db_esp_now_packet->db_esp_now_packet_header.seq_num;
            db_uart_evt.origin = origin;
            /* Drops rather than blocks, for the mirror of the reason in
             * db_send_to_all_espnow: this task is the only one draining
             * db_espnow_send_queue, and db_uart_write_queue is drained only by the
             * control task. With 512 ms budgets on both, each task could sit waiting
             * to hand work to the other - a circular wait that resolves only by
             * timeout and loss. See memory.md 5.4. */
            if (xQueueSend(db_uart_write_queue, &db_uart_evt,
                           pdMS_TO_TICKS(DB_ESPNOW_DATA_QUEUE_DELAY_MS)) != pdTRUE) {
                ESP_LOGW(TAG, "db_uart_write_queue full - dropping %u B downlink",
                         db_uart_evt.data_len);
                db_link_stats_inc(&db_link_stats.drop_uart_queue_full);
                free(db_uart_evt.data);
            } else {
                // all good
            }
        } else if (db_esp_now_packet->db_esp_now_packet_header.packet_type == DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY &&
                   DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR && origin == DB_ESPNOW_ORIGIN_GND) {
            /* This is only called on the AIR side since GND sends telemetry to AIR only */
            if (db_decrypted_data[0] != sizeof(db_esp_now_clients_list_t)) {
                ESP_LOGW(TAG, "Ignoring ESP-NOW internal telemetry with unexpected payload length");
                return;
            }
            db_esp_now_clients_list_t *clients = (db_esp_now_clients_list_t*) &db_decrypted_data[1];
            ESP_LOGD(TAG, "Received internal telemetry frame containing %i entries", clients->size);
            for (int i = 0; i < clients->size; i++) {
                if (memcmp(LOCAL_MAC_ADDRESS, clients->db_esp_now_bpeer_info[i].broadcast_peer_mac, ESP_NOW_ETH_ALEN) == 0) {
                    // found us (this local ESP32 AIR unit) -> update internal telemetry buffer,
                    // so it gets sent with next Mavlink RADIO STATUS in case MAVLink radio status is enabled
                    db_esp_signal_quality.gnd_noise_floor = clients->gnd_noise_floor;
                    db_esp_signal_quality.gnd_rssi = clients->db_esp_now_bpeer_info[i].gnd_rssi;
                    db_esp_signal_quality.gnd_rx_packets_lost = clients->db_esp_now_bpeer_info[i].gnd_rx_lost_packets;
                    break;
                } else {
                    // keep on looking for our MAC
                }
            }
        } else {
            ESP_LOGW(TAG, "Received unknown DroneBridge for ESP32 ESP-NOW packet type");
        }
    } else {
        ESP_LOGE(TAG, "Failed to Decrypt & Authenticate. Ignoring");
        return;
    }
}

/**
 * ESP-NOW sending callback function is called in WiFi task.
 * Do not do lengthy operations from this task. Instead, post necessary data to a queue and handle it from the ESP-NOW
 * task.
 * Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function.
 * So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending
 * has returned.
 *
 * Posts to its own queue with a zero timeout: never blocks the WiFi task. If the post ever fails the ESP-NOW task's
 * deadline (DB_ESPNOW_SEND_CB_TIMEOUT_MS) releases the send path and counts it, so a lost event costs at most that
 * long instead of wedging transmission. A failed send is no longer logged here - logging from the WiFi task slows
 * the radio - but by the ESP-NOW task, which also counts it.
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void db_espnow_send_callback(const esp_now_send_info_t *esp_now_send_info, esp_now_send_status_t status) {
    if (esp_now_send_info == NULL || esp_now_send_info->des_addr == NULL) {
        return;     // nothing to report; the send path deadline recovers
    }
    const uint8_t *mac_addr = esp_now_send_info->des_addr;
#else
static void db_espnow_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (mac_addr == NULL) {
        return;     // nothing to report; the send path deadline recovers
    }
#endif
    db_espnow_event_send_cb_t send_cb;
    memcpy(send_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb.status = status;
    if (db_espnow_send_cb_queue != NULL) {
        (void) xQueueSend(db_espnow_send_cb_queue, &send_cb, 0);
    }
}

/**
 * ESP-NOW receiving callback function is called in WiFi task. This is the entry point when new data is incoming.
 * Do not do lengthy operations from this task. Instead, post necessary data to a queue and handle it from the control
 * module task
 */
static void db_espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    db_espnow_event_t evt;
    db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    const uint16_t minimum_packet_length = db_esp_now_packet_header_len + DB_ESPNOW_AES_TAG_LEN + 1U;

    if (recv_info == NULL || data == NULL || len < minimum_packet_length) {
        ESP_LOGE(TAG, "Receive callback arg error");
        return;
    }
    uint8_t *src_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;
    if (src_addr == NULL || des_addr == NULL) {
        ESP_LOGE(TAG, "Receive callback address error");
        return;
    }

    /* Addressed (unicast) frames are accepted as of 2026-11-11. Upstream rejected
     * them here because it only ever broadcast, and that one early return is what made
     * the 2026-11-07 unicast attempt fail: the send side was switched over while every
     * addressed frame was still discarded at the peer, before it was even decrypted.
     * Accepting them is safe - the payload is AES-GCM authenticated below, so a forged
     * frame gets no further whatever its destination address. See
     * docs/decisions/2026-11-10-link-reliability-architecture.md.
     * des_addr decides which sequence space the packet belongs to (db_espnow_seq_space.h):
     * anything that is not the broadcast address reached us only because it was addressed to
     * us - the radio filters out frames addressed to other units. */

    // The header is authenticated later. This early rejection only avoids allocating and queueing known-disallowed traffic.
    if (!db_espnow_origin_is_allowed(data[0])) {
        ESP_LOGD(TAG, "Receive ESP-NOW data with disallowed origin. Ignoring");
        return;
    }

    evt.id = DB_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, src_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        db_link_stats_inc(&db_link_stats.drop_alloc_fail);
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    recv_cb->addressed = !IS_BROADCAST_ADDR(des_addr);
    recv_cb->rssi = recv_info->rx_ctrl == NULL ? -127 : recv_info->rx_ctrl->rssi;
    recv_cb->noise_floor = 0;
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C2) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C6)
    if (recv_info->rx_ctrl != NULL) {
        recv_cb->noise_floor = recv_info->rx_ctrl->noise_floor;
    }
#endif
    /* Zero timeout: this is the WiFi driver's task. Waiting here (it used to be up to 512 ms)
     * stalls every other receive and every send completion on the unit, i.e. turns one full
     * queue into a radio-wide outage. A packet that does not fit is dropped and counted. No
     * log line either - at a high drop rate the logging itself would slow the WiFi task. */
    if (db_espnow_send_recv_callback_queue == NULL ||
        xQueueSend(db_espnow_send_recv_callback_queue, &evt, 0) != pdTRUE) {
        db_link_stats_inc(&db_link_stats.drop_rx_queue_full);
        free(recv_cb->data);
    }
}

/**
 *  Init/Create structure containing all ESP-NOW broadcast connection quality information
 *  List is used to keep track of the RSSI and signal quality on GND side.
 * @return Structure containing all ESP-NOW broadcast connection quality information
 */
db_esp_now_clients_list_t *db_espnow_broadcast_peers_list_create() {
    db_esp_now_clients_list_t *client_list = malloc(sizeof(db_esp_now_clients_list_t)); // Allocate memory for the list
    if (client_list == NULL) { // Check if the allocation failed
        return NULL; // Return NULL to indicate an error
    }
    client_list->size = 0; // Initialize the size to 0
    return client_list; // Return the pointer to the list
}

/**
 * Destroy structure containing all ESP-NOW broadcast connection quality information
 * @param db_esp_now_clients Structure containing all ESP-NOW broadcast connection quality information
 */
void db_espnow_broadcast_peers_list_destroy(db_esp_now_clients_list_t *esp_now_clients) {
    if (esp_now_clients == NULL) { // Check if the list is NULL
        return; // Do nothing
    }
    free(esp_now_clients); // Free the list
}

/**
 * Init mbedtls aes gcm mode and set encryption key based on the WiFi password specified by the user
 * @param aes_key buffer for saving the generated aes key of len AES_256_KEY_BYTES
 * @return result of mbedtls_gcm_setkey
 */
int init_gcm_encryption_module(uint8_t *aes_key) {
    mbedtls_gcm_init(&aes);
    const char *espnow_secret = strlen(DB_PARAM_ESPNOW_LINK_SECRET) > 0 ? DB_PARAM_ESPNOW_LINK_SECRET : DB_PARAM_PASS;
    generate_pkcs5_key(espnow_secret, aes_key, AES_256_KEY_BYTES);
    int ret = mbedtls_gcm_setkey(&aes, MBEDTLS_CIPHER_ID_AES, aes_key, DB_ESPNOW_AES_KEY_LEN);
    return ret;
}

void deinit_espnow_all(){
    ESP_LOGW(TAG, "De init ESPNOW incl. Queues & AES");
    mbedtls_gcm_free(&aes);
    vSemaphoreDelete(db_espnow_send_recv_callback_queue);
    vSemaphoreDelete(db_espnow_send_cb_queue);
    vSemaphoreDelete(db_espnow_send_queue); // ToDo: Check if that is a good idea since control task might be using it
    vSemaphoreDelete(db_uart_write_queue); // ToDo: Check if that is a good idea since control task might be using it
    db_espnow_broadcast_peers_list_destroy(db_esp_now_clients_list);
    esp_now_deinit();
}

/**
 * Init all relevant structures and Queues for ESP-NOW communication
 * @return ESP_FAIL on failure or ESP_OK on success
 */
esp_err_t db_espnow_init() {
    ESP_LOGI(TAG, "Initializing ESP-NOW parameters");
    db_esp_now_clients_list = db_espnow_broadcast_peers_list_create();
    /* Init Queue for ESP-NOW internal callbacks when packet is finally sent or received */
    db_espnow_send_recv_callback_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_event_t));
    if (db_espnow_send_recv_callback_queue == NULL) {
        ESP_LOGE(TAG, "Create db_espnow_send_recv_callback_queue mutex fail");
        return ESP_FAIL;
    }
    db_espnow_send_cb_queue = xQueueCreate(DB_ESPNOW_SEND_CB_QUEUE_SIZE, sizeof(db_espnow_event_send_cb_t));
    if (db_espnow_send_cb_queue == NULL) {
        ESP_LOGE(TAG, "Create db_espnow_send_cb_queue fail");
        return ESP_FAIL;
    }
    db_espnow_tx_tracker_init(&db_espnow_tx_tracker);
    /* Only the GND addresses more than one destination, so only it needs a space per
     * destination; see db_espnow_seq_space.h for why the AIR deliberately keeps one. */
    db_espnow_seq_tx_init(&db_espnow_seq_tx, DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND);

    /* Init Queues for communication with control task */
    db_espnow_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_queue_event_t));
    db_uart_write_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_uart_queue_event_t));
    if (db_espnow_send_queue == NULL) {
        ESP_LOGE(TAG, "Create db_espnow_send_queue mutex fail");
        return ESP_FAIL;
    }
    if (db_uart_write_queue == NULL) {
        ESP_LOGE(TAG, "Create db_uart_write_queue mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESP-NOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(db_espnow_send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(db_espnow_receive_callback));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    if (!esp_now_is_peer_exist(BROADCAST_MAC)) ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    /* The broadcast MAC is a peer like any other, so the rate applies to it as well.
     * This is what makes DB_PARAM_ESPNOW_RATE testable on its own: broadcast traffic on
     * an LR rate, with addressing left alone. Upstream never set a rate for this peer,
     * which is why it is an open question what rate broadcast has been using. */
    db_espnow_apply_rate_config(BROADCAST_MAC);

    /* Pin the packet size to the ESP-NOW payload limit — in BOTH directions.
     * Every ESP-NOW frame costs a full airtime slot + AES-GCM overhead + a
     * send-callback round trip regardless of payload length, so a value BELOW
     * the limit only fragments the stream (e.g. trans_pack_size=16 would burn
     * three radio slots on a single 40-byte MAVLink telemetry frame), and a
     * value above it cannot be carried in one frame anyway. Runtime-only
     * override: the persisted parameter value is left untouched. */
    if (DB_PARAM_SERIAL_PACK_SIZE != DB_ESPNOW_PAYLOAD_MAXSIZE) {
        ESP_LOGI(TAG, "ESP-NOW: overriding trans_pack_size %i -> %i (radio payload limit)",
                 DB_PARAM_SERIAL_PACK_SIZE, DB_ESPNOW_PAYLOAD_MAXSIZE);
        DB_PARAM_SERIAL_PACK_SIZE = DB_ESPNOW_PAYLOAD_MAXSIZE;
    } else {
        // all good
    }

    /* Init AES GCM encryption module */
    uint8_t aes_key[AES_256_KEY_BYTES];
    int ret = init_gcm_encryption_module(aes_key);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_gcm_setkey returned an error: %i", ret);
        deinit_espnow_all();
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_now_set_pmk(aes_key));  // only first 16 bytes will be used

    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        ESP_LOGI(TAG, "ESP-NOW for DroneBridge init done - acting as ESP-NOW GND station");
    } else {
        ESP_LOGI(TAG, "ESP-NOW for DroneBridge init done - acting as ESP-NOW AIR station");
    }
    return ESP_OK;
}

/**
 * Send ESP-NOW DroneBridge for ESP32 internal telemetry packet containing RSSI and noise floor info from GND to AIR.
 * Used by AIR side to create a RADIO_STATUS MAVLink message sent to GCS.
 *
 * Runs in the ESP-NOW task, which is the only task that drains db_espnow_send_queue - so it must never wait for a
 * slot there (it would be waiting for itself). Zero timeout; the caller's free-slot gate makes a full queue
 * practically impossible, and a refusal just leaves the frame pending for the next send callback.
 * Routed DB_ESPNOW_DEST_AUTO: broadcast in ESPNOW_UNICAST modes 0 and 1 (in mode 1 it is housekeeping, not a control
 * frame, and every AIR reads it), the single known peer in mode 2 as before.
 *
 * @return true when packet was scheduled for sending, false if it will not be sent
 */
bool db_espnow_schedule_internal_telemetry_packet() {
    if (DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        // Only GND sends internal telemetry. This function was called wrongly
        return false;
    } else {
        // continue
    }
    // ToDo: Split list if longer than max payload size
    if (sizeof(db_esp_now_clients_list_t) > DB_ESPNOW_PAYLOAD_MAXSIZE) {
        ESP_LOGE(TAG, "Size of db_esp_now_clients_list_t is > %i", DB_ESPNOW_PAYLOAD_MAXSIZE);
        return false;
    } else {
        /* continue */
    }
    ESP_LOGD(TAG, "Scheduling int. telem. packet");
    uint8_t payload_size = sizeof(db_esp_now_clients_list_t);

    db_espnow_queue_event_t evt = {0};
    // Set payload of DroneBridge for ESP32 ESP-NOW packet
    evt.data = malloc(payload_size);
    if (evt.data == NULL) {
        db_link_stats_inc(&db_link_stats.drop_alloc_fail);
        ESP_LOGE(TAG, "malloc(%u) failed - peer-list frame postponed", payload_size);
        return false;
    }
    memcpy(evt.data, db_esp_now_clients_list, payload_size);
    evt.data_len = payload_size;
    evt.packet_type = DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY;
    evt.dest_mode = DB_ESPNOW_DEST_AUTO;
    if (xQueueSend(db_espnow_send_queue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Send to db_espnow_send_queue queue fail");
        free(evt.data);
        return false;
    } else {
        // all good
        return true;
    }
}

/**
 * Log a failed send, at most once per DB_ESPNOW_SEND_FAIL_LOG_INTERVAL_MS.
 *
 * Failures are already counted in db_link_stats by db_espnow_tx_on_send_cb(); this line only
 * tells someone at the console to look there. Unthrottled it was one line per failure, which at
 * the edge of range with addressed frames is several lines a second on a 115200-baud console,
 * written from the ESP-NOW task's own loop. The line carries how many failures it stands for.
 * Called only from the ESP-NOW task, so the statics need no locking.
 *
 * @param mac Destination MAC of the failed send, as reported by the send callback
 */
static void db_espnow_log_send_failure(const uint8_t mac[ESP_NOW_ETH_ALEN]) {
    static bool logged_once = false;
    static int64_t last_log_us = 0;
    static uint32_t suppressed = 0;
    const int64_t now_us = esp_timer_get_time();
    if (logged_once && now_us - last_log_us < (int64_t) DB_ESPNOW_SEND_FAIL_LOG_INTERVAL_MS * 1000) {
        suppressed++;
        return;
    }
    ESP_LOGW(TAG, "ESP-NOW send failed - MAC: " MACSTR " (+%lu suppressed since last line; all counted in link stats)",
             MAC2STR(mac), (unsigned long) suppressed);
    suppressed = 0;
    last_log_us = now_us;
    logged_once = true;
}

/**
 *  Task that handles all ESP-NOW related data processing. Reads ESP-NOW Callback-Queue, Reads ESP-NOW send queue and
 *  writes to UART-WRITE Queue.
 */
_Noreturn void process_espnow_data() {
    esp_err_t err = db_espnow_init();
    if (err == ESP_OK) {
        // all good. continue
    } else {
        ESP_LOGE(TAG, "Failed to init espnow (db_espnow_init()) aborting start of db_espnow task");
        vTaskDelete(NULL);
    }
    db_espnow_event_t evt;
    db_espnow_event_send_cb_t send_cb_evt;
    /* Whether the next packet may go to the radio now lives in db_espnow_tx_tracker (initialised by db_espnow_init):
     * same single-flight rule as the old ready_to_send flag, plus a deadline so a lost callback cannot wedge it. */
    bool send_internal_telemetry_frame = false;    // flag that indicates a new internal telemetry frame shall be sent
    int delay_timer_cnt = 0;
    /* Starts in the past so the first peer-list frame goes out immediately rather than
     * after a period of silence - the AIR's RADIO_STATUS has nothing to report until it
     * arrives. */
    int64_t last_internal_telemetry_us = -(int64_t) DB_ESPNOW_INTERNAL_TELEMETRY_PERIOD_US;
    while(1) {
        /* Send-callback results first: they are what releases the send path. */
        if (db_espnow_send_cb_queue != NULL && xQueueReceive(db_espnow_send_cb_queue, &send_cb_evt, 0) == pdTRUE) {
            const bool success = send_cb_evt.status == ESP_NOW_SEND_SUCCESS;
            if (!success) {
                db_espnow_log_send_failure(send_cb_evt.mac_addr);
            }
            // the last sending callback has returned, so we keep the order of packets. ESP-NOW by default does not
            // guarantee the order when many packets are sent in quick succession. A late callback (its packet was
            // already written off by the deadline below) releases nothing.
            if (db_espnow_tx_on_send_cb(&db_espnow_tx_tracker, success, &db_link_stats)) {
                /* Housekeeping only takes a queue slot while there are slots to
                 * spare. The peer-list frame shares db_espnow_send_queue with the
                 * GCS's uplink in FIFO order, so it cannot jump ahead of user data -
                 * but at 211 B it can crowd the queue out by sheer volume, and a full
                 * queue is what stalls the control task. Under pressure, drain user
                 * data instead and let the RSSI display arrive late. */
                if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND && send_internal_telemetry_frame &&
                    db_espnow_send_queue != NULL &&
                    uxQueueSpacesAvailable(db_espnow_send_queue) > (ESPNOW_QUEUE_SIZE / 2)) {
                    send_internal_telemetry_frame = !db_espnow_schedule_internal_telemetry_packet();
                } else {
                    // try to immediately send the next packet if available (marks it in flight if sent)
                    (void) db_read_uart_queue_and_send();
                }
            }
        }
        /* The deadline: a send callback that never arrives (event lost, or a driver that never reports) must not stop
         * transmission for good. The packet is written off - its fate is unknown, so it is counted neither as
         * delivered nor as failed - and sending resumes. */
        if (db_espnow_tx_check_timeout(&db_espnow_tx_tracker, (uint32_t) (esp_timer_get_time() / 1000),
                                       DB_ESPNOW_SEND_CB_TIMEOUT_MS, &db_link_stats)) {
            ESP_LOGW(TAG, "No ESP-NOW send callback within %d ms - resuming the send path", DB_ESPNOW_SEND_CB_TIMEOUT_MS);
        }
        if (db_espnow_send_recv_callback_queue == NULL) ESP_LOGE(TAG, "db_espnow_send_recv_callback_queue is NULL!");
        if(db_espnow_send_recv_callback_queue != NULL && xQueueReceive(db_espnow_send_recv_callback_queue, &evt, 0) == pdTRUE) {
            switch (evt.id) {
                case DB_ESPNOW_RECV_CB: {
                    db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                    db_espnow_process_rcv_data(recv_cb->data, recv_cb->data_len, recv_cb->mac_addr, recv_cb->rssi,
                                                recv_cb->noise_floor, recv_cb->addressed);
                    free(recv_cb->data);
                    break;
                }
                default:
                    ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                    break;
            }
        }
        /* process UART -> ESP-NOW */
        if (db_espnow_tx_ready(&db_espnow_tx_tracker)) {
            // send the next packet if available (marks it in flight if sent)
            (void) db_read_uart_queue_and_send();
        } else {
            // do nothing - we are not ready for sending another packet
        }

        if (delay_timer_cnt == 5000) {
            /* all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference */
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
        } else {
            delay_timer_cnt++;
        }
        /* The peer-list frame is scheduled on a wall clock, NOT on the loop counter
         * above. Tying it to iteration count put it on air at roughly 66 Hz - the loop
         * spins through 5000 trivial iterations in a few ms - which is ~113 kbps of
         * 211 B housekeeping frames. Worse, the rate then varied with how busy the loop
         * was, so the load rose exactly when the link could least afford it. */
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_internal_telemetry_us >= DB_ESPNOW_INTERNAL_TELEMETRY_PERIOD_US) {
            last_internal_telemetry_us = now_us;
            send_internal_telemetry_frame = true;
        }
    }
    vTaskDelete(NULL);
}

/**
 * Start task that handles ESP-NOW data
 */
void db_start_espnow_module() {
    xTaskCreate(&process_espnow_data, "db_espnow", 40960, NULL, 5, NULL);
}
