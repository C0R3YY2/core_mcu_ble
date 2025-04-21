/**
 *       _                       __        ___          _
 *      | | ___  _____   ____ _  \ \      / (_)_ __ ___| | ___  ___ ___
 *   _  | |/ _ \/ _ \ \ / / _` |  \ \ /\ / /| | '__/ _ \ |/ _ \/ __/ __|
 *  | |_| |  __/  __/\ V / (_| |   \ V  V / | | | |  __/ |  __/\__ \__ \
 *   \___/ \___|\___| \_/ \__,_|    \_/\_/  |_|_|  \___|_|\___||___/___/
 *
 * @file jv_bt+packet.h
 * @author Jerrold Erickson (jerrold.erickson@jeevawireless.com)
 * @brief Library functions for creating BLE packets for backscatter
 * @date 2022-10-13
 *
 * @copyright Copyright (c) 2022 Jeeva Wireless
 *
 */

#ifndef JV_BT_PACKET_H
#define JV_BT_PACKET_H

#include <stddef.h>
#include <stdint.h>

#define UNCODED_PREAMBLE_SIZE_1MBPS       1
#define UNCODED_PREAMBLE_SIZE_2MBPS       2
#define ACCESS_ADDRESS_SIZE               4
#define HEADER_SIZE                       2
#define ADVERTISING_ADDRESS_SIZE          6
#define MAX_ADVERTISING_DATA_SIZE         31 // true for legacy advertising packets
#define MAX_PDU_SIZE                      (HEADER_SIZE + ADVERTISING_ADDRESS_SIZE + MAX_ADVERTISING_DATA_SIZE)
#define CRC_SIZE                          3
#define WHITENING_SIZE                    (MAX_PDU_SIZE + CRC_SIZE)
#define UNCODED_MAX_PACKET_SIZE           (UNCODED_PREAMBLE_SIZE_2MBPS + ACCESS_ADDRESS_SIZE + WHITENING_SIZE)
#define CODED_PREAMBLE_SIZE               10
#define CODED_FEC1_SIZE                   ((ACCESS_ADDRESS_SIZE * 8) + 2 + 3)
#define CODED_FEC2_S2_SIZE                ((WHITENING_SIZE * 2) + 1)
#define CODED_FEC2_S8_SIZE                ((WHITENING_SIZE * 8) + 3)
#define CODED_MAX_PACKET_SIZE             (CODED_PREAMBLE_SIZE + CODED_FEC1_SIZE + CODED_FEC2_S8_SIZE)

enum pdu_type_t
{
    ADV_IND =         0b0000,
    ADV_DIRECT_IND =  0b0001,
    ADV_NONCONN_IND = 0b0010,
    SCAN_REQ =        0b0011,
    SCAN_RSP =        0b0100,
    CONNECT_IND =     0b0101,
    ADV_SCAN_IND =    0b0110,
    ADV_EXT_IND =     0b0111,
    AUX_CONNECT_RSP = 0b1000
};

typedef enum pdu_type_t pdu_type_t;

enum jv_packet_encoding_t
{
    UNCODED_1MBPS,
    UNCODED_2MBPS,
    CODED_S2,
    CODED_S8
};

typedef enum jv_packet_encoding_t jv_packet_encoding_t;

typedef struct jv_ble_pdu
{
    uint8_t pdu[MAX_PDU_SIZE];
    uint8_t pdu_len;
} jv_ble_pdu;

typedef struct jv_ble_packet
{
    uint8_t whitening_lookup_table[WHITENING_SIZE];
    uint8_t whitened_packet[CODED_MAX_PACKET_SIZE];
    size_t packet_len;
    jv_packet_encoding_t encoding;
} jv_ble_packet;


/**
 * @brief Create a legacy advertising pdu object
 *
 * @param pdu Pointer to a jv_ble_pdu object to be initalized
 * @param AdvA Pointer to an array of Bytes containing the packet advertising address
 * @param AdvA_len Length of AdvA, in Bytes. Must be equal to 6.
 * @param AdvData Pointer to an array of Bytes containing the packet advertising data
 * @param AdvData_len Length of AdvData, in Bytes. Must be 31 or less.
 * @return int -1 if invalid arguments, or 0 if successful
 */
int create_legacy_advertising_pdu(jv_ble_pdu *pdu, uint8_t *AdvA, uint8_t AdvA_len, uint8_t *AdvData, uint8_t AdvData_len);

/**
 * @brief
 *
 * @param packet Pointer to jv_ble_packet object to be initalized
 * @param ch BLE channel number. Must be 39 or less.
 * @param pdu Pointer to jv_ble_pdu object initalized from a successful call to create_legacy_advertising_pdu()
 * @param encoding
 * @return int -1 if invalid arguments, or 0 if successful
 */
int init_packet(jv_ble_packet *packet, uint8_t ch, jv_ble_pdu *pdu, jv_packet_encoding_t encoding);


/**
 * @brief Update the jv_ble_packet object
 *
 * @param packet Pointer to jv_ble_packet object to be updated
 * @param pdu Pointer to jv_ble_pdu initalized from a successful call to create_legacy_advertising_pdu()
 *
 * @warning PDU->pdu_len must be the same as the pdu originally used when init_uncoded_packet() was called.
 */
void update_advertising_packet(jv_ble_packet *packet, jv_ble_pdu *pdu);

size_t encode_packet(uint8_t *dst, jv_ble_packet *packet);

#endif
