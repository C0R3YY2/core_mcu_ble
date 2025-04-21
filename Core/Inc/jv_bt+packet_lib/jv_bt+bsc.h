/**
 *       _                       __        ___          _
 *      | | ___  _____   ____ _  \ \      / (_)_ __ ___| | ___  ___ ___
 *   _  | |/ _ \/ _ \ \ / / _` |  \ \ /\ / /| | '__/ _ \ |/ _ \/ __/ __|
 *  | |_| |  __/  __/\ V / (_| |   \ V  V / | | | |  __/ |  __/\__ \__ \
 *   \___/ \___|\___| \_/ \__,_|    \_/\_/  |_|_|  \___|_|\___||___/___/
 *
 * @file jv_bt+bsc.h
 * @author Jerrold Erickson (jerrold.erickson@jeevawireless.com)
 * @brief Library functions for preparing a BLE packet to be backscattered
 * @date 2022-10-14
 *
 * @copyright Copyright (c) 2022 Jeeva Wireless
 *
 */

#ifndef JV_BT_BSC_H
#define JV_BT_BSC_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Upscale a ble packet for backscatter at 1 Mbps phy
 *
 * @param dst Destination buffer for the upscaled packet
 * @param packet Pointer to whitened buffer from a jv_ble_packet
 * @param packet_len length of whitened buffer
 * @return uint32_t Size of upscaled packet in Bytes
 *
 * @warning No bounds checking on size of dst buffer.
 */
uint32_t jv_bsc_upscale_1Mbps(uint32_t *dst, uint8_t *packet, size_t packet_len);

/**
 * @brief Upscale a ble packet for backscatter at 2 Mbps phy
 *
 * @param dst Destination buffer for the upscaled packet
 * @param packet Pointer to whitened buffer from a jv_ble_packet
 * @param packet_len length of whitened buffer
 * @return uint32_t Size of upscaled packet in Bytes
 *
 * @warning No bounds checking on size of dst buffer.
 */
uint32_t jv_bsc_upscale_2Mbps(uint32_t *dst, uint8_t *packet, size_t packet_len);

#endif
