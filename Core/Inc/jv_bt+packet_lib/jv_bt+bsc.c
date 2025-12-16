/**
 *       _                       __        ___          _
 *      | | ___  _____   ____ _  \ \      / (_)_ __ ___| | ___  ___ ___
 *   _  | |/ _ \/ _ \ \ / / _` |  \ \ /\ / /| | '__/ _ \ |/ _ \/ __/ __|
 *  | |_| |  __/  __/\ V / (_| |   \ V  V / | | | |  __/ |  __/\__ \__ \
 *   \___/ \___|\___| \_/ \__,_|    \_/\_/  |_|_|  \___|_|\___||___/___/
 *
 * @file jv_bt+bsc.c
 * @author Jerrold Erickson (jerrold.erickson@jeevawireless.com)
 * @brief Library functions for preparing a BLE packet to be backscattered
 * @date 2022-10-14
 *
 * @copyright Copyright (c) 2022 Jeeva Wireless
 *
 */
//gpt added
//#include "core_cm4.h"   /* gives us __REV() for byte‑swap */
//#include "stm32l4xx.h"  /* device header – safely defines __REV() */
#include "stm32g0xx.h"	/* device header – safely defines __REV() */

#include "jv_bt+bsc.h"



static const uint8_t upscale_pattern_1Mbps[2] = {
    0xAA, // bit 0
    0xD2  // bit 1
};



#define CONCAT(a, b, c) a##b##c

/*
#ifndef BLE_OFFSET
#define BLE_OFFSET 35
#endif

#if BLE_OFFSET == -15
#define THREE_ENCODING_1Mbps CONCAT(0X, f0f0, f0f0) //f0f0 = 1111000011110000
#define TWO_ENCODING_1Mbps   CONCAT(0X, 00ff, f0f0) //00ff = 0000000011111111
#define ONE_ENCODING_1Mbps   CONCAT(0x, f0f0, 00ff)
#define ZERO_ENCODING_1Mbps  CONCAT(0x, 00ff, 00ff)

#elif BLE_OFFSET == -30
#define THREE_ENCODING_1Mbps CONCAT(0X, 9831, 67ce) //9831 = 1001100000110001
#define TWO_ENCODING_1Mbps   CONCAT(0X, 711c, 67ce) //67ce = 0110011111001110
#define ONE_ENCODING_1Mbps   CONCAT(0x, 9831, 8ee3)
#define ZERO_ENCODING_1Mbps  CONCAT(0x, 711c, 8ee3)

#elif BLE_OFFSET == -35
#define THREE_ENCODING_1Mbps CONCAT(0X, CCCC, CCCC)
#define TWO_ENCODING_1Mbps   CONCAT(0X, 38E7, CCCC)
#define ONE_ENCODING_1Mbps   CONCAT(0x, CCCC, 38E7)
#define ZERO_ENCODING_1Mbps  CONCAT(0x, 38E7, 38E7)

#elif BLE_OFFSET == -40
#define THREE_ENCODING_1Mbps CONCAT(0X, 6c26, 93d9) //6c26 = 0110110000100110
#define TWO_ENCODING_1Mbps   CONCAT(0X, 9831, 93d9) //93d9 = 1001001111011001
#define ONE_ENCODING_1Mbps   CONCAT(0x, 6c26, 67ce)
#define ZERO_ENCODING_1Mbps  CONCAT(0x, 9831, 67ce)

#elif BLE_OFFSET == -45
#define THREE_ENCODING_1Mbps CONCAT(0X, db24, db24) //db24 = 1101101100100100
#define TWO_ENCODING_1Mbps   CONCAT(0X, 3333, db24) //3333 = 0011001100110011
#define ONE_ENCODING_1Mbps   CONCAT(0x, db24, 3333)
#define ZERO_ENCODING_1Mbps  CONCAT(0x, 3333, 3333)

//**************************************************
//**************** Positive offsets ****************
//**************************************************

#elif BLE_OFFSET == 15
#define THREE_ENCODING_1Mbps CONCAT(0x, 00ff, 00ff) //00ff = 0000000011111111
#define TWO_ENCODING_1Mbps   CONCAT(0x, f0f0, 00ff) //f0f0 = 1111000011110000
#define ONE_ENCODING_1Mbps   CONCAT(0X, 00ff, f0f0)
#define ZERO_ENCODING_1Mbps  CONCAT(0X, f0f0, f0f0)

#elif BLE_OFFSET == 30
#define THREE_ENCODING_1Mbps CONCAT(0x, 711c, 8ee3) //711c = 0111000100011100
#define TWO_ENCODING_1Mbps   CONCAT(0x, 9831, 8ee3) //8ee3 = 1000111011100011
#define ONE_ENCODING_1Mbps   CONCAT(0X, 711c, 67ce) //9831 = 1001100000110001
#define ZERO_ENCODING_1Mbps  CONCAT(0X, 9831, 67ce) //67ce = 0110011111001110


#elif BLE_OFFSET == 35
#define THREE_ENCODING_1Mbps CONCAT(0x, 38E7, 38E7)
#define TWO_ENCODING_1Mbps   CONCAT(0x, CCCC, 38E7)
#define ONE_ENCODING_1Mbps   CONCAT(0X, 38E7, CCCC)
#define ZERO_ENCODING_1Mbps  CONCAT(0X, CCCC, CCCC)


#elif BLE_OFFSET == 40
#define THREE_ENCODING_1Mbps CONCAT(0x, 9831, 67ce) //9831 = 1001100000110001
#define TWO_ENCODING_1Mbps   CONCAT(0x, 6c26, 67ce) //67ce = 0110011111001110
#define ONE_ENCODING_1Mbps   CONCAT(0X, 9831, 93d9) //6c26 = 0110110000100110
#define ZERO_ENCODING_1Mbps  CONCAT(0X, 6c26, 93d9) //93d9 = 1001001111011001

#elif BLE_OFFSET == 45
#define THREE_ENCODING_1Mbps CONCAT(0x, 3333, 3333) //3333 = 0011001100110011 (4MHz)
#define TWO_ENCODING_1Mbps   CONCAT(0x, db24, 3333) //db24 = 1101101100100100 (5MHz)
#define ONE_ENCODING_1Mbps   CONCAT(0X, 3333, db24)
#define ZERO_ENCODING_1Mbps  CONCAT(0X, db24, db24)
#else
#error "Invalid BLE_OFFSET"
#endif

// -3 MHz offset for 2M PHY
#define THREE_ENCODING_2Mbps CONCAT(0X, 0000, CCCC) //CCCC = 1100110011001100
#define TWO_ENCODING_2Mbps   CONCAT(0X, 0000, F0CC) //F0CC = 1111000011001100
#define ONE_ENCODING_2Mbps   CONCAT(0x, 0000, CCF0) //CCF0 = 1100110011110000
#define ZERO_ENCODING_2Mbps  CONCAT(0x, 0000, F0F0) //F0F0 = 1111000011110000
*/

/*
const uint32_t upscale_lookup_1Mbps[4] = {
    ZERO_ENCODING_1Mbps,
    ONE_ENCODING_1Mbps,
    TWO_ENCODING_1Mbps,
    THREE_ENCODING_1Mbps};
*/

/*
const uint32_t upscale_lookup_2Mbps[4] = {
    ZERO_ENCODING_2Mbps,
    ONE_ENCODING_2Mbps,
    TWO_ENCODING_2Mbps,
    THREE_ENCODING_2Mbps};
*/

/*
uint32_t jv_bsc_upscale_1Mbps(uint32_t *dst, uint8_t *packet, size_t packet_len)
{

    // We are writing 2 bytes per bit
    //                  = 16 bytes per byte  (times 8 on both sides)
    //                  = 4 uint32s per byte (1 uint32 has 4 bytes bc 4*8=32)
    //   So we stop after writing (4 * packet_len) uint32s
    const uint32_t *stopping_point = dst + (packet_len << 2);
    while (dst < stopping_point)
    {

        *(dst++) = upscale_lookup_1Mbps[(((*packet) & 0xc0) >> 6)];
        *(dst++) = upscale_lookup_1Mbps[(((*packet) & 0x30) >> 4)];
        *(dst++) = upscale_lookup_1Mbps[(((*packet) & 0x0c) >> 2)];
        *(dst++) = upscale_lookup_1Mbps[((*packet) & 0x03)];

        packet++;
    }
    return packet_len << 4;
}
*/

uint32_t jv_bsc_upscale_1Mbps(uint32_t *dst, uint8_t *packet, size_t packet_len)
{
    // this function:
    // - runs SPI at 8 MHz
    // - uses 8 samples per BLE bit (1 µs at 1 Mbps)
    // - encodes each BLE bit as a single 8 bit pattern
    //
    // dst still comes in as uint32_t*, but we treat it as a byte buffer.
    uint8_t *out = (uint8_t *)dst;
    uint8_t *p   = packet;
    uint8_t *end = packet + packet_len;

    while (p < end)
    {
        uint8_t byte = *p++;

        // Expand bits b7..b0 into 8 output bytes (MSB first)
        out[0] = upscale_pattern_1Mbps[(byte >> 7) & 0x01];
        out[1] = upscale_pattern_1Mbps[(byte >> 6) & 0x01];
        out[2] = upscale_pattern_1Mbps[(byte >> 5) & 0x01];
        out[3] = upscale_pattern_1Mbps[(byte >> 4) & 0x01];
        out[4] = upscale_pattern_1Mbps[(byte >> 3) & 0x01];
        out[5] = upscale_pattern_1Mbps[(byte >> 2) & 0x01];
        out[6] = upscale_pattern_1Mbps[(byte >> 1) & 0x01];
        out[7] = upscale_pattern_1Mbps[(byte >> 0) & 0x01];

        out += 8;
    }

    // 8 output bytes per input byte
    return (uint32_t)(packet_len * 8u);
}


/*
uint32_t jv_bsc_upscale_2Mbps(uint32_t *dst, uint8_t *packet, size_t packet_len)
{

    //We are writing 1 byte per bit = 8 bytes per byte = 2 uint32s per byte
    //So we stop after writing (2 * packet_len) uint32s

    const uint32_t *stopping_point = dst + (packet_len << 1);
    while (dst < stopping_point)
    {

        *(dst++) = upscale_lookup_2Mbps[(((*packet) & 0xc0) >> 6)] | (upscale_lookup_2Mbps[(((*packet) & 0x30) >> 4)] << 16);
        *(dst++) = upscale_lookup_2Mbps[(((*packet) & 0x0c) >> 2)] | (upscale_lookup_2Mbps[((*packet) & 0x03)] << 16);

        packet++;
    }
    return packet_len << 3;
}
*/


