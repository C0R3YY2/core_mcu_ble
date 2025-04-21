#include "crc.h"

//////////////////////////////////
// CRC   Generated on Thu Mar 24 14:57:47 2022 by pycrc v0.9.2, https://pycrc.org

// python3 pycrc.py --algorithm table-driven --table-idx-width 4 --width 24 --poly 0x00065b --reflect-in False --xor-in 0x555555 --reflect-out False --xor-out 0x000000 --generate h -o crc.h
// python3 pycrc.py --algorithm table-driven --table-idx-width 4 --width 24 --poly 0x00065b --reflect-in False --xor-in 0x555555 --reflect-out False --xor-out 0x000000 --generate c -o crc.c

//////////////////////////////////
static const crc_t crc_table[16] = {
    0x000000, 0x00065b, 0x000cb6, 0x000aed, 0x00196c, 0x001f37, 0x0015da, 0x001381,
    0x0032d8, 0x003483, 0x003e6e, 0x003835, 0x002bb4, 0x002def, 0x002702, 0x002159};

crc_t crc_update(crc_t crc, const uint8_t *data, size_t data_len)
{
    //crc = 0x555555
    
    uint32_t tbl_idx;

    while (data_len--)
    {
        tbl_idx = (crc >> 20) ^ (*data >> 4);
        crc = crc_table[tbl_idx & 0x0f] ^ (crc << 4);
        tbl_idx = (crc >> 20) ^ (*data);
        crc = crc_table[tbl_idx & 0x0f] ^ (crc << 4);
        data++;
    }
    return crc & 0xffffff;
}
