#pragma once

#include "protocols.h"
#include "utils/biphase.h"

#define FDXB_DATA_SIZE (16)
#define FDXB_RAW_SIZE (128)

typedef struct {
    uint8_t data[FDXB_DATA_SIZE];
    uint64_t raw_hi;
    uint64_t raw_lo;
    uint8_t raw_length;
    biphase *modem;
} fdxb_codec;

extern const protocol fdxb;

uint8_t fdxb_t55xx_writer(uint8_t *uid, uint32_t *blks);
