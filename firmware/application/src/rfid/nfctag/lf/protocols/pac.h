#pragma once

#include "protocols.h"

#define PAC_DATA_SIZE (8)
#define PAC_RAW_SIZE (128)

typedef struct {
    uint8_t data[PAC_DATA_SIZE];
    uint8_t raw[PAC_RAW_SIZE / 8];
    uint8_t raw_length;
} pac_codec;

extern const protocol pac;

uint8_t pac_t55xx_writer(uint8_t *uid, uint32_t *blks);
