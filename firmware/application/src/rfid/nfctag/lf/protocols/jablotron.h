#pragma once

#include "protocols.h"
#include "utils/biphase.h"

#define JABLOTRON_DATA_SIZE (5)
#define JABLOTRON_RAW_SIZE (64)

typedef struct {
    uint8_t data[JABLOTRON_DATA_SIZE];
    uint64_t raw;
    uint8_t raw_length;
    biphase *modem;
    uint16_t last_interval;
    bool preamble_found;
} jablotron_codec;

extern const protocol jablotron;

uint8_t jablotron_t55xx_writer(uint8_t *uid, uint32_t *blks);
