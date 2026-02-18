#pragma once

#include "protocols.h"
#include "utils/fskdemod.h"

#define PARADOX_DATA_SIZE (6)
#define PARADOX_RAW_SIZE (96)

typedef enum {
    PARADOX_STATE_PREAMBLE,
    PARADOX_STATE_DATA,
    PARADOX_STATE_DONE,
} paradox_codec_state_t;

typedef struct {
    uint8_t data[PARADOX_DATA_SIZE];
    uint64_t raw;
    uint8_t raw_length;
    fsk_t *modem;
    paradox_codec_state_t state;
} paradox_codec;

extern const protocol paradox;

uint8_t paradox_t55xx_writer(uint8_t *uid, uint32_t *blks);
