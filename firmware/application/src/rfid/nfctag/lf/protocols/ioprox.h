#pragma once

#include "protocols.h"
#include "utils/fskdemod.h"

#define IOPROX_DATA_SIZE (8)
#define IOPROX_RAW_SIZE (64)

typedef enum {
    IOPROX_STATE_PREAMBLE,
    IOPROX_STATE_DATA,
    IOPROX_STATE_DONE,
} ioprox_codec_state_t;

typedef struct {
    uint8_t data[IOPROX_DATA_SIZE];
    
    uint64_t raw;
    uint8_t raw_length;
    uint8_t preamble_count;
    
    fsk_t *modem;
    ioprox_codec_state_t state;
} ioprox_codec;

extern const protocol ioprox;

uint8_t ioprox_t55xx_writer(uint8_t *uid, uint32_t *blks);
