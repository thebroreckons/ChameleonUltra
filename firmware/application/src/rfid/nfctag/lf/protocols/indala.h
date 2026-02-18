#pragma once

#include "protocols.h"
#include "utils/pskdemod.h"

#define INDALA_64_DATA_SIZE (8)
#define INDALA_224_DATA_SIZE (28)
#define INDALA_64_RAW_SIZE (64)
#define INDALA_224_RAW_SIZE (224)

typedef struct {
    uint8_t data[INDALA_224_DATA_SIZE];
    uint64_t raw_hi;
    uint64_t raw_lo;
    uint64_t raw_ex;
    uint8_t raw_length;
    psk_t *modem;
    bool is_224;
} indala_codec;

extern const protocol indala_64;
extern const protocol indala_224;

uint8_t indala_64_t55xx_writer(uint8_t *uid, uint32_t *blks);
uint8_t indala_224_t55xx_writer(uint8_t *uid, uint32_t *blks);
