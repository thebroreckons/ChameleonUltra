#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint8_t (*biphase_period)(uint8_t interval);

typedef struct {
    bool sync;           // Whether we're in sync state
    bool last_level;     // Last signal level (for edge detection)
    biphase_period rp;   // Period classification function
} biphase;

extern void biphase_reset(biphase *m);
extern void biphase_feed(biphase *m, uint8_t interval, bool *bits, int8_t *bitlen);
