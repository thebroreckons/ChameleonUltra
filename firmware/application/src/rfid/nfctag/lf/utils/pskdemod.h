#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default bitrate for PSK demodulation
#define PSK_DEFAULT_BITRATE (32)

typedef struct {
    uint8_t c;                              // Sample counter
    uint8_t bitrate;                        // Samples per bit
    uint16_t *samples;                      // Sample buffer (dynamic)
    int16_t prev_avg;                       // Previous bit period average
    bool prev_bit;                          // Previous decoded bit
    bool initialized;                       // Whether we have enough samples
} psk_t;

extern psk_t *psk_alloc(void);
extern psk_t *psk_alloc_with_bitrate(uint8_t bitrate);
extern void psk_free(psk_t *m);
extern bool psk_feed(psk_t *m, uint16_t sample, bool *bit);

#ifdef __cplusplus
}
#endif
