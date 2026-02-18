#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default bitrate for HID Prox (RF/50)
#define FSK_DEFAULT_BITRATE (50)

typedef struct {
    uint8_t c;
    uint8_t bitrate;            // Configurable bitrate
    uint16_t *samples;          // Dynamic sample buffer
    float goertzel_fc_8;        // Goertzel coefficient for fc/8
    float goertzel_fc_10;       // Goertzel coefficient for fc/10
} fsk_t;

extern bool fsk_feed(fsk_t *m, uint16_t sample, bool *bit);
extern fsk_t *fsk_alloc(void);
extern fsk_t *fsk_alloc_with_bitrate(uint8_t bitrate);
extern void fsk_free(fsk_t *m);

#ifdef __cplusplus
}
#endif
