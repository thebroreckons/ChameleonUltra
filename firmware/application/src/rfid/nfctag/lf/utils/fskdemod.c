#include "fskdemod.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "math.h"

#define PI 3.14159265358979f
#define GOERTZEL(FREQ, SAMPLE_RATE) (2.0 * cos((2.0 * PI * FREQ) / (SAMPLE_RATE)))

float goertzel_mag(float coef, uint16_t samples[], int n) {
    float z1 = 0;
    float z2 = 0;
    for (int i = 0; i < n; i++) {
        float z0 = coef * z1 - z2 + (float)(samples[i]);
        z2 = z1;
        z1 = z0;
    }
    return sqrt(z1 * z1 + z2 * z2 - coef * z1 * z2);
}

void fsk_free(fsk_t *m) {
    if (m != NULL) {
        if (m->samples != NULL) {
            free(m->samples);
        }
        free(m);
    }
}

bool fsk_feed(fsk_t *m, uint16_t sample, bool *bit) {
    m->samples[m->c++] = sample;
    if (m->c < m->bitrate) {
        return false;
    }
    float bit0 = goertzel_mag(m->goertzel_fc_8, m->samples, m->bitrate);
    float bit1 = goertzel_mag(m->goertzel_fc_10, m->samples, m->bitrate);
    *bit = bit1 > bit0;
    m->c = 0;
    return true;
}

fsk_t *fsk_alloc(void) {
    return fsk_alloc_with_bitrate(FSK_DEFAULT_BITRATE);
}

fsk_t *fsk_alloc_with_bitrate(uint8_t bitrate) {
    fsk_t *m = (fsk_t *)malloc(sizeof(fsk_t));
    if (m == NULL) {
        return NULL;
    }
    
    m->bitrate = bitrate;
    m->samples = (uint16_t *)malloc(bitrate * 2 * sizeof(uint16_t));
    if (m->samples == NULL) {
        free(m);
        return NULL;
    }
    
    m->c = 0;
    // Goertzel coefficients for FSK2a frequencies
    // fc_8 = 125kHz / 8 = 15.625 kHz (bit 0)
    // fc_10 = 125kHz / 10 = 12.5 kHz (bit 1)
    m->goertzel_fc_8 = GOERTZEL(15625.0f, 125000.0f);
    m->goertzel_fc_10 = GOERTZEL(12500.0f, 125000.0f);
    
    return m;
}
