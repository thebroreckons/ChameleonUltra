#include "pskdemod.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

psk_t *psk_alloc(void) {
    return psk_alloc_with_bitrate(PSK_DEFAULT_BITRATE);
}

psk_t *psk_alloc_with_bitrate(uint8_t bitrate) {
    psk_t *m = (psk_t *)malloc(sizeof(psk_t));
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
    m->prev_avg = 0;
    m->prev_bit = false;
    m->initialized = false;
    return m;
}

void psk_free(psk_t *m) {
    if (m != NULL) {
        if (m->samples != NULL) {
            free(m->samples);
        }
        free(m);
    }
}

bool psk_feed(psk_t *m, uint16_t sample, bool *bit) {
    m->samples[m->c++] = sample;
    
    if (m->c < m->bitrate) {
        return false;  // Not enough samples yet
    }
    
    // Calculate average of current bit period
    int32_t sum = 0;
    for (int i = 0; i < m->bitrate; i++) {
        sum += (int16_t)m->samples[i];
    }
    int16_t curr_avg = (int16_t)(sum / m->bitrate);
    
    // Shift samples for next period
    memmove(m->samples, m->samples + m->bitrate / 2, (m->bitrate) * sizeof(uint16_t));
    m->c = m->bitrate / 2;
    
    if (!m->initialized) {
        m->prev_avg = curr_avg;
        m->initialized = true;
        return false;  // Need another period to compare
    }
    
    // PSK1: Detect phase inversion by comparing sign of correlation
    // If the signal inverted, the average will have opposite sign relative to trend
    // A phase inversion = bit '1', no inversion = bit '0'
    
    // Calculate correlation between current and previous
    int32_t correlation = (int32_t)curr_avg * (int32_t)m->prev_avg;
    
    bool phase_changed = (correlation < 0);
    
    // In PSK1, a phase change indicates a '1'
    *bit = phase_changed;
    
    m->prev_avg = curr_avg;
    
    return true;
}
