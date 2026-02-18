#include "biphase.h"

#include <stdlib.h>
#include <string.h>

void biphase_reset(biphase *m) {
    m->sync = true;
    m->last_level = false;
}

void biphase_feed(biphase *m, uint8_t interval, bool *bits, int8_t *bitlen) {
    // Get the period classification
    // 0 = 0.5T (short), 1 = T (normal), 2 = 1.5T (long), 3 = invalid
    uint8_t t = m->rp(interval);
    *bitlen = -1;  // Default: no bit output
    
    if (t == 3) {
        return;  // Invalid period
    }
    
    // Differential Biphase encoding:
    // - Always a transition at bit boundary
    // - Mid-bit transition = '1'
    // - No mid-bit transition = '0'
    
    if (m->sync) {
        // In sync state, looking for complete bit periods
        if (t == 0) {
            // 0.5T - This is a half-period, means we have a mid-bit transition
            // Output '1', stay in sync (next should be another 0.5T)
            *bitlen = 1;
            bits[0] = 1;
        } else if (t == 1) {
            // 1T - Full period with no mid-bit transition
            // Output '0', stay in sync
            *bitlen = 1;
            bits[0] = 0;
        } else if (t == 2) {
            // 1.5T - Could be: 0.5T + 1T (bit '1' then '0') or sync drift
            // Output '1', then next interval should complete
            *bitlen = 1;
            bits[0] = 1;
            m->sync = false;  // Now expecting 1T remainder
        }
    } else {
        // Non-sync state: we're in the middle of a bit period
        if (t == 0) {
            // 0.5T - Completing a partial bit, switch back to sync
            m->sync = true;
        } else if (t == 1) {
            // 1T - This completes one bit and is a full period for '0'
            *bitlen = 1;
            bits[0] = 0;
            m->sync = true;
        } else {
            // Invalid in non-sync state, reset
            m->sync = true;
        }
    }
}
