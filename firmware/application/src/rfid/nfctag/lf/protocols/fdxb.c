#include "fdxb.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"

#define FDXB_T55XX_BLOCK_COUNT (5)
#define FDXB_BITRATE (32)
#define FDXB_HEADER (0x001)

#define FDXB_READ_TIME1 (32)
#define FDXB_READ_TIME2 (48)
#define FDXB_READ_JITTER (8)

#define NRF_LOG_MODULE_NAME fdxb_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static nrf_pwm_values_wave_form_t m_fdxb_pwm_seq_vals[FDXB_RAW_SIZE * 2] = {};

nrf_pwm_sequence_t m_fdxb_pwm_seq = {
    .values.p_wave_form = m_fdxb_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_fdxb_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

static uint16_t crc16_ccitt(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void fdxb_raw_data(uint8_t *uid, uint64_t *hi, uint64_t *lo) {
    uint64_t raw_lo = 0;
    uint64_t raw_hi = 0;
    int bit_count = 0;
    
    uint64_t national = 0;
    uint64_t country = 0;
    uint8_t data_block = 0;
    uint8_t animal = 1;
    
    for (int i = 0; i < 8; i++) {
        national |= ((uint64_t)uid[i]) << (56 - i * 8);
    }
    national &= 0x3FFFFFFFFFFULL;
    
    country = (uid[4] >> 2) | ((uid[5] & 0x03) << 6);
    country &= 0x3FF;
    
    for (int i = 0; i < 11; i++) {
        raw_lo <<= 1;
        if (i == 10) raw_lo |= 1;
        bit_count++;
    }
    
    for (int i = 0; i < 38; i++) {
        if (bit_count >= 64) {
            raw_hi <<= 1;
            raw_hi |= (raw_lo >> 63);
            raw_lo <<= 1;
            raw_lo |= (national >> (37 - i)) & 1;
        } else {
            raw_lo <<= 1;
            raw_lo |= (national >> (37 - i)) & 1;
        }
        bit_count++;
        if ((i + 1) % 8 == 0 && i < 37) {
            if (bit_count >= 64) {
                raw_hi <<= 1;
                raw_hi |= (raw_lo >> 63);
                raw_lo <<= 1;
                raw_lo |= 1;
            } else {
                raw_lo <<= 1;
                raw_lo |= 1;
            }
            bit_count++;
        }
    }
    
    for (int i = 0; i < 10; i++) {
        if (bit_count >= 64) {
            raw_hi <<= 1;
            raw_hi |= (raw_lo >> 63);
            raw_lo <<= 1;
            raw_lo |= (country >> (9 - i)) & 1;
        } else {
            raw_lo <<= 1;
            raw_lo |= (country >> (9 - i)) & 1;
        }
        bit_count++;
    }
    
    if (bit_count >= 64) {
        raw_hi <<= 1;
        raw_hi |= (raw_lo >> 63);
        raw_lo <<= 1;
        raw_lo |= data_block;
    } else {
        raw_lo <<= 1;
        raw_lo |= data_block;
    }
    bit_count++;
    
    if (bit_count >= 64) {
        raw_hi <<= 1;
        raw_hi |= (raw_lo >> 63);
        raw_lo <<= 1;
        raw_lo |= animal;
    } else {
        raw_lo <<= 1;
        raw_lo |= animal;
    }
    bit_count++;
    
    for (int i = 0; i < 16; i++) {
        if (bit_count >= 64) {
            raw_hi <<= 1;
            raw_hi |= (raw_lo >> 63);
            raw_lo <<= 1;
        } else {
            raw_lo <<= 1;
        }
        bit_count++;
    }
    
    *hi = raw_hi;
    *lo = raw_lo;
    
    uint8_t crc_data[10];
    memset(crc_data, 0, 10);
    uint16_t crc = crc16_ccitt(crc_data, 10);
    
    *lo |= ((uint64_t)crc) << 48;
}

static uint8_t fdxb_period(uint8_t interval) {
    if (interval >= FDXB_READ_TIME1 - FDXB_READ_JITTER &&
        interval <= FDXB_READ_TIME1 + FDXB_READ_JITTER) {
        return 0;
    }
    if (interval >= FDXB_READ_TIME2 - FDXB_READ_JITTER &&
        interval <= FDXB_READ_TIME2 + FDXB_READ_JITTER) {
        return 1;
    }
    return 3;
}

static fdxb_codec *fdxb_alloc(void) {
    fdxb_codec *codec = malloc(sizeof(fdxb_codec));
    codec->modem = malloc(sizeof(biphase));
    codec->modem->rp = fdxb_period;
    return codec;
}

static void fdxb_free(fdxb_codec *d) {
    if (d->modem) {
        free(d->modem);
        d->modem = NULL;
    }
    free(d);
}

static uint8_t *fdxb_get_data(fdxb_codec *d) {
    return d->data;
}

static void fdxb_decoder_start(fdxb_codec *d, uint8_t format) {
    memset(d->data, 0, FDXB_DATA_SIZE);
    d->raw_hi = 0;
    d->raw_lo = 0;
    d->raw_length = 0;
    biphase_reset(d->modem);
}

static bool fdxb_decode_feed(fdxb_codec *d, bool bit) {
    if (d->raw_length < 64) {
        d->raw_hi <<= 1;
        d->raw_hi |= bit;
    } else {
        d->raw_lo <<= 1;
        d->raw_lo |= bit;
    }
    d->raw_length++;
    
    if (d->raw_length < FDXB_RAW_SIZE) {
        return false;
    }
    
    uint64_t header = (d->raw_hi >> 53) & 0x7FF;
    if (header != FDXB_HEADER) {
        return false;
    }
    
    for (int i = 0; i < FDXB_DATA_SIZE; i++) {
        d->data[i] = (d->raw_lo >> (120 - i * 8)) & 0xFF;
    }
    
    return true;
}

static bool fdxb_decoder_feed(fdxb_codec *d, uint16_t interval) {
    bool bits[2] = {0};
    int8_t bitlen = 0;
    
    biphase_feed(d->modem, (uint8_t)interval, bits, &bitlen);
    if (bitlen == -1) {
        d->raw_hi = 0;
        d->raw_lo = 0;
        d->raw_length = 0;
        return false;
    }
    
    for (int i = 0; i < bitlen; i++) {
        if (fdxb_decode_feed(d, bits[i])) {
            return true;
        }
    }
    return false;
}

static const nrf_pwm_sequence_t *fdxb_modulator(fdxb_codec *d, uint8_t *buf) {
    uint64_t hi, lo;
    fdxb_raw_data(buf, &hi, &lo);
    
    for (int i = 0; i < 64; i++) {
        uint16_t bit_val = (hi >> (63 - i)) & 1;
        m_fdxb_pwm_seq_vals[i * 2].channel_0 = bit_val ? 16 : 0;
        m_fdxb_pwm_seq_vals[i * 2].counter_top = 32;
        m_fdxb_pwm_seq_vals[i * 2 + 1].channel_0 = bit_val ? 0 : 16;
        m_fdxb_pwm_seq_vals[i * 2 + 1].counter_top = 32;
    }
    
    for (int i = 0; i < 64; i++) {
        uint16_t bit_val = (lo >> (63 - i)) & 1;
        m_fdxb_pwm_seq_vals[(64 + i) * 2].channel_0 = bit_val ? 16 : 0;
        m_fdxb_pwm_seq_vals[(64 + i) * 2].counter_top = 32;
        m_fdxb_pwm_seq_vals[(64 + i) * 2 + 1].channel_0 = bit_val ? 0 : 16;
        m_fdxb_pwm_seq_vals[(64 + i) * 2 + 1].counter_top = 32;
    }
    
    m_fdxb_pwm_seq.length = FDXB_RAW_SIZE * 2 * 4;
    return &m_fdxb_pwm_seq;
}

const protocol fdxb = {
    .tag_type = TAG_TYPE_FDXB,
    .data_size = FDXB_DATA_SIZE,
    .alloc = (codec_alloc)fdxb_alloc,
    .free = (codec_free)fdxb_free,
    .get_data = (codec_get_data)fdxb_get_data,
    .modulator = (modulator)fdxb_modulator,
    .decoder = {
        .start = (decoder_start)fdxb_decoder_start,
        .feed = (decoder_feed)fdxb_decoder_feed,
    },
};

uint8_t fdxb_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    uint64_t hi, lo;
    fdxb_raw_data(uid, &hi, &lo);
    
    blks[0] = T5577_FDXB_CONFIG;
    blks[1] = (hi >> 32) & 0xFFFFFFFF;
    blks[2] = hi & 0xFFFFFFFF;
    blks[3] = (lo >> 32) & 0xFFFFFFFF;
    blks[4] = lo & 0xFFFFFFFF;
    
    return FDXB_T55XX_BLOCK_COUNT;
}
