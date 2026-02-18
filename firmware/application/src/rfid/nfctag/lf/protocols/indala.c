#include "indala.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"

#define INDALA_64_T55XX_BLOCK_COUNT (3)
#define INDALA_224_T55XX_BLOCK_COUNT (8)
#define INDALA_BITRATE (32)

#define INDALA_PREAMBLE_BIT_0 (1)
#define INDALA_PREAMBLE_BIT_2 (1)
#define INDALA_PREAMBLE_BIT_32 (1)

#define NRF_LOG_MODULE_NAME indala_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static nrf_pwm_values_wave_form_t m_indala_64_pwm_seq_vals[INDALA_64_RAW_SIZE] = {};
static nrf_pwm_values_wave_form_t m_indala_224_pwm_seq_vals[INDALA_224_RAW_SIZE] = {};

nrf_pwm_sequence_t m_indala_64_pwm_seq = {
    .values.p_wave_form = m_indala_64_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_indala_64_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

nrf_pwm_sequence_t m_indala_224_pwm_seq = {
    .values.p_wave_form = m_indala_224_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_indala_224_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

static const uint8_t indala_fc_map[] = {57, 49, 44, 47, 48, 53, 39, 58};
static const uint8_t indala_cn_map[] = {42, 45, 43, 40, 52, 36, 35, 51, 46, 33, 37, 54, 56, 59, 50, 41};

static uint64_t indala_64_raw_data(uint8_t *uid) {
    uint8_t bits[64] = {0};
    uint8_t fc = uid[0];
    uint16_t cn = (uid[1] << 8) | uid[2];
    
    bits[0] = 1;
    bits[2] = 1;
    bits[32] = 1;
    
    for (int i = 0; i < 8; i++) {
        bits[indala_fc_map[i]] = (fc >> (7 - i)) & 1;
    }
    
    for (int i = 0; i < 16; i++) {
        bits[indala_cn_map[i]] = (cn >> (15 - i)) & 1;
    }
    
    uint8_t p1 = 1, p2 = 1;
    for (int i = 33; i < 64; i++) {
        if (i % 2) p1 ^= bits[i];
        else p2 ^= bits[i];
    }
    bits[34] = p1;
    bits[38] = p2;
    
    uint8_t chk = 0;
    chk += ((cn >> 14) & 1);
    chk += ((cn >> 12) & 1);
    chk += ((cn >> 9) & 1);
    chk += ((cn >> 8) & 1);
    chk += ((cn >> 6) & 1);
    chk += ((cn >> 5) & 1);
    chk += ((cn >> 2) & 1);
    chk += (cn & 1);
    
    if ((chk & 1) == 0) {
        bits[62] = 1;
        bits[63] = 0;
    } else {
        bits[62] = 0;
        bits[63] = 1;
    }
    
    uint64_t raw = 0;
    for (int i = 0; i < 64; i++) {
        raw <<= 1;
        raw |= bits[i];
    }
    
    return raw;
}

static indala_codec *indala_alloc(void) {
    indala_codec *codec = malloc(sizeof(indala_codec));
    codec->modem = psk_alloc_with_bitrate(INDALA_BITRATE);
    codec->is_224 = false;
    return codec;
}

static void indala_free(indala_codec *d) {
    if (d->modem) {
        psk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
}

static uint8_t *indala_get_data(indala_codec *d) {
    return d->data;
}

static void indala_decoder_start(indala_codec *d, uint8_t format) {
    memset(d->data, 0, INDALA_224_DATA_SIZE);
    d->raw_hi = 0;
    d->raw_lo = 0;
    d->raw_ex = 0;
    d->raw_length = 0;
    d->is_224 = (format == 1);
}

static bool indala_decode_feed(indala_codec *d, bool bit) {
    d->raw_hi <<= 1;
    d->raw_hi |= bit;
    d->raw_length++;
    
    if (d->raw_length == 64 && !d->is_224) {
        if ((d->raw_hi & 0xC000000000000001ULL) == 0x4000000000000001ULL) {
            uint8_t fc = 0;
            for (int i = 0; i < 8; i++) {
                fc |= (((d->raw_hi >> (63 - indala_fc_map[i])) & 1) << (7 - i));
            }
            
            uint16_t cn = 0;
            for (int i = 0; i < 16; i++) {
                cn |= (((d->raw_hi >> (63 - indala_cn_map[i])) & 1) << (15 - i));
            }
            
            d->data[0] = fc;
            d->data[1] = (cn >> 8) & 0xFF;
            d->data[2] = cn & 0xFF;
            
            return true;
        }
    }
    
    return false;
}

static bool indala_decoder_feed(indala_codec *d, uint16_t val) {
    bool bit = false;
    if (!psk_feed(d->modem, val, &bit)) {
        return false;
    }
    return indala_decode_feed(d, bit);
}

static const nrf_pwm_sequence_t *indala_64_modulator(indala_codec *d, uint8_t *buf) {
    uint64_t raw = indala_64_raw_data(buf);
    
    for (int i = 0; i < INDALA_64_RAW_SIZE; i++) {
        uint16_t phase = (raw >> (INDALA_64_RAW_SIZE - i - 1)) & 1;
        m_indala_64_pwm_seq_vals[i].channel_0 = phase ? 0x8010 : 0x0010;
        m_indala_64_pwm_seq_vals[i].counter_top = 32;
    }
    
    return &m_indala_64_pwm_seq;
}

static const nrf_pwm_sequence_t *indala_224_modulator(indala_codec *d, uint8_t *buf) {
    for (int i = 0; i < INDALA_224_DATA_SIZE; i++) {
        d->data[i] = buf[i];
    }
    
    for (int i = 0; i < INDALA_224_RAW_SIZE; i++) {
        uint16_t phase = 0;
        if (i < 64) {
            phase = (d->raw_hi >> (63 - i)) & 1;
        } else if (i < 128) {
            phase = (d->raw_lo >> (127 - i)) & 1;
        } else {
            phase = (d->raw_ex >> (223 - i)) & 1;
        }
        m_indala_224_pwm_seq_vals[i].channel_0 = phase ? 0x8010 : 0x0010;
        m_indala_224_pwm_seq_vals[i].counter_top = 32;
    }
    
    return &m_indala_224_pwm_seq;
}

const protocol indala_64 = {
    .tag_type = TAG_TYPE_INDALA,
    .data_size = INDALA_64_DATA_SIZE,
    .alloc = (codec_alloc)indala_alloc,
    .free = (codec_free)indala_free,
    .get_data = (codec_get_data)indala_get_data,
    .modulator = (modulator)indala_64_modulator,
    .decoder = {
        .start = (decoder_start)indala_decoder_start,
        .feed = (decoder_feed)indala_decoder_feed,
    },
};

const protocol indala_224 = {
    .tag_type = TAG_TYPE_INDALA_224,
    .data_size = INDALA_224_DATA_SIZE,
    .alloc = (codec_alloc)indala_alloc,
    .free = (codec_free)indala_free,
    .get_data = (codec_get_data)indala_get_data,
    .modulator = (modulator)indala_224_modulator,
    .decoder = {
        .start = (decoder_start)indala_decoder_start,
        .feed = (decoder_feed)indala_decoder_feed,
    },
};

uint8_t indala_64_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    uint64_t raw = indala_64_raw_data(uid);
    blks[0] = T5577_INDALA_64_CONFIG;
    blks[1] = (raw >> 32) & 0xFFFFFFFF;
    blks[2] = raw & 0xFFFFFFFF;
    return INDALA_64_T55XX_BLOCK_COUNT;
}

uint8_t indala_224_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    blks[0] = T5577_INDALA_224_CONFIG;
    for (int i = 0; i < 7; i++) {
        uint32_t word = 0;
        for (int j = 0; j < 4; j++) {
            word <<= 8;
            word |= uid[i * 4 + j];
        }
        blks[i + 1] = word;
    }
    return INDALA_224_T55XX_BLOCK_COUNT;
}
