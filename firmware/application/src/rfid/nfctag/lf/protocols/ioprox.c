#include "ioprox.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"

#define IOPROX_T55XX_BLOCK_COUNT (3)
#define IOPROX_BITRATE (64)

#define IOPROX_FSK_LO_FREQ_LOOP (5)
#define IOPROX_FSK_LO_FREQ_TOP (8)
#define IOPROX_FSK_HI_FREQ_LOOP (6)
#define IOPROX_FSK_HI_FREQ_TOP (10)

#define NRF_LOG_MODULE_NAME ioprox_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static nrf_pwm_values_wave_form_t m_ioprox_pwm_seq_vals[IOPROX_RAW_SIZE * 6] = {};

nrf_pwm_sequence_t m_ioprox_pwm_seq = {
    .values.p_wave_form = m_ioprox_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_ioprox_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

static uint64_t ioprox_raw_data(uint8_t *uid) {
    uint64_t raw = 0;
    uint8_t facility = uid[0];
    uint8_t version = uid[1];
    uint16_t cardnum = (uid[2] << 8) | uid[3];
    
    raw = 0;
    
    for (int i = 0; i < 9; i++) {
        raw <<= 1;
    }
    
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= (0xF0 >> i) & 1;
    }
    raw <<= 1;
    raw |= 1;
    
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= (facility >> i) & 1;
    }
    raw <<= 1;
    raw |= 1;
    
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= (version >> i) & 1;
    }
    raw <<= 1;
    raw |= 1;
    
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= ((cardnum >> 8) >> i) & 1;
    }
    raw <<= 1;
    raw |= 1;
    
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= (cardnum >> i) & 1;
    }
    raw <<= 1;
    raw |= 1;
    
    uint8_t checksum = 0xFF - (0xF0 + facility + version + (cardnum >> 8) + (cardnum & 0xFF));
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= (checksum >> i) & 1;
    }
    
    raw <<= 2;
    raw |= 0x03;
    
    return raw;
}

static ioprox_codec *ioprox_alloc(void) {
    ioprox_codec *codec = malloc(sizeof(ioprox_codec));
    codec->modem = fsk_alloc_with_bitrate(IOPROX_BITRATE);
    codec->state = IOPROX_STATE_PREAMBLE;
    return codec;
}

static void ioprox_free(ioprox_codec *d) {
    if (d->modem) {
        fsk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
}

static uint8_t *ioprox_get_data(ioprox_codec *d) {
    return d->data;
}

static void ioprox_decoder_start(ioprox_codec *d, uint8_t format) {
    memset(d->data, 0, IOPROX_DATA_SIZE);
    d->raw = 0;
    d->raw_length = 0;
    d->preamble_count = 0;
    d->state = IOPROX_STATE_PREAMBLE;
}

static bool ioprox_decode_feed(ioprox_codec *d, bool bit) {
    if (d->state == IOPROX_STATE_PREAMBLE) {
        if (!bit) {
            d->preamble_count++;
        } else {
            if (d->preamble_count >= 8) {
                d->state = IOPROX_STATE_DATA;
                d->raw = 0;
                d->raw_length = 0;
            } else {
                d->preamble_count = 0;
            }
        }
        return false;
    }
    
    if (d->state == IOPROX_STATE_DATA) {
        d->raw <<= 1;
        d->raw_length++;
        if (bit) {
            d->raw |= 1;
        }
        
        if (d->raw_length >= 55) {
            uint8_t fixed = (d->raw >> 47) & 0xFF;
            if (fixed != 0xF0) {
                d->state = IOPROX_STATE_PREAMBLE;
                d->preamble_count = 0;
                return false;
            }
            
            uint8_t facility = (d->raw >> 38) & 0xFF;
            uint8_t version = (d->raw >> 29) & 0xFF;
            uint8_t card_hi = (d->raw >> 20) & 0xFF;
            uint8_t card_lo = (d->raw >> 11) & 0xFF;
            uint8_t checksum = d->raw & 0xFF;
            
            uint8_t calc_checksum = 0xFF - (0xF0 + facility + version + card_hi + card_lo);
            
            if (checksum == calc_checksum) {
                d->data[0] = facility;
                d->data[1] = version;
                d->data[2] = card_hi;
                d->data[3] = card_lo;
                d->data[4] = checksum;
                d->state = IOPROX_STATE_DONE;
                return true;
            }
            
            d->state = IOPROX_STATE_PREAMBLE;
            d->preamble_count = 0;
        }
        
        return false;
    }
    
    return false;
}

static bool ioprox_decoder_feed(ioprox_codec *d, uint16_t val) {
    bool bit = false;
    if (!fsk_feed(d->modem, val, &bit)) {
        return false;
    }
    return ioprox_decode_feed(d, bit);
}

static const nrf_pwm_sequence_t *ioprox_modulator(ioprox_codec *d, uint8_t *buf) {
    uint64_t raw = ioprox_raw_data(buf);
    int k = 0;
    
    for (int i = 0; i < IOPROX_RAW_SIZE; i++) {
        bool bit = (raw >> (IOPROX_RAW_SIZE - i - 1)) & 1;
        
        if (!bit) {
            for (int j = 0; j < IOPROX_FSK_HI_FREQ_LOOP; j++) {
                m_ioprox_pwm_seq_vals[k].channel_0 = IOPROX_FSK_HI_FREQ_TOP / 2;
                m_ioprox_pwm_seq_vals[k].counter_top = IOPROX_FSK_HI_FREQ_TOP;
                k++;
            }
        } else {
            for (int j = 0; j < IOPROX_FSK_LO_FREQ_LOOP; j++) {
                m_ioprox_pwm_seq_vals[k].channel_0 = IOPROX_FSK_LO_FREQ_TOP / 2;
                m_ioprox_pwm_seq_vals[k].counter_top = IOPROX_FSK_LO_FREQ_TOP;
                k++;
            }
        }
    }
    
    m_ioprox_pwm_seq.length = k * 4;
    return &m_ioprox_pwm_seq;
}

const protocol ioprox = {
    .tag_type = TAG_TYPE_IOPROX,
    .data_size = IOPROX_DATA_SIZE,
    .alloc = (codec_alloc)ioprox_alloc,
    .free = (codec_free)ioprox_free,
    .get_data = (codec_get_data)ioprox_get_data,
    .modulator = (modulator)ioprox_modulator,
    .decoder = {
        .start = (decoder_start)ioprox_decoder_start,
        .feed = (decoder_feed)ioprox_decoder_feed,
    },
};

uint8_t ioprox_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    uint64_t raw = ioprox_raw_data(uid);
    blks[0] = T5577_IOPROX_CONFIG;
    blks[1] = raw >> 32;
    blks[2] = raw & 0xFFFFFFFF;
    return IOPROX_T55XX_BLOCK_COUNT;
}
