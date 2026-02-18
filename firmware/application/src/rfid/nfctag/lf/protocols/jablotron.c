#include "jablotron.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"

#define JABLOTRON_T55XX_BLOCK_COUNT (3)
#define JABLOTRON_BITRATE (64)

#define JABLOTRON_PREAMBLE (0xFFFF)

#define JABLOTRON_READ_TIME1 (32)
#define JABLOTRON_READ_TIME2 (48)
#define JABLOTRON_READ_JITTER (8)

#define NRF_LOG_MODULE_NAME jablotron_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static nrf_pwm_values_wave_form_t m_jablotron_pwm_seq_vals[JABLOTRON_RAW_SIZE * 2] = {};

nrf_pwm_sequence_t m_jablotron_pwm_seq = {
    .values.p_wave_form = m_jablotron_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_jablotron_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

static uint64_t jablotron_raw_data(uint8_t *uid) {
    uint64_t raw = JABLOTRON_PREAMBLE;
    uint64_t card_id = 0;
    
    for (int i = 0; i < 5; i++) {
        card_id <<= 8;
        card_id |= uid[i];
    }
    card_id &= 0xFFFFFFFFFFULL;
    
    for (int i = 39; i >= 0; i--) {
        raw <<= 1;
        raw |= (card_id >> i) & 1;
    }
    
    uint8_t checksum = 0;
    for (int i = 0; i < 5; i++) {
        checksum ^= uid[i];
    }
    checksum ^= 0x3A;
    
    for (int i = 7; i >= 0; i--) {
        raw <<= 1;
        raw |= (checksum >> i) & 1;
    }
    
    return raw;
}

static uint8_t jablotron_period(uint8_t interval) {
    if (interval >= JABLOTRON_READ_TIME1 - JABLOTRON_READ_JITTER &&
        interval <= JABLOTRON_READ_TIME1 + JABLOTRON_READ_JITTER) {
        return 0;
    }
    if (interval >= JABLOTRON_READ_TIME2 - JABLOTRON_READ_JITTER &&
        interval <= JABLOTRON_READ_TIME2 + JABLOTRON_READ_JITTER) {
        return 1;
    }
    return 3;
}

static jablotron_codec *jablotron_alloc(void) {
    jablotron_codec *codec = malloc(sizeof(jablotron_codec));
    codec->modem = malloc(sizeof(biphase));
    codec->modem->rp = jablotron_period;
    return codec;
}

static void jablotron_free(jablotron_codec *d) {
    if (d->modem) {
        free(d->modem);
        d->modem = NULL;
    }
    free(d);
}

static uint8_t *jablotron_get_data(jablotron_codec *d) {
    return d->data;
}

static void jablotron_decoder_start(jablotron_codec *d, uint8_t format) {
    memset(d->data, 0, JABLOTRON_DATA_SIZE);
    d->raw = 0;
    d->raw_length = 0;
    d->preamble_found = false;
    biphase_reset(d->modem);
}

static bool jablotron_decode_feed(jablotron_codec *d, bool bit) {
    d->raw <<= 1;
    d->raw_length++;
    if (bit) {
        d->raw |= 1;
    }
    
    if (d->raw_length < JABLOTRON_RAW_SIZE) {
        return false;
    }
    
    uint16_t preamble = (d->raw >> 48) & 0xFFFF;
    if (preamble != JABLOTRON_PREAMBLE) {
        return false;
    }
    
    uint64_t card_id = (d->raw >> 8) & 0xFFFFFFFFFFULL;
    uint8_t checksum = d->raw & 0xFF;
    
    uint8_t calc_checksum = 0;
    for (int i = 0; i < 5; i++) {
        calc_checksum ^= (card_id >> (32 - i * 8)) & 0xFF;
    }
    calc_checksum ^= 0x3A;
    
    if (checksum != calc_checksum) {
        return false;
    }
    
    for (int i = 0; i < 5; i++) {
        d->data[i] = (card_id >> (32 - i * 8)) & 0xFF;
    }
    
    return true;
}

static bool jablotron_decoder_feed(jablotron_codec *d, uint16_t interval) {
    bool bits[2] = {0};
    int8_t bitlen = 0;
    
    biphase_feed(d->modem, (uint8_t)interval, bits, &bitlen);
    if (bitlen == -1) {
        d->raw = 0;
        d->raw_length = 0;
        return false;
    }
    
    for (int i = 0; i < bitlen; i++) {
        if (jablotron_decode_feed(d, bits[i])) {
            return true;
        }
    }
    return false;
}

static const nrf_pwm_sequence_t *jablotron_modulator(jablotron_codec *d, uint8_t *buf) {
    uint64_t raw = jablotron_raw_data(buf);
    
    for (int i = 0; i < JABLOTRON_RAW_SIZE; i++) {
        uint16_t bit_val = (raw >> (JABLOTRON_RAW_SIZE - i - 1)) & 1;
        
        m_jablotron_pwm_seq_vals[i * 2].channel_0 = bit_val ? 16 : 0;
        m_jablotron_pwm_seq_vals[i * 2].counter_top = 32;
        m_jablotron_pwm_seq_vals[i * 2 + 1].channel_0 = bit_val ? 16 : 0;
        m_jablotron_pwm_seq_vals[i * 2 + 1].counter_top = 32;
    }
    
    m_jablotron_pwm_seq.length = JABLOTRON_RAW_SIZE * 2 * 4;
    return &m_jablotron_pwm_seq;
}

const protocol jablotron = {
    .tag_type = TAG_TYPE_JABLOTRON,
    .data_size = JABLOTRON_DATA_SIZE,
    .alloc = (codec_alloc)jablotron_alloc,
    .free = (codec_free)jablotron_free,
    .get_data = (codec_get_data)jablotron_get_data,
    .modulator = (modulator)jablotron_modulator,
    .decoder = {
        .start = (decoder_start)jablotron_decoder_start,
        .feed = (decoder_feed)jablotron_decoder_feed,
    },
};

uint8_t jablotron_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    uint64_t raw = jablotron_raw_data(uid);
    blks[0] = T5577_JABLOTRON_CONFIG;
    blks[1] = raw >> 32;
    blks[2] = raw & 0xFFFFFFFF;
    return JABLOTRON_T55XX_BLOCK_COUNT;
}
