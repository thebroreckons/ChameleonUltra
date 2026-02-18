#include "paradox.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"

#define PARADOX_T55XX_BLOCK_COUNT (4)
#define PARADOX_BITRATE (50)
#define PARADOX_PREAMBLE (0x0F)

#define PARADOX_FSK_LO_FREQ_LOOP (5)
#define PARADOX_FSK_LO_FREQ_TOP (8)
#define PARADOX_FSK_HI_FREQ_LOOP (6)
#define PARADOX_FSK_HI_FREQ_TOP (10)

#define NRF_LOG_MODULE_NAME paradox_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static nrf_pwm_values_wave_form_t m_paradox_pwm_seq_vals[PARADOX_RAW_SIZE * 6] = {};

nrf_pwm_sequence_t m_paradox_pwm_seq = {
    .values.p_wave_form = m_paradox_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_paradox_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

static uint8_t crc8_maxim(uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint64_t paradox_raw_data(uint8_t *uid) {
    uint8_t manchester[12] = {0};
    uint16_t fc = (uid[0] << 8) | uid[1];
    uint32_t cn = (uid[2] << 24) | (uid[3] << 16) | (uid[4] << 8) | uid[5];
    
    manchester[0] = PARADOX_PREAMBLE;
    manchester[1] = 0x55;
    manchester[2] = 0x55;
    manchester[3] = 0x55;
    
    for (int i = 0; i < 16; i++) {
        int bit = (fc >> (15 - i)) & 1;
        int pos = 4 + (i / 8);
        if (bit) {
            manchester[pos] |= (0x80 >> (i % 8));
        }
    }
    
    for (int i = 0; i < 32; i++) {
        int bit = (cn >> (31 - i)) & 1;
        int pos = 6 + (i / 8);
        if (bit) {
            manchester[pos] |= (0x80 >> (i % 8));
        }
    }
    
    uint8_t crc = (crc8_maxim(manchester + 1, 9) ^ 0x6) & 0xFF;
    manchester[10] = crc;
    manchester[11] = 0x0A;
    
    for (int i = 1; i < 11; i++) {
        manchester[i] = (manchester[i] << 4) | (manchester[i + 1] >> 4);
    }
    
    uint64_t raw = 0;
    for (int i = 0; i < 8; i++) {
        raw <<= 8;
        raw |= manchester[i];
    }
    
    return raw;
}

static paradox_codec *paradox_alloc(void) {
    paradox_codec *codec = malloc(sizeof(paradox_codec));
    codec->modem = fsk_alloc_with_bitrate(PARADOX_BITRATE);
    codec->state = PARADOX_STATE_PREAMBLE;
    return codec;
}

static void paradox_free(paradox_codec *d) {
    if (d->modem) {
        fsk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
}

static uint8_t *paradox_get_data(paradox_codec *d) {
    return d->data;
}

static void paradox_decoder_start(paradox_codec *d, uint8_t format) {
    memset(d->data, 0, PARADOX_DATA_SIZE);
    d->raw = 0;
    d->raw_length = 0;
    d->state = PARADOX_STATE_PREAMBLE;
}

static bool paradox_decode_feed(paradox_codec *d, bool bit) {
    d->raw <<= 1;
    d->raw_length++;
    if (bit) {
        d->raw |= 1;
    }
    
    if (d->state == PARADOX_STATE_PREAMBLE) {
        if (d->raw_length >= 8) {
            uint8_t preamble = d->raw & 0xFF;
            if (preamble == PARADOX_PREAMBLE) {
                d->state = PARADOX_STATE_DATA;
            }
        }
        return false;
    }
    
    if (d->raw_length < PARADOX_RAW_SIZE) {
        return false;
    }
    
    uint16_t fc = (d->raw >> 44) & 0x03FF;
    uint32_t cn = (d->raw >> 12) & 0xFFFFFFFF;
    
    d->data[0] = (fc >> 8) & 0xFF;
    d->data[1] = fc & 0xFF;
    d->data[2] = (cn >> 24) & 0xFF;
    d->data[3] = (cn >> 16) & 0xFF;
    d->data[4] = (cn >> 8) & 0xFF;
    d->data[5] = cn & 0xFF;
    
    d->state = PARADOX_STATE_DONE;
    return true;
}

static bool paradox_decoder_feed(paradox_codec *d, uint16_t val) {
    bool bit = false;
    if (!fsk_feed(d->modem, val, &bit)) {
        return false;
    }
    return paradox_decode_feed(d, bit);
}

static const nrf_pwm_sequence_t *paradox_modulator(paradox_codec *d, uint8_t *buf) {
    uint64_t raw = paradox_raw_data(buf);
    int k = 0;
    
    for (int i = 0; i < PARADOX_RAW_SIZE; i++) {
        bool bit = (raw >> (PARADOX_RAW_SIZE - i - 1)) & 1;
        
        if (!bit) {
            for (int j = 0; j < PARADOX_FSK_HI_FREQ_LOOP; j++) {
                m_paradox_pwm_seq_vals[k].channel_0 = PARADOX_FSK_HI_FREQ_TOP / 2;
                m_paradox_pwm_seq_vals[k].counter_top = PARADOX_FSK_HI_FREQ_TOP;
                k++;
            }
        } else {
            for (int j = 0; j < PARADOX_FSK_LO_FREQ_LOOP; j++) {
                m_paradox_pwm_seq_vals[k].channel_0 = PARADOX_FSK_LO_FREQ_TOP / 2;
                m_paradox_pwm_seq_vals[k].counter_top = PARADOX_FSK_LO_FREQ_TOP;
                k++;
            }
        }
    }
    
    m_paradox_pwm_seq.length = k * 4;
    return &m_paradox_pwm_seq;
}

const protocol paradox = {
    .tag_type = TAG_TYPE_PARADOX,
    .data_size = PARADOX_DATA_SIZE,
    .alloc = (codec_alloc)paradox_alloc,
    .free = (codec_free)paradox_free,
    .get_data = (codec_get_data)paradox_get_data,
    .modulator = (modulator)paradox_modulator,
    .decoder = {
        .start = (decoder_start)paradox_decoder_start,
        .feed = (decoder_feed)paradox_decoder_feed,
    },
};

uint8_t paradox_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    uint64_t raw = paradox_raw_data(uid);
    blks[0] = T5577_PARADOX_CONFIG;
    blks[1] = 0;
    blks[2] = (raw >> 32) & 0xFFFFFFFF;
    blks[3] = raw & 0xFFFFFFFF;
    return PARADOX_T55XX_BLOCK_COUNT;
}
