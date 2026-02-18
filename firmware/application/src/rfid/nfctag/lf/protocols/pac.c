#include "pac.h"
#include "protocols.h"
#include "t55xx.h"
#include "../../parity.h"
#include <stdlib.h>
#include <string.h>

#define PAC_PREAMBLE_LEN 8
#define PAC_PREAMBLE_BITS 0xFF

static void *pac_alloc(void) {
    pac_codec *codec = calloc(1, sizeof(pac_codec));
    return codec;
}

static void pac_free(void *codec) {
    free(codec);
}

static void pac_build_frame(uint8_t *data, uint8_t data_len, uint8_t *raw) {
    uint8_t bitpos = 0;
    
    // Preamble: 8 mark bits
    for (int i = 0; i < PAC_PREAMBLE_LEN; i++) {
        raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
        bitpos++;
    }
    
    // STX (0x02) with framing: start(0) + 7 data bits LSB + odd parity + stop(1)
    uint8_t stx = 0x02;
    raw[bitpos / 8] &= ~(1 << (7 - (bitpos % 8))); // start bit = 0
    bitpos++;
    for (int i = 0; i < 7; i++) {
        if ((stx >> i) & 1) {
            raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
        }
        bitpos++;
    }
    if (oddparity8(stx & 0x7F)) {
        raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
    }
    bitpos++;
    raw[bitpos / 8] |= (1 << (7 - (bitpos % 8))); // stop bit = 1
    bitpos++;
    
    // '2' (0x32) with framing
    uint8_t ch2 = '2';
    raw[bitpos / 8] &= ~(1 << (7 - (bitpos % 8)));
    bitpos++;
    for (int i = 0; i < 7; i++) {
        if ((ch2 >> i) & 1) {
            raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
        }
        bitpos++;
    }
    if (oddparity8(ch2 & 0x7F)) {
        raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
    }
    bitpos++;
    raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
    bitpos++;
    
    // '0' (0x30) with framing
    uint8_t ch0 = '0';
    raw[bitpos / 8] &= ~(1 << (7 - (bitpos % 8)));
    bitpos++;
    for (int i = 0; i < 7; i++) {
        if ((ch0 >> i) & 1) {
            raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
        }
        bitpos++;
    }
    if (oddparity8(ch0 & 0x7F)) {
        raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
    }
    bitpos++;
    raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
    bitpos++;
    
    // Card ID bytes (8 bytes) with framing
    uint8_t checksum = 0;
    for (int byte_idx = 0; byte_idx < data_len; byte_idx++) {
        uint8_t byte = data[byte_idx];
        checksum ^= byte;
        
        raw[bitpos / 8] &= ~(1 << (7 - (bitpos % 8))); // start bit
        bitpos++;
        for (int i = 0; i < 7; i++) {
            if ((byte >> i) & 1) {
                raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
            }
            bitpos++;
        }
        if (oddparity8(byte & 0x7F)) {
            raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
        }
        bitpos++;
        raw[bitpos / 8] |= (1 << (7 - (bitpos % 8))); // stop bit
        bitpos++;
    }
    
    // Checksum byte with framing
    raw[bitpos / 8] &= ~(1 << (7 - (bitpos % 8)));
    bitpos++;
    for (int i = 0; i < 7; i++) {
        if ((checksum >> i) & 1) {
            raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
        }
        bitpos++;
    }
    if (oddparity8(checksum & 0x7F)) {
        raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
    }
    bitpos++;
    raw[bitpos / 8] |= (1 << (7 - (bitpos % 8)));
}

static uint8_t *pac_get_data(void *codec) {
    return ((pac_codec *)codec)->data;
}

static const nrf_pwm_sequence_t *pac_modulator(void *codec, uint8_t *data) {
    pac_codec *pac = (pac_codec *)codec;
    memcpy(pac->data, data, PAC_DATA_SIZE);
    
    memset(pac->raw, 0, sizeof(pac->raw));
    pac_build_frame(pac->data, PAC_DATA_SIZE, pac->raw);
    pac->raw_length = PAC_RAW_SIZE;
    
    // Build PWM sequence for NRZ modulation at RF/32
    // Each bit = 32 RF cycles
    // For NRZ: 1 = carrier on, 0 = carrier off
    uint16_t *seq_values = calloc(PAC_RAW_SIZE, sizeof(uint16_t));
    for (int i = 0; i < PAC_RAW_SIZE; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = 7 - (i % 8);
        if (pac->raw[byte_idx] & (1 << bit_idx)) {
            seq_values[i] = 0x8000 | 32; // carrier on for 32 cycles
        } else {
            seq_values[i] = 32; // carrier off for 32 cycles
        }
    }
    
    nrf_pwm_sequence_t *seq = calloc(1, sizeof(nrf_pwm_sequence_t));
    seq->values.p_raw = seq_values;
    seq->length = PAC_RAW_SIZE;
    seq->repeats = 0;
    seq->end_delay = 0;
    
    return seq;
}

static void pac_decoder_start(void *codec, uint8_t preamble) {
    pac_codec *pac = (pac_codec *)codec;
    memset(pac->data, 0, PAC_DATA_SIZE);
    memset(pac->raw, 0, sizeof(pac->raw));
    pac->raw_length = 0;
}

static bool pac_decoder_feed(void *codec, uint16_t value) {
    // PAC uses NRZ encoding - decoding not implemented for reader mode
    // This would require SAADC-based signal processing
    return false;
}

uint8_t pac_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    if (uid == NULL || blks == NULL) {
        return 0;
    }
    
    // Block 0: T5577 configuration
    // NRZ modulation, RF/32, 4 data blocks
    blks[0] = T5577_MODULATION_DIRECT | T5577_BITRATE_RF_32 | (4 << T5577_MAXBLOCK_SHIFT);
    
    // Build the 128-bit frame
    uint8_t raw[16] = {0};
    pac_build_frame(uid, PAC_DATA_SIZE, raw);
    
    // Blocks 1-4: Raw data (128 bits = 4 x 32-bit blocks)
    blks[1] = (raw[0] << 24) | (raw[1] << 16) | (raw[2] << 8) | raw[3];
    blks[2] = (raw[4] << 24) | (raw[5] << 16) | (raw[6] << 8) | raw[7];
    blks[3] = (raw[8] << 24) | (raw[9] << 16) | (raw[10] << 8) | raw[11];
    blks[4] = (raw[12] << 24) | (raw[13] << 16) | (raw[14] << 8) | raw[15];
    
    return 5; // 5 blocks total
}

const protocol pac = {
    .alloc = pac_alloc,
    .free = pac_free,
    .data_size = PAC_DATA_SIZE,
    .get_data = pac_get_data,
    .modulator = pac_modulator,
    .decoder.start = pac_decoder_start,
    .decoder.feed = pac_decoder_feed,
};
