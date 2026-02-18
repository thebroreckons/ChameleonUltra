#include <stdio.h>

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "lf_reader_main.h"
#include "nrfx_saadc.h"
#include "protocols/indala.h"
#include "time.h"
#include "rfid_main.h"

#define NRF_LOG_MODULE_NAME lf_indala_read
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#define INDALA_BUFFER_SIZE (12288)

static circular_buffer cb;

static void saadc_cb(nrf_saadc_value_t *vals, size_t size) {
    for (int i = 0; i < size; i++) {
        nrf_saadc_value_t val = vals[i];
        if (!cb_push_back(&cb, &val)) {
            return;
        }
    }
}

static void init_indala_hw(void) {
    lf_125khz_radio_saadc_enable(saadc_cb);
}

static void uninit_indala_hw(void) {
    lf_125khz_radio_saadc_disable();
}

bool indala_64_read(uint8_t *data, uint32_t timeout_ms) {
    void *codec = indala_64.alloc();
    indala_64.decoder.start(codec, 0);

    cb_init(&cb, INDALA_BUFFER_SIZE, sizeof(uint16_t));
    init_indala_hw();
    start_lf_125khz_radio();

    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            if (indala_64.decoder.feed(codec, val)) {
                memcpy(data, indala_64.get_data(codec), indala_64.data_size);
                ok = true;
                break;
            }
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_indala_hw();
    cb_free(&cb);

    indala_64.free(codec);
    return ok;
}

bool indala_224_read(uint8_t *data, uint32_t timeout_ms) {
    void *codec = indala_224.alloc();
    indala_224.decoder.start(codec, 0);

    cb_init(&cb, INDALA_BUFFER_SIZE, sizeof(uint16_t));
    init_indala_hw();
    start_lf_125khz_radio();

    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            if (indala_224.decoder.feed(codec, val)) {
                memcpy(data, indala_224.get_data(codec), indala_224.data_size);
                ok = true;
                break;
            }
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_indala_hw();
    cb_free(&cb);

    indala_224.free(codec);
    return ok;
}
