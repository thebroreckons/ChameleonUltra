#include <stdio.h>

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "lf_reader_main.h"
#include "nrfx_saadc.h"
#include "protocols/fdxb.h"
#include "time.h"
#include "rfid_main.h"

#define NRF_LOG_MODULE_NAME lf_fdxb_read
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#define FDXB_BUFFER_SIZE (8192)

static circular_buffer cb;

static void saadc_cb(nrf_saadc_value_t *vals, size_t size) {
    for (int i = 0; i < size; i++) {
        nrf_saadc_value_t val = vals[i];
        if (!cb_push_back(&cb, &val)) {
            return;
        }
    }
}

static void init_fdxb_hw(void) {
    lf_125khz_radio_saadc_enable(saadc_cb);
}

static void uninit_fdxb_hw(void) {
    lf_125khz_radio_saadc_disable();
}

bool fdxb_read(uint8_t *data, uint32_t timeout_ms) {
    void *codec = fdxb.alloc();
    fdxb.decoder.start(codec, 0);

    cb_init(&cb, FDXB_BUFFER_SIZE, sizeof(uint16_t));
    init_fdxb_hw();
    start_lf_125khz_radio();

    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            if (fdxb.decoder.feed(codec, val)) {
                memcpy(data, fdxb.get_data(codec), fdxb.data_size);
                ok = true;
                break;
            }
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_fdxb_hw();
    cb_free(&cb);

    fdxb.free(codec);
    return ok;
}
