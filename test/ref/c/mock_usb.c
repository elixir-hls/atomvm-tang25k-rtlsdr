/* The reference for test/ref/transfers.sh: the C driver (rtlsdr_live.c, the earlier C port
 * of librtlsdr from usb-rtl-sdr-tv) on a mock USB that prints every vendor control transfer
 * and answers IN transfers with bytes from a fixed rule (the same rule as the Elixir mock).
 * usage: mock_usb <seed> <rf_hz> <gain_tenths> <tune_hz> */
#include <stdio.h>
#include <stdlib.h>
#include "rtlsdr_live.h"

static uint32_t seed, n_in;

uint32_t usb_ctrl_out(uint32_t wvalue, uint32_t windex, const uint8_t *data, uint32_t n)
{
    printf("O %04x %04x ", wvalue, windex);
    for (uint32_t i = 0; i < n; i++) printf("%02x", data[i]);
    printf("\n");
    return 1;
}

uint32_t usb_ctrl_in(uint32_t wvalue, uint32_t windex, uint8_t *dst, uint32_t n)
{
    printf("I %04x %04x %u\n", wvalue, windex, n);
    for (uint32_t k = 0; k < n; k++) dst[k] = (uint8_t)(n_in * 37u + k * 11u + seed * 101u + (n_in >> 1) * seed);
    n_in++;
    return 1;
}

void usb_wait_ms(uint32_t ms) { (void)ms; }

int main(int argc, char **argv)
{
    if (argc != 5) return 2;
    seed = (uint32_t)atoi(argv[1]);
    rtlsdr_params_t p = { .rf_hz = (uint32_t)atoi(argv[2]), .gain_tenths = (uint32_t)atoi(argv[3]),
                          .rate = 960000, .flags = RTLSDR_F_ZERO_IF };
    static rtlsdr_result_t r;
    rtlsdr_live_init(&p, &r);
    printf("init n_ctrl=%u n_fail=%u init_ok=%u freq_ok=%u gain=%u\n", r.n_ctrl, r.n_fail, r.init_ok, r.freq_ok, r.gain_total);
    printf("tune %u\n", rtlsdr_live_set_freq((uint32_t)atoi(argv[4]), &r));
    return 0;
}
