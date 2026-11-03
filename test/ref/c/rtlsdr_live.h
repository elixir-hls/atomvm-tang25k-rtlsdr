/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/* Copied unchanged from usb-rtl-sdr firmware/rtlsdr_live.h (master b5795c5). The three USB
 * primitives it calls (usb_ctrl_out / usb_ctrl_in / usb_wait_ms) are in rtlsdr_usb.c. */
/* rtlsdr_live.h — librtlsdr の RTL2832U + R828D 初期化を **ライブ RMW** で行う移植 (firmware / host test 共用)。
 *
 * ★正本: pi/rtl_r82xx_live_init.py (tuner_r82xx.c + librtlsdr.c を逐語移植し Pi で lock+受信を実証した版)。
 * ★この header の API は firmware (hs_min.c) と host 側 bit-exact test (eval/stage6-fm/test_rtlsdr_live_port.py)
 *   の両方が使う。USB の原始操作 3 本は **呼び手が提供する** (extern、下記)。
 */
#ifndef RTLSDR_LIVE_H
#define RTLSDR_LIVE_H
#include <stdint.h>

/* ★flags */
#define RTLSDR_F_TESTMODE   0x01u   /* set_sample_rate の後に demod page0 0x19 <- 0x03 (8bit counter payload) */
#define RTLSDR_F_SKIP_TUNER 0x02u   /* baseband + rate + reset_buffer だけ (= gen_min_rom.py の minimal 相当) */
#define RTLSDR_F_ZERO_IF    0x04u   /* 土台 (FM ラジオ) の設定のまま: Zero-IF、I/Q 両 ADC、スペクトル反転なし (比較用) */

typedef struct {
    uint32_t rf_hz;        /* 受信周波数 [Hz]、例 82500000 */
    uint32_t gain_tenths;  /* 例 402 = 40.2 dB、0 = tuner AGC */
    uint32_t rate;         /* sample rate [Hz]、例 2400000 / 240000 */
    uint32_t flags;        /* RTLSDR_F_* */
} rtlsdr_params_t;

typedef struct {
    uint32_t n_ctrl;        /* 撃った control transfer の総数 */
    uint32_t n_fail;        /* 失敗 (ACK/DATA が返らなかった) 数 */
    uint32_t first_fail;    /* 最初に失敗した transfer の通し番号 (1 始まり、0 = 失敗なし) */
    uint32_t init_ok;       /* r82xx_init (set_tv_standard の filter cal PLL lock) */
    uint32_t freq_ok;       /* set_freq の PLL lock */
    uint32_t has_lock;      /* 最後の set_pll の lock bit */
    uint32_t gain_total;    /* set_gain が積んだ合計 (tenths dB) */
    uint32_t div_num;       /* 最後の set_pll の div_num (vco_fine_tune 調整後、8bit) */
    uint32_t vco_fine_tune; /* 最後の set_pll が読んだ vco_fine_tune (0..3) */
    uint32_t fil_cal_code;  /* set_tv_standard が読んだ filter cal code */
    uint8_t  last_rd[5];    /* 最後の r82xx_read の raw byte (bitrev 前、線の順) */
} rtlsdr_result_t;

/* ★★USB 原始操作 — 呼び手 (hs_min.c / host mock) が定義する。戻り値 1 = 成功。
 *   ★vendor control (bRequest 0)。OUT は bmRequestType 0x40、IN は 0xC0。 */
extern uint32_t usb_ctrl_out(uint32_t wvalue, uint32_t windex, const uint8_t *data, uint32_t n);
extern uint32_t usb_ctrl_in(uint32_t wvalue, uint32_t windex, uint8_t *dst, uint32_t n);
extern void     usb_wait_ms(uint32_t ms);

void rtlsdr_live_init(const rtlsdr_params_t *p, rtlsdr_result_t *r);
/* tuner gain change while streaming (after rtlsdr_live_init); returns the gain set (tenths dB) */
uint32_t rtlsdr_live_set_gain(uint32_t gain_tenths, rtlsdr_result_t *r);
/* retune while streaming (after rtlsdr_live_init); returns 1 when the PLL locked */
uint32_t rtlsdr_live_set_freq(uint32_t rf_hz, rtlsdr_result_t *r);

#endif
