/* The reference for test/ref/transfers.sh: the C driver this package had before the Elixir
 * one (lib/rtlsdr.ex), unchanged but for this note and the build guard. */
/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/* Copied from usb-rtl-sdr firmware/rtlsdr_live.c (master b5795c5). The three USB
 * primitives it calls (usb_ctrl_out / usb_ctrl_in / usb_wait_ms) are in rtlsdr_usb.c.
 * One change: gain_tenths == 0 selects the tuner AGC (from the original's 21c431d, reverted
 * there because its demodulator's output grew with the amplitude squared; rtl/fm_demod.v's
 * does not). */
/* ==========================================================================
 * rtlsdr_live.c — librtlsdr の RTL2832U baseband + R828D tuner 初期化を **ライブ RMW** で行う。
 *
 * ★★なぜ ROM replay ではないのか (2026-09-18 の実測、verdict-tuner-fixed-replay-insufficient):
 *   固定 replay ROM は一度は PLL lock したが、電源サイクルで R828D の VCO auto-cal (vco_fine_tune)
 *   が drift すると同じ値では lock しない。librtlsdr は set_pll で vco_fine_tune を **read** して
 *   div_num を調整し、lock bit を **read** して未 lock なら VCO 電流を上げて **retry** する。
 *   set_tv_standard も filter cal code を read する。★read 結果で書く値が変わる = CRC16 も変わる ∴
 *   **ROM (定数 CRC) では原理的に組めない**。shadow register を持つ本物のアルゴリズムが要る。
 *
 * ★★正本 = pi/rtl_r82xx_live_init.py (Pi + pyusb で 79.5MHz を電源状態非依存で lock+受信を実証)。
 *   ★この file は **その Python を逐語で C にしたもの**。関数名・順序・定数を変えない。
 *   ★bit-exact 検証 = eval/stage6-fm/test_rtlsdr_live_port.py (同じ mock USB に両者を流し、
 *     transfer 列 (wValue/wIndex/data) が一致することを要求する)。
 *
 * ★★RV32I (SERV) の制約: 乗算/除算命令が無い。`*` `/` は gcc が __mulsi3/__udivsi3 を呼ぶので
 *   hs_min.c 側が shift-add / shift-subtract の実装を提供する (libgcc は link しない)。
 *   ★この file は移植の逐語性を優先して `*` `/` をそのまま書く (host test では x86 native)。
 *
 * ★★USB 原始操作は rtlsdr_live.h の extern 3 本 (usb_ctrl_out / usb_ctrl_in / usb_wait_ms)。
 *   firmware では hs_min.c が SETUP+CRC16 を組んで撃ち、host test では mock が記録する。
 * ========================================================================== */
#include <stdint.h>
#include "rtlsdr_live.h"

/* ---------------- librtlsdr の定数 (逐語) -------------------------------- */
#define USBB   1u
#define SYSB   2u
#define IICB   6u
#define USB_SYSCTL      0x2000u
#define USB_EPA_CTL     0x2148u
#define USB_EPA_MAXPKT  0x2158u
#define DEMOD_CTL       0x3000u
#define DEMOD_CTL_1     0x300bu
#define XTAL            28800000u   /* ★V4: tuner xtal = RTL xtal 28.8MHz (トレースで検証) */
#define IF_FREQ         3570000u    /* R82XX_IF_FREQ */
#define I2C_ADDR        0x74u
#define MAX_I2C         8u
#define VER_NUM         49u
#define VCO_POWER_REF   1u          /* CHIP_R828D */
#define NREGS           27u         /* r82xx shadow: reg 0x05..0x1f */
#define REG_SHADOW_START 5u

/* ★FIR_DEFAULT (int16 x16) を librtlsdr の詰め方 (int8 x8 + int12 x8) で並べた 20 byte。
 *   ★計算は tools/gen_min_rom.py::set_fir / Python 正本と同じ。★host test が bit-exact を要求する。 */
/* r82xx_init_array: regs 0x05..0x1f (27 bytes) */
static const uint8_t INIT_ARRAY[NREGS] = {
    0x83, 0x30, 0x75, 0xc0, 0x40, 0xd6, 0x6c, 0xf5, 0x63, 0x75, 0x68,
    0x6c, 0x83, 0x80, 0x00, 0x0f, 0x00, 0xc0, 0x30, 0x48, 0xcc, 0x60, 0x00,
    0x54, 0xae, 0x4a, 0xc0 };
/* freq_ranges (used fields only): start_MHz, open_d, rf_mux_ploy, tf_c, xtal_cap0p */
#define NRANGES 21u
static const uint16_t RANGE_MHZ[NRANGES] = {
    0, 50, 55, 60, 65, 70, 75, 80, 90, 100, 110, 120, 140, 180, 220, 250, 280, 310, 450, 588, 650 };
static const uint8_t RANGE_OPEN_D[NRANGES] = {
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t RANGE_RF_MUX[NRANGES] = {
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
    0x02, 0x02, 0x41, 0x41, 0x40, 0x40 };
static const uint8_t RANGE_TF_C[NRANGES] = {
    0xdf, 0xbe, 0x8b, 0x7b, 0x69, 0x58, 0x44, 0x44, 0x34, 0x34, 0x24, 0x24, 0x14, 0x13, 0x13,
    0x11, 0x00, 0x00, 0x00, 0x00, 0x00 };
/* xtal_cap0p は全 range 0x00 (V4 の表) ∴ 表を持たない */
static const uint8_t BITREV_LUT[16] = {
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
static const int8_t LNA_STEPS[16] = { 0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13 };
static const int8_t MIX_STEPS[16] = { 0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8 };
/* r82xx_set_bandwidth (tuner_r82xx.c 逐語、pi/rtl_r82xx_live_init.py::set_bandwidth と同一) */
static const uint32_t IF_LPF_BW[10] = {
    1700000u, 1600000u, 1550000u, 1450000u, 1200000u, 900000u, 700000u, 550000u, 450000u, 350000u };
#define FILT_HP_BW1 350000u
#define FILT_HP_BW2 380000u

/* ---------------- tuner state (Python の Tuner インスタンス変数) ------------ */
static uint8_t  T_regs[NREGS];
static uint32_t T_has_lock;
static uint32_t T_fil_cal_code;
static uint32_t T_int_freq;
static rtlsdr_result_t *R;          /* ★結果の書き場 (rtlsdr_live_init が設定) */

static uint32_t bitrev(uint32_t b)
{
    return (BITREV_LUT[b & 0xfu] << 4) | BITREV_LUT[(b >> 4) & 0xfu];
}

/* ★全 USB transfer はここを通る (計数と first_fail のため) */
static void tally(uint32_t ok)
{
    R->n_ctrl = R->n_ctrl + 1u;
    if (!ok) {
        R->n_fail = R->n_fail + 1u;
        if (R->first_fail == 0u) {
            R->first_fail = R->n_ctrl;
        }
    }
}

/* ---------------- RTL2832 register access (librtlsdr) ---------------------- */
static void rtl_wr(uint32_t block, uint32_t addr, uint32_t val, uint32_t ln)
{
    uint8_t d[2];
    if (ln == 1u) {
        d[0] = (uint8_t)(val & 0xffu);
    } else {
        d[0] = (uint8_t)((val >> 8) & 0xffu);
        d[1] = (uint8_t)(val & 0xffu);
    }
    tally(usb_ctrl_out(addr, (block << 8) | 0x10u, d, ln));
}

static uint32_t rtl_rd(uint32_t block, uint32_t addr)
{
    uint8_t d[1];
    d[0] = 0u;
    tally(usb_ctrl_in(addr, (block << 8), d, 1u));
    return d[0];
}

/* ★demod write = librtlsdr rtlsdr_demod_write_reg 逐語: write の後に **trailing dummy read**
 *   (page 0x0a reg 0x01、1 byte) を撃つ。
 *   ★2026-09-19 の移植では「gen_min_rom.py と同じく不要と実証済 (旧 RTL、gap 18us)」として省いていた。
 *   ★2026-09-21 (5) 実機 (12 KB RAM、stack 正常): dummy read 無しだと **30 本目の demod write
 *     (page1 reg 0x2a) から device が OUT+DATA1 を STALL** し、以後 73/109 が失敗
 *     (eval/stage6-fm/verdict-stage3-lilog-1.json の ★★li_log)。09-12 の 548/548 は usbmon trace の
 *     replay = read 込みだった。∴ Python 正本 / librtlsdr と同じ形に戻して A/B する。
 *   ★dummy read は tally しない (n_ctrl の番号を Python 正本の write 列と揃えるため)。 */
static void demod_wr(uint32_t page, uint32_t addr, uint32_t val, uint32_t ln)
{
    uint8_t d[2];
    if (ln == 1u) {
        d[0] = (uint8_t)(val & 0xffu);
    } else {
        d[0] = (uint8_t)((val >> 8) & 0xffu);
        d[1] = (uint8_t)(val & 0xffu);
    }
    tally(usb_ctrl_out((addr << 8) | 0x20u, 0x10u | page, d, ln));
    (void)usb_ctrl_in((0x01u << 8) | 0x20u, 0x0au, d, 1u);      /* ★trailing dummy read */
}

static void i2c_repeater(uint32_t on)
{
    demod_wr(1u, 0x01u, on ? 0x18u : 0x10u, 1u);
}

/* ★set_if_freq(IF_FREQ) — Python: iff = (int(freq * (1<<22) // XTAL)) * -1 ; iff &= 0x3fffff
 *   ★freq << 22 は 32bit を溢れる (1.5e13)。★RV32I に 64bit 除算 (__udivdi3) を持ち込まないため、
 *     引数は IF_FREQ 固定にして **整数定数式** で畳む (gcc が compile-time に評価する)。 */
#define IFF_Q   ((uint32_t)((((uint64_t)IF_FREQ) << 22) / (uint64_t)XTAL))
#define IFF_VAL ((0u - IFF_Q) & 0x3fffffu)
static void set_if_freq(void)
{
    const uint32_t iff = IFF_VAL;
    demod_wr(1u, 0x19u, (iff >> 16) & 0x3fu, 1u);
    demod_wr(1u, 0x1au, (iff >> 8) & 0xffu, 1u);
    demod_wr(1u, 0x1bu, iff & 0xffu, 1u);
}

/* ★set_if_freq_dyn(freq) — set_if_freq と同じ式だが freq が実行時に決まる (set_bandwidth の
 *   戻り値、rate に依存)。★freq << 22 は 32bit を溢れるので set_sample_rate と同じ long division
 *   (freq/XTAL の商 0 に、余りへ 22 bit を 1 bit ずつ入れる) で計算する。★freq < XTAL (28.8MHz)
 *   が前提 (int_freq は常に高々 4.57MHz)。 */
static void set_if_freq_dyn(uint32_t freq)
{
    uint32_t q = freq / XTAL;              /* 常に 0 (freq < XTAL) */
    uint32_t rem = freq - q * XTAL;
    uint32_t i, iff;
    for (i = 0u; i < 22u; i++) {
        rem <<= 1; q <<= 1;
        if (rem >= XTAL) { rem -= XTAL; q |= 1u; }
    }
    iff = (0u - q) & 0x3fffffu;
    demod_wr(1u, 0x19u, (iff >> 16) & 0x3fu, 1u);
    demod_wr(1u, 0x1au, (iff >> 8) & 0xffu, 1u);
    demod_wr(1u, 0x1bu, iff & 0xffu, 1u);
}

/* ---------------- R828D I2C (shadow-based, r82xx_write/read verbatim) ------ */
static void i2c_write(uint32_t reg, const uint8_t *vals, uint32_t n)
{
    uint32_t i, pos, r, rem, size;
    uint8_t buf[MAX_I2C];

    for (i = 0u; i < n; i++) {
        uint32_t idx = reg - REG_SHADOW_START + i;
        if (idx < NREGS) {
            T_regs[idx] = vals[i];
        }
    }
    pos = 0u; r = reg; rem = n;
    while (rem > 0u) {
        size = (rem < (MAX_I2C - 1u)) ? rem : (MAX_I2C - 1u);
        buf[0] = (uint8_t)r;
        for (i = 0u; i < size; i++) {
            buf[1u + i] = vals[pos + i];
        }
        tally(usb_ctrl_out(I2C_ADDR, (IICB << 8) | 0x10u, buf, size + 1u));
        r += size; pos += size; rem -= size;
    }
}

static void write_reg(uint32_t reg, uint32_t val)
{
    uint8_t v = (uint8_t)val;
    i2c_write(reg, &v, 1u);
}

static uint32_t cache(uint32_t reg)
{
    uint32_t i = reg - REG_SHADOW_START;
    return (i < NREGS) ? T_regs[i] : 0u;
}

static void write_reg_mask(uint32_t reg, uint32_t val, uint32_t mask)
{
    uint32_t cur = cache(reg);
    write_reg(reg, (cur & ~mask) | (val & mask));
}

/* ★定数だけの write_reg_mask 列を table で回す (image 縮小)。1 entry = {reg, val, mask}。 */
static void wrm_seq(const uint8_t *t, uint32_t n)
{
    uint32_t i;
    for (i = 0u; i < n; i += 3u) {
        write_reg_mask(t[i], t[i + 1u], t[i + 2u]);
    }
}

/* r82xx_read from reg 0, bit-reversed. ★dst は bitrev 後、R->last_rd は raw。 */
static void r82xx_read(uint8_t *dst, uint32_t n)
{
    uint8_t zero[1];
    uint32_t i;
    zero[0] = 0x00u;
    tally(usb_ctrl_out(I2C_ADDR, (IICB << 8) | 0x10u, zero, 1u));
    for (i = 0u; i < n; i++) dst[i] = 0u;
    tally(usb_ctrl_in(I2C_ADDR, (IICB << 8), dst, n));
    for (i = 0u; i < n; i++) {
        if (i < 5u) R->last_rd[i] = dst[i];
        dst[i] = (uint8_t)bitrev(dst[i]);
    }
}

/* ---------------- r82xx_set_pll (★LIVE: reads vco_fine_tune + lock, retries) -- */
static uint32_t set_pll(uint32_t freq)
{
    const uint32_t pll_ref = XTAL;
    const uint32_t freq_khz = (freq + 500u) / 1000u;
    const uint32_t pll_ref_khz = (pll_ref + 500u) / 1000u;
    const uint32_t vco_min = 1770000u;
    const uint32_t vco_max = vco_min * 2u;
    uint32_t mix_div = 2u, div_num = 0u, div_buf;
    uint32_t vco_fine_tune, vco_freq, nint, vco_fra, ni, si, n_sdm, sdm, i;
    uint8_t data[5];

    static const uint8_t head[] = {
        0x10, 0x00, 0x10,   /* refdiv2=0 */
        0x1a, 0x00, 0x0c,   /* pll autotune 128kHz */
        0x12, 0x06, 0xff }; /* VCO current MAX (Blog mod) */
    wrm_seq(head, sizeof(head));
    while (mix_div <= 64u) {
        if ((vco_min <= freq_khz * mix_div) && (freq_khz * mix_div < vco_max)) {
            div_buf = mix_div;
            while (div_buf > 2u) {
                div_buf >>= 1; div_num += 1u;
            }
            break;
        }
        mix_div <<= 1;
    }
    r82xx_read(data, 5u);                       /* ★LIVE read: vco_fine_tune */
    vco_fine_tune = (data[4] & 0x30u) >> 4;
    if (vco_fine_tune > VCO_POWER_REF)      div_num -= 1u;
    else if (vco_fine_tune < VCO_POWER_REF) div_num += 1u;
    R->vco_fine_tune = vco_fine_tune;
    R->div_num = div_num & 0xffu;
    write_reg_mask(0x10u, (div_num << 5) & 0xffu, 0xe0u);
    vco_freq = freq * mix_div;
    nint = vco_freq / (2u * pll_ref);
    vco_fra = (vco_freq - 2u * pll_ref * nint) / 1000u;
    if (nint > ((128u / VCO_POWER_REF) - 1u)) {
        return 0u;                              /* [R82XX] no valid PLL */
    }
    ni = (nint - 13u) / 4u; si = nint - 4u * ni - 13u;
    write_reg(0x14u, ni + (si << 6));
    write_reg_mask(0x12u, (vco_fra == 0u) ? 0x08u : 0x00u, 0x08u);
    n_sdm = 2u; sdm = 0u;
    while (vco_fra > 1u) {
        if (vco_fra > (2u * pll_ref_khz / n_sdm)) {
            sdm += 32768u / (n_sdm / 2u);
            vco_fra -= 2u * pll_ref_khz / n_sdm;
            if (n_sdm >= 0x8000u) break;
        }
        n_sdm <<= 1;
    }
    write_reg(0x16u, sdm >> 8);
    write_reg(0x15u, sdm & 0xffu);
    T_has_lock = 0u;
    for (i = 0u; i < 2u; i++) {
        usb_wait_ms(10u);
        r82xx_read(data, 3u);                   /* ★LIVE read: lock bit */
        if (data[2] & 0x40u) {
            T_has_lock = 1u; break;
        }
        if (i == 0u) {
            /* ★2026-09-23 修正: upstream tuner_r82xx.c は元の non-Blog driver の
             *   0x60/0xe0 (VCO current field 部分書換) をコメントアウトし、
             *   "RTL-SDR Blog Hack: Set max current" として 0x06/0xff (全byte
             *   上書き、set_pll 冒頭の初期化と同じ値) に置き換えている。本 port は
             *   コメントアウトされた側の値を誤って採用していた (github.com/
             *   rtlsdrblog/rtl-sdr-blog master, tuner_r82xx.c:554-561 で確認)。 */
            write_reg_mask(0x12u, 0x06u, 0xffu);  /* RTL-SDR Blog Hack: max current, retry */
        }
    }
    R->has_lock = T_has_lock;
    return T_has_lock;
}

/* ---------------- r82xx_set_mux ------------------------------------------- */
static void set_mux(uint32_t freq)
{
    uint32_t fmhz = freq / 1000000u;
    uint32_t j, rng = NRANGES - 1u;
    for (j = 0u; j < NRANGES - 1u; j++) {
        if (fmhz < RANGE_MHZ[j + 1u]) {
            rng = j; break;
        }
    }
    write_reg_mask(0x17u, RANGE_OPEN_D[rng], 0x08u);
    write_reg_mask(0x1au, RANGE_RF_MUX[rng], 0xc3u);
    write_reg(0x1bu, RANGE_TF_C[rng]);
    {
        static const uint8_t tail[] = { 0x10, 0x00, 0x0b,  0x08, 0x00, 0x3f,  0x09, 0x00, 0x3f };
        wrm_seq(tail, sizeof(tail));    /* xtal_cap0p (all 0x00) | XTAL_HIGH_CAP_0P */
    }
}

/* ---------------- r82xx_set_tv_standard (★LIVE: filter cal read) ----------- */
static uint32_t set_tv_standard(void)
{
    const uint32_t if_khz = 3570u, filt_cal_lo = 56000u;
    const uint32_t filt_gain = 0x30u, img_r = 0x00u, filt_q = 0x10u, hp_cor = 0x6bu;
    const uint32_t ext_enable = 0x60u, loop_through = 0x80u, lt_att = 0x00u;
    const uint32_t flt_ext_widest = 0x00u, polyfil_cur = 0x60u;
    uint32_t i;
    uint8_t data[5];

    static const uint8_t head[] = {
        0x0c, 0x00, 0x0f,  0x13, VER_NUM, 0x3f,  0x1d, 0x00, 0x38 };   /* type != ANALOG_TV */
    static const uint8_t tail[] = {
        0x0b, hp_cor, 0xef,  0x07, img_r, 0x80,  0x06, filt_gain, 0x30,  0x1e, ext_enable, 0x60,
        0x05, loop_through, 0x80,  0x1f, lt_att, 0x80,  0x0f, flt_ext_widest, 0x80,
        0x19, polyfil_cur, 0x60 };
    for (i = 0u; i < NREGS; i++) T_regs[i] = INIT_ARRAY[i];   /* shadow = init_array */
    wrm_seq(head, sizeof(head));
    T_int_freq = if_khz * 1000u;
    for (i = 0u; i < 2u; i++) {
        static const uint8_t pre[] = {
            0x0b, hp_cor, 0x60,  0x0f, 0x04, 0x04,  0x10, 0x00, 0x03 };   /* cali clk on / xtal cap 0pF */
        static const uint8_t post[] = {
            0x0b, 0x10, 0x10,  0x0b, 0x00, 0x10,  0x0f, 0x00, 0x04 };     /* start / stop trigger / clk off */
        wrm_seq(pre, sizeof(pre));
        if (!set_pll(filt_cal_lo * 1000u) || !T_has_lock) {
            return 0u;
        }
        wrm_seq(post, sizeof(post));
        r82xx_read(data, 5u);                   /* ★LIVE read: cal code */
        T_fil_cal_code = data[4] & 0x0fu;
        if (T_fil_cal_code && (T_fil_cal_code != 0x0fu)) break;
    }
    if (T_fil_cal_code == 0x0fu) T_fil_cal_code = 0u;
    R->fil_cal_code = T_fil_cal_code;
    write_reg_mask(0x0au, filt_q | T_fil_cal_code, 0x1fu);
    wrm_seq(tail, sizeof(tail));
    return 1u;
}

/* ---------------- r82xx_sysfreq_sel (DVB-T default branch) ----------------- */
static void sysfreq_sel(void)
{
    /* mixer_top 0x24 / lna_top 0xe5 / lna_vth_l 0x53 / mixer_vth_l 0x75 / air_cable1_in 0 / cable2_in 0
     * cp_cur 0x38 / div_buf_cur 0xa0 (Blog hack) / filter_cur 0x40 / lna_discharge 14。
     * ★write_reg(reg, v) は write_reg_mask(reg, v, 0xff) と同じ線に出る (shadow も同じ)。 */
    static const uint8_t seq[] = {
        0x1d, 0xe5, 0xc7,  0x1c, 0x24, 0xf8,  0x0d, 0x53, 0xff,  0x0e, 0x75, 0xff,
        0x05, 0x00, 0x60,  0x06, 0x00, 0x08,  0x11, 0x38, 0x38,  0x17, 0xa0, 0x30,  0x0a, 0x40, 0x60,
        /* LNA (type != ANALOG_TV) */
        0x1d, 0x00, 0x38,  0x1c, 0x00, 0x04,  0x06, 0x00, 0x40,  0x1a, 0x30, 0x30,
        0x1d, 0x18, 0x38,  0x1c, 0x24, 0x04,  0x1e, 14, 0x1f,    0x1a, 0x20, 0x30 };
    wrm_seq(seq, sizeof(seq));
}

static void set_vga_gain(void)
{
    write_reg_mask(0x0cu, 0x08u, 0x9fu);        /* 16.3 dB VGA */
}

/* ---------------- r82xx_set_bandwidth (★LIVE: rate に合わせて IF filter 帯域 + int_freq を
 *   再計算する。pi/rtl_r82xx_live_init.py::Tuner.set_bandwidth と同一)。
 *   ★usbmon 実測 (rate=240000): reg 0x0a=0xc8 (bit4=0 = 狭帯域枝) / reg 0x0b=0xe6 と一致確認済み。
 *   ★これが無いと IF filter が汎用初期化 (set_tv_standard) の広い帯域のまま残り、rate に対して
 *   帯域が広すぎる (隣接ノイズ/干渉が混入する) — 2026-09-22 の実測で確定した欠落。 */
static uint32_t set_bandwidth(uint32_t bw)
{
    uint32_t reg_0a, reg_0b, real_bw = 0u, int_freq, i;
    if (bw > 7000000u) {
        reg_0a = 0x10u; reg_0b = 0x0bu; int_freq = 4570000u;
    } else if (bw > 6000000u) {
        reg_0a = 0x10u; reg_0b = 0x2au; int_freq = 4570000u;
    } else if (bw > IF_LPF_BW[0] + FILT_HP_BW1 + FILT_HP_BW2) {
        reg_0a = 0x10u; reg_0b = 0x6bu; int_freq = 3570000u;
    } else {
        reg_0a = 0x00u; reg_0b = 0x80u; int_freq = 2300000u;
        if (bw > IF_LPF_BW[0] + FILT_HP_BW1) {
            bw -= FILT_HP_BW2; int_freq += FILT_HP_BW2; real_bw += FILT_HP_BW2;
        } else {
            reg_0b |= 0x20u;
        }
        if (bw > IF_LPF_BW[0]) {
            bw -= FILT_HP_BW1; int_freq += FILT_HP_BW1; real_bw += FILT_HP_BW1;
        } else {
            reg_0b |= 0x40u;
        }
        i = 0u;
        while (i < 10u) {
            if (bw > IF_LPF_BW[i]) break;
            i += 1u;
        }
        i -= 1u;
        reg_0b |= 15u - i;
        real_bw += IF_LPF_BW[i];
        int_freq -= real_bw / 2u;
    }
    write_reg_mask(0x0au, reg_0a, 0x10u);
    write_reg_mask(0x0bu, reg_0b, 0xefu);
    T_int_freq = int_freq;
    return int_freq;
}

static uint32_t tuner_init(void)
{
    i2c_write(0x05u, INIT_ARRAY, NREGS);        /* init_array */
    if (!set_tv_standard()) return 0u;
    sysfreq_sel();
    return 1u;
}

/* ---------------- RTL2832 GPIO (for the V4 upconverter switch) ------------- */
static void set_gpio_output(uint32_t gpio)
{
    uint32_t g = 1u << gpio, r;
    r = rtl_rd(SYSB, 0x3004u);                  /* GPD */
    rtl_wr(SYSB, 0x3004u, r & ~g, 1u);
    r = rtl_rd(SYSB, 0x3003u);                  /* GPOE */
    rtl_wr(SYSB, 0x3003u, r | g, 1u);
}

static void set_gpio_bit(uint32_t gpio, uint32_t val)
{
    uint32_t g = 1u << gpio, r;
    r = rtl_rd(SYSB, 0x3001u);                  /* GPO */
    rtl_wr(SYSB, 0x3001u, val ? (r | g) : (r & ~g), 1u);
}

static void set_bias_tee_gpio(uint32_t gpio, uint32_t on)
{
    set_gpio_output(gpio);
    set_gpio_bit(gpio, on);
}

/* ---------------- r82xx_set_freq (V4 FM: no upconvert; notch/tracking for FM) - */
static uint32_t set_freq(uint32_t rf)
{
    uint32_t lo = rf + T_int_freq;
    uint32_t open_d, cable_2_in, cable_1_in, air_in;
    uint32_t band;                              /* 0 HF / 1 VHF / 2 UHF */

    set_mux(lo);
    set_vga_gain();
    if (!set_pll(lo) || !T_has_lock) {
        return 0u;                              /* PLL not locked */
    }
    /* V4 notch: open_d 0x00 inside notch band, else 0x08 */
    open_d = ((rf <= 2200000u) || ((85000000u <= rf) && (rf <= 112000000u)) ||
              ((172000000u <= rf) && (rf <= 242000000u))) ? 0x00u : 0x08u;
    write_reg_mask(0x17u, open_d, 0x08u);
    /* ★V4 tuner-input selection (r82xx_set_freq, was missing -> 30x weak signal). */
    if (rf <= 28800000u)       band = 0u;
    else if (rf < 250000000u)  band = 1u;
    else                       band = 2u;
    if (band == 0u) {
        write_reg_mask(0x1au, 0x40u, 0xc3u);    /* bypass tracking filter */
        write_reg(0x1bu, 0x00u);
    }
    cable_2_in = (band == 0u) ? 0x08u : 0x00u;  /* cable 2 = HF input */
    cable_1_in = (band == 1u) ? 0x40u : 0x00u;  /* cable 1 = VHF input */
    air_in     = (band == 2u) ? 0x00u : 0x20u;  /* air = UHF input */
    write_reg_mask(0x06u, cable_2_in, 0x08u);
    set_bias_tee_gpio(5u, cable_2_in ? 0u : 1u); /* upconverter switch */
    write_reg_mask(0x05u, cable_1_in, 0x40u);
    write_reg_mask(0x05u, air_in, 0x20u);
    return 1u;
}

static uint32_t set_gain(uint32_t gain_tenths)
{
    uint32_t k, li = 0u, mi = 0u;
    int32_t total = 0;
    uint8_t data[4];
    if (gain_tenths == 0u) {
        /* tuner AGC (librtlsdr rtlsdr_set_tuner_gain_mode(0)): LNA / Mixer automatic, VGA 16.3 dB */
        write_reg_mask(0x05u, 0x00u, 0x10u);    /* LNA auto on */
        write_reg_mask(0x07u, 0x10u, 0x10u);    /* Mixer auto on */
        set_vga_gain();
        return 0u;
    }
    write_reg_mask(0x05u, 0x10u, 0x10u);        /* LNA auto off */
    write_reg_mask(0x07u, 0x00u, 0x10u);        /* Mixer auto off */
    r82xx_read(data, 4u);                       /* (librtlsdr reads; value unused) */
    set_vga_gain();
    for (k = 0u; k < 15u; k++) {
        if (total >= (int32_t)gain_tenths) break;
        li += 1u; total += LNA_STEPS[li];
        if (total >= (int32_t)gain_tenths) break;
        mi += 1u; total += MIX_STEPS[mi];
    }
    write_reg_mask(0x05u, li, 0x0fu);
    write_reg_mask(0x07u, mi, 0x0fu);
    return (uint32_t)total;
}

/* ---------------- RTL2832 baseband (librtlsdr rtlsdr_init_baseband, absolute) - */
/* ★★table 駆動 (2026-09-19、image を bank A+B に収めるため)。1 entry = 5 byte:
 *   kind (0 rtl_wr USBB / 1 rtl_wr SYSB / 2 demod page0 / 3 demod page1) | len<<4、addr lo, addr hi, val lo, val hi。
 *   ★順序・値は Python 正本 init_baseband と同一 (host test が bit-exact を要求する)。 */
#define BB(kind, len, addr, val) (uint8_t)((kind) | ((len) << 4)), (uint8_t)((addr) & 0xff), (uint8_t)((addr) >> 8), \
                                 (uint8_t)((val) & 0xff), (uint8_t)((val) >> 8)
static const uint8_t BASEBAND_SEQ[] = {
    BB(0, 1, USB_SYSCTL, 0x09), BB(0, 2, USB_EPA_MAXPKT, 0x0002), BB(0, 2, USB_EPA_CTL, 0x1002),
    BB(1, 1, DEMOD_CTL_1, 0x22), BB(1, 1, DEMOD_CTL, 0xe8),
    BB(3, 1, 0x01, 0x14), BB(3, 1, 0x01, 0x10),           /* reset demod */
    BB(3, 1, 0x15, 0x00), BB(3, 2, 0x16, 0x0000),
    BB(3, 1, 0x16, 0x00), BB(3, 1, 0x17, 0x00), BB(3, 1, 0x18, 0x00),
    BB(3, 1, 0x19, 0x00), BB(3, 1, 0x1a, 0x00), BB(3, 1, 0x1b, 0x00),
    /* FIR (FIR_DEFAULT を int8 x8 + int12 x8 に詰めた 20 byte) */
    BB(3, 1, 0x1c, 0xca), BB(3, 1, 0x1d, 0xdc), BB(3, 1, 0x1e, 0xd7), BB(3, 1, 0x1f, 0xd8),
    BB(3, 1, 0x20, 0xe0), BB(3, 1, 0x21, 0xf2), BB(3, 1, 0x22, 0x0e), BB(3, 1, 0x23, 0x35),
    BB(3, 1, 0x24, 0x06), BB(3, 1, 0x25, 0x50), BB(3, 1, 0x26, 0x9c), BB(3, 1, 0x27, 0x0d),
    BB(3, 1, 0x28, 0x71), BB(3, 1, 0x29, 0x11), BB(3, 1, 0x2a, 0x14), BB(3, 1, 0x2b, 0x71),
    BB(3, 1, 0x2c, 0x74), BB(3, 1, 0x2d, 0x19), BB(3, 1, 0x2e, 0x41), BB(3, 1, 0x2f, 0xa5),
    BB(2, 1, 0x19, 0x05),                                 /* SDR mode */
    BB(3, 1, 0x93, 0xf0), BB(3, 1, 0x94, 0x0f),
    BB(3, 1, 0x11, 0x00), BB(3, 1, 0x04, 0x00),
    BB(2, 1, 0x61, 0x60), BB(2, 1, 0x06, 0x80),
    BB(3, 1, 0xb1, 0x1b), BB(2, 1, 0x0d, 0x83),
};

static void init_baseband(void)
{
    uint32_t i;
    for (i = 0u; i < sizeof(BASEBAND_SEQ); i += 5u) {
        const uint8_t *e = &BASEBAND_SEQ[i];
        uint32_t kind = e[0] & 0x0fu, len = e[0] >> 4;
        uint32_t addr = e[1] | ((uint32_t)e[2] << 8), val = e[3] | ((uint32_t)e[4] << 8);
        if (kind < 2u) {
            rtl_wr(kind ? SYSB : USBB, addr, val, len);
        } else {
            demod_wr(kind - 2u, addr, val, len);
        }
    }
}

static void set_sample_rate(uint32_t rate)
{
    /* ratio = (28800000 * (1<<22)) // rate ; ratio &= 0x0ffffffc   (Python / librtlsdr 逐語)
     * ★被除数 28800000 << 22 は 47bit。★RV32I に 64bit 除算を持ち込まないため 32bit の
     *   long division で同じ商を出す: q = XTAL / rate を先に取り、余りに 22 bit を 1 bit ずつ
     *   入れる。★余りは常に < rate < 2^22 なので 32bit に収まり、商も < 2^30 (rate >= 225001)。
     *   ★host test が Python の 64bit 計算と bit 一致を要求する。 */
    uint32_t q = XTAL / rate;
    uint32_t rem = XTAL - q * rate;
    uint32_t i, ratio;
    for (i = 0u; i < 22u; i++) {
        rem <<= 1; q <<= 1;
        if (rem >= rate) {
            rem -= rate; q |= 1u;
        }
    }
    ratio = q & 0x0ffffffcu;
    demod_wr(1u, 0x9fu, (ratio >> 16) & 0xffffu, 2u);
    demod_wr(1u, 0xa1u, ratio & 0xffffu, 2u);
    demod_wr(1u, 0x01u, 0x14u, 1u); demod_wr(1u, 0x01u, 0x10u, 1u);
}

static void set_testmode(uint32_t on)
{
    demod_wr(0u, 0x19u, on ? 0x03u : 0x05u, 1u);   /* rtlsdr_set_testmode */
}

static void reset_buffer(void)
{
    rtl_wr(USBB, USB_EPA_CTL, 0x1002u, 2u);
    rtl_wr(USBB, USB_EPA_CTL, 0x0000u, 2u);
}

/* ==========================================================================
 * ★★entry — pi/rtl_r82xx_live_init.py::main の init 部分と同じ順序。
 * ========================================================================== */
uint32_t rtlsdr_live_set_gain(uint32_t gain_tenths, rtlsdr_result_t *r)
{
    uint32_t g;
    R = r;
    i2c_repeater(1u);
    g = set_gain(gain_tenths);
    i2c_repeater(0u);
    return g;
}

uint32_t rtlsdr_live_set_freq(uint32_t rf_hz, rtlsdr_result_t *r)
{
    uint32_t ok;
    R = r;
    i2c_repeater(1u);
    ok = set_freq(rf_hz);
    i2c_repeater(0u);
    return ok;
}

void rtlsdr_live_init(const rtlsdr_params_t *p, rtlsdr_result_t *r)
{
    uint32_t i;
    R = r;
    r->n_ctrl = 0u; r->n_fail = 0u; r->first_fail = 0u;
    r->init_ok = 0u; r->freq_ok = 0u; r->has_lock = 0u; r->gain_total = 0u;
    r->div_num = 0u; r->vco_fine_tune = 0u; r->fil_cal_code = 0u;
    for (i = 0u; i < 5u; i++) r->last_rd[i] = 0u;
    for (i = 0u; i < NREGS; i++) T_regs[i] = 0u;
    T_has_lock = 0u; T_fil_cal_code = 0u; T_int_freq = IF_FREQ;

    init_baseband();
    if ((p->flags & (RTLSDR_F_SKIP_TUNER | RTLSDR_F_ZERO_IF)) == 0u) {
        /* librtlsdr rtlsdr_open の R82xx 用の設定 (土台はこれを飛ばしていた): R828D は IF 3.57 MHz の
         * low-IF を I 側の ADC だけに出す。Zero-IF のままだと使わない Q 側の ADC も入り、
         * LO が RF より上 (lo = rf + IF) なのでスペクトルが反転する (ISDB-T の SP の並びで見つけた、
         * docs/progress.md TV0)。 */
        demod_wr(1u, 0xb1u, 0x1au, 1u);         /* Zero-IF をやめる */
        demod_wr(0u, 0x08u, 0x4du, 1u);         /* I 側の ADC だけ */
        demod_wr(1u, 0x15u, 0x01u, 1u);         /* スペクトル反転を有効に */
    }
    if ((p->flags & RTLSDR_F_SKIP_TUNER) == 0u) {
        i2c_repeater(1u);
        r->init_ok = tuner_init();
        /* ★rate に合わせて IF filter 帯域 + int_freq を再計算してから周波数を決める
         *   (set_freq の lo = rf + T_int_freq が古い int_freq を使わないように、この順序が必須)。 */
        set_if_freq_dyn(set_bandwidth(p->rate));
        r->freq_ok = set_freq(p->rf_hz);
        r->gain_total = set_gain(p->gain_tenths);
        i2c_repeater(0u);
    } else {
        set_if_freq();                          /* DDC IF = 3.57MHz (skip_tuner は変更なし) */
    }
    set_sample_rate(p->rate);
    if (p->flags & RTLSDR_F_TESTMODE) {
        set_testmode(1u);
    }
    reset_buffer();
}
