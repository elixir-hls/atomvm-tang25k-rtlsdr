# SPDX-License-Identifier: GPL-2.0-or-later
#
# A port of librtlsdr (osmocom rtl-sdr / rtl-sdr-blog: librtlsdr.c, tuner_r82xx.c), and so under its
# license, the GNU General Public License version 2 or (at your option) any later version (LICENSE).
# The code it was ported from is
#   Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
#   Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>              (librtlsdr.c)
#   Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
#   Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>                   (tuner_r82xx.c)
#
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.

defmodule RTLSDR do
  @moduledoc """
  An RTL-SDR (RTL2832U + R820T / R828D) driver in Elixir, on the USB host of
  atomvm-tang25k-usb-host (`:usb_host`). Elixir opens and tunes the device; once started,
  its IQ goes from the host's bulk IN engine into the pipeline's `usb_bulk_in()` without
  passing through the CPU.

      RTLSDR.open(82_500_000, 0)   # bus reset, enumeration, RTL2832U + R82xx init at 960 kS/s
      RTLSDR.start()               # IQ bytes (I, Q, I, Q, offset binary) into the pipeline
      RTLSDR.tune(80_000_000)      # true when the tuner's PLL locked
      RTLSDR.stats()               # {packets, naks, errors} of the bulk IN engine
      RTLSDR.debug()               # the USB host's state (`:usb_host.debug/0`)
      RTLSDR.stop()

  The tuner driver follows librtlsdr (rtlsdr.c, tuner_r82xx.c) as the earlier C port of
  usb-rtl-sdr-tv did, transfer for transfer (test/ref/transfers.sh compares the two): the
  R82xx registers are kept in a shadow, and the PLL setup reads the VCO fine tune and the
  lock bit back and retries, so the values written depend on what the tuner answers.

  The shadow and the device's address live in the calling process (its dictionary): open,
  tune and gain from one process.
  """
  import Bitwise

  @rate 960_000
  @iq_ep 1

  # librtlsdr's constants
  @usbb 1
  @sysb 2
  @iicb 6
  @usb_sysctl 0x2000
  @usb_epa_ctl 0x2148
  @usb_epa_maxpkt 0x2158
  @demod_ctl 0x3000
  @demod_ctl_1 0x300B
  @xtal 28_800_000
  @i2c_addr 0x74
  @max_i2c 8
  @ver_num 49
  @vco_power_ref 1
  @nregs 27
  @shadow_start 5

  # r82xx_init_array: regs 0x05..0x1f
  @init_array [0x83, 0x30, 0x75, 0xC0, 0x40, 0xD6, 0x6C, 0xF5, 0x63, 0x75, 0x68, 0x6C, 0x83,
               0x80, 0x00, 0x0F, 0x00, 0xC0, 0x30, 0x48, 0xCC, 0x60, 0x00, 0x54, 0xAE, 0x4A,
               0xC0]
  # freq_ranges (the fields used): start MHz, open_d, rf_mux_ploy, tf_c (xtal_cap0p all 0)
  @range_mhz {0, 50, 55, 60, 65, 70, 75, 80, 90, 100, 110, 120, 140, 180, 220, 250, 280, 310,
              450, 588, 650}
  @range_open_d {0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  @range_rf_mux {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
                 0x02, 0x02, 0x02, 0x02, 0x41, 0x41, 0x40, 0x40}
  @range_tf_c {0xDF, 0xBE, 0x8B, 0x7B, 0x69, 0x58, 0x44, 0x44, 0x34, 0x34, 0x24, 0x24, 0x14,
               0x13, 0x13, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00}
  @bitrev_lut {0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE, 0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF}
  @lna_steps {0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13}
  @mix_steps {0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8}
  @if_lpf_bw {1_700_000, 1_600_000, 1_550_000, 1_450_000, 1_200_000, 900_000, 700_000,
              550_000, 450_000, 350_000}
  @filt_hp_bw1 350_000
  @filt_hp_bw2 380_000

  # rtlsdr_init_baseband: {kind (:usb / :sys: rtl_wr, 0 / 1: demod page), len, addr, val}
  @baseband [
    {:usb, 1, @usb_sysctl, 0x09}, {:usb, 2, @usb_epa_maxpkt, 0x0002},
    {:usb, 2, @usb_epa_ctl, 0x1002}, {:sys, 1, @demod_ctl_1, 0x22}, {:sys, 1, @demod_ctl, 0xE8},
    {1, 1, 0x01, 0x14}, {1, 1, 0x01, 0x10},
    {1, 1, 0x15, 0x00}, {1, 2, 0x16, 0x0000},
    {1, 1, 0x16, 0x00}, {1, 1, 0x17, 0x00}, {1, 1, 0x18, 0x00},
    {1, 1, 0x19, 0x00}, {1, 1, 0x1A, 0x00}, {1, 1, 0x1B, 0x00},
    # FIR (FIR_DEFAULT packed as librtlsdr does: int8 x8 + int12 x8, 20 bytes)
    {1, 1, 0x1C, 0xCA}, {1, 1, 0x1D, 0xDC}, {1, 1, 0x1E, 0xD7}, {1, 1, 0x1F, 0xD8},
    {1, 1, 0x20, 0xE0}, {1, 1, 0x21, 0xF2}, {1, 1, 0x22, 0x0E}, {1, 1, 0x23, 0x35},
    {1, 1, 0x24, 0x06}, {1, 1, 0x25, 0x50}, {1, 1, 0x26, 0x9C}, {1, 1, 0x27, 0x0D},
    {1, 1, 0x28, 0x71}, {1, 1, 0x29, 0x11}, {1, 1, 0x2A, 0x14}, {1, 1, 0x2B, 0x71},
    {1, 1, 0x2C, 0x74}, {1, 1, 0x2D, 0x19}, {1, 1, 0x2E, 0x41}, {1, 1, 0x2F, 0xA5},
    {0, 1, 0x19, 0x05},
    {1, 1, 0x93, 0xF0}, {1, 1, 0x94, 0x0F},
    {1, 1, 0x11, 0x00}, {1, 1, 0x04, 0x00},
    {0, 1, 0x61, 0x60}, {0, 1, 0x06, 0x80},
    {1, 1, 0xB1, 0x1B}, {0, 1, 0x0D, 0x83}
  ]

  # -- the API --

  @doc """
  Bus reset (and a PHY reset if it does not answer), enumeration (directly or behind a hub),
  RTL2832U + R82xx init at `freq` Hz, 960 kS/s zero-IF IQ. `gain`: tuner gain in 0.1 dB,
  0 = the tuner's AGC.
  """
  def open(freq, gain) do
    usb = usb()
    usb.bulk_in_stop()

    with :ok <- reset(usb),
         {:ok, addr, _dev} <- enumerate(usb) do
      :erlang.put(:rtlsdr_addr, addr)
      r = live_init(freq, gain)
      if r.n_fail > 0 or r.freq_ok == 0, do: {:error, {:init, r.n_fail}}, else: :ok
    end
  end

  @doc "The bulk IN engine on: the IQ goes into the pipeline."
  def start, do: usb().bulk_in_start(addr(), @iq_ep, false)

  @doc "The bulk IN engine off."
  def stop, do: usb().bulk_in_stop()

  @doc "Retune while streaming; true when the PLL locked."
  def tune(freq) do
    i2c_repeater(1)
    ok = set_freq(freq)
    i2c_repeater(0)
    ok == 1
  end

  @doc "Tuner gain while streaming (0.1 dB, 0 = AGC); returns the gain set."
  def gain(g) do
    i2c_repeater(1)
    total = set_gain(g)
    i2c_repeater(0)
    total
  end

  @doc "{packets, naks, errors} of the bulk IN engine."
  def stats, do: usb().bulk_in_stats()

  @doc "The USB host's state (`:usb_host.debug/0`)."
  def debug, do: usb().debug()

  # -- the USB host (a module with :usb_host's functions; tests put a mock in :rtlsdr_usb) --

  defp usb do
    case :erlang.get(:rtlsdr_usb) do
      :undefined -> :usb_host
      m -> m
    end
  end

  defp addr, do: :erlang.get(:rtlsdr_addr)

  defp reset(usb) do
    case usb.bus_reset() do
      :ok ->
        :ok

      {:error, _} ->
        usb.phy_reset()

        case usb.bus_reset() do
          :ok -> :ok
          {:error, e} -> {:error, {:reset, e}}
        end
    end
  end

  defp enumerate(usb) do
    case usb.enumerate() do
      {:ok, _, _} = ok -> ok
      {:error, step} -> {:error, {:enumerate, step}}
    end
  end

  # vendor control transfers (bRequest 0): IN = bmRequestType 0xC0, OUT = 0x40; 1 = success
  defp setup(bm, wvalue, windex, n),
    do: <<bm, 0, wvalue::16-little, windex::16-little, n::16-little>>

  defp ctrl_in(wvalue, windex, n) do
    case usb().ctrl_in(addr(), setup(0xC0, wvalue, windex, n), n) do
      {:ok, d} when byte_size(d) == n -> {1, d}
      _ -> {0, :erlang.list_to_binary(:lists.duplicate(n, 0))}
    end
  end

  # a failed OUT is retried after a vendor read (page 0x0a reg 0x01, librtlsdr's dummy
  # read): a vendor read clears the state in which the RTL2832U stalls vendor writes
  defp ctrl_out(wvalue, windex, data), do: ctrl_out(wvalue, windex, data, 0)

  defp ctrl_out(_wvalue, _windex, _data, 3), do: 0

  defp ctrl_out(wvalue, windex, data, t) do
    if t > 0, do: ctrl_in(0x0120, 0x0A, 1)

    case usb().ctrl_out(addr(), setup(0x40, wvalue, windex, byte_size(data)), data) do
      :ok -> 1
      _ -> ctrl_out(wvalue, windex, data, t + 1)
    end
  end

  # every transfer of the driver goes through here (counted)
  defp tally(ok) do
    :erlang.put(:rtlsdr_n_ctrl, :erlang.get(:rtlsdr_n_ctrl) + 1)
    if ok == 0, do: :erlang.put(:rtlsdr_n_fail, :erlang.get(:rtlsdr_n_fail) + 1)
    ok
  end

  # -- RTL2832 registers --

  defp bytes(val, 1), do: <<val &&& 0xFF>>
  defp bytes(val, 2), do: <<val >>> 8 &&& 0xFF, val &&& 0xFF>>

  defp rtl_wr(block, a, val, ln), do: tally(ctrl_out(a, block <<< 8 ||| 0x10, bytes(val, ln)))

  defp rtl_rd(block, a) do
    {ok, <<v>>} = ctrl_in(a, block <<< 8, 1)
    tally(ok)
    v
  end

  # a demod write, then librtlsdr's trailing dummy read (not counted)
  defp demod_wr(page, a, val, ln) do
    tally(ctrl_out(a <<< 8 ||| 0x20, 0x10 ||| page, bytes(val, ln)))
    ctrl_in(0x0120, 0x0A, 1)
  end

  defp i2c_repeater(on), do: demod_wr(1, 0x01, if(on == 1, do: 0x18, else: 0x10), 1)

  # set_if_freq: iff = -(freq << 22 // XTAL) & 0x3fffff
  defp set_if_freq(freq) do
    iff = -div(freq <<< 22, @xtal) &&& 0x3FFFFF
    demod_wr(1, 0x19, iff >>> 16 &&& 0x3F, 1)
    demod_wr(1, 0x1A, iff >>> 8 &&& 0xFF, 1)
    demod_wr(1, 0x1B, iff &&& 0xFF, 1)
  end

  # -- R82xx over I2C, with the shadow of regs 0x05..0x1f --

  defp regs, do: :erlang.get(:rtlsdr_regs)

  defp i2c_write(reg, vals) do
    shadow(reg, vals)
    i2c_chunks(reg, vals)
  end

  defp shadow(_reg, []), do: :ok

  defp shadow(reg, [v | rest]) do
    i = reg - @shadow_start
    if i >= 0 and i < @nregs, do: :erlang.put(:rtlsdr_regs, put_elem(regs(), i, v))
    shadow(reg + 1, rest)
  end

  # at most MAX_I2C - 1 values per transfer, after the register number
  defp i2c_chunks(_reg, []), do: :ok

  defp i2c_chunks(reg, vals) do
    {now, rest} = split(vals, @max_i2c - 1, [])
    tally(ctrl_out(@i2c_addr, @iicb <<< 8 ||| 0x10, :erlang.list_to_binary([reg | now])))
    i2c_chunks(reg + length(now), rest)
  end

  defp split(rest, 0, acc), do: {:lists.reverse(acc), rest}
  defp split([], _n, acc), do: {:lists.reverse(acc), []}
  defp split([v | rest], n, acc), do: split(rest, n - 1, [v | acc])

  defp write_reg(reg, val), do: i2c_write(reg, [val &&& 0xFF])

  defp cache(reg) do
    i = reg - @shadow_start
    if i >= 0 and i < @nregs, do: elem(regs(), i), else: 0
  end

  defp write_reg_mask(reg, val, mask),
    do: write_reg(reg, (cache(reg) &&& bnot(mask)) ||| (val &&& mask))

  defp wrm([]), do: :ok

  defp wrm([{reg, val, mask} | rest]) do
    write_reg_mask(reg, val, mask)
    wrm(rest)
  end

  # r82xx_read from reg 0: n bytes, bit-reversed
  defp r82xx_read(n) do
    tally(ctrl_out(@i2c_addr, @iicb <<< 8 ||| 0x10, <<0>>))
    {ok, d} = ctrl_in(@i2c_addr, @iicb <<< 8, n)
    tally(ok)
    for b <- :erlang.binary_to_list(d), do: bitrev(b)
  end

  defp bitrev(b), do: elem(@bitrev_lut, b &&& 0xF) <<< 4 ||| elem(@bitrev_lut, b >>> 4 &&& 0xF)

  # -- r82xx_set_pll: reads the VCO fine tune and the lock bit, retries with more current --

  defp set_pll(freq) do
    freq_khz = div(freq + 500, 1000)
    pll_ref_khz = div(@xtal + 500, 1000)
    # refdiv2 = 0, PLL autotune 128 kHz, VCO current max (RTL-SDR Blog)
    wrm([{0x10, 0x00, 0x10}, {0x1A, 0x00, 0x0C}, {0x12, 0x06, 0xFF}])
    {mix_div, div_num} = mix_div(freq_khz, 2)
    [_, _, _, _, d4] = r82xx_read(5)
    vco_fine_tune = (d4 &&& 0x30) >>> 4

    div_num =
      cond do
        vco_fine_tune > @vco_power_ref -> div_num - 1 &&& 0xFFFFFFFF
        vco_fine_tune < @vco_power_ref -> div_num + 1
        true -> div_num
      end

    write_reg_mask(0x10, div_num <<< 5 &&& 0xFF, 0xE0)
    vco_freq = freq * mix_div &&& 0xFFFFFFFF
    nint = div(vco_freq, 2 * @xtal)
    vco_fra = div(vco_freq - 2 * @xtal * nint, 1000)

    if nint > div(128, @vco_power_ref) - 1 do
      # no valid PLL
      0
    else
      ni = div(nint - 13 &&& 0xFFFFFFFF, 4)
      si = nint - 4 * ni - 13 &&& 0xFFFFFFFF
      write_reg(0x14, ni + (si <<< 6))
      write_reg_mask(0x12, if(vco_fra == 0, do: 0x08, else: 0x00), 0x08)
      sdm = sdm(vco_fra, 2, 0, pll_ref_khz)
      write_reg(0x16, sdm >>> 8)
      write_reg(0x15, sdm &&& 0xFF)
      :erlang.put(:rtlsdr_has_lock, 0)
      lock(0)
    end
  end

  # the mixer divider putting the VCO in 1.77..3.54 GHz, and its log2 - 1
  defp mix_div(freq_khz, m) when m <= 64 do
    if 1_770_000 <= freq_khz * m and freq_khz * m < 3_540_000,
      do: {m, log2(m) - 1},
      else: mix_div(freq_khz, m * 2)
  end

  defp mix_div(_freq_khz, m), do: {m, 0}

  defp log2(1), do: 0
  defp log2(m), do: 1 + log2(m >>> 1)

  defp sdm(vco_fra, n_sdm, sdm, ref_khz) when vco_fra > 1 do
    step = div(2 * ref_khz, n_sdm)

    if vco_fra > step do
      sdm = sdm + div(32768, div(n_sdm, 2))
      if n_sdm >= 0x8000, do: sdm, else: sdm(vco_fra - step, n_sdm <<< 1, sdm, ref_khz)
    else
      sdm(vco_fra, n_sdm <<< 1, sdm, ref_khz)
    end
  end

  defp sdm(_vco_fra, _n_sdm, sdm, _ref_khz), do: sdm

  # two tries for the lock bit; after the first, max VCO current (RTL-SDR Blog hack)
  defp lock(2), do: 0

  defp lock(i) do
    :timer.sleep(10)
    [_, _, d2] = r82xx_read(3)

    if (d2 &&& 0x40) != 0 do
      :erlang.put(:rtlsdr_has_lock, 1)
      1
    else
      if i == 0, do: write_reg_mask(0x12, 0x06, 0xFF)
      lock(i + 1)
    end
  end

  # -- r82xx_set_mux --

  defp set_mux(freq) do
    rng = range(div(freq, 1_000_000), 0)
    write_reg_mask(0x17, elem(@range_open_d, rng), 0x08)
    write_reg_mask(0x1A, elem(@range_rf_mux, rng), 0xC3)
    write_reg(0x1B, elem(@range_tf_c, rng))
    wrm([{0x10, 0x00, 0x0B}, {0x08, 0x00, 0x3F}, {0x09, 0x00, 0x3F}])
  end

  defp range(_fmhz, 20), do: 20
  defp range(fmhz, j), do: if(fmhz < elem(@range_mhz, j + 1), do: j, else: range(fmhz, j + 1))

  # -- r82xx_set_tv_standard: the filter calibration reads its code --

  defp set_tv_standard do
    :erlang.put(:rtlsdr_regs, :erlang.list_to_tuple(@init_array))
    wrm([{0x0C, 0x00, 0x0F}, {0x13, @ver_num, 0x3F}, {0x1D, 0x00, 0x38}])
    :erlang.put(:rtlsdr_int_freq, 3_570_000)

    case filter_cal(0) do
      :fail ->
        0

      code ->
        code = if code == 0x0F, do: 0, else: code
        write_reg_mask(0x0A, 0x10 ||| code, 0x1F)
        # hp_cor, img_r, filt_gain, ext_enable, loop_through, lt_att, flt_ext_widest, polyfil_cur
        wrm([{0x0B, 0x6B, 0xEF}, {0x07, 0x00, 0x80}, {0x06, 0x30, 0x30}, {0x1E, 0x60, 0x60},
             {0x05, 0x80, 0x80}, {0x1F, 0x00, 0x80}, {0x0F, 0x00, 0x80}, {0x19, 0x60, 0x60}])
        1
    end
  end

  defp filter_cal(i) do
    # cali clk on, xtal cap 0 pF
    wrm([{0x0B, 0x6B, 0x60}, {0x0F, 0x04, 0x04}, {0x10, 0x00, 0x03}])

    if set_pll(56_000 * 1000) == 0 or :erlang.get(:rtlsdr_has_lock) == 0 do
      :fail
    else
      # start, stop the trigger, clk off
      wrm([{0x0B, 0x10, 0x10}, {0x0B, 0x00, 0x10}, {0x0F, 0x00, 0x04}])
      [_, _, _, _, d4] = r82xx_read(5)
      code = d4 &&& 0x0F
      if (code != 0 and code != 0x0F) or i == 1, do: code, else: filter_cal(i + 1)
    end
  end

  # -- r82xx_sysfreq_sel (the DVB-T default branch), the VGA --

  defp sysfreq_sel do
    wrm([{0x1D, 0xE5, 0xC7}, {0x1C, 0x24, 0xF8}, {0x0D, 0x53, 0xFF}, {0x0E, 0x75, 0xFF},
         {0x05, 0x00, 0x60}, {0x06, 0x00, 0x08}, {0x11, 0x38, 0x38}, {0x17, 0xA0, 0x30},
         {0x0A, 0x40, 0x60},
         # LNA
         {0x1D, 0x00, 0x38}, {0x1C, 0x00, 0x04}, {0x06, 0x00, 0x40}, {0x1A, 0x30, 0x30},
         {0x1D, 0x18, 0x38}, {0x1C, 0x24, 0x04}, {0x1E, 14, 0x1F}, {0x1A, 0x20, 0x30}])
  end

  defp set_vga_gain, do: write_reg_mask(0x0C, 0x08, 0x9F)

  # -- r82xx_set_bandwidth: the IF filter for the sample rate, and the IF --

  defp set_bandwidth(bw) do
    {reg_0a, reg_0b, int_freq} =
      cond do
        bw > 7_000_000 -> {0x10, 0x0B, 4_570_000}
        bw > 6_000_000 -> {0x10, 0x2A, 4_570_000}
        bw > elem(@if_lpf_bw, 0) + @filt_hp_bw1 + @filt_hp_bw2 -> {0x10, 0x6B, 3_570_000}
        true -> narrow(bw)
      end

    write_reg_mask(0x0A, reg_0a, 0x10)
    write_reg_mask(0x0B, reg_0b, 0xEF)
    :erlang.put(:rtlsdr_int_freq, int_freq)
    int_freq
  end

  defp narrow(bw) do
    lpf0 = elem(@if_lpf_bw, 0)

    {bw, int_freq, real_bw, reg_0b} =
      if bw > lpf0 + @filt_hp_bw1,
        do: {bw - @filt_hp_bw2, 2_300_000 + @filt_hp_bw2, @filt_hp_bw2, 0x80},
        else: {bw, 2_300_000, 0, 0x80 ||| 0x20}

    {bw, int_freq, real_bw, reg_0b} =
      if bw > lpf0,
        do: {bw - @filt_hp_bw1, int_freq + @filt_hp_bw1, real_bw + @filt_hp_bw1, reg_0b},
        else: {bw, int_freq, real_bw, reg_0b ||| 0x40}

    i = lpf_index(bw, 0) - 1
    real_bw = real_bw + elem(@if_lpf_bw, i)
    {0x00, reg_0b ||| 15 - i, int_freq - div(real_bw, 2)}
  end

  defp lpf_index(_bw, 10), do: 10
  defp lpf_index(bw, i), do: if(bw > elem(@if_lpf_bw, i), do: i, else: lpf_index(bw, i + 1))

  defp tuner_init do
    i2c_write(0x05, @init_array)

    if set_tv_standard() == 0 do
      0
    else
      sysfreq_sel()
      1
    end
  end

  # -- RTL2832 GPIO (the V4's upconverter switch) --

  defp set_bias_tee_gpio(gpio, on) do
    g = 1 <<< gpio
    rtl_wr(@sysb, 0x3004, rtl_rd(@sysb, 0x3004) &&& bnot(g), 1)
    rtl_wr(@sysb, 0x3003, rtl_rd(@sysb, 0x3003) ||| g, 1)
    r = rtl_rd(@sysb, 0x3001)
    rtl_wr(@sysb, 0x3001, if(on == 1, do: r ||| g, else: r &&& bnot(g)), 1)
  end

  # -- r82xx_set_freq (the V4: no upconverter for FM; notch, tracking filter, input) --

  defp set_freq(rf) do
    lo = rf + :erlang.get(:rtlsdr_int_freq)
    set_mux(lo)
    set_vga_gain()

    if set_pll(lo) == 0 or :erlang.get(:rtlsdr_has_lock) == 0 do
      0
    else
      notch = rf <= 2_200_000 or (85_000_000 <= rf and rf <= 112_000_000) or
                (172_000_000 <= rf and rf <= 242_000_000)

      write_reg_mask(0x17, if(notch, do: 0x00, else: 0x08), 0x08)
      band = cond do
        rf <= 28_800_000 -> :hf
        rf < 250_000_000 -> :vhf
        true -> :uhf
      end

      if band == :hf do
        write_reg_mask(0x1A, 0x40, 0xC3)
        write_reg(0x1B, 0x00)
      end

      cable_2_in = if band == :hf, do: 0x08, else: 0x00
      write_reg_mask(0x06, cable_2_in, 0x08)
      set_bias_tee_gpio(5, if(cable_2_in != 0, do: 0, else: 1))
      write_reg_mask(0x05, if(band == :vhf, do: 0x40, else: 0x00), 0x40)
      write_reg_mask(0x05, if(band == :uhf, do: 0x00, else: 0x20), 0x20)
      1
    end
  end

  # -- the gain: 0 = AGC (LNA and mixer automatic), else LNA and mixer steps --

  defp set_gain(0) do
    write_reg_mask(0x05, 0x00, 0x10)
    write_reg_mask(0x07, 0x10, 0x10)
    set_vga_gain()
    0
  end

  defp set_gain(g) do
    write_reg_mask(0x05, 0x10, 0x10)
    write_reg_mask(0x07, 0x00, 0x10)
    # librtlsdr reads here; the value is unused
    r82xx_read(4)
    set_vga_gain()
    {li, mi, total} = gain_steps(g, 0, 0, 0, 0)
    write_reg_mask(0x05, li, 0x0F)
    write_reg_mask(0x07, mi, 0x0F)
    total
  end

  defp gain_steps(_g, 15, li, mi, total), do: {li, mi, total}

  defp gain_steps(g, k, li, mi, total) do
    cond do
      total >= g ->
        {li, mi, total}

      total + elem(@lna_steps, li + 1) >= g ->
        {li + 1, mi, total + elem(@lna_steps, li + 1)}

      true ->
        total = total + elem(@lna_steps, li + 1)
        gain_steps(g, k + 1, li + 1, mi + 1, total + elem(@mix_steps, mi + 1))
    end
  end

  # -- RTL2832 baseband, sample rate, buffer --

  defp init_baseband([]), do: :ok

  defp init_baseband([{kind, len, a, val} | rest]) do
    case kind do
      :usb -> rtl_wr(@usbb, a, val, len)
      :sys -> rtl_wr(@sysb, a, val, len)
      page -> demod_wr(page, a, val, len)
    end

    init_baseband(rest)
  end

  defp set_sample_rate(rate) do
    ratio = div(@xtal <<< 22, rate) &&& 0x0FFFFFFC
    demod_wr(1, 0x9F, ratio >>> 16 &&& 0xFFFF, 2)
    demod_wr(1, 0xA1, ratio &&& 0xFFFF, 2)
    demod_wr(1, 0x01, 0x14, 1)
    demod_wr(1, 0x01, 0x10, 1)
  end

  defp reset_buffer do
    rtl_wr(@usbb, @usb_epa_ctl, 0x1002, 2)
    rtl_wr(@usbb, @usb_epa_ctl, 0x0000, 2)
  end

  # -- the init: librtlsdr's order (zero-IF, as the FM radio uses it) --

  @doc false
  def live_init(rf, g) do
    :erlang.put(:rtlsdr_n_ctrl, 0)
    :erlang.put(:rtlsdr_n_fail, 0)
    :erlang.put(:rtlsdr_regs, :erlang.make_tuple(@nregs, 0))
    :erlang.put(:rtlsdr_has_lock, 0)
    :erlang.put(:rtlsdr_int_freq, 3_570_000)
    init_baseband(@baseband)
    i2c_repeater(1)
    init_ok = tuner_init()
    # the IF filter for the rate first: set_freq's lo = rf + int_freq uses its IF
    set_if_freq(set_bandwidth(@rate))
    freq_ok = set_freq(rf)
    gain_total = set_gain(g)
    i2c_repeater(0)
    set_sample_rate(@rate)
    reset_buffer()

    %{n_ctrl: :erlang.get(:rtlsdr_n_ctrl), n_fail: :erlang.get(:rtlsdr_n_fail),
      init_ok: init_ok, freq_ok: freq_ok, gain_total: gain_total}
  end
end
