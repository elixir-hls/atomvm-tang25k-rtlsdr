# atomvm-tang25k-rtlsdr

An RTL-SDR (RTL2832U + R820T / R828D) driver for
[atomvm-tang25k-hls](https://github.com/elixir-hls/atomvm-tang25k-hls) (with [elixir-hls](https://github.com/elixir-hls/elixir-hls)), on the USB
host of [atomvm-tang25k-usb-host](https://github.com/elixir-hls/atomvm-tang25k-usb-host): Elixir
opens and tunes the RTL-SDR, its IQ goes from the host's bulk IN engine into an FPGA pipeline
(`usb_bulk_in()`) without passing through the CPU.

```sh
# the application's hls.env: this package and the USB host
EHLS_BLOCKS=../atomvm-tang25k-usb-host:../atomvm-tang25k-rtlsdr
```

```elixir
use EHLS, blocks: [EHLS.USBHost]

RTLSDR.open(82_500_000, 0)   # bus reset (and a PHY reset if it does not answer), enumeration
                             # (directly or behind a hub), RTL2832U + R82xx init at 960 kS/s;
                             # gain in 0.1 dB, 0 = the tuner's AGC
RTLSDR.start()               # the bulk IN engine on: IQ bytes (I, Q, I, Q, offset binary)
RTLSDR.tune(80_000_000)      # true when the tuner's PLL locked
RTLSDR.gain(402)             # tuner gain while streaming (0 = AGC)
RTLSDR.stats()               # {packets, naks, errors} of the bulk IN engine
RTLSDR.debug()               # the USB host's state (ULPI link, SOF, PHY registers)
RTLSDR.stop()
```

The driver is Elixir (lib/rtlsdr.ex), on the USB host's Elixir API (`:usb_host` of
atomvm-tang25k-usb-host: control transfers for the chips' registers, the bulk IN engine for
the IQ). It sets the RTL2832U up and drives the R82xx tuner through the RTL2832U's I2C, as
librtlsdr does: the tuner's registers are kept in a shadow, and the PLL setup reads the VCO
fine tune and the lock bit back and retries, so the values written depend on the tuner's
answers. The shadow and the device's address live in the process that called `open/2`: tune
and set the gain from that process.

It depends on atomvm-tang25k-usb-host, so the application's hls.env names both (above).

## Checks

`test/ref/transfers.sh` compares the driver with the C driver it replaced (test/ref/c/, the
port of librtlsdr from usb-rtl-sdr-tv): both run on a mock USB that records every vendor
control transfer and answers reads by the same rule, over 200 cases (seeds steering the
lock bit, the VCO fine tune and the filter calibration; FM, HF, VHF and UHF frequencies;
AGC and fixed gains). The transfers (wValue, wIndex, data) and the results must be
identical.

```sh
test/ref/transfers.sh        # gcc and Elixir on the host
```

## Example

`examples/sdr_capture.ex`: `usb_bulk_in()` as an instance's result, a slice of the IQ read back to
memory and summed up.

```sh
cd examples
H=../../atomvm-tang25k-hls/scripts
$H/build_kernel.sh lib/sdr_capture.ex && $H/build_soc.sh && $H/build_atomvm.sh && $H/flash.sh vm lib
$H/build_ex.sh sdr_capture SdrCapture && $H/run.sh sdr_capture
```

## License

Apache-2.0 (see [LICENSE](LICENSE)).
