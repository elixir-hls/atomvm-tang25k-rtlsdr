defmodule SdrCapture do
  @moduledoc """
  P1: the RTL-SDR's IQ flowing into the FPGA. Elixir opens and tunes the RTL-SDR
  through NIFs; the USB host's bulk IN goes into the FPGA; a slice of it is read back to check
  that it looks like IQ.
  """
  use EHLS, blocks: [EHLS.USBHost]
  import Bitwise

  definstance iq, overflow: :drop do
    usb_bulk_in()
  end

  def start(freq \\ 82_500_000) do
    IO.puts("rtlsdr: open at #{freq} Hz: #{inspect(RTLSDR.open(freq, 0))}")
    :ok = RTLSDR.start()
    :timer.sleep(200)
    {p0, _, _} = RTLSDR.stats()
    :timer.sleep(1000)
    {p1, naks, errs} = RTLSDR.stats()
    IO.puts("rtlsdr: #{p1 - p0} packets in 1 s (#{(p1 - p0) * 512} bytes), naks #{naks}, errors #{errs}")
    bin = iq(:out, 4096 * 4)
    bytes = for <<w::little-32 <- bin>>, do: w &&& 0xFF
    {is, qs} = split_iq(bytes, [], [])
    IO.puts("I: #{describe(is)}")
    IO.puts("Q: #{describe(qs)}")
    RTLSDR.stop()
    :ok
  end

  defp split_iq([i, q | rest], is, qs), do: split_iq(rest, [i | is], [q | qs])
  defp split_iq(_, is, qs), do: {is, qs}

  # mean and RMS around it (unsigned bytes, centre 127.5), min / max
  defp describe(xs) do
    n = length(xs)
    sum = Enum.reduce(xs, 0, &(&1 + &2))
    sq = Enum.reduce(xs, 0, fn x, acc -> acc + (x * 2 - 255) * (x * 2 - 255) end)
    "n=#{n} mean=#{div(sum * 10, n) / 10} rms=#{round(:math.sqrt(sq / n) * 5) / 10} " <>
      "min=#{:lists.min(xs)} max=#{:lists.max(xs)}"
  end
end
