# The Elixir side of test/ref/transfers.sh: RTLSDR (lib/rtlsdr.ex) on a mock :usb_host that
# prints every vendor control transfer like test/ref/c/mock_usb.c and answers IN transfers
# by the same rule. Also checks that each SETUP packet is the vendor request it should be.
# usage: elixir -pa <beams> mock_usb_host.exs <seed> <rf_hz> <gain_tenths> <tune_hz>
defmodule MockUSBHost do
  import Bitwise

  def bus_reset, do: :ok
  def phy_reset, do: :ok
  def enumerate, do: {:ok, 2, <<0::144>>}
  def bulk_in_start(_, _, _), do: :ok
  def bulk_in_stop, do: :ok

  def ctrl_in(2, <<0xC0, 0, wv::16-little, wi::16-little, n::16-little>>, n) do
    IO.puts(:io_lib.format("I ~4.16.0b ~4.16.0b ~b", [wv, wi, n]))
    c = :erlang.get(:mock_n_in)
    :erlang.put(:mock_n_in, c + 1)
    seed = :erlang.get(:mock_seed)
    {:ok, :erlang.list_to_binary(for k <- 0..(n - 1)//1, do: c * 37 + k * 11 + seed * 101 + (c >>> 1) * seed &&& 0xFF)}
  end

  def ctrl_in(a, s, n), do: raise("bad ctrl_in #{inspect({a, s, n})}")

  def ctrl_out(2, <<0x40, 0, wv::16-little, wi::16-little, n::16-little>>, data) when byte_size(data) == n do
    hex = for <<b <- data>>, into: "", do: :io_lib.format("~2.16.0b", [b]) |> IO.iodata_to_binary()
    IO.puts(:io_lib.format("O ~4.16.0b ~4.16.0b ~s", [wv, wi, hex]))
    :ok
  end

  def ctrl_out(a, s, d), do: raise("bad ctrl_out #{inspect({a, s, d})}")
end

[seed, rf, g, tune] = Enum.map(System.argv(), &String.to_integer/1)
:erlang.put(:mock_seed, seed)
:erlang.put(:mock_n_in, 0)
:erlang.put(:rtlsdr_usb, MockUSBHost)
:erlang.put(:rtlsdr_addr, 2)
r = RTLSDR.live_init(rf, g)
IO.puts("init n_ctrl=#{r.n_ctrl} n_fail=#{r.n_fail} init_ok=#{r.init_ok} freq_ok=#{r.freq_ok} gain=#{r.gain_total}")
IO.puts("tune #{if RTLSDR.tune(tune), do: 1, else: 0}")
