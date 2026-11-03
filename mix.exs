defmodule AtomVMTang25KRTLSDR.MixProject do
  use Mix.Project

  def project do
    [
      app: :atomvm_tang25k_rtlsdr,
      version: "0.1.0",
      elixir: "~> 1.18",
      elixirc_paths: ["lib"],
      deps: deps()
    ]
  end

  def application, do: []

  defp deps do
    [
      {:atomvm_tang25k_usb_host, git: "https://github.com/elixir-hls/atomvm-tang25k-usb-host.git"}
    ]
  end
end
