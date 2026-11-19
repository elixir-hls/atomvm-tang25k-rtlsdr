defmodule AtomVMTang25KRTLSDR.MixProject do
  use Mix.Project

  def project do
    [
      app: :atomvm_tang25k_rtlsdr,
      version: "0.1.0",
      elixir: "~> 1.18",
      elixirc_paths: ["lib"],
      deps: deps(),
      description: "An RTL-SDR driver for Elixir HLS on the Tang Primer 25K, on atomvm-tang25k-usb-host: its IQ into an FPGA pipeline",
      package: [
        licenses: ["Apache-2.0"],
        links: %{"GitHub" => "https://github.com/elixir-hls/atomvm-tang25k-rtlsdr"},
        files: ~w(lib mix.exs README.md LICENSE),
        exclude_patterns: [~r/__pycache__/, ~r/\.pyc$/]
      ]
    ]
  end

  def application, do: []

  defp deps do
    [
      {:atomvm_tang25k_usb_host, "~> 0.1"}
    ]
  end
end
