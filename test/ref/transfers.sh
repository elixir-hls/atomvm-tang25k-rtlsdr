#!/usr/bin/env bash
# The Elixir driver (lib/rtlsdr.ex) against the C driver it replaces (test/ref/c/): both on a
# mock USB that records every vendor control transfer (wValue, wIndex, data) and answers
# reads by the same rule. The reads steer the driver (the PLL's lock bit, the VCO fine tune,
# the filter calibration code), so each seed takes other branches; every case's transfers,
# init results and tune result must be identical.
# usage: test/ref/transfers.sh [seeds, default 40]
set -euo pipefail
cd "$(dirname "$0")"
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
gcc -O1 -Wall -o "$W/c" c/rtlsdr_live.c c/mock_usb.c
elixirc -o "$W/beams" ../../lib/rtlsdr.ex > /dev/null
n=0
for seed in $(seq 0 $((${1:-40} - 1))); do
    for args in "82500000 0 79500000" "79500000 402 82500000" "24000000 150 433000000" \
                "433000000 0 700000000" "150000000 900 28000000"; do
        "$W/c" $seed $args > "$W/c.txt"
        elixir -pa "$W/beams" mock_usb_host.exs $seed $args > "$W/ex.txt"
        if ! cmp -s "$W/c.txt" "$W/ex.txt"; then
            echo "MISMATCH: seed $seed, rf / gain / tune $args"
            diff "$W/c.txt" "$W/ex.txt" | head -8
            exit 1
        fi
        n=$((n + 1))
    done
done
echo "transfers: $n cases, Elixir = C (the last: $(grep -c '^[IO] ' "$W/c.txt") transfers, $(grep '^init' "$W/c.txt"))"
