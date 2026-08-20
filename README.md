# embarch-dev-bench

Zephyr-based C firmware for the EmbArch dev-bench: the physical rig that plays
a DUT's BLE counterpart (advertise/connect/GATT exchange) and samples power
during a `Study`. See
[embarch-doc/embarch-dev-bench/design.md](https://github.com/gabrieltetar/embarch-doc/blob/main/embarch-dev-bench/design.md)
for the full architecture and design decisions this is a mechanical
translation of — this README only covers building it.

One shared application (`app/`) built three ways, from three independent west
workspaces:

- `workspaces/native_sim/` — vanilla Zephyr, runs as a native Linux process.
  No hardware required; BLE is a canned-outcome stub (`ble_bridge_stub.c`).
- `workspaces/nordic/` — nRF Connect SDK (NCS), targets the nRF54L15DK
  (`ble_bridge_real.c`, real Zephyr BT host calls). Currently inactive —
  Milestone 2's target board moved to the ESP32-C5 below
  (`embarch-decision-reversals.md` row 13); this workspace stays in the tree,
  built but unflashed, for a future re-run.
- `workspaces/espressif/` — vanilla Zephyr, targets the real
  ESP32-C5-WROOM-1 DK (`ble_bridge_real.c`, same real Zephyr BT host calls as
  `nordic`) — Milestone 2's actual target board.

## Prerequisites

Standard Zephyr toolchain setup — see
[Zephyr's Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
(`west`, CMake, Ninja, the Zephyr SDK, and — for `workspaces/nordic` only —
NCS's `nrfutil`/`nrf-command-line-tools`). `workspaces/native_sim` only needs
a host C toolchain (gcc/clang) alongside the above; no cross-compiler, no
embedded hardware. `workspaces/espressif` needs the Zephyr SDK's
`riscv64-zephyr-elf` toolchain (covers the ESP32-C5's RV32IMAC core via
multilib) — no separate ESP-IDF toolchain install, since flashing goes
through `embarch-core`'s own ESP-JTAG support (`embarch-core/design.md` §3
decision 18), not `west flash`/`esptool`'s UART-bootloader path.

## Building: native_sim

```sh
cd workspaces/native_sim
west init -l manifest
west update
west build -b native_sim app
./build/zephyr/zephyr.exe
```

Run the serial-protocol unit tests instead of (or alongside) the main app:

```sh
west build -b native_sim ../../app/tests/serial_protocol
./build/zephyr/zephyr.exe
# or, across every test in app/tests/:
west twister -p native_sim -T ../../app/tests
```

## Building: nordic (nRF54L15DK)

```sh
cd workspaces/nordic
west init -l manifest
west update
west build -b nrf54l15dk/nrf54l15/cpuapp app
west flash
```

The `manifest/west.yml` pin (NCS version) hasn't been validated against real
hardware yet — see the design doc's changelog and §4 open items before
assuming a clean `west update` on the first try.

## Building: espressif (ESP32-C5-WROOM-1 DK)

```sh
cd workspaces/espressif
west init -l manifest
west update
west build -b esp32c5_devkitc/esp32c5/hpcore app
```

This produces `build/zephyr/zephyr.bin` — flash it via `embarch-core`'s
`POST /flash` (`format: "bin"`, `base_address: "0x2000"`, the address
Zephyr's own build merges to), not `west flash`; decision 13's manual-flash
default is reversed for this board (design doc decision 13/18). Port
selection has no SEGGER J-Link to auto-detect against (this board enumerates
as a plain USB Serial/JTAG device, no VCOM) — use `EMBARCH_DEV_BENCH_PORT`
(design doc decision 26).

## Repo layout

See design doc §2 for the full annotated tree. Short version: `app/` is the
one shared, vendor-agnostic C application; each `workspaces/<vendor>/`
directory is an independent west topdir with its own manifest, symlinking in
`app/` rather than copying it.
