# embarch-dev-bench

Zephyr-based C firmware for the EmbArch dev-bench: the physical rig that plays
a DUT's BLE counterpart (advertise/connect/GATT exchange) and samples power
during a `Study`. See
[embarch-doc/embarch-dev-bench/spec.md](https://github.com/gabrieltetar/embarch-doc/blob/main/embarch-dev-bench/spec.md)
for what is true now and
[decisions.md](https://github.com/gabrieltetar/embarch-doc/blob/main/embarch-dev-bench/decisions.md)
for why — this README only covers building it.

One shared application (`app/`) built three ways, from three independent west
workspaces:

- `workspaces/native_sim/` — vanilla Zephyr, runs as a native Linux process.
  No hardware required; BLE is a canned-outcome stub (`ble_bridge_stub.c`).
- `workspaces/nordic/` — nRF Connect SDK (NCS), targets the nRF54L15DK
  (`ble_bridge_real.c`, real Zephyr BT host calls). **The current bench**
  (decision 43).
- `workspaces/espressif/` — vanilla Zephyr, targets the real
  ESP32-C5-WROOM-1 DK (`ble_bridge_real.c`, same real Zephyr BT host calls as
  `nordic`). Stood in as the bench for one milestone; stays in the tree and
  working, but not currently flashed (decision 43).

## Prerequisites

Standard Zephyr toolchain setup — see
[Zephyr's Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
(`west`, CMake, Ninja, the Zephyr SDK, and — for `workspaces/nordic` only —
NCS's `nrfutil`/`nrf-command-line-tools`). `workspaces/native_sim` only needs
a host C toolchain (gcc/clang) alongside the above; no cross-compiler, no
embedded hardware. `workspaces/espressif` needs the Zephyr SDK's
`riscv64-zephyr-elf` toolchain (covers the ESP32-C5's RV32IMAC core via
multilib) — no separate ESP-IDF toolchain install, since flashing goes
through `embarch-core`'s own ESP-JTAG support, not `west flash`/`esptool`'s
UART-bootloader path
([embarch-core decisions.md](https://github.com/gabrieltetar/embarch-doc/blob/main/embarch-core/decisions.md)
decision 18).

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

## Building: nordic (nRF54L15DK) — the current bench

```sh
cd workspaces/nordic
west init -l manifest
west update
west build -b nrf54l15dk/nrf54l15/cpuapp app
west flash
```

**Enrol this board with `link_port_interface = 2`** before running a study
through `embarch-core`. The DK's onboard J-Link exposes two VCOMs under one
USB serial, and this DK's console is wired to the second one (VCOM1);
detection's lowest-index fallback lands on a port that accepts bytes and
never answers (decision 43).

## Building: espressif (ESP32-C5-WROOM-1 DK)

```sh
cd workspaces/espressif
west init -l manifest
west update
west build -b esp32c5_devkitc/esp32c5/hpcore app
```

This produces `build/zephyr/zephyr.bin` — flash it via `embarch-core`'s
`POST /flash` (`format: "bin"`, `base_address: "0x2000"`, the address
Zephyr's own build merges to), not `west flash`
([embarch-core decisions.md](https://github.com/gabrieltetar/embarch-doc/blob/main/embarch-core/decisions.md)
decision 18). This board is not the current bench (decision 43); it is not
enrolled for a study link today, and its port-selection story lives in
`embarch-core`'s own docs, not here.

## Repo layout

See
[spec.md §2](https://github.com/gabrieltetar/embarch-doc/blob/main/embarch-dev-bench/spec.md#2-repository-layout)
for the full annotated tree. Short version: `app/` is the one shared,
vendor-agnostic C application; each `workspaces/<vendor>/` directory is an
independent west topdir with its own manifest, symlinking in `app/` rather
than copying it.

## License

MIT — see [LICENSE](LICENSE). That covers this repo's own contents
(`app/` and each `workspaces/<vendor>/` manifest); the west-managed trees
`west update` fetches into those workspaces are not checked in here and keep
their upstream licenses.
