# pd-bench

A USB Power Delivery sink that a host scripts over its shell. It is the PD
partner for a DUT whose USB-C ports are under test: it asks for fixed PDOs
and PPS levels on command, keeps a PPS contract alive, and reports the
contract and VBUS it sees. An electronic load on its VBUS output turns it
into a scriptable PD/PPS sink. Design: `embarch-doc/embarch-host-study-proposal.md`,
phase 1; why it is a second app here: `embarch-dev-bench` decision 48.

This is **not** the dev-bench wire. The console is a Zephyr shell, driven
with Core's `signal_exchange` on a declared `Direct` signal.

## Hardware

NUCLEO-G0B1RE with an X-NUCLEO-SNK1M1 (TCPP01-M12 sink protection),
powered from the ST-LINK USB. On the SNK1M1:

- **JP3 (LDO_OUT) and JP4 (RESET) off.** Fitted, they select dead-battery
  mode: JP4 holds the MCU in reset until a USB-C source powers the board, so
  the shell is silent and SWD cannot halt the core.
- Solder bridges J1 and J2 in position 1-2.

This is the setup ST's X-CUBE-TCPP `SNK1M1_Sink_PPS` example for the
NUCLEO-G071RB gives.

| Function | Pin |
|---|---|
| CC1 / CC2 (UCPD1) | PA8 / PB15 |
| VBUS sense, 200 k over 40 k | PB1 (ADC1_IN9) |
| TCPP01 VCC_OUT, driven high | PC10 |
| TCPP01 DB_OUT, driven high | PB6 |
| Console and shell (ST-LINK VCOM) | USART2, 115200 |

The pins come from ST's X-CUBE-TCPP SNK1M1 examples for the Nucleo-64
pinout.

## Build and flash

It needs a Zephyr USB-C stack carrying the PD fixes that are not upstream
yet, so it builds from a west workspace that has them, not from
`workspaces/`:

```sh
cd <that workspace>
west build -b nucleo_g0b1re -d build/pd-bench <path to>/embarch-dev-bench/pd-bench
```

Enrol the Nucleo as a **bench board** (no role, `embarch-topology`
decision 41) and flash it through a project that names its probe serial:

```sh
embarch-api enroll-probe --chip STM32G0B1VE --probe-serial <ST-LINK serial> --name pd-bench
embarch-api build-and-flash <project>
```

The ST-LINK's mass-storage drive refuses this image: it reports 32 KB.

## Shell

Every `pd` command ends its output with one line, `pd: ok` or
`pd: error <reason>`; an exchange waits for `pd: `.

| Command | Does |
|---|---|
| `pd status` | Rp current, whether the partner is PD, the contract in force, Rejects seen, VBUS |
| `pd caps` | The partner's Source_Capabilities, one PDO per line |
| `pd cc` | What each CC line sees: `open`, `ra`, `rd`, `rp-default`, `rp-1.5a`, `rp-3a` |
| `pd req <pos> [mA]` | Ask for fixed PDO `pos`, at its current or `mA` |
| `pd pps <pos> <mV> <mA>` | Ask for PPS APDO `pos` at `mV` / `mA` |
| `pd max` | Ask for the highest-power fixed PDO, at most 3 A (the default) |
| `pd getcaps` | Send Get_Source_Cap |
| `pd hardreset` | Send Hard Reset Signaling |

`devmem` is enabled too, for reading the UCPD and GPIO registers without
halting the core.

A choice is checked against the caps held and refused at once when it is
not offered. `pd: ok` means the Request was queued, not accepted: read the
outcome back with `pd status`. A Reject leaves the earlier contract in force
and counts in `rejects`. A PPS contract is renewed every 8 s, under
tPPSRequest (10 s), by repeating the contract, not the last choice.

## Two things the bench taught

- **The USB-C thread runs at the shell's priority (14).** Zephyr's
  AttachWait.SNK skips its sleep while it sees Rp and no VBUS, so at a
  higher priority a source that withholds VBUS starves the shell and the
  log. Time slicing keeps both answering.
- **The VBUS ADC samples for the longest time.** The divider is about
  33 kΩ; the shortest sampling read 5.0 V as 4.36 V, under the 4.75 V
  vSafe5V floor, and the sink never attached.

