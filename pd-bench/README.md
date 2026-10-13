# pd-bench

A USB Power Delivery partner that a host scripts over its shell, for a DUT
whose USB-C ports are under test. Two builds, one per expansion board:

- **X-NUCLEO-SNK1M1 (default): a sink.** It asks for fixed PDOs and PPS
  levels on command, keeps a PPS contract alive, and reports the contract
  and VBUS it sees. An electronic load on its VBUS output turns it into a
  scriptable PD/PPS sink.
- **X-NUCLEO-DRP1M1: a dual-role port.** Everything the sink does, and as a
  source it switches a 5 V bench supply onto VBUS, answers the sink's
  Request with Accept, Reject or Wait as the host sets, sends and answers
  PR_Swap, and offers the Source_Capabilities the host sets. It tests a
  DUT's Power Role Swap and its behaviour as a sink against a scriptable
  source.

Design: `embarch-doc/embarch-host-study-proposal.md`, phase 1; why it is a
second app here: `embarch-dev-bench` decision 48.

This is **not** the dev-bench wire. The console is a Zephyr shell, driven
with Core's `signal_exchange` on a declared `Direct` signal.

## Hardware

A NUCLEO-G0B1RE powered from the ST-LINK USB, with one of the two boards.
Each board is a shield of this app (`boards/shields/`); the UCPD1 setup
both share is in `boards/nucleo_g0b1re.overlay`.

### X-NUCLEO-SNK1M1

TCPP01-M12 sink protection. On the SNK1M1:

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

### X-NUCLEO-DRP1M1

TCPP03-M20 dual-role protection, with back-to-back N-MOSFETs on a provider
path (CN3, `SOURCE`) and a consumer path (CN2, `SINK`) to VBUS.

- **No jumper on JP1.** Its two jumpers feed the Nucleo's 3.3 V and NRST
  from the shield's LDO; with the ST-LINK powering the Nucleo they stay off
  (UM2891 3.2.1).
- **CN3: a 5 V bench supply, current-limited, at least the current offered**
  (3 A by default; ST's examples offer 1.5 A). Pin 1 is the supply, pin 2
  GND (UM2891 Fig. 9). The provider path switches CN3 onto VBUS as it is,
  with no regulation, so the bench only offers 5 V and refuses to source,
  or to accept a PR_Swap to source, unless CN3 reads 4.75 to 5.5 V.
- **CN2: the electronic load** for the sink role, as on the SNK1M1 output.
- Shield defaults kept: TCPP03 at I2C 0x34 (R28 fitted, SH16 open), VBUS
  OVP at 22 V (R0), OCP at 6 A with the 7 mΩ shunt R5, R19/R20 fitted.

| Function | Pin | Morpho |
|---|---|---|
| CC1 / CC2 (UCPD1) | PA8 / PB15 | CN10-23 / CN10-26 |
| TCPP03 I2C SCL / SDA (I2C1, 1 kΩ pull-ups on the shield) | PB8 / PB9 | CN10-3 / CN10-5 |
| TCPP03 ENABLE, driven high | PC8 | CN10-2 |
| TCPP03 FLGn, falling on a fault | PC5 (USART1 RX disabled) | CN10-37 |
| VBUS at the connector, 200 k over 40.2 k | PA0 (ADC1_IN0) | CN7-28 |
| Provider path (CN3), 200 k over 40.2 k | PA1 (ADC1_IN1) | CN7-30 |
| Consumer path (CN2), not used | PA4 (ADC1_IN4) | CN7-32 |
| VBUS current, IANA at 42 V/V over 7 mΩ, not used | PB11 (ADC1_IN15) | CN7-36 |
| Console and shell (ST-LINK VCOM) | USART2, 115200 | |

The TCPP03 stays in Normal mode: its dead-battery Rd is off, so the UCPD
toggles Rp and Rd itself, and FLGn falls only on a fault. Register 0 is a
control byte written whole:

| Bits | Meaning |
|---|---|
| 1:0 | VCONN switch: open (the bench never sources VCONN) |
| 2 | GDP: 1 closes the provider path |
| 3 | GDC: 1 opens the consumer path |
| 5:4 | Mode: 01 Normal |
| 6 | VBUS discharge |

Idle is `0x10`: consumer path closed, as the sink needs. Sourcing writes
`0x18` (consumer path open), waits 2 ms, then `0x1c`; the TCPP03 refuses
both paths closed. Turning off writes `0x58` (provider path open, VBUS
discharging) until VBUS is under 0.6 V, then `0x10`. Register 2 holds the
fault flags (OCP VCONN, OCP VBUS, OVP VBUS, OVP CC, over-temperature); a
fault opens both paths until the recovery word `0x18` clears it.

#### Sources

- ST X-CUBE-TCPP, `Projects/NUCLEO-G071RB/Applications/USB_PD/DRP1M1_DRP`:
  `TCPP/Target/drp1m1_conf.h` (I2C1 PB8/PB9, ENABLE PC8, FLGn PC5, VBUSc
  PA0 ADC channel 0, provider PA1, consumer PA4, IANA PB11, divider 200/40,
  gain 42, shunt 7 mΩ), `DRP1M1_DRP.ioc` (UCPD1 CC1 PA8, CC2 PB15),
  `USBPD/App/usbpd_pdo_defs.h` and the VIF (one 5 V 1.5 A source PDO, no
  VCONN, no Try.SRC/Try.SNK), `README.md` (a supply on CN3 to act as a
  source, no jumper).
- ST X-CUBE-TCPP `Drivers/BSP/X-NUCLEO-DRP1M1/drp1m1_usbpd_pwr.c`: address
  `0x68` ("0x34"), the VBUS on and off sequences, the fault and recovery
  handling.
- ST `stm32-tcpp0203`, `tcpp0203.h` and `tcpp0203.c`: the register map.
- UM2891 Rev 3: 1.6 (I2C address), 1.7 (dividers, IANA), 1.8 and Fig. 9
  (provider and consumer paths, CN2/CN3), 1.9 (OCP, recovery words), 1.10
  (OVP), 1.11 (JP1), Fig. 7 (morpho pins), 3.2.1 (no jumper with the
  ST-LINK).
- Zephyr `boards/st/nucleo_g0b1re/st_morpho_connector.dtsi`: morpho pin to
  GPIO on the G0B1RE.

## Build and flash

It needs a Zephyr USB-C stack carrying the PD fixes that are not upstream
yet, so it builds from a west workspace that has them, not from
`workspaces/`. The SNK1M1 build is the default; the DRP1M1 build names its
shield:

```sh
cd <that workspace>
west build -b nucleo_g0b1re -d build/pd-bench <path to>/embarch-dev-bench/pd-bench
west build -b nucleo_g0b1re -d build/pd-bench-drp <path to>/embarch-dev-bench/pd-bench -- -DSHIELD=x_nucleo_drp1m1
```

The DRP1M1 build enables `CONFIG_USBC_CSM_DRP`, I2C and a 2 KB USB-C
thread stack (`boards/shields/x_nucleo_drp1m1/x_nucleo_drp1m1.conf`).

Enrol the Nucleo as a **bench board** (no role, `embarch-topology`
decision 41) and flash it through a project that names its probe serial;
a project for the DRP1M1 build passes `-DSHIELD=x_nucleo_drp1m1`:

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
| `pd status` | Rp current, whether the partner is PD, the contract in force, Rejects and Waits seen, the data and power roles with the swap answers and outcomes, VBUS; on the DRP1M1 also the Requests answered and the TCPP03 registers and faults |
| `pd role` | Power role (`source`, `sink` or `none`), data role, contract |
| `pd caps` | The partner's Source_Capabilities, one PDO per line |
| `pd cc` | What each CC line sees: `open`, `ra`, `rd`, `rp-default`, `rp-1.5a`, `rp-3a` |
| `pd req <pos> [mA]` | Ask for fixed PDO `pos`, at its current or `mA` (sink role) |
| `pd pps <pos> <mV> <mA>` | Ask for PPS APDO `pos` at `mV` / `mA` (sink role) |
| `pd max` | Ask for the highest-power fixed PDO, at most 3 A (the default) |
| `pd getcaps` | Send Get_Source_Cap |
| `pd hardreset` | Send Hard Reset Signaling |
| `pd drswap` | Send DR_Swap |
| `pd keepalive <on\|off>` | Off, a PPS contract gets no keep-alive Request, to test a source's tPPSTimeout Hard Reset (on by default) |
| `pd dr <accept\|reject\|nosupport>` | How the bench answers a partner's DR_Swap (default accept); nosupport also clears the 5 V PDOs' Dual-Role Data bit |
| `pd ask <status\|ppsstatus\|battstatus\|snkcaps>` | Ask the partner and print its answer: Get_Status, Get_PPS_Status (sink role), Get_Battery_Status, Get_Sink_Cap (source role) |
| `pd prswap` | Send PR_Swap (DRP1M1) |
| `pd pr <accept\|reject>` | How the bench answers a partner's PR_Swap (default accept; DRP1M1) |
| `pd answer <accept\|reject\|wait>` | How the bench, as source, answers a sink's Request (default accept; DRP1M1) |
| `pd srccaps [<mV>:<mA> ...]` | The bench's Source_Capabilities: listed, or set and sent at once to a sink attached (DRP1M1) |

On the SNK1M1 the last four answer `pd: error not a DRP build`. `devmem`
is enabled too, for reading the UCPD and GPIO registers without halting
the core.

A choice is checked against the caps held and refused at once when it is
not offered. `pd status` also shows the data role, the DR_Swap answer and
the outcome of the bench's own last DR_Swap, and the power role, the
PR_Swap answer and the outcome of its own last PR_Swap: `accepted`,
`rejected`, `not supported`, `wait`, `no response` (PR_Swap), `failed`
(a PR_Swap that ended in a Hard Reset or ErrorRecovery), or `pending` when
the stack did not send it, as after a refusal until the next negotiation
(USB PD 7.4.2) or within tPRSwapWait of a Wait.

`pd: ok` means the Request was queued, not accepted: read the
outcome back with `pd status`. A Reject leaves the earlier contract in force
and counts in `rejects`. A Wait counts in `waits`, and the stack repeats
the Request after tSinkRequest (100 ms). A PPS contract is renewed every 8 s, under
tPPSRequest (10 s), by repeating the contract, not the last choice.

`pd ask` waits up to 1 s for the answer. `pd: error no answer` means the
stack dropped the request (not in a Ready state, or a PD 2.0 partner) or
the partner did not reply; `the partner sent none` means Not_Supported,
Reject or no reply in time. Get_Source_Info, Get_Revision,
Get_Source_Cap_Extended, Get_Manufacturer_Info and Get_Battery_Cap have
no DPM request in the stack, so there is no `pd ask` for them.

### As a source (DRP1M1)

- **The offer.** The devicetree's `source-pdos` is the most the bench
  offers: one fixed 5 V 3 A PDO with the Dual-Role Power bit. `pd srccaps`
  takes PDOs in rising voltage, the first at 5000 mV, each a voltage the
  devicetree lists at no more than its current, so `pd srccaps 5000:1500`
  is ST's certified offer. Rp follows the 5 V PDO: 3 A, 1.5 A or USB
  default.
- **The answer.** `accept` still rejects a Request above the PDO it names.
  After a Reject or Wait to the first Request there is no contract, and the
  stack waits for new Source_Capabilities: `pd srccaps ...` sends them.
- **PR_Swap.** The stack answers a partner's PR_Swap with Accept or Reject
  only; `pd pr wait` answers `pd: error`. Two DRPs attach in either role
  (the stack has no Try.SRC or Try.SNK): read `pd role`, then `pd prswap`
  for the other one.

### Safety (DRP1M1)

- The bench sources only when the stack asks (attach as source, PR_Swap,
  the end of a Hard Reset), only with CN3 at vSafe5V, and returns only once
  VBUS is at vSafe5V; otherwise it turns off and the stack goes to
  ErrorRecovery.
- Turning off discharges VBUS under 0.6 V, within tSafe0V (650 ms): on a
  detach, ErrorRecovery, a PR_Swap to sink, and a Hard Reset, after which
  VBUS stays off for tSrcRecover (700 ms) before it returns.
- An MCU reset opens both paths: the TCPP03 keeps its state through a
  reset, and the first thing the bench writes is the recovery word.
- A TCPP03 fault (FLGn) is counted in `pd status`, cleared, and ends in
  ErrorRecovery; the port attaches again from scratch.
- The bench never sources VCONN.

## Two things the bench taught

- **The USB-C thread runs at the shell's priority (14).** Zephyr's
  AttachWait.SNK skips its sleep while it sees Rp and no VBUS, so at a
  higher priority a source that withholds VBUS starves the shell and the
  log. Time slicing keeps both answering.
- **The VBUS ADC samples for the longest time.** The divider is about
  33 kΩ; the shortest sampling read 5.0 V as 4.36 V, under the 4.75 V
  vSafe5V floor, and the sink never attached. The DRP1M1's dividers are the
  same.
