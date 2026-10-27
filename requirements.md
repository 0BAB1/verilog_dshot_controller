# HOLY DSHOT – 4-channel DShot ESC controller IP – Requirements

Version 1.0 · target: Holy Drone flight controller (Holy Core SoC, Arty S7-50)

This document is the complete specification for an autonomous coding agent. It defines
**what** to build, **how** it must behave, **how** the repository is laid out and **how** it
must be verified. When something is not specified here, follow the conventions of the
HOLY CORE project (https://0bab1.github.io/HOLY_CORE_COURSE) and record the decision in
`readme.md` under "Design decisions".

Requirement IDs (`REQ-xx`) and test IDs (`T-xx`) are referenced in the traceability matrix
(section 11). Every REQ must be covered by at least one test.

---

## 1. Context

- **System:** a quadcopter flight controller running on the HOLY CORE (RV32IM + Zicsr,
  single cycle, 25 MHz) on an Arty S7-50 (Spartan-7 XC7S50). Peripherals are reached
  through the HOLY CORE platform's external **AXI4-Lite** interface (external peripheral
  space `0x10000000–0x2FFFFFFF`). Interrupts go to the platform PLIC (`irq_in`).
- **ESCs:** 4 custom ESCs running **AM32** firmware. AM32 accepts servo PWM and DShot
  (300/600), plus bidirectional DShot. This IP implements **standard (unidirectional)
  DShot300** only.
- **Control:** open loop. The flight software sends a throttle command per motor every
  control step (arbitrary frequency). No RPM or telemetry is read back.
  
## 2. Scope

**In scope**
- 4 DShot output channels, DShot300, frame encoding and CRC in hardware.
- AXI4-Lite slave register interface, polled by software.
- SystemVerilog core + plain Verilog wrapper for Vivado block designs.
- cocotb + Verilator testbenches.
- IP must be optimised for space and timoing (by applying standard good measures)

**Out of scope (non-goals)**
- Bidirectional DShot, eRPM/telemetry reception, KISS telemetry UART.
- Servo PWM, OneShot, MultiShot.
- Any closed-loop speed control.
- Clock-domain crossing: the IP runs in the AXI clock domain only.
- Addtionaal complicated features, this IP is plain and simple, it simply sends X throttle from the slave AXIL rgister to DSHOT 300, that's it.

## 3. DShot background (normative)

### 3.1 Frame format
A frame is 16 bits, sent **MSB first**:

| Bits | Field | Meaning |
|---|---|---|
| [15:5] | throttle (11 bits) | 0 = motor stop, 1–47 = special commands, 48–2047 = throttle |
| [4] | telemetry request | passed through from the register, unused by this system |
| [3:0] | CRC | see below |

With the 12-bit payload `v = {throttle[10:0], telemetry}`:

```
crc = (v ^ (v >> 4) ^ (v >> 8)) & 0xF        // = v[3:0] ^ v[7:4] ^ v[11:8]
frame = (v << 4) | crc
```

Reference vectors (register value → frame), register format in section 6:

| Register value | Throttle | Telemetry | Frame |
|---|---|---|---|
| `0x000` | 0 | 0 | `0x0000` |
| `0x030` | 48 | 0 | `0x0606` |
| `0x416` | 1046 | 0 | `0x82C6` |
| `0xFFF` | 2047 | 1 | `0xFFFF` |

### 3.2 Bit encoding
- The line **idles low**. Every bit starts with a rising edge.
- Bit period `T_BIT`; the line stays high for `T1H` (bit = 1) or `T0H` (bit = 0), then low
  for the rest of the bit.

| Speed | Bit rate | T_BIT | T1H (≈ 75 %) | T0H (≈ 37.5 %) |
|---|---|---|---|---|
| DShot600 | 600 kbit/s | 1.667 µs | 1.250 µs | 0.625 µs |
| DShot300 | 300 kbit/s | 3.333 µs | 2.500 µs | 1.250 µs |
| DShot150 | 150 kbit/s | 6.667 µs | 5.000 µs | 2.500 µs |

### 3.3 Integer timing (normative rounding)
With `CLK_HZ` the clock frequency and `RATE` the bit rate, all values in clock cycles:

```
T_BIT = (CLK_HZ + RATE/2) / RATE          // round to nearest
T1H   = (3*T_BIT + 2) / 4                 // round(0.75  * T_BIT)
T0H   = (3*T_BIT + 4) / 8                 // round(0.375 * T_BIT)
```

At `CLK_HZ = 25 MHz`: DShot600 → 42 / 32 / 16, DShot300 → 83 / 62 / 31,
DShot150 → 167 / 125 / 63.

### 3.4 ESC behaviour assumptions (verify on hardware, not in RTL)
- AM32 arms after receiving valid "stop" frames (throttle 0) for a while after power-up.
- AM32 stops the motor if frames stop arriving (signal-loss failsafe).
- Special commands (1–47) are only meaningful with motors stopped and must be repeated.

## 4. Functional requirements

### I/O based.

I/O are sepcified. Requirements shall be distilled to the bare, yet safe and industry opreational-ready minumum.

- INPUTS :
 - 4 control registerswhich contains the throttle per motor
 - 1 satatus register that displays whatever is necessary, if nothing is omportant, no status register is required.
 - 1 control register to enable (throttle *can* then be applied on all 4 motors) / disable (also acts as an hard stop on all 4 motors) and a softreset of needed to reset all regs to 0

## MANUAL NOTES

Please add a 1 documentation readme.md exmplaining what the projects does, how to use in in an SoC (regmapsand register functionnalities) et how it works overall. You can add a second page explinning the project structure.

You have 64KB of register space.

This is a simple project. You shall only have 1 simple main .sv file with all the logic. Only if truly need you can add other files if it really help with readabilty and maintability.

Project should be easuily maintanable by hand and by a human, easily readble.

Project should not be complex and spagetthi. Only a src/ folder (with the very few source files) and a tb/ folder containning the cocotb tbs.

Make sure you floow the DSHOT standard and ensure AM32 compatibility on a cmd pin.

The IP should be very simple in design and usage.