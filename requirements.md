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
  DShot** only.
- **Control:** open loop. The flight software sends a throttle command per motor every
  control step (1 kHz nominal, possibly 500 Hz). No RPM or telemetry is read back.
- **Prior art:** a first trigger-only version exists (`holy_dshot.v`, plain Verilog). It
  may be used as a reference for the frame engine; this specification supersedes it.

## 2. Scope

**In scope**
- 4 DShot output channels, DShot150/300/600, frame encoding and CRC in hardware.
- AXI4-Lite slave register interface, polled by software.
- Two output modes: TRIGGER (one frame per software request) and AUTO (periodic frames).
- Safety: watchdog, fault state forcing "motor stop" frames, sticky error flags, soft reset.
- One level interrupt output for faults.
- SystemVerilog core + plain Verilog wrapper for Vivado block designs.
- cocotb + Verilator testbenches, C driver and a bring-up program.

**Out of scope (non-goals)**
- Bidirectional DShot, eRPM/telemetry reception, KISS telemetry UART.
- Servo PWM, OneShot, MultiShot.
- Any closed-loop speed control.
- Clock-domain crossing: the IP runs in the AXI clock domain only.

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

### 4.1 Outputs and timing
- **REQ-01** The IP drives `N_CH = 4` outputs `dshot_o[3:0]`. All four frames of one frame
  event start on the **same clock edge** (aligned to the cycle).
- **REQ-02** Outputs are driven from flip-flops (glitch-free), idle **low** whenever no frame
  is being sent, and are **low** in reset, when disabled, and immediately (≤ 1 cycle) after a
  soft reset.
- **REQ-03** Speed is selected by `CTRL.SPEED` (600/300/150). Timing follows section 3.3
  exactly (tolerance 0 cycles vs. the formula) for any `CLK_HZ` parameter value from
  10 MHz to 200 MHz.
- **REQ-04** Frame encoding and CRC follow section 3.1 bit-exactly for all 4096 register
  values.
- **REQ-05** A configuration change (`SPEED`) while a frame is being sent takes effect at
  the next frame only. The frame in flight is never altered.

### 4.2 Staging and commit
- **REQ-06** Software writes per-channel values into **staged** registers `THROTTLE0..3`.
  Staged values never affect the outputs directly.
- **REQ-07** A write to `COMMIT` copies all four staged values to the **active** values
  atomically (same clock edge). This guarantees that the four motors always use values from
  the same control step.
- **REQ-08** A frame latches the active values at its start; a commit during a frame affects
  the next frame only.
- **REQ-09** A commit refreshes the watchdog (section 4.4).

### 4.3 Modes
- **REQ-10 TRIGGER mode** (`CTRL.MODE = 0`): each write to `COMMIT` commits the values and
  starts **one** frame on all channels if no frame is in flight. If a frame is in flight, the
  values are still committed, no new frame starts, and `STATUS.ERR_OVERRUN` is set.
- **REQ-11 AUTO mode** (`CTRL.MODE = 1`): while running, a frame starts every
  `FRAME_PERIOD` clock cycles (start-to-start, exact to the cycle), using the active values.
  `COMMIT` only commits values and refreshes the watchdog.
- **REQ-12** `FRAME_PERIOD` minimum is `18 × T_BIT` (frame plus a 2-bit idle gap) for the
  current speed. A smaller value is clamped to that minimum and sets `STATUS.ERR_CFG`.
- **REQ-13** Switching MODE takes effect after the frame in flight, if any.

### 4.4 Watchdog
- **REQ-14** The watchdog is **always active in AUTO mode**, and active in TRIGGER mode
  when `CTRL.WDG_EN = 1`.
- **REQ-15** It counts clock cycles since the last `COMMIT` (or since entering RUNNING).
  When the count reaches `WDG_TIMEOUT`, the IP enters **FAULT** with cause "watchdog".
- **REQ-16** `WDG_TIMEOUT = 0` is invalid: the reset value is used instead and
  `STATUS.ERR_CFG` is set.

### 4.5 States, faults and errors
States: `DISABLED`, `RUNNING`, `FAULT`.

| From | Event | To | Effect |
|---|---|---|---|
| any | reset, or `CTRL.SOFT_RESET` | DISABLED | outputs low, active and staged values = 0, errors and fault cleared, config registers to reset values |
| DISABLED | `CTRL.EN` 0 → 1 | RUNNING | active values = 0 (stop), watchdog counter cleared |
| RUNNING | `CTRL.EN` 1 → 0 | DISABLED | current frame aborted, outputs low |
| RUNNING | watchdog timeout | FAULT | `NEED_RESET = 1`, `FAULT_CAUSE = 1`, `fault_irq = 1` |
| FAULT | `CTRL.SOFT_RESET` | DISABLED | as reset |

- **REQ-17** In **FAULT** with `CTRL.EN = 1`, the IP sends **stop frames** (all channels
  payload 0, telemetry 0) every `FRAME_PERIOD` cycles, regardless of MODE, ignoring active
  values and `COMMIT`. With `CTRL.EN = 0` in FAULT, outputs stay low.
- **REQ-18** FAULT is latched: only `CTRL.SOFT_RESET` (or the hardware reset) leaves it.
  `COMMIT` and `EN` toggles do not clear it.
- **REQ-19 Invalid command protection:** if `CTRL.CMD_EN = 0`, a commit whose staged value
  for a channel has throttle in 1–47 is applied for the other channels only. That
  channel keeps its previous active value, and `STATUS.ERR_INVALID` is set. With
  `CMD_EN = 1`, values 1–47 are committed normally.
- **REQ-20** `ERR_OVERRUN`, `ERR_INVALID` and `ERR_CFG` are **sticky** and cleared by writing
  1 to the bit (W1C). They do not stop operation.
- **REQ-21** `fault_irq` is a level output equal to `STATUS.NEED_RESET`.
- **REQ-22** `STATUS.READY = 1` iff state is RUNNING. `STATUS.BUSY = 1` iff a frame is
  being shifted out.

### 4.6 Soft reset
- **REQ-23** `CTRL.SOFT_RESET` is write-1, self-clearing (reads 0). It has the effect of the
  hardware reset on all state and registers (including `CTRL`, which returns to its reset
  value), except the `FRAMES` counter which is preserved for debugging.

## 5. Interfaces and parameters

### 5.1 Parameters
| Parameter | Default | Meaning |
|---|---|---|
| `CLK_HZ` | 25 000 000 | clock frequency, used for all timing |
| `N_CH` | 4 | fixed at 4 (register map assumes 4); keep as a localparam-checked parameter |

### 5.2 Ports of the SystemVerilog core `holy_dshot`
Naming follows the HOLY CORE style (`clk`, `rst_n`, synchronous active-low reset).

| Port | Dir | Width | Description |
|---|---|---|---|
| `clk` | in | 1 | clock (AXI clock) |
| `rst_n` | in | 1 | synchronous active-low reset |
| `s_axi_awaddr` | in | 12 | AXI4-Lite write address |
| `s_axi_awprot` | in | 3 | ignored |
| `s_axi_awvalid` / `s_axi_awready` | in / out | 1 | |
| `s_axi_wdata` | in | 32 | |
| `s_axi_wstrb` | in | 4 | see REQ-26 |
| `s_axi_wvalid` / `s_axi_wready` | in / out | 1 | |
| `s_axi_bresp` | out | 2 | OKAY / SLVERR |
| `s_axi_bvalid` / `s_axi_bready` | out / in | 1 | |
| `s_axi_araddr` | in | 12 | |
| `s_axi_arprot` | in | 3 | ignored |
| `s_axi_arvalid` / `s_axi_arready` | in / out | 1 | |
| `s_axi_rdata` | out | 32 | |
| `s_axi_rresp` | out | 2 | OKAY |
| `s_axi_rvalid` / `s_axi_rready` | out / in | 1 | |
| `dshot_o` | out | 4 | DShot lines to the ESCs |
| `fault_irq` | out | 1 | level, = `NEED_RESET` |

Flat `s_axi_*` ports (no SystemVerilog interface) are required: they are what Vivado infers
as an AXI4-Lite slave and what `cocotbext-axi` binds to with `from_prefix(dut, "s_axi")`.

### 5.3 AXI4-Lite requirements
- **REQ-24** Fully AXI4-Lite compliant slave: AW and W may arrive in any order or in the same
  cycle; no deadlock under any valid/ready sequence; one outstanding write and one
  outstanding read; `VALID` never depends on `READY`.
- **REQ-25** Address decode on `addr[7:2]`; unmapped reads return 0 with OKAY; unmapped
  writes are ignored with OKAY.
- **REQ-26** Writes with `wstrb != 4'hF` are ignored and answered with **SLVERR**
  (the driver always writes full words).
- **REQ-27** Reads have no side effects.

### 5.4 Verilog wrapper `holy_dshot_wrapper` (fpga/)
- **REQ-28** Plain Verilog-2001 file instantiating `holy_dshot`, with ports `aclk`,
  `aresetn`, `s_axi_*`, `dshot_o[3:0]`, `fault_irq` and the same parameters. Required because
  Vivado block-design module references do not accept a SystemVerilog top. It must contain
  no logic, only the instance and connections.

## 6. Register map

Base address assigned in Vivado (suggested `0x10080000`). All registers are 32 bits,
word-aligned. Access: RW read/write, R read-only, W write-only (reads 0), W1C write 1 to
clear, W1S write 1, self-clearing.

| Offset | Name | Access | Reset | Fields |
|---|---|---|---|---|
| `0x00` | `CTRL` | RW | `0x0` | `[0]` EN · `[1]` SOFT_RESET (W1S) · `[3:2]` SPEED (0 = 600, 1 = 300, 2 = 150, 3 = reserved → treated as 600 + `ERR_CFG`) · `[4]` MODE (0 = TRIGGER, 1 = AUTO) · `[5]` WDG_EN (TRIGGER mode only) · `[6]` CMD_EN |
| `0x04` | `STATUS` | R / W1C | `0x0` | `[0]` READY · `[1]` BUSY · `[2]` NEED_RESET · `[7:4]` FAULT_CAUSE (0 none, 1 watchdog) · `[8]` ERR_OVERRUN (W1C) · `[9]` ERR_INVALID (W1C) · `[10]` ERR_CFG (W1C) · `[13:12]` STATE (0 DISABLED, 1 RUNNING, 2 FAULT) |
| `0x08` | `COMMIT` | W | – | any full-word write: commit staged → active, refresh watchdog, start a frame (TRIGGER mode) |
| `0x0C` | `FRAME_PERIOD` | RW | `CLK_HZ / 1000` | `[31:0]` AUTO and FAULT frame period, in cycles (REQ-12) |
| `0x10` | `WDG_TIMEOUT` | RW | `CLK_HZ / 20` | `[31:0]` watchdog timeout, in cycles (50 ms) |
| `0x14` | `FRAMES` | R | `0x0` | `[31:0]` frames sent since reset (wraps; preserved by SOFT_RESET) |
| `0x18` | `INFO` | R | – | `[15:0]` T_BIT for current SPEED · `[23:16]` N_CH (4) · `[31:24]` VERSION (`0x01`) |
| `0x20`–`0x2C` | `THROTTLE0..3` | RW | `0x0` | staged value: `[10:0]` throttle · `[11]` telemetry request |
| `0x30`–`0x3C` | `ACTIVE0..3` | R | `0x0` | active (committed) value, same format |

Writes to `FRAME_PERIOD` store the clamped value (REQ-12); reads return what is used.

## 7. Non-functional requirements

- **REQ-29 Language and style:** core in SystemVerilog-2012 (`logic`, `always_ff`,
  `always_comb`, `typedef enum`), constants and types in `packages/holy_dshot_pkg.sv`,
  synchronous active-low reset `if (~rst_n)` as in the HOLY CORE, `snake_case`, comments on
  every block.
- **REQ-30 Synthesizable** in Vivado for Spartan-7: no latches, no initial blocks for logic,
  no `#` delays, single clock domain.
- **REQ-31 Lint clean:** `verilator --lint-only -Wall` on the core and the wrapper with zero
  warnings, or each remaining warning waived inline with a justification comment.
- **REQ-32 Size target:** under 400 LUTs and 300 FFs (indicative).
- **REQ-33 Timing:** no combinational path from AXI inputs to `dshot_o`; target ≥ 100 MHz.

## 8. Repository layout (mirrors the HOLY CORE edition layout)

```
holy_dshot/
├── Makefile                     cleanup only (see 9.6)
├── readme.md                    overview, register map, design decisions, how to test
├── requirements.txt             cocotb, cocotbext-axi, pytest (pinned versions)
├── packages/
│   └── holy_dshot_pkg.sv        register offsets, field positions, enums, timing functions
├── src/
│   ├── holy_dshot.sv            top core: AXI-Lite regs + control FSM
│   ├── holy_dshot_regs.sv       (optional) AXI-Lite slave + register file
│   └── holy_dshot_engine.sv     frame encoder + bit timing for 4 channels
├── fpga/
│   ├── holy_dshot_wrapper.v     plain Verilog wrapper (REQ-28)
│   └── holy_dshot_example.xdc   example pin constraints
├── sw/
│   ├── hc_dshot.h               C driver (section 10)
│   ├── hc_dshot.c
│   └── dshot_test/main.c        bring-up program
└── tb/
    ├── test_runner.py           pytest runner for all module testbenches
    ├── holy_dshot_engine/
    │   ├── Makefile
    │   └── test_holy_dshot_engine.py
    ├── holy_dshot/
    │   ├── Makefile
    │   └── test_holy_dshot.py
    └── holy_dshot_wrapper/
        ├── Makefile
        └── test_holy_dshot_wrapper.py
```

Rules:
- One `tb/<module>/` folder per tested module, containing a `Makefile` and
  `test_<module>.py`. The cocotb `TOPLEVEL` is `<module>`, the `MODULE` is `test_<module>`.
- The package file is always compiled first.
- If a module is split differently, keep one testbench folder per module.

## 9. Testing

### 9.1 Tech stack
| Tool | Version | Role |
|---|---|---|
| Python | ≥ 3.10 | testbench language |
| cocotb | 1.9.x (pin in `requirements.txt`) | testbench framework (frontend) |
| Verilator | ≥ 5.020 | simulator (backend) |
| cocotbext-axi | ≥ 0.1.24 | AXI4-Lite master BFM (`AxiLiteMaster`, `AxiLiteBus`) |
| pytest | ≥ 7 | runs all testbenches through `tb/test_runner.py` |
| GTKWave | any | optional waveform viewing (`dump.vcd`) |

If cocotb 2.x is used instead, the only change is the runner import
(`from cocotb_tools.runner import get_runner`); record it in `readme.md`.

### 9.2 Per-module Makefile (HOLY CORE convention)
Each `tb/<module>/Makefile` follows this template:

```make
# Makefile

# defaults
SIM ?= verilator
TOPLEVEL_LANG ?= verilog
EXTRA_ARGS += --trace --trace-structs
WAVES = 1

VERILOG_SOURCES += $(PWD)/../../packages/holy_dshot_pkg.sv
VERILOG_SOURCES += $(PWD)/../../src/holy_dshot_engine.sv
VERILOG_SOURCES += $(PWD)/../../src/holy_dshot.sv
# wrapper testbench only:
# VERILOG_SOURCES += $(PWD)/../../fpga/holy_dshot_wrapper.v

# parameters (Verilator -G), default 25 MHz
CLK_HZ ?= 25000000
EXTRA_ARGS += -GCLK_HZ=$(CLK_HZ)

TOPLEVEL = holy_dshot
MODULE = test_holy_dshot

include $(shell cocotb-config --makefiles)/Makefile.sim
```

`cd tb/<module> && make` must run that module's tests and leave a `dump.vcd`.

### 9.3 Test runner (HOLY CORE convention)
`tb/test_runner.py` runs every testbench with pytest (`cd tb && pytest test_runner.py`), in
the style of the HOLY CORE `generic_tb_runner`:

```python
# test_runner.py
import os
from pathlib import Path
from cocotb.runner import get_runner

PROJ = Path(__file__).resolve().parent.parent

def generic_tb_runner(design_name, extra_sources=(), parameters=None, tag=""):
    sim = os.getenv("SIM", "verilator")
    sources = [PROJ / "packages" / "holy_dshot_pkg.sv"]
    sources += sorted((PROJ / "src").glob("*.sv"))
    sources += [PROJ / s for s in extra_sources]
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel=design_name,
        build_dir=f"./{design_name}/sim_build{tag}",
        parameters=parameters or {},
        build_args=["--trace", "--trace-structs"],
    )
    runner.test(hdl_toplevel=design_name,
                test_module=f"test_{design_name}",
                test_dir=f"./{design_name}")

def test_holy_dshot_engine():
    generic_tb_runner("holy_dshot_engine")

def test_holy_dshot_25mhz():
    generic_tb_runner("holy_dshot", parameters={"CLK_HZ": 25_000_000}, tag="_25")

def test_holy_dshot_100mhz():
    generic_tb_runner("holy_dshot", parameters={"CLK_HZ": 100_000_000}, tag="_100")

def test_holy_dshot_wrapper():
    generic_tb_runner("holy_dshot_wrapper",
                      extra_sources=["fpga/holy_dshot_wrapper.v"])

if __name__ == "__main__":
    test_holy_dshot_engine()
    test_holy_dshot_25mhz()
    test_holy_dshot_100mhz()
    test_holy_dshot_wrapper()
```

Testbenches read the clock frequency from the DUT parameter when possible, or from an
environment variable set by the runner, so the same test file works for both builds.

### 9.4 Testbench architecture
Each `test_<module>.py` contains:

1. **Clock and reset:** `Clock(dut.clk, period, units="ns")` matching `CLK_HZ`, and a
   `reset(dut)` coroutine (HOLY CORE style: drive reset low for a few cycles, release,
   check reset values).
2. **Sampling convention:** after `await RisingEdge(dut.clk)`, wait `await Timer(1, units="ns")`
   (or `ReadOnly()`) before reading outputs, consistently in every test.
3. **AXI-Lite BFM:** `AxiLiteMaster(AxiLiteBus.from_prefix(dut, "s_axi"), dut.clk, dut.rst_n, reset_active_level=False)`,
   helpers `wr(addr, val)` / `rd(addr)` built on `write_dword` / `read_dword`.
4. **Python reference model** (`DShotModel`), independent from the RTL:
   - `crc4(v)`, `frame_of(reg_value)` (section 3.1), `timing(clk_hz, speed)` (section 3.3);
   - a register/state model (staged, active, mode, state, errors, watchdog counter) used as a
     scoreboard in the random test.
5. **Pin monitor:** a coroutine sampling `dshot_o` every cycle into per-channel lists. A
   decoder turns samples into frames (rising edges, high time per bit, bit period), returning
   the bits, high times, bit periods, first-edge cycle and the gap to the next frame.
6. **Checks:** frame value vs. the model, every high time ∈ {T1H, T0H}, every bit period ==
   T_BIT, channel alignment, idle-low, frame start-to-start period in AUTO mode.
7. **Randomness:** `random` seeded by cocotb (seed printed in the log); every test must
   pass for any seed.

Internal signals may be probed through the hierarchy (`dut.<instance>.<signal>`), as the HOLY
CORE testbenches do, but black-box checks on ports and registers are preferred.

### 9.5 Test list

| ID | Module | Test | Pass criteria |
|---|---|---|---|
| T-01 | engine | Encode all 4096 values on each channel | frame == `frame_of(v)` for all values |
| T-02 | engine | Bit timing at 600/300/150 | periods == T_BIT, highs ∈ {T1H, T0H}, 0 tolerance |
| T-03 | engine | 4-channel alignment with different values | identical first-edge cycle on all channels |
| T-04 | holy_dshot | Reset values | every register == section 6 reset value; outputs low |
| T-05 | holy_dshot | AXI-Lite ordering | AW-before-W, W-before-AW, same cycle, back-to-back, random B/R ready back-pressure: all complete, correct data, no deadlock |
| T-06 | holy_dshot | Unmapped access, partial strobe | unmapped read = 0 / OKAY; `wstrb != 0xF` → SLVERR, no state change |
| T-07 | holy_dshot | TRIGGER: one frame per COMMIT | exactly 1 frame per commit, FRAMES += 1, BUSY during frame |
| T-08 | holy_dshot | TRIGGER: overrun | COMMIT during a frame → ERR_OVERRUN, values committed, no extra frame; W1C clears it |
| T-09 | holy_dshot | Staging and atomic commit | staged writes do not change output; after COMMIT all 4 change in the same frame; ACTIVE regs match |
| T-10 | holy_dshot | Commit during a frame | frame in flight unchanged, next frame uses new values |
| T-11 | holy_dshot | AUTO period | start-to-start == FRAME_PERIOD ± 0 cycles over ≥ 50 frames |
| T-12 | holy_dshot | FRAME_PERIOD clamp | value < 18·T_BIT → clamped, ERR_CFG set, readback = clamped |
| T-13 | holy_dshot | Watchdog timeout AUTO | no commit for WDG_TIMEOUT cycles → FAULT, NEED_RESET, FAULT_CAUSE = 1, fault_irq = 1, stop frames (0x0000) every FRAME_PERIOD |
| T-14 | holy_dshot | Watchdog refresh | commits every < WDG_TIMEOUT for ≥ 5 timeouts → never FAULT |
| T-15 | holy_dshot | Watchdog TRIGGER mode | WDG_EN = 0 → no fault; WDG_EN = 1 → fault as T-13 |
| T-16 | holy_dshot | FAULT is latched | COMMIT / EN toggles do not leave FAULT; SOFT_RESET → DISABLED, all cleared |
| T-17 | holy_dshot | Invalid command | CMD_EN = 0, staged 1–47 → channel keeps old active value, others committed, ERR_INVALID; CMD_EN = 1 → committed |
| T-18 | holy_dshot | Soft reset mid-frame | outputs low ≤ 1 cycle after the write, state DISABLED, registers reset, FRAMES preserved |
| T-19 | holy_dshot | Disable / enable | EN = 0 → no frames, outputs low; EN 0 → 1 → RUNNING, active = 0 |
| T-20 | holy_dshot | Speed change in flight | frame in flight keeps old timing, next frame uses new timing; INFO.T_BIT follows SPEED |
| T-21 | holy_dshot | Reserved SPEED = 3 | behaves as 600, ERR_CFG set |
| T-22 | holy_dshot | Random stress vs model | ≥ 20 000 cycles of random register operations; outputs, STATUS and ACTIVE match the Python model at every check |
| T-23 | holy_dshot | Two clock frequencies | T-02, T-11 and T-13 pass at CLK_HZ = 25 MHz and 100 MHz |
| T-24 | wrapper | Smoke test through the Verilog wrapper | reset values, one TRIGGER frame decoded correctly, one AUTO period checked |
| T-25 | all | Lint | `verilator --lint-only -Wall` clean on core and wrapper (REQ-31) |

### 9.6 Cleanup Makefile (project root, HOLY CORE convention)

```make
.PHONY: clean

clean:
	@find ./tb -type d -name "__pycache__" -exec rm -rf {} +
	@find ./tb -type d -name "sim_build*" -exec rm -rf {} +
	@find ./tb -type f -name "results.xml" -exec rm -f {} +
	@find ./tb -type f -name "*.None" -exec rm -f {} +
	@find ./tb -type d -name ".pytest_cache" -exec rm -rf {} +
	@find ./tb -type f -name "dump.vcd" -exec rm -f {} +
```

### 9.7 How the agent must run the verification
1. `verilator --lint-only -Wall packages/holy_dshot_pkg.sv src/*.sv` (and the wrapper).
2. `cd tb/<module> && make` for each module while developing.
3. `cd tb && pytest test_runner.py` must pass completely before delivery.
4. If a run fails without an obvious reason, run `make clean` at the root and retry
   (HOLY CORE tip).

## 10. Software deliverables

### 10.1 Driver `sw/hc_dshot.[ch]`
Freestanding C (no libc), builds without warnings with
`riscv32-unknown-elf-gcc -march=rv32im -mabi=ilp32 -Os -Wall -Wextra -ffreestanding`.
Register access through `volatile uint32_t` pointers only, 32-bit accesses only, no 64-bit
divisions.

| Function | Behaviour |
|---|---|
| `int dshot_init(dshot_speed_t speed, dshot_mode_t mode, uint32_t frame_period_us, uint32_t wdg_timeout_ms)` | soft reset, configure, check INFO (VERSION, N_CH), enable; returns 0 or a negative error |
| `int dshot_commit(const uint16_t v[4])` | write THROTTLE0..3, then COMMIT; never waits |
| `uint16_t dshot_throttle(uint32_t permille)` | 0 → 0 (stop); 1..1000 → 48..2047 (rounded) |
| `uint32_t dshot_status(void)` | raw STATUS |
| `int dshot_need_reset(void)` | 1 if NEED_RESET |
| `void dshot_clear_errors(void)` | W1C all sticky errors |
| `void dshot_soft_reset(void)` | SOFT_RESET |
| `int dshot_command(uint8_t motor, uint8_t cmd, uint8_t repeat)` | sets CMD_EN, sends cmd (telemetry bit set) `repeat` times at 1 ms, others at 0, clears CMD_EN |
| `void dshot_arm(uint32_t ms)` | commits zeros every 1 ms for `ms` |

Timing helpers use the HOLY CORE CLINT (`hc_mtime()`, `HC_CLK_HZ`) from `hc_time.h`.
Error codes: `DSHOT_OK 0`, `DSHOT_ERR_NODEV -400`, `DSHOT_ERR_ARG -402`, `DSHOT_ERR_FAULT -403`.

### 10.2 Bring-up program `sw/dshot_test/main.c`
- `MODE_SCOPE` (default, no ESC connected): AUTO mode 1 kHz, fixed pattern
  `{0, 48, 1046, 2047|telemetry}` (frames `0x0000 0x0606 0x82C6 0xFFFF`), prints frames/s
  and STATUS once per second.
- `MODE_ESC` (one ESC, props off): arm 3 s, ramp one motor 0 → 10 % → 0 over 12 s, then
  commit stop forever. Prints progress every 0.5 s.
- `MODE_WDG`: stop committing after 2 s; must report NEED_RESET and stop frames, then
  soft reset and resume.

## 11. Traceability matrix

| Requirement | Tests |
|---|---|
| REQ-01 alignment | T-03, T-07 |
| REQ-02 idle low, registered | T-04, T-18, T-19 |
| REQ-03 timing | T-02, T-20, T-23 |
| REQ-04 encoding | T-01, T-22 |
| REQ-05 speed change | T-20 |
| REQ-06 staging | T-09 |
| REQ-07 atomic commit | T-09 |
| REQ-08 latch at frame start | T-10 |
| REQ-09 commit refreshes watchdog | T-14 |
| REQ-10 TRIGGER mode | T-07, T-08 |
| REQ-11 AUTO mode | T-11, T-23 |
| REQ-12 period clamp | T-12 |
| REQ-13 mode switch | T-22 |
| REQ-14 watchdog activation | T-13, T-15 |
| REQ-15 watchdog timeout | T-13, T-23 |
| REQ-16 WDG_TIMEOUT = 0 | T-22 (directed case included) |
| REQ-17 stop frames in FAULT | T-13 |
| REQ-18 FAULT latched | T-16 |
| REQ-19 invalid command | T-17 |
| REQ-20 sticky W1C errors | T-08, T-12, T-17 |
| REQ-21 fault_irq | T-13, T-16 |
| REQ-22 READY / BUSY | T-07, T-16, T-19 |
| REQ-23 soft reset | T-16, T-18 |
| REQ-24 AXI ordering | T-05 |
| REQ-25 unmapped access | T-06 |
| REQ-26 partial strobe | T-06 |
| REQ-27 side-effect-free reads | T-22 |
| REQ-28 wrapper | T-24 |
| REQ-29, REQ-30, REQ-31 | T-25 + review |
| REQ-32, REQ-33 | Vivado synthesis report, if available (not blocking) |

## 12. Integration notes (for the readme)

- Add `fpga/holy_dshot_wrapper.v` to the Vivado block design as a module reference, connect
  `s_axi` to the AXI4-Lite crossbar fed by the HOLY CORE platform's external `m_axi_lite`,
  `aclk`/`aresetn` to the peripheral clock and reset, `fault_irq` to a free `irq_in` bit of
  the platform PLIC. Assign an address in `0x10000000–0x2FFFFFFF` (suggested `0x10080000`).
- Set `CLK_HZ` to the real clock of `aclk`.
- Software must access the IP as **non-cacheable** (the platform default range
  `0x00000000–0x7FFFFFFF` set through CSRs `0x7C1`–`0x7C4` already covers it).
- Example XDC, per output (gentle edges, since the ESC wiring runs next to power):
  `set_property -dict {PACKAGE_PIN <pin> IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports {dshot_o[0]}]`.
  Hardware: ~100 Ω series resistor near the FPGA pin, common ground with the ESCs.

## 13. Definition of done

- [ ] All files of section 8 exist; RTL implements every REQ.
- [ ] `cd tb && pytest test_runner.py` passes (all T-01 … T-24), for at least 3 different seeds.
- [ ] `make` works in every `tb/<module>/` folder and produces `dump.vcd`.
- [ ] Lint clean (T-25).
- [ ] Driver and bring-up program compile warning-free with the flags of 10.1.
- [ ] `readme.md` contains: overview, register map, state diagram (text or mermaid),
      integration notes, how to run tests, design decisions and any deviation from this document.


## MANUAL NOTES

Please add a 1 documentation readme.md exmplaining what the projects does, how to use in in an SoC (regmaps) et how it works overall. You can add a second page explinning the project structure.

This is a simple project. You shall only have 1 simple main .sv file with all the logic. Only if truly need you can add other files if it really help with readabilty and maintability.

Project should be easuily maintanable by hand and by a human, easily readble.

Project should not be complex and spagetthi. Only a src/ folder (with the very few source files) and a tb/ folder containning the cocotb tbs.

Make sure you floow the DSHOT standard and ensure AM32 compatibility on a cmd pin