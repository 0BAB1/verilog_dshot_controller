# HOLY DSHOT - 4-channel DShot ESC controller

HOLY DSHOT is an AXI4-Lite peripheral that drives four ESCs (AM32 firmware) with
standard, unidirectional DShot: DShot150, DShot300 or DShot600. The FPGA builds the
frames and their timing in hardware, so software only writes throttle values and
commits them. It was written for the HOLY CORE SoC (RV32IM, 25 MHz) on an Arty S7-50,
but it has no dependency on the SoC beyond the AXI4-Lite bus and one interrupt line.

Files: `src/holy_dshot.sv` (the whole core), `src/holy_dshot_wrapper.v` (Verilog
entry point for Vivado block designs), `tb/` (cocotb testbench). The layout is
described in [STRUCTURE.md](STRUCTURE.md).

---

## 1. What it does

- Drives `dshot_o[3:0]`, one DShot line per ESC. The four frames of one frame event
  start on the same clock cycle.
- Two output modes:
  - **TRIGGER**: each write to `COMMIT` sends one frame on all channels.
  - **AUTO**: a frame is sent every `FRAME_PERIOD` cycles, using the last committed
    values.
- Staged/commit: software writes the four throttles into staging registers, then
  one `COMMIT` write makes all four active at once. The four motors always get values
  from the same control step.
- Safety: a watchdog. If no `COMMIT` arrives within `WDG_TIMEOUT` cycles, the IP
  latches a FAULT, sends stop frames (throttle 0) and raises `fault_irq`. Only a soft
  reset clears a fault.
- Software can read back the active values, the status, and a frame counter.

Out of scope: bidirectional DShot and telemetry, OneShot/Multishot, servo PWM, closed-loop
speed control.

## 2. How to use it in an SoC

### 2.1 Connections

| Signal | Connect to |
|---|---|
| `aclk` | the peripheral clock. `CLK_HZ` must be set to its frequency |
| `aresetn` | the peripheral reset (active low) |
| `s_axi_*` | an AXI4-Lite slave port of the crossbar (the HOLY CORE external `m_axi_lite`) |
| `dshot_o[3:0]` | the four ESC signal lines, through a ~100 Ω series resistor. Use gentle edges (`SLEW SLOW`, `DRIVE 4`). Common ground with the ESCs |
| `fault_irq` | a free `irq_in` bit of the platform PLIC. It is a level: high while `STATUS.NEED_RESET` is set |

Address: the suggested base is `0x10080000` (any address in `0x10000000–0x2FFFFFFF`).
Software must access it as non-cacheable (the default range covers it).

Parameters of `holy_dshot_wrapper` (or `holy_dshot`):

| Parameter | Default | Meaning |
|---|---|---|
| `CLK_HZ` | `25000000` | clock of `aclk`. Used for all DShot timing and reset values |
| `N_CH` | `4` | must stay 4 (checked at elaboration) |

Use `holy_dshot_wrapper.v` as a module reference in a Vivado block design. Vivado
module references do not accept the SystemVerilog top.

### 2.2 Register map

Base + offset. All registers are 32-bit and word aligned. Access: RW, R, W (reads 0),
W1C (write 1 to clear), W1S (write 1, self-clearing).

| Offset | Name | Access | Reset | Fields |
|---|---|---|---|---|
| `0x00` | `CTRL` | RW | `0x0` | `[0]` EN · `[1]` SOFT_RESET (W1S) · `[3:2]` SPEED (0 = 600, 1 = 300, 2 = 150, 3 = reserved, acts as 600 and sets ERR_CFG) · `[4]` MODE (0 = TRIGGER, 1 = AUTO) · `[5]` WDG_EN (TRIGGER only) · `[6]` CMD_EN |
| `0x04` | `STATUS` | R / W1C | `0x0` | `[0]` READY · `[1]` BUSY · `[2]` NEED_RESET · `[7:4]` FAULT_CAUSE (1 = watchdog) · `[8]` ERR_OVERRUN (W1C) · `[9]` ERR_INVALID (W1C) · `[10]` ERR_CFG (W1C) · `[13:12]` STATE (0 DISABLED, 1 RUNNING, 2 FAULT) |
| `0x08` | `COMMIT` | W | – | any full-word write: staged → active (all 4 at once), refresh watchdog, start a frame in TRIGGER mode |
| `0x0C` | `FRAME_PERIOD` | RW | `CLK_HZ/1000` | `[31:0]` AUTO and stop-frame period, in cycles. Clamped to `18 × T_BIT` (see 3.4) |
| `0x10` | `WDG_TIMEOUT` | RW | `CLK_HZ/20` | `[31:0]` watchdog timeout, in cycles (50 ms). 0 is refused: reset value used, ERR_CFG set |
| `0x14` | `FRAMES` | R | `0x0` | `[31:0]` frames started since hardware reset (wraps; kept by SOFT_RESET) |
| `0x18` | `INFO` | R | – | `[15:0]` T_BIT for the selected SPEED · `[23:16]` N_CH (4) · `[31:24]` VERSION (`0x01`) |
| `0x20`–`0x2C` | `THROTTLE0..3` | RW | `0x0` | staged value: `[10:0]` throttle · `[11]` telemetry request. Staged values never reach the pins by themselves |
| `0x30`–`0x3C` | `ACTIVE0..3` | R | `0x0` | active value (what the frames carry), same format |

Other behaviour of the bus interface:
- Accesses decode `addr[7:2]` only. Unmapped reads return 0 with OKAY, and unmapped writes are ignored with OKAY.
- A write with a strobe other than `0xF` returns SLVERR and changes nothing.
- Writes to read-only registers are ignored with OKAY.
- One outstanding write and one outstanding read.

### 2.3 Throttle encoding

A DShot throttle value is 11 bits:

| Value | Meaning |
|---|---|
| 0 | motor stop |
| 1–47 | special commands (only sent when `CMD_EN = 1`, see 3.3) |
| 48–2047 | throttle, 48 = minimum, 2047 = full |

Convert a permille throttle `p` (0–1000) to a register value with
`p == 0 ? 0 : 48 + round(p × 1999 / 1000)` (the C driver from the spec is not part of this delivery, see section 6).

### 2.4 Typical sequences

**Bring-up (TRIGGER mode, one frame per control step, 1 kHz):**

```
CTRL      = EN | SPEED<<2 | WDG_EN<<5          (TRIGGER mode = 0, speed 600)
WDG_TO    = 50 ms in cycles                    (optional, reset value is 50 ms)
loop every control step:
    THROTTLE0..3 = values                       (staged)
    COMMIT       = any                          (one frame on all channels)
```

**Continuous output (AUTO mode, 1 kHz):**

```
FRAME_PERIOD = CLK_HZ / 1000
CTRL         = EN | MODE<<4 | SPEED<<2          (AUTO)
loop every control step:
    THROTTLE0..3 = values
    COMMIT       = any                          (new values from the next frame on)
```

**Arming an ESC:** enable, then send stop frames (throttle 0) for a few seconds
before any throttle. In AUTO mode this happens automatically after EN.

**Recovering from a fault:** read `STATUS`. If `NEED_RESET` is set, write
`CTRL = SOFT_RESET`, then enable and resume as above. A soft reset also clears the
staged and active values.

## 3. How it works

### 3.1 Block overview

```
 AXI4-Lite  ──► handshake ──► register file ──► staging (THROTTLE0..3)
                                   │                 │ COMMIT (atomic)
                                   │                 ▼
                                   │           active (ACTIVE0..3)
                                   │                 │ latched at frame start
                                   ▼                 ▼
                         control FSM ────► frame engine ──► dshot_o[3:0]
                         (DISABLED /        (16-bit shift register per
                          RUNNING / FAULT,   channel, shared bit timer,
                          watchdog,          registered outputs)
                          period timer)
```

### 3.2 Frame and bit timing

A frame is 16 bits, MSB first: throttle `[15:5]`, telemetry request `[4]` and a CRC
`[3:0]` over the 12-bit payload `{throttle, telemetry}`:
`crc = v[3:0] ^ v[7:4] ^ v[11:8]`.

Every bit starts with a rising edge. The line is high for `T1H` cycles for a 1 bit or
`T0H` cycles for a 0 bit, then low for the rest of the bit period `T_BIT`:

| Speed | T_BIT | T1H | T0H |
|---|---|---|---|
| DShot600 | 42 | 32 | 16 |
| DShot300 | 83 | 62 | 31 |
| DShot150 | 167 | 125 | 63 |

These are the values at 25 MHz. They are computed as
`T_BIT = round(CLK_HZ / rate)`, `T1H = round(0.75 × T_BIT)`, `T0H = round(0.375 × T_BIT)`
(exact integer formulas, no tolerance).

All four channels share one bit counter, so their frames are aligned to the same
cycle by construction. A frame lasts `16 × T_BIT` cycles. Frame-to-frame spacing in AUTO
mode is exactly `FRAME_PERIOD`.

### 3.3 Commit, modes and CMD_EN

- `COMMIT` copies the four staged values to the active registers in the same
  cycle. A frame latches the active values when it starts.
- **TRIGGER**: a `COMMIT` starts one frame if none is in flight. If one is in flight,
  the values are still committed, no frame starts, and `ERR_OVERRUN` is set.
- **AUTO**: a frame starts every `FRAME_PERIOD` cycles. `COMMIT` only updates the values
  and refreshes the watchdog.
- **CMD_EN = 0** (default): a staged throttle of 1–47 is rejected for that channel only.
  The channel keeps its previous active value and `ERR_INVALID` is set. The other
  channels are committed normally. **CMD_EN = 1** sends values 1–47 as they are.

### 3.4 Frame period and speed

`FRAME_PERIOD` must be at least `18 × T_BIT` (one frame plus two idle bits). A smaller
write is raised to that minimum and sets `ERR_CFG`. Reads return the value in use.
When `SPEED` changes, the period is clamped again to the new minimum (also setting
`ERR_CFG` if it changed).

### 3.5 States

```
            EN 0→1                 watchdog timeout
 DISABLED ───────────► RUNNING ─────────────────────► FAULT
    ▲                    │  ▲                            │
    │      EN 1→0        │  │                            │
    └────────────────────┘  │   (FAULT is latched:      │
                            │    only SOFT_RESET leaves) │
    SOFT_RESET (from any state) ──► DISABLED ◄───────────┘
```

| State | Outputs |
|---|---|
| DISABLED | low |
| RUNNING | TRIGGER: one frame per COMMIT. AUTO: a frame every `FRAME_PERIOD` |
| FAULT (EN = 1) | stop frames (throttle 0) every `FRAME_PERIOD`, whatever MODE is |
| FAULT (EN = 0) | low |

- `EN 1→0` aborts the frame in flight and drives the outputs low at once.
- Entering DISABLED → RUNNING clears the active values to 0 (stop).
- `SOFT_RESET` aborts the frame in flight, resets all registers (including `CTRL`) and
  clears the errors and the fault. `FRAMES` is kept.

### 3.6 Watchdog

- Counts cycles since the last `COMMIT` (or since entering RUNNING). When the count
  reaches `WDG_TIMEOUT`, the state becomes FAULT with `FAULT_CAUSE = 1`.
- Active in AUTO always, in TRIGGER only with `WDG_EN`.

### 3.7 Errors

`ERR_OVERRUN`, `ERR_INVALID` and `ERR_CFG` are sticky, cleared by writing 1 to the bit,
and never stop operation. `fault_irq` = `NEED_RESET` = "state is FAULT".

## 4. Design decisions

Decisions taken where the spec was open, or where the spec and the manual notes
differ:

1. **File layout.** Following the manual notes, all RTL lives in `src/`: one main
   file `holy_dshot.sv` and the Verilog wrapper (required by Vivado). There is no
   separate package or engine file, and there is no `fpga/` or `sw/` folder. The
   constants live in the module.
2. **Testbench.** The cocotb testbench uses its own small AXI4-Lite master
   (`tb/dshot_tb.py`) instead of `cocotbext-axi`'s `AxiLiteMaster`. The BFM keeps
   `bready`/`rready` high, so it consumes responses before a test can sample them.
   The testbench also bounds every handshake wait (200 cycles) and every test has a
   simulation-time limit, so a stuck bus fails with a message instead of hanging.
3. **Commit in any state.** `COMMIT` copies staged → active in every state. Only the
   RUNNING state starts frames and refreshes the watchdog. Enabling clears the active
   values anyway.
4. **Watchdog counter runs in every state.** It restarts on a COMMIT and on entering
   RUNNING. Enabling `WDG_EN` after a long idle time therefore faults at once if the
   timeout has already passed. Write a COMMIT first.
5. **A watchdog fault does not abort the frame in flight.** The stop frames start at
   the next period. Only `EN 1→0` and `SOFT_RESET` abort a frame.
6. **AUTO start waits for a free channel.** If a speed change makes the frame in flight
   longer than the new period, the next AUTO frame is delayed until the frame ends.
   The spacing is then longer than `FRAME_PERIOD` once. This happens only in that
   corner case.
7. **`FRAMES` counts every frame start**, including stop frames sent in FAULT.
8. **Register decode uses `addr[7:2]`** as specified, so addresses 0x100 apart alias.
9. **`dshot_o` is registered.** The next-state logic is combinational, so each output
   changes one cycle after the state that drives it. All four channels share that
   timing.
10. **Soft reset and the AXI handshake.** The soft reset is a combinational path from
    the register write to the control registers. It is not a path to `dshot_o`:
    the output flops are reset on the next edge, which meets "low ≤ 1 cycle after the
    write".

## 5. Verification

The testbench in `tb/` runs 21 cocotb tests that follow the test list in
`requirements.md` section 9.5 (T-01, T-02 with T-03 in one test, T-04 … T-22) and a
wrapper smoke test (T-24). Checks are black-box: AXI registers and the `dshot_o` pins
against a Python reference model. All tests run at 25 MHz.

```bash
cd tb
pytest test_runner.py              # core + wrapper, once each (~95 s)
make                               # core tests with waveform (dump.vcd)
make MODULE=test_holy_dshot_wrapper TOPLEVEL=holy_dshot_wrapper   # wrapper only
make clean
```

Lint (T-25): `verilator --lint-only -Wall --top-module holy_dshot src/holy_dshot.sv`
and the same for `holy_dshot_wrapper` with `src/holy_dshot_wrapper.v`. Both are clean.

Not run: the 100 MHz build (T-23) and multi-seed runs. The design is parameterised
by `CLK_HZ`, but this is not verified at other clock frequencies.

Requirements: Python ≥ 3.10, cocotb 1.9.x, Verilator ≥ 5.020, pytest ≥ 7
(see `tb/requirements.txt`).

## 6. Not covered by simulation

- Timing closure and utilization on the Spartan-7 (REQ-32, REQ-33). Run Vivado synthesis.
- ESC behaviour: arming, signal-loss failsafe, reaction to special commands. Verify on
  hardware with the ESCs props off.
- The C driver and bring-up program from the spec (section 10) are not part of this
  delivery, following the manual note to keep the project to `src/` and `tb/`.
