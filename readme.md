# HOLY DSHOT – 4-channel DShot300 ESC output

> [!DANGER]
> This repo was heavily vibe coded

An AXI4-Lite peripheral that sends DShot300 throttle to four ESCs (AM32).

## USER GUIDE

Registers (base address + offset, 32-bit, word aligned):

| Offset | Name | Access | Reset | Meaning |
|---|---|---|---|---|
| `0x00` | `CTRL` | RW | `0` | `[0]` EN · `[1]` SOFT_RESET (write 1, reads 0) |
| `0x04` | `THROTTLE0` | RW | `0` | `[10:0]` throttle, channel 0 |
| `0x08` | `THROTTLE1` | RW | `0` | `[10:0]` throttle, channel 1 |
| `0x0C` | `THROTTLE2` | RW | `0` | `[10:0]` throttle, channel 2 |
| `0x10` | `THROTTLE3` | RW | `0` | `[10:0]` throttle, channel 3 |

**Throttle values:**

| Value | Output |
|---|---|
| 0 | stop |
| 1–47 | stop (DShot commands are blocked) |
| 48–2047 | throttle, 48 = minimum, 2047 = maximum |

**Typical sequence:**

```
CTRL = 1                      enable: all channels send stop, throttles are cleared
wait a few seconds            AM32 arms on stop frames
THROTTLE0..3 = value          change at any time, it applies to the next frame
...
CTRL = 0                      hard stop: all channels send stop frames
CTRL = 1                      enable again: starts at stop, write throttles again
CTRL = 2                      soft reset: registers and EN cleared, stop frames
```

**Other behaviour:**

- Writes with `wstrb != 0xF` return SLVERR and change nothing.
- Unmapped addresses: reads return 0, writes are ignored (both OKAY).
- Reads have no side effects.

## Safety

- **Stop by default.** After reset, and whenever EN = 0, every channel sends a
  stop frame (throttle 0). The ESCs stay armed and stopped, and never see a missing signal.
- **No commands.** Throttles 1–47 are never sent, so a wrong value cannot trigger an
  ESC command.
- **Hard stop.** `CTRL.EN = 0` takes effect on the next frame, within about 60 µs.
- **Enable starts stopped.** Every enable (EN 0→1) clears the throttle registers, so a
  stale value cannot spin a motor. Write the throttles after enabling.
- **Reset is safe.** Reset and soft reset both clear the registers to 0 and disable
  the output, so the motors go to stop. A soft reset keeps the frame timing. A hard
  reset (`rst_n`) drops the lines low at once, so a frame in flight can leave one short
  pulse (a runt). ESCs ignore runts, and the next frames are complete.

## Project structure

```
dshot_controller/
├── requirements.md          specification
├── readme.md                usage, register map, behaviour
├── STRUCTURE.md             this page
├── src/
│   ├── holy_dshot.sv        the core: AXI4-Lite slave, registers, frame engine
│   └── holy_dshot_wrapper.v plain Verilog wrapper for Vivado (no logic)
└── tb/
    ├── Makefile             cocotb + Verilator run
    ├── requirements.txt     cocotb 1.9.2
    └── test_holy_dshot.py   all tests (core and wrapper)
```

## How it works

Each channel sends a 16-bit frame every 18 bit times (about 60 µs at 25 MHz). The frame
has the throttle, telemetry bit 0, and a 4-bit CRC (`crc = v[3:0]^v[7:4]^v[11:8]`).
All four channels share one bit counter, so their frames start on the same cycle.

At each frame start the engine reads the register values and sends them, so a register
write applies to the next frame of that channel. The four registers are written one at a
time, so a single frame may carry one new and three old values. This is fine for an
open-loop motor output.

DShot300 timing at `CLK_HZ` (cycles): `T_BIT = (CLK_HZ + 150 000) / 300 000`,
`T1H = round(0.75·T_BIT)`, `T0H = round(0.375·T_BIT)`. At 25 MHz: 83 / 62 / 31.

Outputs come directly from flip-flops and nothing combinational reaches them from AXI.

## Build and test

```bash
cd tb
make                              # core, 12 tests (~55 s)
make TOPLEVEL=holy_dshot_wrapper  # Vivado wrapper, same tests
make clean
```

Lint:

```bash
verilator --lint-only -Wall --top-module holy_dshot src/holy_dshot.sv
verilator --lint-only -Wall --top-module holy_dshot_wrapper src/holy_dshot.sv src/holy_dshot_wrapper.v
```

Parameter: `CLK_HZ` (default 25 MHz). It sets the DShot timing. Set it to the real `aclk`
frequency.

## Not included

- No status register and no interrupt: nothing in the spec needs them.
- No watchdog. If software stops writing, the motors keep the last throttle. Add a
  timeout in software, or ask for a hardware watchdog.
- No ESC feedback. A missing ESC cannot be detected.
- Timing closure on the Spartan-7 is not checked here. Run Vivado synthesis.
