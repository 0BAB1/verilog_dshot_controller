# HOLY DSHOT – usage

Four DShot300 outputs for AM32 ESCs, controlled over AXI4-Lite. Write a throttle per
motor. The IP sends it continuously. No commit, no telemetry.

## 1. Registers

Base address + offset. 32-bit, word-aligned, full-word writes only.

| Offset | Name | Access | Reset | Bits |
|---|---|---|---|---|
| `0x00` | `CTRL` | RW | `0` | `[0]` EN · `[1]` SOFT_RESET (write-only, reads 0) |
| `0x04` | `THROTTLE0` | RW | `0` | `[10:0]` throttle, motor 0 |
| `0x08` | `THROTTLE1` | RW | `0` | `[10:0]` throttle, motor 1 |
| `0x0C` | `THROTTLE2` | RW | `0` | `[10:0]` throttle, motor 2 |
| `0x10` | `THROTTLE3` | RW | `0` | `[10:0]` throttle, motor 3 |

Bits not listed are ignored on write and read as 0.

### Throttle values

| Value | Motor output |
|---|---|
| `0` | stop |
| `1` – `47` | stop (these are DShot commands, the IP never sends them) |
| `48` – `2047` | throttle; 48 = minimum, 2047 = maximum |

### Converting a percentage

```c
// p = 0..1000 (0.0 %..100.0 %)
uint32_t thr = (p == 0) ? 0 : 48 + (p * 1999) / 1000;
```

## 2. Bus behaviour

- Write with `wstrb != 0xF` → `BRESP = SLVERR`, nothing changes. Always write full words.
- Read or write of an unmapped address → read returns 0, write is ignored, both `OKAY`.
- Reads have no side effects.
- The IP is one AXI write and one AXI read at a time. Wait for `BVALID` / `RVALID`.

## 3. Procedures

### 3.1 Start the motors

1. Enable: `CTRL = 1`.
   All four channels now send stop frames. Throttles are cleared to 0.
2. Wait for the ESCs to arm. AM32 arms after it has received stop frames for a while.
   The exact time depends on the ESC firmware. Measure it on your ESC.
3. Write the throttles: `THROTTLE0..3 = value`.
   Each new value applies to the next frame of its motor (within 500 µs).

Do not write throttles before step 1. Enable clears them, so an earlier write is lost.
Do not skip step 2. Writing a throttle before arming can make the ESC ignore the signal.

### 3.2 Change speed

Write the throttle register. Nothing else is needed. Writes can be as frequent as the
control loop wants. A new value is used from the next frame.

### 3.3 Stop the motors

Either:

- **Set throttles to 0** (soft stop). Motors stop, the IP keeps running, ESCs stay armed.
- **Write `CTRL = 0`** (hard stop). Every channel sends stop frames within one frame
  (within 500 µs). Throttle registers keep their values.

Use `CTRL = 0` for emergencies. To run again, write `CTRL = 1`, wait for arming, then
write the throttles again (enable cleared them).

### 3.4 Reset

| Write | Effect |
|---|---|
| `CTRL = 2` (soft reset) | EN and all throttles cleared. Motors get stop frames. |
| `rst_n` (hard reset) | Same as soft reset. Outputs are low during reset. |

After any reset, the motors are stopped. The IP sends stop frames again once reset is
released. To run again, follow section 3.1.

A hard reset in the middle of a frame can leave one short pulse on a line. ESCs ignore
it. The frames after reset are complete.

### 3.5 Reading state

- `CTRL` bit 0 = EN, the current enable state.
- `THROTTLEn` = the value last written, or 0 after enable or reset.

There is no status register, no interrupt and no fault flag.

## 4. Behaviour by state

| Current state | Event | Result |
|---|---|---|
| Disabled | write throttle | stored, not sent. Still stop on the line. |
| Disabled | `CTRL = 1` | enabled. Throttles cleared. Stop frames. |
| Enabled | write throttle | new value on the next frame |
| Enabled | write `CTRL = 1` again | no change. Throttles kept. |
| Enabled | `CTRL = 0` | stop frames within one frame. Throttles kept. |
| Enabled | `CTRL = 2` / reset | throttles and EN cleared. Stop frames. |
| Any | `CTRL = 3` | same as soft reset (SOFT_RESET wins over EN) |

## 5. Driver checklist

- Initialise: `CTRL = 2` (known state), then `CTRL = 1`.
- Wait for the ESC arming time before the first throttle write.
- Clamp every throttle to `0` or `48..2047` before writing. Never write `1..47`.
- Check `BRESP == OKAY` on every write.
- Send the throttle at least once per control step. Nothing stops the motors if you go
  silent: there is no watchdog. Use a software timeout that writes 0 or `CTRL = 0`.
- Before reset or power-down, write `CTRL = 0` (hard stop) and let the motors stop.

## 6. Timing

- DShot300 at `CLK_HZ` (default 25 MHz). Bit period 83 cycles, `1` high 62, `0` high 31.
- Each channel sends one 16-bit frame every 500 µs (2 kHz). The frame itself is 18 bit times (about 60 µs); the rest is idle, line low.
- All four channels start their frames on the same clock cycle.
- Frame format: throttle (11 bits), telemetry bit (always 0), CRC (4 bits).
- Set `CLK_HZ` to the real `aclk` frequency. The timing is computed from it.

## 7. Not included

- Status or fault reporting, interrupts, watchdog.
- Telemetry, RPM, bidirectional DShot.
- DShot commands (1–47): blocked on purpose.
- ESC presence detection: the IP cannot tell whether an ESC is connected.
- Atomic write of all four throttles: each register is written separately, so one
  frame can carry a mix of new and old values.
