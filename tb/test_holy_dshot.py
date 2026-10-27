# =============================================================================
# test_holy_dshot.py - cocotb tests for the holy_dshot core
#
# Test IDs (T-xx) follow requirements.md section 9.5. Checks are black-box:
# AXI registers and the dshot_o pins, compared with the reference model in
# dshot_tb.py. Run with `make` (or `CLK_HZ=100000000 make` for 100 MHz).
# =============================================================================
import random

import cocotb
from cocotb.triggers import FallingEdge, ReadOnly

from dshot_tb import (
    ACT, BUSY, CLK_HZ, COMMIT, CTRL, DISABLED, EN, ERR_CFG, ERR_INVALID, ERR_OVERRUN,
    FAULT, FPERIOD, FRAMES, INFO, NEED_RESET, READY, RUNNING, SOFT_RESET, SPD150,
    SPD300, SPD600, SPD_RSVD, STATUS, THR, TRIGGER, AUTO, WDG_TO, Bench, complete,
    ctrl, decode, fault_cause_of, frame_of, idle, state_of, t_bit, values_of,
)


async def start(dut):
    """Create the bench, start clock and monitor, reset the DUT."""
    bench = Bench(dut, dut.clk, dut.rst_n)
    await bench.start()
    return bench


async def wait_until(bench, addr, mask, want, max_cycles):
    """Poll a register until (value & mask) == want. Returns elapsed cycles."""
    t0 = bench.mon.cycle
    while (await bench.rd(addr)) & mask != want:
        if bench.mon.cycle - t0 > max_cycles:
            raise AssertionError(f"timeout waiting for reg 0x{addr:02x} & 0x{mask:x} == 0x{want:x}")
    return bench.mon.cycle - t0


def rand_vals(rng):
    """Random register values with throttle 48..2047 (valid with CMD_EN=0 too)."""
    return [rng.randrange(48, 2048) | (rng.getrandbits(1) << 11) for _ in range(4)]


def check_frames(frames, vals):
    """Each channel carries one frame per entry with the expected value."""
    for ch in range(4):
        got = values_of(frames[ch])
        exp = [frame_of(vals[ch])] * len(got)
        assert got == exp, f"channel {ch}: got {[hex(g) for g in got]}, expected {[hex(e) for e in exp]}"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t04_reset_values(dut):
    """T-04: every register at its reset value, outputs low."""
    b = await start(dut)
    assert int(dut.dshot_o.value) == 0, "outputs not low after reset"
    assert int(dut.fault_irq.value) == 0
    assert await b.rd(CTRL) == 0
    assert await b.rd(STATUS) == 0
    assert await b.rd(FPERIOD) == CLK_HZ // 1000, "FRAME_PERIOD reset value"
    assert await b.rd(WDG_TO) == CLK_HZ // 20, "WDG_TIMEOUT reset value"
    assert await b.rd(FRAMES) == 0
    assert await b.rd(INFO) == (1 << 24) | (4 << 16) | t_bit(SPD600), "INFO"
    for addr in THR + ACT:
        assert await b.rd(addr) == 0, f"reg 0x{addr:02x} not zero"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t05_axi_ordering(dut):
    """T-05: AW/W order, same cycle, back-to-back, B/R back-pressure. No deadlock."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    cases = [(0, 0, 0), (0, 4, 0), (4, 0, 0), (0, 0, 5), (2, 2, 3), (7, 1, 0)]
    for aw_d, w_d, b_d in cases:
        val = rng.getrandbits(12)
        resp = await b.raw_write(THR[0], val, aw_delay=aw_d, w_delay=w_d, b_delay=b_d)
        assert resp == 0, f"BRESP {resp} for aw={aw_d} w={w_d} b={b_d}"
        data, rresp = await b.raw_read(THR[0], r_delay=rng.randrange(4))
        assert rresp == 0 and data == val, f"readback {data:#x} != {val:#x}"
    # Back-to-back writes with no gaps, then read everything back
    for ch in range(4):
        assert await b.raw_write(THR[ch], 0x100 + ch) == 0
    for ch in range(4):
        assert (await b.raw_read(THR[ch]))[0] == 0x100 + ch


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t06_unmapped_and_strobe(dut):
    """T-06: unmapped reads 0/OKAY, unmapped writes ignored, partial strobe -> SLVERR, no change."""
    b = await start(dut)
    await b.wr(THR[0], 0x123)
    assert await b.raw_write(THR[0], 0xABC, strb=0x3) == 2, "partial strobe must be SLVERR"
    assert await b.rd(THR[0]) == 0x123, "partial strobe changed state"
    assert await b.raw_write(0x40, 0xFFFF_FFFF) == 0, "unmapped write must be OKAY"
    assert (await b.raw_read(0x40)) == (0, 0), "unmapped read must be 0/OKAY"
    assert await b.raw_write(0x40, 0x1, strb=0x1) == 2
    # Read-only register: write ignored, reads unchanged
    assert await b.raw_write(FRAMES, 0x55) == 0
    assert await b.rd(FRAMES) == 0
    # Partial strobe on CTRL must not enable the IP
    assert await b.raw_write(CTRL, ctrl(en=1, speed=SPD150), strb=0x1) == 2
    assert await b.rd(CTRL) == 0
    assert state_of(await b.rd(STATUS)) == DISABLED


# -----------------------------------------------------------------------------
# 1024 frames x ~800 cycles each: needs more simulated time than the others
@cocotb.test(timeout_time=80, timeout_unit="ms")
async def t01_encoding_all_values(dut):
    """T-01 / T-04: every 12-bit register value encodes bit-exactly on all channels."""
    b = await start(dut)
    # CMD_EN=1: the sweep includes command values 1..47, which must be sent as-is
    await b.wr(CTRL, ctrl(en=1, speed=SPD600, mode=TRIGGER, cmd=1))
    expected = [[], [], [], []]
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    for base in range(0, 4096, 4):
        vals = [base + ch for ch in range(4)]
        await b.commit_values(vals)
        await b.wait_frames_done(tb=b.tb)
        for ch in range(4):
            expected[ch].append(frame_of(vals[ch]))
    for ch in range(4):
        got = values_of(decode(b.pulses(ch)[n0[ch]:], b.tb))
        assert len(got) == 1024, f"channel {ch}: {len(got)} frames"
        assert got == expected[ch], f"channel {ch}: first mismatch " + str(
            next((i, hex(g), hex(e)) for i, (g, e) in enumerate(zip(got, expected[ch])) if g != e))


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t02_t03_timing_and_alignment(dut):
    """T-02 / T-03 / T-23: bit timing exact at 600/300/150, identical frame start on all channels."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    for speed in (SPD600, SPD300, SPD150):
        tb = t_bit(speed)
        await b.wr(CTRL, ctrl(en=1, speed=speed, mode=TRIGGER))
        assert await b.rd(INFO) & 0xFFFF == tb, f"INFO.T_BIT for speed {speed}"
        vals = rand_vals(rng)
        n0 = [len(b.pulses(ch)) for ch in range(4)]
        await b.commit_values(vals)
        await b.wait_frames_done(tb=tb)
        frames = [decode(b.pulses(ch)[n0[ch]:], tb) for ch in range(4)]   # strict: 0-cycle tolerance
        check_frames(frames, vals)
        starts = {frames[ch][0][0] for ch in range(4)}
        assert len(starts) == 1, f"channels not aligned: {starts}"
        dut._log.info(f"speed {speed}: T_BIT={tb} T1H={t1h_of(tb)} T0H={t0h_of(tb)} OK")


def t1h_of(tb):
    return (3 * tb + 2) // 4


def t0h_of(tb):
    return (3 * tb + 4) // 8


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t07_trigger_one_frame(dut):
    """T-07: TRIGGER: exactly one frame per COMMIT, FRAMES += 1, BUSY during the frame."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER))
    for _ in range(3):
        vals = rand_vals(rng)
        f0 = await b.rd(FRAMES)
        n0 = [len(b.pulses(ch)) for ch in range(4)]
        await b.commit_values(vals)
        assert await b.rd(STATUS) & BUSY, "BUSY not set during the frame"
        await b.wait_frames_done()
        assert await b.rd(FRAMES) == f0 + 1, "FRAMES must count one frame"
        frames = [complete(b.pulses(ch)[n0[ch]:], b.tb) for ch in range(4)]
        assert all(len(frames[ch]) == 1 for ch in range(4)), "not exactly one frame per commit"
        check_frames(frames, vals)
        assert not await b.rd(STATUS) & BUSY


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t08_trigger_overrun(dut):
    """T-08: COMMIT during a frame: ERR_OVERRUN, values committed, no extra frame; W1C clears."""
    b = await start(dut)
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER))
    val_a = [0x100, 0x200, 0x300, 0x400]
    val_b = [0x0A0, 0x0B0, 0x0C0, 0x0D0]
    val_c = [0x010, 0x020, 0x030, 0x040]
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.commit_values(val_a)        # starts frame A
    await b.commit_values(val_b)        # frame A still in flight -> overrun
    assert await b.rd(STATUS) & ERR_OVERRUN, "ERR_OVERRUN not set"
    for ch in range(4):
        assert await b.rd(ACT[ch]) == val_b[ch], "overrun commit must still update ACTIVE"
    await b.wait_frames_done()
    frames = [complete(b.pulses(ch)[n0[ch]:], b.tb) for ch in range(4)]
    assert all(len(frames[ch]) == 1 for ch in range(4)), "overrun must not start a second frame"
    check_frames(frames, val_a)
    await b.wr(STATUS, ERR_OVERRUN)
    assert not await b.rd(STATUS) & ERR_OVERRUN, "W1C did not clear ERR_OVERRUN"
    # A commit while idle does not set the error
    await b.commit_values(val_c)
    await b.wait_frames_done()
    assert not await b.rd(STATUS) & ERR_OVERRUN


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t09_staging_atomic(dut):
    """T-09: staged writes do not change outputs; one COMMIT switches all 4 channels in the same frame."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    period = 20 * b.tb
    await b.wr(FPERIOD, period)
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    await b.wait(3 * period)
    vals = rand_vals(rng)
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    for ch in range(4):
        await b.wr(THR[ch], vals[ch])
    await b.wait(4 * period)
    frames = [complete(b.pulses(ch)[n0[ch]:], b.tb) for ch in range(4)]
    for ch in range(4):
        assert all(v == 0 for v in values_of(frames[ch])), "staged writes reached the outputs"
        assert await b.rd(ACT[ch]) == 0, "staged write changed ACTIVE"
    n1 = [len(b.pulses(ch)) for ch in range(4)]
    await b.wr(COMMIT, 0)
    await b.wait(4 * period)
    frames = [complete(b.pulses(ch)[n1[ch]:], b.tb) for ch in range(4)]
    switch = []
    for ch in range(4):
        got = values_of(frames[ch])
        k = next(i for i, v in enumerate(got) if v != 0)
        assert got[k:] == [frame_of(vals[ch])] * (len(got) - k), f"channel {ch} not new after switch"
        switch.append((k, frames[ch][k][0]))
        assert await b.rd(ACT[ch]) == vals[ch]
    assert len({s for s in switch}) == 1, "channels switch in different frames"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t10_commit_during_frame(dut):
    """T-10: a commit during a frame does not alter that frame; the next frame uses the new values."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    period = 20 * b.tb
    await b.wr(FPERIOD, period)
    val_a, val_b = rand_vals(rng), rand_vals(rng)
    # Enable first: EN 0->1 clears ACTIVE, so the values are committed while RUNNING
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    await b.commit_values(val_a)
    await b.wait(2 * period)
    # Wait for the start of the next frame (its first pulse has just fallen)
    n0 = len(b.pulses(0))
    for _ in range(2 * period):
        if len(b.pulses(0)) != n0:
            break
        await FallingEdge(dut.clk)
    else:
        raise AssertionError("no AUTO frame started within 2 periods")
    await b.wait(5 * b.tb)                  # well inside that frame
    await b.commit_values(val_b)
    await b.wait(4 * period)
    frames = [complete(b.pulses(ch)[n0:], b.tb) for ch in range(4)]
    for ch in range(4):
        got = values_of(frames[ch])
        assert len(got) >= 3
        assert got[0] == frame_of(val_a[ch]), "frame in flight was altered by the commit"
        assert got[1:] == [frame_of(val_b[ch])] * (len(got) - 1), "next frames use the new values"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t11_auto_period(dut):
    """T-11: AUTO start-to-start equals FRAME_PERIOD exactly, over 50+ frames."""
    b = await start(dut)
    for period in (18 * b.tb, 25 * b.tb, 60 * b.tb):
        await b.wr(CTRL, 0)                              # restart the scheduler
        await b.wr(FPERIOD, period)
        assert await b.rd(FPERIOD) == period
        await b.wr(CTRL, ctrl(en=1, mode=AUTO))
        n0 = len(b.pulses(0))
        await b.wait(56 * period)
        frames = complete(b.pulses(0)[n0:], b.tb)
        starts = [s for s, _ in frames]
        assert len(starts) >= 50, f"only {len(starts)} frames"
        diffs = {y - x for x, y in zip(starts, starts[1:])}
        assert diffs == {period}, f"start-to-start {diffs}, expected {period}"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t12_period_clamp(dut):
    """T-12: FRAME_PERIOD below 18*T_BIT is clamped and sets ERR_CFG; speed change re-clamps."""
    b = await start(dut)
    await b.wr(FPERIOD, 10)
    assert await b.rd(FPERIOD) == 18 * b.tb, "clamped readback"
    assert await b.rd(STATUS) & ERR_CFG, "ERR_CFG not set"
    await b.wr(STATUS, ERR_CFG)
    assert not await b.rd(STATUS) & ERR_CFG, "W1C did not clear ERR_CFG"
    await b.wr(FPERIOD, 100_000)
    assert await b.rd(FPERIOD) == 100_000 and not await b.rd(STATUS) & ERR_CFG
    # Speed change to 150 must raise the minimum: period re-clamped, ERR_CFG set
    await b.wr(FPERIOD, 18 * b.tb)
    assert not await b.rd(STATUS) & ERR_CFG
    await b.wr(CTRL, ctrl(en=0, speed=SPD150))
    assert await b.rd(FPERIOD) == 18 * t_bit(SPD150), "speed change did not re-clamp"
    assert await b.rd(STATUS) & ERR_CFG
    # WDG_TIMEOUT = 0 is invalid: reset value used, ERR_CFG set
    await b.wr(STATUS, ERR_CFG)
    await b.wr(WDG_TO, 0)
    assert await b.rd(WDG_TO) == CLK_HZ // 20
    assert await b.rd(STATUS) & ERR_CFG


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t13_watchdog_auto(dut):
    """T-13: AUTO with no COMMIT -> FAULT (watchdog), NEED_RESET, fault_irq, stop frames every FRAME_PERIOD."""
    b = await start(dut)
    period = 20 * b.tb
    wdg = 4 * period
    await b.wr(FPERIOD, period)
    await b.wr(WDG_TO, wdg)
    t0 = b.mon.cycle
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    elapsed = await wait_until(b, STATUS, NEED_RESET, NEED_RESET, 4 * wdg)
    elapsed = b.mon.cycle - t0
    assert wdg <= elapsed <= wdg + 50, f"fault after {elapsed} cycles, expected ~{wdg}"
    st = await b.rd(STATUS)
    assert state_of(st) == FAULT and fault_cause_of(st) == 1, f"STATUS {st:#x}"
    assert int(dut.fault_irq.value) == 1, "fault_irq not asserted"
    n0 = len(b.pulses(0))
    await b.wait(6 * period)
    frames = complete(b.pulses(0)[n0:], b.tb)
    assert len(frames) >= 5, "no stop frames in FAULT"
    assert all(v == 0x0000 for v in values_of(frames)), "FAULT must send stop frames (0x0000)"
    diffs = {y - x for (x, _), (y, _) in zip(frames, frames[1:])}
    assert diffs == {period}, f"stop-frame period {diffs}, expected {period}"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t14_watchdog_refresh(dut):
    """T-14: a COMMIT every half timeout for 5+ timeouts never faults."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    wdg = 30 * b.tb
    await b.wr(FPERIOD, 20 * b.tb)
    await b.wr(WDG_TO, wdg)
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    await b.commit_values(rand_vals(rng))
    total = 0
    while total < 6 * wdg:
        await b.wait(wdg // 2)
        total += wdg // 2
        await b.commit_values(rand_vals(rng))
    st = await b.rd(STATUS)
    assert st & READY and not st & NEED_RESET and state_of(st) == RUNNING, f"STATUS {st:#x}"
    assert int(dut.fault_irq.value) == 0


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t15_watchdog_trigger(dut):
    """T-15: TRIGGER: WDG_EN=0 never faults; WDG_EN=1 faults after the timeout."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    wdg = 20 * b.tb
    await b.wr(WDG_TO, wdg)
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, wdg=0))
    await b.commit_values(rand_vals(rng))
    await b.wait_frames_done()
    await b.wait(3 * wdg)
    st = await b.rd(STATUS)
    assert st & READY and not st & NEED_RESET, "fault with WDG_EN=0 in TRIGGER"
    # Re-enter RUNNING with WDG_EN=1: the timeout counts from entering RUNNING
    await b.wr(CTRL, ctrl(en=0))
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, wdg=1))
    elapsed = await wait_until(b, STATUS, NEED_RESET, NEED_RESET, 4 * wdg)
    assert wdg <= elapsed <= wdg + 50, f"fault after {elapsed} cycles, expected ~{wdg}"
    assert fault_cause_of(await b.rd(STATUS)) == 1


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t16_fault_latched(dut):
    """T-16: FAULT is latched through COMMIT and EN toggles; SOFT_RESET clears everything but FRAMES."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    wdg = 20 * b.tb
    await b.wr(WDG_TO, wdg)
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, wdg=1))
    await b.commit_values(rand_vals(rng))
    await b.wait_frames_done()
    await wait_until(b, STATUS, NEED_RESET, NEED_RESET, 4 * wdg)
    await b.commit_values(rand_vals(rng))
    await b.wr(CTRL, ctrl(en=0, mode=TRIGGER, wdg=1))           # EN 1->0
    assert int(dut.dshot_o.value) == 0 and state_of(await b.rd(STATUS)) == FAULT
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, wdg=1))           # EN 0->1
    st = await b.rd(STATUS)
    assert state_of(st) == FAULT and st & NEED_RESET and not st & READY
    await b.commit_values(rand_vals(rng))
    assert state_of(await b.rd(STATUS)) == FAULT
    frames_before = await b.rd(FRAMES)
    assert frames_before > 0
    await b.wr(CTRL, SOFT_RESET)
    st = await b.rd(STATUS)
    assert st == 0, f"STATUS after soft reset {st:#x}"
    assert int(dut.fault_irq.value) == 0
    assert await b.rd(CTRL) == 0
    assert await b.rd(FPERIOD) == CLK_HZ // 1000
    for addr in THR + ACT:
        assert await b.rd(addr) == 0
    frames_after = await b.rd(FRAMES)
    assert frames_after >= frames_before, "FRAMES must be preserved by SOFT_RESET"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t17_invalid_command(dut):
    """T-17: CMD_EN=0 keeps the old value for command throttles 1..47; CMD_EN=1 commits them."""
    b = await start(dut)
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, cmd=0))
    base = [0x200, 0x300, 0x400, 0x500]
    await b.commit_values(base)
    await b.wait_frames_done()
    staged = [5, 0x111, 47, 48]          # 5 and 47 are commands, 48 is the lowest throttle
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.commit_values(staged)
    await b.wait_frames_done()
    expected = [base[0], staged[1], base[2], staged[3]]
    for ch in range(4):
        assert await b.rd(ACT[ch]) == expected[ch], f"ACT{ch}"
    st = await b.rd(STATUS)
    assert st & ERR_INVALID, "ERR_INVALID not set"
    frames = [complete(b.pulses(ch)[n0[ch]:], b.tb) for ch in range(4)]
    check_frames(frames, expected)
    await b.wr(STATUS, ERR_INVALID)
    assert not await b.rd(STATUS) & ERR_INVALID
    # CMD_EN=1: the same staged values are committed normally
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, cmd=1))
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.commit_values(staged)
    await b.wait_frames_done()
    for ch in range(4):
        assert await b.rd(ACT[ch]) == staged[ch]
    assert not await b.rd(STATUS) & ERR_INVALID
    frames = [complete(b.pulses(ch)[n0[ch]:], b.tb) for ch in range(4)]
    check_frames(frames, staged)


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t18_soft_reset_midframe(dut):
    """T-18: SOFT_RESET during a frame: outputs low within 1 cycle, registers reset, FRAMES kept."""
    b = await start(dut)
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER))
    await b.commit_values([0x123, 0x456, 0x789, 0x7FF])
    await b.wait(8 * b.tb)                 # mid-frame
    frames_before = await b.rd(FRAMES)
    n_commit = [len(b.pulses(ch)) for ch in range(4)]
    await b.wr(CTRL, SOFT_RESET)
    assert int(dut.dshot_o.value) == 0, "outputs not low right after SOFT_RESET"
    await b.wait(2)
    assert int(dut.dshot_o.value) == 0
    n_after = [len(b.pulses(ch)) for ch in range(4)]
    await b.wait(3 * 16 * b.tb)
    assert [len(b.pulses(ch)) for ch in range(4)] == n_after, "pulses continued after soft reset"
    # The frame in flight at the reset is aborted: it has started but has fewer than 16 bits
    for ch in range(4):
        n_bits = len(b.pulses(ch)) - n_commit[ch]
        assert 0 < n_bits < 16, f"channel {ch}: {n_bits} bits sent, expected an aborted frame"
    st = await b.rd(STATUS)
    assert st == 0, f"STATUS after soft reset {st:#x}"
    assert await b.rd(CTRL) == 0
    for addr in THR + ACT:
        assert await b.rd(addr) == 0
    assert await b.rd(FRAMES) >= frames_before


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t19_disable_enable(dut):
    """T-19: EN=0 stops frames and drives outputs low; EN 0->1 restarts with active values = 0."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    period = 20 * b.tb
    await b.wr(FPERIOD, period)
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    await b.commit_values(rand_vals(rng))
    await b.wait(2 * period)
    await b.wr(CTRL, ctrl(en=0, mode=AUTO))
    assert int(dut.dshot_o.value) == 0, "outputs not low right after EN=0"
    st = await b.rd(STATUS)
    assert state_of(st) == DISABLED and not st & READY
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.wait(3 * period)
    assert [len(b.pulses(ch)) for ch in range(4)] == n0, "frames while disabled"
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    assert state_of(await b.rd(STATUS)) == RUNNING
    for ch in range(4):
        assert await b.rd(ACT[ch]) == 0, "active values not stop (0) on enable"
    await b.wait(4 * period)
    frames = complete(b.pulses(0)[n0[0]:], b.tb)
    assert len(frames) >= 3 and all(v == 0 for v in values_of(frames)), "not stop frames after enable"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t20_speed_change_inflight(dut):
    """T-20: a speed change during a frame: that frame keeps its timing, the next uses the new speed."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    await b.wr(CTRL, ctrl(en=1, speed=SPD600, mode=TRIGGER))
    val1 = rand_vals(rng)
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.commit_values(val1)                       # frame at 600 starts
    await b.wait(2 * b.tb)
    await b.wr(CTRL, ctrl(en=1, speed=SPD150, mode=TRIGGER))   # mid-frame speed change
    assert await b.rd(INFO) & 0xFFFF == t_bit(SPD150), "INFO.T_BIT follows SPEED"
    await b.wait_frames_done(tb=b.tb)
    frames = [decode(b.pulses(ch)[n0[ch]:], t_bit(SPD600)) for ch in range(4)]   # 600 timing, strict
    check_frames(frames, val1)
    assert all(len(frames[ch]) == 1 for ch in range(4))
    tb150 = t_bit(SPD150)
    n1 = [len(b.pulses(ch)) for ch in range(4)]
    val2 = rand_vals(rng)
    await b.commit_values(val2)
    await b.wait_frames_done(tb=tb150)
    frames = [decode(b.pulses(ch)[n1[ch]:], tb150) for ch in range(4)]           # 150 timing, strict
    check_frames(frames, val2)


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t21_reserved_speed(dut):
    """T-21: SPEED=3 behaves as 600 and sets ERR_CFG."""
    b = await start(dut)
    await b.wr(CTRL, ctrl(en=1, speed=SPD_RSVD, mode=TRIGGER))
    assert await b.rd(CTRL) == ctrl(en=1, speed=SPD_RSVD, mode=TRIGGER)
    assert await b.rd(STATUS) & ERR_CFG, "reserved SPEED must set ERR_CFG"
    assert await b.rd(INFO) & 0xFFFF == t_bit(SPD600), "reserved SPEED must time as 600"
    rng = random.Random(cocotb.RANDOM_SEED)
    vals = rand_vals(rng)
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.commit_values(vals)
    await b.wait_frames_done(tb=t_bit(SPD600))
    frames = [decode(b.pulses(ch)[n0[ch]:], t_bit(SPD600)) for ch in range(4)]
    check_frames(frames, vals)


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t22_random_stress(dut):
    """T-22: random register operations; ACTIVE, STATUS and every frame match the Python model."""
    b = await start(dut)
    rng = random.Random(cocotb.RANDOM_SEED)
    cmd_en = 0
    staged = [0] * 4
    active = [0] * 4
    invalid = False
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, cmd=cmd_en))
    commits = 0
    for _ in range(400):
        r = rng.random()
        if r < 0.35:                                      # stage one value
            ch = rng.randrange(4)
            val = rng.randrange(0, 60) if rng.random() < 0.3 else rng.getrandbits(12)
            await b.wr(THR[ch], val)
            staged[ch] = val
        elif r < 0.50:                                    # commit (wait for the frame in flight first)
            await b.wait_frames_done()
            n0 = [len(b.pulses(ch)) for ch in range(4)]
            await b.wr(COMMIT, 0)
            commits += 1
            for ch in range(4):
                if not cmd_en and 1 <= (staged[ch] & 0x7FF) <= 47:
                    invalid = True                        # keep the previous active value
                else:
                    active[ch] = staged[ch]
            await b.wait_frames_done()
            frames = [complete(b.pulses(ch)[n0[ch]:], b.tb) for ch in range(4)]
            assert all(len(frames[ch]) == 1 for ch in range(4)), "commit did not give one frame"
            check_frames(frames, active)
        elif r < 0.60:                                    # toggle CMD_EN
            cmd_en ^= 1
            await b.wr(CTRL, ctrl(en=1, mode=TRIGGER, cmd=cmd_en))
        elif r < 0.68:                                    # clear the sticky error
            await b.wr(STATUS, ERR_INVALID)
            invalid = False
        else:                                             # read back
            ch = rng.randrange(4)
            assert await b.rd(ACT[ch]) == active[ch], f"ACT{ch} mismatch"
            st = await b.rd(STATUS)
            assert bool(st & ERR_INVALID) == invalid, "ERR_INVALID mismatch"
            assert not st & ERR_OVERRUN, "unexpected overrun"
            assert state_of(st) == RUNNING
    assert commits > 10
    for ch in range(4):
        assert await b.rd(ACT[ch]) == active[ch]
        assert await b.rd(THR[ch]) == staged[ch]
    dut._log.info(f"random stress: {commits} commits, seed {cocotb.RANDOM_SEED}")
