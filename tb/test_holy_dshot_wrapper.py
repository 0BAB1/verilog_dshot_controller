# =============================================================================
# test_holy_dshot_wrapper.py - T-24: smoke test through the plain Verilog wrapper
# (holy_dshot_wrapper.v, Vivado block-design entry point). Uses the wrapper's
# own port names: aclk / aresetn.
# Run with: make MODULE=test_holy_dshot_wrapper TOPLEVEL=holy_dshot_wrapper
# =============================================================================
import random

import cocotb

from dshot_tb import (
    AUTO, COMMIT, CTRL, FPERIOD, INFO, STATUS, THR, TRIGGER, Bench, complete, ctrl, decode,
    frame_of, t_bit, values_of, SPD600, ACT, CLK_HZ,
)


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def t24_wrapper_smoke(dut):
    """T-24: reset values, one TRIGGER frame decoded correctly, one AUTO period checked."""
    b = Bench(dut, dut.aclk, dut.aresetn)
    await b.start()
    # Reset values
    assert int(dut.dshot_o.value) == 0
    assert await b.rd(CTRL) == 0
    assert await b.rd(FPERIOD) == CLK_HZ // 1000
    assert await b.rd(INFO) & 0xFFFF == t_bit(SPD600)
    # One TRIGGER frame
    await b.wr(CTRL, ctrl(en=1, mode=TRIGGER))
    vals = [0x030, 0x416, 0x7FF, 0x000]
    n0 = [len(b.pulses(ch)) for ch in range(4)]
    await b.commit_values(vals)
    await b.wait_frames_done()
    for ch in range(4):
        frames = complete(b.pulses(ch)[n0[ch]:], b.tb)
        assert values_of(frames) == [frame_of(vals[ch])], f"channel {ch}"
        assert await b.rd(ACT[ch]) == vals[ch]
    # One AUTO period (18 bit times, minimum)
    period = 18 * b.tb
    await b.wr(CTRL, 0)
    await b.wr(FPERIOD, period)
    await b.wr(CTRL, ctrl(en=1, mode=AUTO))
    n0 = len(b.pulses(0))
    await b.wait(6 * period)
    frames = complete(b.pulses(0)[n0:], b.tb)
    starts = [s for s, _ in frames]
    assert len(starts) >= 5
    assert {y - x for x, y in zip(starts, starts[1:])} == {period}
    assert await b.rd(STATUS) & 0x1 == 0x1, "not READY in AUTO"
