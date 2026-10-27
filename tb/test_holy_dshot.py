# =============================================================================
# test_holy_dshot.py - cocotb tests for holy_dshot (DShot300, 4 channels).
#
# Checks are on the pins (decoded frames) and on the AXI registers.
# Run with `make` (core) or `make TOPLEVEL=holy_dshot_wrapper` (wrapper).
# =============================================================================
import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer

CLK_HZ = int(os.environ.get("CLK_HZ", "25000000"))

# DShot300 timing (requirements section 3.3)
T = (CLK_HZ + 150_000) // 300_000          # bit period, cycles
T1H = (3 * T + 2) // 4
T0H = (3 * T + 4) // 8
FRAME = 18 * T                              # 16 bits + 2 idle bit times

# Register map
A_CTRL = 0x0000
A_THR = [0x0004, 0x0008, 0x000C, 0x0010]

HS_TIMEOUT = 200                            # cycles to wait for any AXI handshake


def frame_of(thr, en=1):
    """Expected DShot300 frame for a throttle register value (section 3.1)."""
    t = thr if (en and thr >= 48) else 0    # stop unless enabled and a real throttle
    v = t << 1                              # {throttle, telemetry = 0}
    return (v << 4) | ((v ^ (v >> 4) ^ (v >> 8)) & 0xF)


class Bench:
    def __init__(self, dut):
        self.dut = dut
        # The core uses clk / rst_n, the wrapper uses aclk / aresetn
        self.clk = dut.clk if hasattr(dut, "clk") else dut.aclk
        self.rst = dut.rst_n if hasattr(dut, "rst_n") else dut.aresetn
        self.pulses = [[] for _ in range(4)]   # (rise_cycle, fall_cycle) per channel
        self.cycle = 0
        self.reset_cycle = 0

    async def start(self):
        cocotb.start_soon(Clock(self.clk, 1e9 / CLK_HZ, units="ns").start())
        cocotb.start_soon(self._monitor())
        d = self.dut
        for sig in (d.s_axi_awvalid, d.s_axi_wvalid, d.s_axi_arvalid,
                    d.s_axi_bready, d.s_axi_rready):
            sig.value = 0
        await self.reset()

    async def reset(self):
        """Hardware reset: rst_n low for 5 cycles."""
        self.rst.value = 0
        await self.wait(5)
        self.rst.value = 1
        await self.wait(2)
        self.reset_cycle = self.cycle          # frames before this may be runts (see readme)

    async def _monitor(self):
        prev = [0] * 4
        rise = [0] * 4
        while True:
            await RisingEdge(self.clk)
            await ReadOnly()
            v = int(self.dut.dshot_o.value)
            for ch in range(4):
                bit = (v >> ch) & 1
                if bit and not prev[ch]:
                    rise[ch] = self.cycle
                elif prev[ch] and not bit:
                    self.pulses[ch].append((rise[ch], self.cycle))
                prev[ch] = bit
            self.cycle += 1

    async def wait(self, n):
        for _ in range(n):
            await RisingEdge(self.clk)
        await Timer(1, units="ns")

    async def _accept(self, valid, ready, what):
        """valid is high: return after the rising edge where the handshake happens."""
        for _ in range(HS_TIMEOUT):
            await FallingEdge(self.clk)
            await ReadOnly()
            if int(ready.value):
                break
        else:
            raise AssertionError(f"AXI {what}: ready never high")
        await RisingEdge(self.clk)
        await Timer(1, units="ns")
        valid.value = 0

    async def write(self, addr, data, strb=0xF, aw_delay=0, w_delay=0):
        """AXI write with explicit AW/W timing. Returns BRESP."""
        d = self.dut

        async def aw_phase():
            await self.wait(aw_delay)
            d.s_axi_awaddr.value = addr
            d.s_axi_awvalid.value = 1
            await self._accept(d.s_axi_awvalid, d.s_axi_awready, "AW")

        async def w_phase():
            await self.wait(w_delay)
            d.s_axi_wdata.value = data
            d.s_axi_wstrb.value = strb
            d.s_axi_wvalid.value = 1
            await self._accept(d.s_axi_wvalid, d.s_axi_wready, "W")

        a = cocotb.start_soon(aw_phase())
        w = cocotb.start_soon(w_phase())
        await a
        await w
        d.s_axi_bready.value = 1
        for _ in range(HS_TIMEOUT):
            await FallingEdge(self.clk)
            await ReadOnly()
            if int(d.s_axi_bvalid.value):
                break
        else:
            raise AssertionError("AXI B: response never came")
        resp = int(d.s_axi_bresp.value)
        await RisingEdge(self.clk)
        await Timer(1, units="ns")
        d.s_axi_bready.value = 0
        return resp

    async def read(self, addr):
        """AXI read. Returns (data, RRESP)."""
        d = self.dut
        d.s_axi_araddr.value = addr
        d.s_axi_arvalid.value = 1
        await self._accept(d.s_axi_arvalid, d.s_axi_arready, "AR")
        d.s_axi_rready.value = 1
        for _ in range(HS_TIMEOUT):
            await FallingEdge(self.clk)
            await ReadOnly()
            if int(d.s_axi_rvalid.value):
                break
        else:
            raise AssertionError("AXI R: data never came")
        data, resp = int(d.s_axi_rdata.value), int(d.s_axi_rresp.value)
        await RisingEdge(self.clk)
        await Timer(1, units="ns")
        d.s_axi_rready.value = 0
        return data, resp

    # Simple helpers: full-word access that must succeed
    async def wr(self, addr, val):
        assert await self.write(addr, val) == 0, f"write {addr:#x} failed"

    async def rd(self, addr):
        data, resp = await self.read(addr)
        assert resp == 0, f"read {addr:#x} failed"
        return data

    async def frames(self, n, settle=2):
        """Per-channel lists of n complete frames, all starting after `settle` frames.

        The settle time lets the write take effect. Frames that were already in
        flight when the call started are ignored.
        """
        t0 = self.cycle + settle * FRAME
        await self.wait((settle + n + 1) * FRAME)
        out = []
        for ch in range(4):
            fr = [f for f in decode(self.pulses[ch], self.reset_cycle) if f[0] >= t0]
            assert len(fr) >= n, f"channel {ch}: only {len(fr)} frames"
            out.append(fr[:n])
        starts = [f[0] for f in out[0]]
        for ch in range(1, 4):
            assert [f[0] for f in out[ch]] == starts, "channels not aligned"
        for s0, s1 in zip(starts, starts[1:]):
            assert s1 - s0 == FRAME, f"frame spacing {s1 - s0}, expected {FRAME}"
        return [[v for _, v in out[ch]] for ch in range(4)]


def decode(pulses, skip_before=0):
    """Pulses -> complete frames (start, value). A frame still in flight at the end is dropped.

    A frame is 16 pulses, rising edges T apart, high for T1H (1) or T0H (0).
    Frames starting before `skip_before` (a hard reset cut them short) are ignored.
    Anything else is a bug and fails the test.
    """
    frames = []
    i = 0
    while i < len(pulses):
        start = pulses[i][0]
        bits = []
        j = i
        while j < len(pulses) and len(bits) < 16 and pulses[j][0] == start + len(bits) * T:
            high = pulses[j][1] - pulses[j][0]
            bits.append(1 if high == T1H else 0 if high == T0H else None)
            j += 1
        if start < skip_before:
            i = j
            continue
        if None in bits:
            raise AssertionError(f"bad pulse in frame at cycle {start}")
        if len(bits) == 16:
            frames.append((start, int("".join(map(str, bits)), 2)))
        elif j < len(pulses):
            raise AssertionError(f"broken frame at cycle {start}")
        i = j
    return frames


async def start(dut):
    bench = Bench(dut)
    await bench.start()
    return bench


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_reset_sends_stop(dut):
    """After reset: registers are 0, EN is 0, all channels send stop frames."""
    b = await start(dut)
    assert await b.rd(A_CTRL) == 0
    for addr in A_THR:
        assert await b.rd(addr) == 0
    for ch, vals in enumerate(await b.frames(3)):
        assert vals == [0, 0, 0], f"channel {ch}: {vals}"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_throttle_frames(dut):
    """EN=1 and four throttles: every channel sends its frame with the right CRC and timing."""
    b = await start(dut)
    thr = [48, 1046, 2047, 1000]
    await b.wr(A_CTRL, 1)
    for ch in range(4):
        await b.wr(A_THR[ch], thr[ch])
    frames = await b.frames(3)
    for ch in range(4):
        assert frames[ch] == [frame_of(thr[ch])] * 3, f"channel {ch}: {[hex(v) for v in frames[ch]]}"
        assert await b.rd(A_THR[ch]) == thr[ch]
    assert await b.rd(A_CTRL) == 1


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_hard_stop(dut):
    """EN=0 stops all channels within one frame. EN=1 starts from stop, never from a stale value."""
    b = await start(dut)
    await b.wr(A_CTRL, 1)
    await b.wr(A_THR[0], 1000)
    assert (await b.frames(2))[0] == [frame_of(1000)] * 2
    await b.wr(A_CTRL, 0)
    await b.wait(FRAME)                       # at most one frame of latency
    for vals in await b.frames(2):
        assert vals == [0, 0]
    await b.wr(A_CTRL, 1)
    assert (await b.frames(2))[0] == [0, 0]
    assert await b.rd(A_THR[0]) == 0


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_commands_blocked(dut):
    """Throttle 1..47 is a DShot command: it is never sent, the channel sends stop instead."""
    b = await start(dut)
    thr = [0, 1, 47, 48]
    await b.wr(A_CTRL, 1)
    for ch in range(4):
        await b.wr(A_THR[ch], thr[ch])
    frames = await b.frames(2)
    assert [f[0] for f in frames] == [0, 0, 0, frame_of(48)]
    assert [await b.rd(a) for a in A_THR] == thr   # the register keeps what was written


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_soft_reset(dut):
    """SOFT_RESET clears the registers and EN. Outputs go to stop."""
    b = await start(dut)
    await b.wr(A_CTRL, 1)
    await b.wr(A_THR[2], 500)
    assert (await b.frames(2))[2] == [frame_of(500)] * 2
    await b.wr(A_CTRL, 2)                     # SOFT_RESET
    assert await b.rd(A_CTRL) == 0
    for addr in A_THR:
        assert await b.rd(addr) == 0
    for vals in await b.frames(2):
        assert vals == [0, 0]


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_axi_access(dut):
    """Partial strobe is refused. Unmapped access is harmless. AW/W order does not matter."""
    b = await start(dut)
    assert await b.write(A_THR[0], 0x123) == 0
    assert await b.write(A_THR[0], 0x456, strb=0x3) == 2       # SLVERR, no change
    assert await b.rd(A_THR[0]) == 0x123
    assert await b.write(0x0040, 0xFFFFFFFF) == 0               # unmapped write: ignored
    assert await b.read(0x0040) == (0, 0)                       # unmapped read: 0
    assert await b.write(A_THR[1], 0x222, aw_delay=3) == 0      # AW first
    assert await b.write(A_THR[2], 0x333, w_delay=3) == 0       # W first
    assert await b.rd(A_THR[1]) == 0x222
    assert await b.rd(A_THR[2]) == 0x333


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=60, timeout_unit="ms")
async def test_random(dut):
    """Random writes, enable toggles, soft resets and waits. Every frame is stop or a value written."""
    b = await start(dut)
    rng = random.Random(1)
    allowed = {0}                              # frame values that may legitimately appear
    for _ in range(150):
        r = rng.random()
        if r < 0.5:
            ch = rng.randrange(4)
            val = rng.choice([rng.randrange(0, 48), rng.randrange(48, 2048)])
            await b.wr(A_THR[ch], val)
            allowed.add(frame_of(val))
        elif r < 0.7:
            await b.wr(A_CTRL, rng.randrange(2))   # EN only
        elif r < 0.75:
            await b.wr(A_CTRL, 2)                  # SOFT_RESET
        else:
            await b.wait(rng.randrange(1, 3) * FRAME)
    frames = await b.frames(20)                # also checks spacing and alignment
    for ch in range(4):
        assert set(frames[ch]) <= allowed, f"channel {ch}: unexpected frame value"
    # And the whole run stayed valid: every complete frame decodes with exact timing
    decode(b.pulses[0])


# -----------------------------------------------------------------------------
# AM32-oriented tests
# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_enable_starts_stopped(dut):
    """AM32 arms on stop frames. A stale throttle must not spin the motor on enable."""
    b = await start(dut)
    await b.wr(A_THR[1], 500)                 # written while disabled
    await b.wr(A_CTRL, 1)
    for ch, vals in enumerate(await b.frames(300)):
        assert set(vals) == {0}, f"channel {ch} spun up from a stale value"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=120, timeout_unit="ms")
async def test_edge_throttles(dut):
    """Edge values: stop, command limit, minimum, powers of two and maximum give the right frame."""
    b = await start(dut)
    await b.wr(A_CTRL, 1)
    edges = [0, 1, 47, 48, 49, 127, 128, 255, 256, 511, 512, 1023, 1024, 2046, 2047]
    for i in range(0, len(edges), 4):
        batch = (edges[i:i + 4] + [0, 0, 0, 0])[:4]
        for ch in range(4):
            await b.wr(A_THR[ch], batch[ch])
        frames = await b.frames(1)
        for ch in range(4):
            assert frames[ch] == [frame_of(batch[ch])], f"throttle {batch[ch]} on channel {ch}"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_change_mid_frame(dut):
    """A throttle write in the middle of a frame: frames are old values, then new ones, never a mix."""
    b = await start(dut)
    old, new = 100, 800
    await b.wr(A_CTRL, 1)
    await b.wr(A_THR[0], old)
    await b.wait(2 * FRAME)                    # old value settled
    t0 = b.cycle
    await b.wait(FRAME + FRAME // 2)           # the write lands inside a frame
    await b.wr(A_THR[0], new)
    await b.wait(3 * FRAME)
    seq = [f[1] for f in decode(b.pulses[0]) if f[0] >= t0]
    assert seq, "no frames"
    allowed = [frame_of(old), frame_of(new)]
    assert set(seq) <= set(allowed), "frame mixes old and new values"
    switch = seq.index(allowed[1])             # from here on only the new value
    assert all(v == allowed[1] for v in seq[switch:]), "value went back to old"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=60, timeout_unit="ms")
async def test_reset_in_each_state(dut):
    """Soft and hard reset from disabled, running and mid-frame: stop, clean registers, whole frames."""
    b = await start(dut)

    async def setup(state):
        if state == "disabled":
            await b.wr(A_THR[0], 700)
            await b.wr(A_THR[1], 900)
        else:
            await b.wr(A_CTRL, 1)
            await b.wr(A_THR[0], 700)
            await b.wr(A_THR[1], 900)
            if state == "mid_frame":
                await b.wait(FRAME // 2 + 7)   # reset lands inside a frame

    for state in ("disabled", "running", "mid_frame"):
        for kind in ("soft", "hard"):
            await setup(state)
            if kind == "soft":
                await b.wr(A_CTRL, 2)
            else:
                await b.reset()
            assert await b.rd(A_CTRL) == 0, f"{state}/{kind}: CTRL not cleared"
            for addr in A_THR:
                assert await b.rd(addr) == 0, f"{state}/{kind}: throttle not cleared"
            for ch, vals in enumerate(await b.frames(3)):
                assert set(vals) == {0}, f"{state}/{kind}: channel {ch} not stopped"


# -----------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_rewrite_enable_and_soft_reset_bit(dut):
    """Writing EN=1 again while running keeps the throttles. SOFT_RESET wins over EN in the same write."""
    b = await start(dut)
    await b.wr(A_CTRL, 1)
    await b.wr(A_THR[0], 700)
    await b.wr(A_CTRL, 1)                       # no edge: must not stop the motor
    assert (await b.frames(2))[0] == [frame_of(700)] * 2
    assert await b.rd(A_THR[0]) == 700
    await b.wr(A_CTRL, 3)                       # EN and SOFT_RESET together
    assert await b.rd(A_CTRL) == 0
    assert await b.rd(A_THR[0]) == 0
    for vals in await b.frames(2):
        assert vals == [0, 0]
