# =============================================================================
# dshot_tb.py - shared helpers for the holy_dshot cocotb testbenches
#   - reference model (frame encoding, bit timing), independent of the RTL
#   - pin monitor: records every DShot pulse on dshot_o, cycle accurate
#   - bench: AXI4-Lite access (cocotbext-axi BFM) plus raw AXI with explicit
#     AW/W/B/R timing for the ordering and strobe tests
#
# Timing convention: testbench inputs change 1 ns after a rising edge, outputs
# are sampled in ReadOnly after a rising edge, and handshakes are detected in
# ReadOnly after a falling edge (inputs are stable there), so no race.
# =============================================================================
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge, Timer

CLK_HZ = int(os.environ.get("CLK_HZ", "25000000"))

# A handshake that is not seen within this many cycles is a bug (DUT or testbench),
# so fail with a message instead of waiting forever.
HS_TIMEOUT_CYCLES = 200

# ---- Register map (section 6) ----------------------------------------------
CTRL, STATUS, COMMIT, FPERIOD, WDG_TO, FRAMES, INFO = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18
THR = [0x20, 0x24, 0x28, 0x2C]
ACT = [0x30, 0x34, 0x38, 0x3C]

EN, SOFT_RESET = 1 << 0, 1 << 1
READY, BUSY, NEED_RESET = 1 << 0, 1 << 1, 1 << 2
ERR_OVERRUN, ERR_INVALID, ERR_CFG = 1 << 8, 1 << 9, 1 << 10
DISABLED, RUNNING, FAULT = 0, 1, 2
SPD600, SPD300, SPD150, SPD_RSVD = 0, 1, 2, 3
TRIGGER, AUTO = 0, 1


def ctrl(en=0, speed=0, mode=0, wdg=0, cmd=0):
    """Build a CTRL register value."""
    return en | (speed << 2) | (mode << 4) | (wdg << 5) | (cmd << 6)


def state_of(status):
    """STATUS.STATE field."""
    return (status >> 12) & 0x3


def fault_cause_of(status):
    """STATUS.FAULT_CAUSE field."""
    return (status >> 4) & 0xF


# ---- Reference model (sections 3.1 - 3.3) ----------------------------------
def t_bit(speed, clk=CLK_HZ):
    """Bit period in cycles, rounded to nearest. Reserved speed acts as 600."""
    rate = {SPD300: 300_000, SPD150: 150_000}.get(speed, 600_000)
    return (clk + rate // 2) // rate


def t1h(tb):
    return (3 * tb + 2) // 4


def t0h(tb):
    return (3 * tb + 4) // 8


def crc4(v):
    return (v ^ (v >> 4) ^ (v >> 8)) & 0xF


def frame_of(reg):
    """Register value {tel, throttle[10:0]} -> 16-bit DShot frame."""
    thr = reg & 0x7FF
    tel = (reg >> 11) & 1
    v = (thr << 1) | tel
    return (v << 4) | crc4(v)


# ---- Pin monitor -----------------------------------------------------------
class PinMonitor:
    """Records (rise_cycle, fall_cycle) of every high pulse on each channel.

    cycle counts sampled clock edges since the monitor started. A pulse is
    stored when its line falls, so a frame in progress is not yet complete.
    """

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.cycle = 0
        self.pulses = [[] for _ in range(4)]
        self._prev = [0] * 4
        self._rise = [0] * 4
        self._task = None

    def start(self):
        self._task = cocotb.start_soon(self._run())

    async def _run(self):
        while True:
            await RisingEdge(self.clk)
            await ReadOnly()
            v = int(self.dut.dshot_o.value)
            for ch in range(4):
                bit = (v >> ch) & 1
                if bit and not self._prev[ch]:
                    self._rise[ch] = self.cycle
                elif self._prev[ch] and not bit:
                    self.pulses[ch].append((self._rise[ch], self.cycle))
                self._prev[ch] = bit
            self.cycle += 1


def decode(pulses, tb, strict=True):
    """Group pulses into 16-bit frames. Returns a list of (start_cycle, value).

    A frame is 16 pulses whose rising edges are exactly tb cycles apart. Each
    high time must be exactly T1H (bit 1) or T0H (bit 0). With strict=False an
    incomplete frame (aborted by EN or a soft reset) is returned with value None.
    """
    t1, t0 = t1h(tb), t0h(tb)
    frames = []
    i = 0
    while i < len(pulses):
        start = pulses[i][0]
        bits = []
        j = i
        while j < len(pulses) and len(bits) < 16 and pulses[j][0] == start + len(bits) * tb:
            high = pulses[j][1] - pulses[j][0]
            if high == t1:
                bits.append(1)
            elif high == t0:
                bits.append(0)
            else:
                raise AssertionError(
                    f"bit at cycle {pulses[j][0]}: high time {high}, expected {t0} or {t1}")
            j += 1
        if len(bits) == 16:
            frames.append((start, int("".join(map(str, bits)), 2)))
        elif strict:
            raise AssertionError(f"incomplete frame at cycle {start}: {len(bits)} bits")
        else:
            frames.append((start, None))
        i = j
    return frames


def complete(pulses, tb):
    """Only the complete frames (drops a frame still in flight at the window end)."""
    return [f for f in decode(pulses, tb, strict=False) if f[1] is not None]


def values_of(frames):
    return [v for _, v in frames]


# ---- Bench -----------------------------------------------------------------
# All AXI traffic goes through the raw driver below. cocotbext-axi's AxiLiteMaster
# keeps bready/rready high, so it would consume responses that the testbench is
# still waiting to sample (e.g. after a delay), so it is not used on this bus.
async def idle(clk, n=1):
    """Wait n rising edges, then move 1 ns past the edge (safe to drive)."""
    for _ in range(n):
        await RisingEdge(clk)
    await Timer(1, units="ns")


class Bench:
    """Owns the clock, reset, pin monitor and AXI access for one DUT."""

    def __init__(self, dut, clk, rst, clk_hz=CLK_HZ):
        self.dut = dut
        self.clk = clk
        self.rst = rst
        self.clk_hz = clk_hz
        self.tb = t_bit(SPD600, clk_hz)
        self.mon = PinMonitor(dut, clk)

    async def start(self):
        cocotb.start_soon(Clock(self.clk, 1e9 / self.clk_hz, units="ns").start())
        self.mon.start()
        self.dut.s_axi_awvalid.value = 0
        self.dut.s_axi_wvalid.value = 0
        self.dut.s_axi_arvalid.value = 0
        self.dut.s_axi_bready.value = 0
        self.dut.s_axi_rready.value = 0
        self.rst.value = 0
        await ClockCycles(self.clk, 5)
        await Timer(1, units="ns")
        self.rst.value = 1
        await idle(self.clk, 2)

    # ---- Register access (normal operations, via the raw AXI driver below) ----
    async def wr(self, addr, val):
        resp = await self.raw_write(addr, val)
        assert resp == 0, f"write to {addr:#x} returned BRESP {resp}"

    async def rd(self, addr):
        data, resp = await self.raw_read(addr)
        assert resp == 0, f"read of {addr:#x} returned RRESP {resp}"
        return data

    async def wait(self, n):
        await idle(self.clk, n)

    async def wait_frames_done(self, tb=None, extra=4):
        """Wait until the frame in flight has finished (tb: bit period in cycles)."""
        await idle(self.clk, 16 * (tb or self.tb) + extra)

    async def commit_values(self, vals):
        """Stage four values and commit them (TRIGGER: starts one frame)."""
        for ch in range(4):
            await self.wr(THR[ch], vals[ch])
        await self.wr(COMMIT, 0)

    def pulses(self, ch):
        return self.mon.pulses[ch]

    # ---- Raw AXI4-Lite (explicit AW/W/B/R timing) ----
    async def _wait_flag(self, sig, what, max_cycles=HS_TIMEOUT_CYCLES):
        """Wait (falling edge, ReadOnly) until sig is high. Fails instead of hanging."""
        for _ in range(max_cycles):
            await FallingEdge(self.clk)
            await ReadOnly()
            if int(sig.value):
                return
        d = self.dut
        state = (f"awv={d.s_axi_awvalid.value} awr={d.s_axi_awready.value} "
                 f"wv={d.s_axi_wvalid.value} wr={d.s_axi_wready.value} "
                 f"bv={d.s_axi_bvalid.value} br={d.s_axi_bready.value} "
                 f"arv={d.s_axi_arvalid.value} arr={d.s_axi_arready.value} "
                 f"rv={d.s_axi_rvalid.value} rr={d.s_axi_rready.value}")
        raise AssertionError(f"AXI handshake timeout: {what} never high after {max_cycles} cycles [{state}]")

    async def _handshake_out(self, valid, ready, what):
        """valid is already high. Return after the rising edge where it was accepted."""
        await self._wait_flag(ready, what)
        await RisingEdge(self.clk)
        await Timer(1, units="ns")
        valid.value = 0

    async def raw_write(self, addr, data, strb=0xF, aw_delay=0, w_delay=0, b_delay=0):
        """AXI write with delays on AW, W and B. Returns BRESP."""
        d = self.dut

        async def aw_phase():
            await idle(self.clk, aw_delay)
            d.s_axi_awaddr.value = addr
            d.s_axi_awvalid.value = 1
            await self._handshake_out(d.s_axi_awvalid, d.s_axi_awready, f"AW {addr:#x}")

        async def w_phase():
            await idle(self.clk, w_delay)
            d.s_axi_wdata.value = data
            d.s_axi_wstrb.value = strb
            d.s_axi_wvalid.value = 1
            await self._handshake_out(d.s_axi_wvalid, d.s_axi_wready, f"W {addr:#x}")

        a = cocotb.start_soon(aw_phase())
        w = cocotb.start_soon(w_phase())
        await a
        await w
        await idle(self.clk, b_delay)
        d.s_axi_bready.value = 1
        await self._wait_flag(d.s_axi_bvalid, f"B {addr:#x}")
        resp = int(d.s_axi_bresp.value)
        await RisingEdge(self.clk)
        await Timer(1, units="ns")
        d.s_axi_bready.value = 0
        return resp

    async def raw_read(self, addr, r_delay=0):
        """AXI read with a delay on R. Returns (data, RRESP)."""
        d = self.dut
        d.s_axi_araddr.value = addr
        d.s_axi_arvalid.value = 1
        await self._handshake_out(d.s_axi_arvalid, d.s_axi_arready, f"AR {addr:#x}")
        await idle(self.clk, r_delay)
        d.s_axi_rready.value = 1
        await self._wait_flag(d.s_axi_rvalid, f"R {addr:#x}")
        data = int(d.s_axi_rdata.value)
        resp = int(d.s_axi_rresp.value)
        await RisingEdge(self.clk)
        await Timer(1, units="ns")
        d.s_axi_rready.value = 0
        return data, resp
