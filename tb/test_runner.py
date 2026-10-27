# =============================================================================
# test_runner.py - runs every holy_dshot testbench through pytest.
#   cd tb && pytest test_runner.py
# Core tests run at 25 MHz with three random seeds, and at 100 MHz (T-23).
# The wrapper smoke test (T-24) runs through holy_dshot_wrapper.v.
# =============================================================================
import os
from pathlib import Path

import pytest
from cocotb.runner import get_runner

TB = Path(__file__).resolve().parent
SRC = TB.parent / "src"
SIM = os.getenv("SIM", "verilator")


def run(top, test_module, sources, clk_hz, seed):
    """Build (once per clock) and run one testbench module."""
    build = TB / f"sim_build_{top}_{clk_hz}"
    runner = get_runner(SIM)
    runner.build(
        sources=sources,
        hdl_toplevel=top,
        parameters={"CLK_HZ": clk_hz},
        build_dir=build,
        waves=False,
    )
    os.environ["CLK_HZ"] = str(clk_hz)          # read by dshot_tb.py
    runner.test(
        hdl_toplevel=top,
        test_module=test_module,
        test_dir=TB,
        build_dir=build,
        seed=seed,
        waves=False,   # full traces make the 1M-cycle tests very slow; use `make` for waveforms
    )


def test_holy_dshot():
    run("holy_dshot", "test_holy_dshot", [SRC / "holy_dshot.sv"], 25_000_000, None)


def test_holy_dshot_wrapper():
    run("holy_dshot_wrapper", "test_holy_dshot_wrapper",
        [SRC / "holy_dshot.sv", SRC / "holy_dshot_wrapper.v"], 25_000_000, None)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
