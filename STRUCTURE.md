# Project structure

```
dshot_controller/
├── readme.md                 overview, how to use it in an SoC, register map, design decisions
├── STRUCTURE.md              this page
├── requirements.md           specification (requirement and test IDs)
├── src/
│   ├── holy_dshot.sv         the whole core: AXI4-Lite slave, control, frame engine
│   └── holy_dshot_wrapper.v  plain Verilog wrapper for Vivado block designs (no logic)
└── tb/
    ├── Makefile              cocotb + Verilator run (core, or wrapper with MODULE/TOPLEVEL)
    ├── requirements.txt      Python packages (cocotb 1.9.2, cocotbext-axi, pytest)
    ├── test_runner.py        pytest entry point: runs the core and the wrapper once each
    ├── test_holy_dshot.py    core tests (T-01 … T-22)
    ├── test_holy_dshot_wrapper.py   wrapper smoke test (T-24)
    └── dshot_tb.py           shared helpers: reference model, pin monitor, AXI driver
```

## Inside `src/holy_dshot.sv`

The file has five sections, in this order:

1. Constants and helper functions (bit timing, CRC, command check, period clamp)
2. Storage (all registers and frame-engine state)
3. AXI4-Lite slave (handshake, write decode, read mux)
4. Control (commit, state machine, watchdog, frame scheduler)
5. Frame engine (shift registers, bit timing, registered `dshot_o`)

## Inside `tb/`

- `dshot_tb.py` holds the parts the tests share: DShot reference model, pin decoder,
  and an AXI4-Lite driver with explicit AW/W/B/R timing.
- The test files use only these helpers and the DUT ports, so they stay readable.
