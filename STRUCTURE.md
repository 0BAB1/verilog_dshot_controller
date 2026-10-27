# Project structure

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

`holy_dshot.sv` sections, top to bottom: constants and the frame function, the frame
engine (bit and slot counters, shift registers, registered outputs), the AXI4-Lite
slave, and the register read and write logic.
