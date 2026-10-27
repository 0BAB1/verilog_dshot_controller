# tb/Makefile - cocotb + Verilator testbench for holy_dshot (HOLY CORE convention)
#
#   make                                                   core tests, 25 MHz
#   CLK_HZ=100000000 make                                  core tests, 100 MHz
#   make MODULE=test_holy_dshot_wrapper TOPLEVEL=holy_dshot_wrapper   wrapper smoke test
#   make clean                                             remove build products

SIM ?= verilator
TOPLEVEL_LANG ?= verilog
WAVES = 1

CLK_HZ ?= 25000000
export CLK_HZ

TOPLEVEL ?= holy_dshot
MODULE ?= test_holy_dshot

VERILOG_SOURCES += $(PWD)/../src/holy_dshot.sv
ifeq ($(TOPLEVEL),holy_dshot_wrapper)
VERILOG_SOURCES += $(PWD)/../src/holy_dshot_wrapper.v
endif

EXTRA_ARGS += --trace --trace-structs
EXTRA_ARGS += -GCLK_HZ=$(CLK_HZ)

include $(shell cocotb-config --makefiles)/Makefile.sim
