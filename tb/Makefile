# tb/Makefile - cocotb + Verilator
#
#   make                                  core (holy_dshot)
#   make TOPLEVEL=holy_dshot_wrapper      Vivado wrapper
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog
WAVES ?= 0

CLK_HZ ?= 25000000
export CLK_HZ

TOPLEVEL ?= holy_dshot
MODULE = test_holy_dshot

VERILOG_SOURCES = $(PWD)/../src/holy_dshot.sv
ifeq ($(TOPLEVEL),holy_dshot_wrapper)
VERILOG_SOURCES += $(PWD)/../src/holy_dshot_wrapper.v
endif

EXTRA_ARGS += -GCLK_HZ=$(CLK_HZ)

SIM_BUILD = sim_build/$(TOPLEVEL)

include $(shell cocotb-config --makefiles)/Makefile.sim
