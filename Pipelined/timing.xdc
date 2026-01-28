
# System clock for the Processor module (50 MHz → 20 ns period)
create_clock -name clk -period 20.000 [get_ports clk]

set_property PACKAGE_PIN UNASSIGNED [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports rst]

set_property SEVERITY {Warning} [get_drc_checks NSTD-1]

set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

set_property SEVERITY {Warning} [get_drc_checks RTSTAT-1]
#set_property SEVERITY {Warning} [get_drc_checks PLCK-1]
#set_property SEVERITY {Warning} [get_drc_checks REQP-183]


