
# 50 MHz clock (20 ns period) 
create_clock -name clock_proc -period 20.000 [get_ports clock_proc]
create_clock -name clock_mem  -period 20.000 [get_ports clock_mem]

# Allow bitstream generation even if IO standard is DEFAULT
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]

# Allow bitstream generation even if ports have NO pin assignment (LOC)
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]


set_property SEVERITY {Warning} [get_drc_checks RTSTAT-1]
set_property SEVERITY {Warning} [get_drc_checks PLCK-1]
set_property SEVERITY {Warning} [get_drc_checks REQP-183]
