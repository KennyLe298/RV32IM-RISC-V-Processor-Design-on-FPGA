# timing.xdc
# 10 ns = 100 MHz
create_clock -name clock_proc -period 20.000 [get_ports clock_proc]
create_clock -name clock_mem  -period 20.000 [get_ports clock_mem]

# Allow bitstream generation even if IO standard is DEFAULT
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]

# Allow bitstream generation even if ports have no LOC
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

