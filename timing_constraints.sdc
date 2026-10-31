## timing_constraints.sdc  (ARM single-cycle CPU, board build)
## Same structure as the RISC-V projects.
## clk = 50 MHz board clock (PIN_23). cpu_clk = clkdiv[1] = 50 MHz / 4 = 12.5 MHz (made in top_cpu_arm.v).

create_clock -name clk -period 20.000 [get_ports {clk}]
create_generated_clock -name cpu_clk -source [get_ports {clk}] -divide_by 4 [get_registers {clkdiv[1]}]
derive_clock_uncertainty
set_false_path -from [get_ports {KEY[*]}]
