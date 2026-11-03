create_clock -name sys_clk -period 20.000 [get_ports {clk_50m}]
# ADC and asynchronous external interfaces require board-level verification.
# No guessed ADC tCO/input-delay values are presented as timing sign-off.
