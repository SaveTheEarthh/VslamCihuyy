# Main clock constraint: 100 MHz
# Clock period = 10 ns

create_clock -name clk -period 10.000 [get_ports {clk}]

# Automatically derive PLL-generated clocks
derive_pll_clocks

# Apply clock uncertainty
derive_clock_uncertainty