#=========================================================================
# Design Constraints File
#=========================================================================

# This constraint sets the target clock period for the chip in
# nanoseconds. Note that the first parameter is the name of the clock
# signal in your verilog design. If you called it something different than
# clk you will need to change this. You should set this constraint
# carefully. If the period is unrealistically small then the tools will
# spend forever trying to meet timing and ultimately fail. If the period
# is too large the tools will have no trouble but you will get a very
# conservative implementation.

set clock_net  clock
set clock_name ideal_clock

create_clock -name ${clock_name} \
             -period ${dc_clock_period} \
             [get_ports ${clock_net}]

set jtag_clock_net jtag_TCK
set jtag_clock_name jtag_clock
set jtag_clock_period [expr ${dc_clock_period}*20]

create_clock -name ${jtag_clock_name} \
             -period [expr ${jtag_clock_period}] \
             [get_ports ${jtag_clock_net}]

# Make clocks asynchronous
set_clock_groups -asynchronous -group [get_clocks ${clock_name}] \
		 -group [get_clocks ${jtag_clock_name}]

# This constraint sets the load capacitance in picofarads of the
# output pins of your design.

set_load -pin_load $ADK_TYPICAL_ON_CHIP_LOAD [all_outputs]

# This constraint sets the input drive strength of the input pins of
# your design. We specifiy a specific standard cell which models what
# would be driving the inputs. This should usually be a small inverter
# which is reasonable if another block of on-chip logic is driving
# your inputs.

set_driving_cell -no_design_rule \
  -lib_cell $ADK_DRIVING_CELL [all_inputs]

# set_input_delay constraints for input ports
#
# - make this non-zero to avoid hold buffers on input-registered designs

set_input_delay -clock ${clock_name} [expr ${dc_clock_period}/2.0] \
			[get_ports "reset_wire_reset"] 
      # serial_tl_bits_in_valid serial_tl_bits_in_bits serial_tl_bits_out_ready"]
set_input_delay -clock ${jtag_clock_name} [expr ${jtag_clock_period}/2.0] \
			[get_ports "jtag_TMS jtag_TDI"]


# set_output_delay constraints for output ports

# set_output_delay -clock ${clock_name} 0 \
# 			[get_ports "serial_tl_clock serial_tl_bits_in_ready serial_tl_bits_out_valid serial_tl_bits_out_bits"]
set_output_delay -clock ${jtag_clock_name} 0 \
			[get_ports "jtag_TDO_data jtag_TDO_driven"]

# Make all signals limit their fanout

set_max_fanout 20 $dc_design_name

# Make all signals meet good slew

set_max_transition [expr 0.25*${dc_clock_period}] $dc_design_name

#set_input_transition 1 [all_inputs]
#set_max_transition 10 [all_outputs]