#=========================================================================
# pin-assignments.tcl
#=========================================================================
# The ports of this design become physical pins along the perimeter of the
# design. The commands below will spread the pins along the left and right
# perimeters of the core area. This will work for most designs, but a
# detail-oriented project should customize or replace this section.
#
# Author : George Klimiashvili
# Date   : April 4, 2021

#-------------------------------------------------------------------------
# Pin Assignments
#-------------------------------------------------------------------------

# Here pin assignments are done keeping in mind the location of the SRAM pins
# If you update pin assignments below you should rerun the pin-placement step 
# before re-running init step

# We are assigning pins clockwise here, starting from the top side we go left
# to right, then on the right side we go top to bottom, then on the bottom
# side, we go right to left, then on the left side we go bottom to top.

# Pins on the top side. The first pin in this list (here jtag_TCK) is on the
# top left and the last pin is on the top right.

set pins_top {\
  jtag_TCK  jtag_TMS   jtag_TDI  jtag_TDO_data  jtag_TDO_driven\
}

# Pins on the right side. In this example we are not placing pins on the right
# side, since we haven't routed out the pins on the right side of the SRAM. In
# your design, you can use the right side as well.

set pins_right {\
  gpio_0_0 gpio_0_1 gpio_0_2 spi_0_cs_0 spi_0_sck spi_0_dq_0 spi_0_dq_1 \
  spi_0_dq_2 spi_0_dq_3 \
}

# Pins on the bottom side from right (dout0[0]) to left (din0[31]). I list pins
# out explicitly here because the dout0 and din0 pins on the SRAM macro are
# interleaved somewhat randomly, but if in your case the pins of the same bus
# are to be kept together then you can generate this pin list using a tcl for
# loop.

set pins_bottom {\
  uart_0_txd    uart_0_rxd\
}

# Pins on the left side from bottom (rst_n) to top (addr0[0]).

set pins_left {\
  reset_wire_reset    clock
}

# Spread the pins evenly along the sides of the block

set ports_layer M4

editPin -layer $ports_layer -pin $pins_right  -side RIGHT  -spreadType SIDE
editPin -layer $ports_layer -pin $pins_left   -side LEFT   -spreadType SIDE
editPin -layer $ports_layer -pin $pins_bottom -side BOTTOM -spreadType SIDE
editPin -layer $ports_layer -pin $pins_top    -side TOP    -spreadType SIDE

