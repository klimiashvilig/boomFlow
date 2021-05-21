#=========================================================================
# floorplan.tcl
#=========================================================================
#
# Author : George 
# Date   : May 18, 2021

#-------------------------------------------------------------------------
# Floorplan
#-------------------------------------------------------------------------


# set core_aspect_ratio   1.00; # Aspect ratio 1.0 for a square chip
# set core_density_target 0.75; # Placement density of 70% is reasonable

set core_width  2920;  # Based on https://github.com/efabless/caravel_user_project/blob/main/lef/user_project_wrapper.lef
set core_height 3520;  # Based on https://github.com/efabless/caravel_user_project/blob/main/lef/user_project_wrapper.lef

set pwr_net_list {VDD VSS}; # List of power nets in the core power ring

set M1_min_width   [dbGet [dbGetLayerByZ 1].minWidth]
set M1_min_spacing [dbGet [dbGetLayerByZ 1].minSpacing]

set savedvars(p_ring_width)   [expr 48 * $M1_min_width];   # Arbitrary!
set savedvars(p_ring_spacing) [expr 24 * $M1_min_spacing]; # Arbitrary!

# Core bounding box margins

set core_margin_t [expr ([llength $pwr_net_list] * ($savedvars(p_ring_width) + $savedvars(p_ring_spacing))) + $savedvars(p_ring_spacing)]
set core_margin_b [expr ([llength $pwr_net_list] * ($savedvars(p_ring_width) + $savedvars(p_ring_spacing))) + $savedvars(p_ring_spacing)]
set core_margin_r [expr ([llength $pwr_net_list] * ($savedvars(p_ring_width) + $savedvars(p_ring_spacing))) + $savedvars(p_ring_spacing)]
set core_margin_l [expr ([llength $pwr_net_list] * ($savedvars(p_ring_width) + $savedvars(p_ring_spacing))) + $savedvars(p_ring_spacing)]

#-------------------------------------------------------------------------
# Floorplan
#-------------------------------------------------------------------------

# Calling floorPlan with the "-r" flag sizes the floorplan according to
# the core aspect ratio and a density target (70% is a reasonable
# density).
#

# floorPlan -r $core_aspect_ratio $core_density_target \
#              $core_margin_l $core_margin_b $core_margin_r $core_margin_t


floorPlan -s $core_width $core_height \
             $core_margin_l $core_margin_b $core_margin_r $core_margin_t

setFlipping ss

# planDesign
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 200 200
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 1200 200
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 200 950
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 1200 950
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_2__sram 200 1700
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_2__sram 1200 1700
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_3__sram 200 2450
placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_3__sram 1200 2450


# width = 405.72, height = 399.84
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 200 200 
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 200 750
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_2__sram 200 1300
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_3__sram 200 1850
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 750 200 
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 750 750
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_2__sram 750 1300
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_3__sram 750 1850

# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_4__sram 1300 200 
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_5__sram 1300 750
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_6__sram 1300 1300
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_0__depth_macro_7__sram 1300 1850
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_4__sram 1850 200 
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_5__sram 1850 750
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_6__sram 1850 1300
# placeInstance system/spad/mem/mem_ext/dffram/genblk1_width_macro_1__depth_macro_7__sram 1850 1850

# # width = 399.28, height = 106.08
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_0/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 200 3300
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_0/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 800 3300
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_0/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 200 3000
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_0/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 800 3000

# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_1/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 200 2700
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_1/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 800 2700
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_1/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 200 2400
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_1/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 800 2400

# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_2/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 1400 3300
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_2/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 2000 3300
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_2/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 1400 3000
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_2/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 2000 3000

# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_3/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 1400 2700
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_3/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 2000 2700
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_3/cc_banks_0_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 1400 2400
# placeInstance system/subsystem_l2_wrapper/l2/mods_0/bankedStore/cc_banks_3/cc_banks_0_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 2000 2400

# # width = 399.28, height = 33.72
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/meta_0/tag_array/tag_array_0_ext/dffram_0/genblk1_width_macro_0__sram 2400 2075
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/meta_0/tag_array/tag_array_0_ext/dffram_1/genblk1_width_macro_0__sram 2400 2250
# # flipOrRotateObject -name system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/meta_0/tag_array/tag_array_0_ext/dffram_0/genblk1_width_macro_0__sram -rotate R90
# # flipOrRotateObject -name system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/meta_0/tag_array/tag_array_0_ext/dffram_1/genblk1_width_macro_0__sram -rotate R90

# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/victimCache/dataArrayWay_0/dataArrayWay_0_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 2400 1375
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/victimCache/dataArrayWay_0/dataArrayWay_0_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 2400 1550
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/victimCache/dataArrayWay_0/dataArrayWay_0_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 2400 1725
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/dcache/victimCache/dataArrayWay_0/dataArrayWay_0_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 2400 1900

# # width = 399.28, height = 106.08
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/ptw/l2_tlb_ram/l2_tlb_ram_ext/dffram/genblk1_width_macro_0__depth_macro_0__sram 2400 200
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/ptw/l2_tlb_ram/l2_tlb_ram_ext/dffram/genblk1_width_macro_1__depth_macro_0__sram 2400 500
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/ptw/l2_tlb_ram/l2_tlb_ram_ext/dffram/genblk1_width_macro_0__depth_macro_1__sram 2400 800
# placeInstance system/tile_prci_domain/tile_reset_domain/boom_tile/ptw/l2_tlb_ram/l2_tlb_ram_ext/dffram/genblk1_width_macro_1__depth_macro_1__sram 2400 1100

