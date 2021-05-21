lef read rtk-tech-nolicon.lef
lef read inputs/adk/stdcells.lef
lef read inputs/dffram/DFFRAM.lef

def read design.def

load $::env(design_name)

# Extract for LVS
extract do local
extract no capacitance
extract no coupling
extract no resisitance
extract no adjust
extract unique
extract
ext2spice lvs
ext2spice subcircuit on
ext2spice subcircuit top on
ext2spice -o outputs/design_extracted.spice

quit

