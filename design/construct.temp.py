#! /usr/bin/env python
#=========================================================================
# construct.py
#=========================================================================
# Demo 

import os
import sys

from mflowgen.components import Graph, Step

def construct():

  g = Graph()

  #-----------------------------------------------------------------------
  # Parameters
  #-----------------------------------------------------------------------
  
  adk_name = 'skywater-130nm-adk'
  adk_view = 'view-standard'

  parameters = {
    'construct_path' : __file__,
    'design_name'    : 'ChipTop',
    'clock_period'   : 5.0,
    'adk'            : adk_name,
    'adk_view'       : adk_view,
    'topographical'  : True,
  }

  #-----------------------------------------------------------------------
  # Create nodes
  #-----------------------------------------------------------------------

  this_dir = os.path.dirname( os.path.abspath( __file__ ) )


  # ADK step

  g.set_adk( adk_name )
  adk = g.get_adk_step()


  # Custom steps

  rtl             = Step( this_dir + '/rtl'                             )
  plugin_dc_retime= Step( this_dir + '/plugin-dc-retime'                )
  macros          = Step( this_dir + '/macros'                          )
  plugin_dc       = Step( this_dir + '/plugin-dc'                       )
  iflow           = Step( this_dir + '/iflow'                  )
  constraints     = Step( this_dir + '/constraints'                     )
  pin_placement   = Step( this_dir + '/pin-placement'                   )
  floorplan       = Step( this_dir + '/floorplan'                       )

  # testbench       = Step( this_dir + '/testbench'                       )
  
  # # Power node is custom because power and gnd pins are named differently in
  # # the standard cells compared to the default node, and the layer numbering is
  # # different because of li layer, the default assumes metal 1 is the lowest
  # # layer
  
  power           = Step( this_dir + '/cadence-innovus-power'           ) 

  # # Signoff is custom because it has to output def that the default step does
  # # not do. This is because we use the def instead of gds for generating spice
  # # from layout for LVS
  
  signoff         = Step( this_dir + '/cadence-innovus-signoff'         ) 
  
  # pt_power_rtl    = Step( this_dir + '/synopsys-ptpx-rtl'               )

  # magic_drc       = Step( this_dir + '/open-magic-drc'                  )
  # magic_def2spice = Step( this_dir + '/open-magic-def2spice'            )
  # netgen_lvs      = Step( this_dir + '/open-netgen-lvs'                 )


  # Default steps

  info            = Step( 'info',                          default=True )
  dc              = Step( 'synopsys-dc-synthesis',         default=True )
  
  # Need to use clone if you want to instantiate the same node more than once
  # in your graph but configure it differently, for example, RTL simulation and
  # gate-level simulation use the same VCS node

  # vcs_sim         = Step( 'synopsys-vcs-sim',              default=True )
  # rtl_sim         = vcs_sim.clone()
  # gl_sim          = vcs_sim.clone()
  # rtl_sim.set_name( 'rtl-sim' )
  # gl_sim.set_name( 'gl-sim' )
  
  #iflow           = Step( 'cadence-innovus-flowsetup',     default=True )
  init            = Step( 'cadence-innovus-init',          default=True )
  place           = Step( 'cadence-innovus-place',         default=True )
  cts             = Step( 'cadence-innovus-cts',           default=True )
  postcts_hold    = Step( 'cadence-innovus-postcts_hold',  default=True )
  route           = Step( 'cadence-innovus-route',         default=True )
  postroute       = Step( 'cadence-innovus-postroute',     default=True )
  postroute_hold   = Step( 'cadence-innovus-postroute_hold', default=True)
  # gdsmerge        = Step( 'mentor-calibre-gdsmerge',       default=True )
  synopsys_pt_eco = Step( 'synopsys-pt-eco', default=True)
  open_icc2innovus = Step('open-icc2innovus', default=True)
  cadence_innovus_eco = Step('cadence-innovus-eco', default=True)
  pt_timing       = Step( 'synopsys-pt-timing-signoff',    default=True)

  signoff_posteco = signoff.clone()
  signoff_posteco.set_name("signoff_posteco")
  
  # gen_saif        = Step( 'synopsys-vcd2saif-convert',     default=True )
  # gen_saif_rtl    = gen_saif.clone()
  # gen_saif_gl     = gen_saif.clone()
  # gen_saif_rtl.set_name( 'gen-saif-rtl' )
  # gen_saif_gl.set_name( 'gen-saif-gl' )
  
  # pt_power_gl     = Step( 'synopsys-ptpx-gl',              default=True )
  

  #-----------------------------------------------------------------------
  # Graph -- Add nodes
  #-----------------------------------------------------------------------

  g.add_step( info            )
  g.add_step( rtl             )
  # g.add_step( testbench       )
  # g.add_step( rtl_sim         )
  g.add_step( constraints     )
  g.add_step( plugin_dc_retime)
  g.add_step( macros          )
  g.add_step( plugin_dc       )
  g.add_step( dc              )
  g.add_step( iflow           )
  g.add_step( pin_placement   )
  g.add_step( floorplan       )
  g.add_step( init            )
  g.add_step( power           )
  g.add_step( place           )
  g.add_step( cts             )
  g.add_step( postcts_hold    )
  g.add_step( route           )
  g.add_step( postroute       )
  g.add_step( postroute_hold  )
  g.add_step( signoff         )
  g.add_step( synopsys_pt_eco )
  g.add_step( open_icc2innovus)
  g.add_step( cadence_innovus_eco)
  g.add_step( signoff_posteco)
  # g.add_step( gdsmerge        )
  g.add_step( pt_timing       )
  # g.add_step( gen_saif_rtl    )
  # g.add_step( pt_power_rtl    )
  # g.add_step( gl_sim          )
  # g.add_step( gen_saif_gl     )
  # g.add_step( pt_power_gl     )
  # g.add_step( magic_drc       )
  # g.add_step( magic_def2spice )
  # g.add_step( netgen_lvs      )

  # #-----------------------------------------------------------------------
  # # Graph -- Add edges
  # #-----------------------------------------------------------------------
  
  # # Dynamically add edges

  # rtl_sim.extend_inputs(['design.v'])
  # rtl_sim.extend_inputs(['test_vectors.txt'])
  # gl_sim.extend_inputs(['test_vectors.txt'])

  # Connect by name
  g.connect_by_name( adk,             dc              )
  # g.connect_by_name( adk,             testbench       )
  g.connect_by_name( adk,             iflow           )
  g.connect_by_name( adk,             init            )
  g.connect_by_name( adk,             power           )
  g.connect_by_name( adk,             place           )
  g.connect_by_name( adk,             cts             )
  g.connect_by_name( adk,             postcts_hold    )
  g.connect_by_name( adk,             postroute_hold  )
  g.connect_by_name( adk,             route           )
  g.connect_by_name( adk,             postroute       )
  g.connect_by_name( adk,             signoff         )
  g.connect_by_name( adk,             synopsys_pt_eco )
  g.connect_by_name( adk,             open_icc2innovus) 
  g.connect_by_name( adk,             cadence_innovus_eco) 
  g.connect_by_name( adk,             signoff_posteco) 

  g.connect_by_name( signoff,         synopsys_pt_eco )
  g.connect_by_name(synopsys_pt_eco,  open_icc2innovus)
  g.connect_by_name(open_icc2innovus, cadence_innovus_eco)
  g.connect_by_name(cadence_innovus_eco, signoff_posteco)
  # g.connect_by_name( adk,             gdsmerge        )
  # g.connect_by_name( adk,             magic_drc       )
  # g.connect_by_name( adk,             magic_def2spice )
  # g.connect_by_name( adk,             netgen_lvs      )
  g.connect_by_name( adk,             pt_timing       )
  # g.connect_by_name( adk,             pt_power_rtl    )
  # g.connect_by_name( adk,             pt_power_gl     )

  # g.connect_by_name( rtl,             rtl_sim         ) # design.v
  # g.connect_by_name( testbench,       rtl_sim         ) # testbench.sv
  # g.connect( rtl_sim.o( 'design.vpd' ), gen_saif_rtl.i( 'run.vcd' ) ) # TODO: FIX
  # # FIXME: VCS sim node generates a VCD file but gives it a VPD extension
  dc.extend_inputs(macros.all_outputs())
  dc.extend_inputs(plugin_dc.all_outputs())
  dc.extend_inputs(plugin_dc_retime.all_outputs())
  iflow.extend_inputs(macros.all_outputs())
  init.extend_inputs(macros.all_outputs())
  init.extend_inputs(['floorplan.tcl', 'pin-assignments.tcl'])


  g.connect_by_name( plugin_dc       ,dc              )
  g.connect_by_name( macros          ,dc              )
  g.connect_by_name( plugin_dc_retime,dc              )
  g.connect_by_name( rtl,             dc              )
  g.connect_by_name( adk        ,     dc              )
  g.connect_by_name( constraints,     dc              )
  # g.connect_by_name( gen_saif_rtl,    dc              ) # run.saif

  g.connect_by_name( macros,          iflow           )
  g.connect_by_name( dc,              iflow           )
  g.connect_by_name( macros,          init            )
  g.connect_by_name( dc,              init            )
  g.connect_by_name( dc,              power           )
  g.connect_by_name( dc,              place           )
  g.connect_by_name( dc,              cts             )
  # g.connect_by_name( dc,              pt_power_rtl    ) # design.namemap
  g.connect_by_name( floorplan,       init            )
  g.connect_by_name( pin_placement,   init            )
  g.connect_by_name( iflow,           init            )
  g.connect_by_name( iflow,           power           )
  g.connect_by_name( iflow,           place           )
  g.connect_by_name( iflow,           cts             )
  g.connect_by_name( iflow,           postcts_hold    )
  g.connect_by_name( iflow,           route           )
  g.connect_by_name( iflow,           postroute       )
  g.connect_by_name( iflow,           postroute_hold  )
  g.connect_by_name( iflow,           signoff         )
  #g.connect_by_name( iflow,             synopsys_pt_eco )
 # g.connect_by_name( iflow,             open_icc2innovus) 
  g.connect_by_name( iflow,             cadence_innovus_eco) 
 # g.connect_by_name( iflow,             signoff_posteco) 
  
  # # Core place and route flow
  g.connect_by_name( init,            power           )
  g.connect_by_name( power,           place           )
  g.connect_by_name( place,           cts             )
  g.connect_by_name( cts,             postcts_hold    )
  g.connect_by_name( postcts_hold,    route           )
  g.connect_by_name( route,           postroute       )
  g.connect_by_name( postroute,       postroute_hold  )
  g.connect_by_name( postroute_hold,  signoff         )
  # g.connect_by_name( signoff,         gdsmerge        )
  
  # # DRC, LVS, timing signoff and power signoff
  # g.connect_by_name( gdsmerge,        magic_drc       )
  # g.connect_by_name( signoff,         magic_def2spice )
  # g.connect_by_name( signoff,         netgen_lvs      )
  # g.connect_by_name( magic_def2spice, netgen_lvs      )
  g.connect_by_name( signoff_posteco,         pt_timing       )
  # g.connect_by_name( signoff,         pt_power_rtl    )
  # g.connect_by_name( gen_saif_rtl,    pt_power_rtl    ) # run.saif
  # g.connect_by_name( signoff,         pt_power_gl     )
  # g.connect_by_name( gen_saif_gl,     pt_power_gl     ) # run.saif

  # # Gate level simulation
  # g.connect_by_name( adk,             gl_sim          )
  # g.connect( signoff.o(   'design.vcs.pg.v'  ), gl_sim.i( 'design.vcs.v'     ) )
  # g.connect( pt_timing.o( 'design.sdf'       ), gl_sim.i( 'design.sdf'       ) )
  # g.connect( testbench.o( 'testbench.sv'     ), gl_sim.i( 'testbench.sv'     ) )
  # g.connect( testbench.o( 'design.args.gls'  ), gl_sim.i( 'design.args'      ) )
  # # g.connect( testbench.o( 'test_vectors.txt' ), gl_sim.i( 'test_vectors.txt' ) )

  # g.connect( gl_sim.o( 'design.vpd' ), gen_saif_gl.i( 'run.vcd' ) ) # TODO: FIX
  # FIXME: VCS sim node generates a VCD file but gives it a VPD extension


  #-----------------------------------------------------------------------
  # Parameterize
  #-----------------------------------------------------------------------

  g.update_params( parameters )

  dc_script_order = dc.get_param('order')
  index = dc_script_order.index('compile.tcl')
  dc_script_order.insert(index, 'retime.tcl')
  dc.update_params({'order': dc_script_order})

  #postroute_hold.set_param('hold_target_slack', 0.2)
  #synopsys_pt_eco.set_param('eco_types', ['timing'])

  return g

if __name__ == '__main__':
  g = construct()
  g.plot()
