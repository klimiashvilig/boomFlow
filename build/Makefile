#=========================================================================
# Makefile
#=========================================================================
# Generator : /tmp/ee272b/flow/mflowgen/mflowgen/backends/make_backend.py

SHELL=/usr/bin/env bash -euo pipefail

# $1 -- $dst
# $2 -- $src
# $3 -- $stamp

define cpdir
	rm -rf ./$1
	cp -aL $2 $1 || true
	chmod -R +w $1
	touch $3
endef

# $1 -- $dst
# $2 -- $src
# $3 -- $stamp

define cpdir-and-parameterize
	rm -rf ./$1
	cp -aL $2 $1 || true
	chmod -R +w $1
	cp .mflowgen/$1/configure.yml $1
	touch $3
endef

# $1 -- $dst
# $2 -- $src
# $3 -- $stamp

define mkdir-and-symlink
	rm -rf ./$1
	mkdir -p $1
	cd $1 && ln -sf ../$2/* . && cd ..
	rm $1/configure.yml && cp .mflowgen/$1/configure.yml $1
	touch $3
endef

# $1 -- $dst_dir
# $2 -- $dst
# $3 -- $src
# $4 -- $stamp

define symlink
	mkdir -p $1
	cd $1 && ln -sf $3 $2 && touch $4
endef

# $1 -- $stamp

define stamp
	touch $1
endef

# Default

default: constraints floorplan info macros pin-placement plugin-dc plugin-dc-retime rtl skywater-130nm synopsys-dc-synthesis cadence-innovus-flowsetup cadence-innovus-init cadence-innovus-power cadence-innovus-place cadence-innovus-cts cadence-innovus-postcts_hold cadence-innovus-route cadence-innovus-postroute cadence-innovus-postroute_hold cadence-innovus-signoff mentor-calibre-gdsmerge open-magic-def2spice synopsys-pt-eco open-icc2innovus open-magic-drc open-netgen-lvs cadence-innovus-eco signoff_posteco synopsys-pt-timing-signoff

# ------------------------------------------------------------------------
# constraints
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 0-constraints/.prebuilt)","")
0-constraints/.stamp:
	$(call cpdir-and-parameterize,0-constraints,../design/constraints,0-constraints/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 0-constraints/.prebuilt)","")

define 0_constraints_commands_rule
	mkdir -p 0-constraints/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t constraints && chmod +x .mflowgen/0-constraints/mflowgen-run && cp -f .mflowgen/0-constraints/mflowgen-run 0-constraints && if [[ -e .mflowgen/0-constraints/mflowgen-debug ]]; then chmod +x .mflowgen/0-constraints/mflowgen-debug && cp -f .mflowgen/0-constraints/mflowgen-debug 0-constraints; fi && if [[ -e .mflowgen/0-constraints/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/0-constraints/mflowgen-check-preconditions.py && cp -f .mflowgen/0-constraints/mflowgen-check-preconditions.py 0-constraints; fi && if [[ -e .mflowgen/0-constraints/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/0-constraints/mflowgen-check-postconditions.py && cp -f .mflowgen/0-constraints/mflowgen-check-postconditions.py 0-constraints; fi && cd 0-constraints && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 0-constraints/outputs/*
endef

0-constraints/.execstamp: 0-constraints/.stamp
	$(call 0_constraints_commands_rule)
	touch $@

0-constraints/outputs/.execstamp.constraints.tcl: 0-constraints/.execstamp
	touch $@

endif

# collect outputs

0-constraints/outputs/.stamp.constraints.tcl: 0-constraints/.execstamp 0-constraints/outputs/.execstamp.constraints.tcl
	$(call stamp,0-constraints/outputs/.stamp.constraints.tcl)

# post-conditions

define 0_constraints_post_conditions_commands_rule
	cd 0-constraints && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

0-constraints/.postconditions.stamp: 0-constraints/.execstamp 0-constraints/outputs/.execstamp.constraints.tcl 0-constraints/outputs/.stamp.constraints.tcl
	$(call 0_constraints_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: constraints

constraints: 0-constraints/.execstamp 0-constraints/.postconditions.stamp 0-constraints/outputs/.execstamp.constraints.tcl 0-constraints/outputs/.stamp.constraints.tcl

.PHONY: 0

0: 0-constraints/.execstamp 0-constraints/.postconditions.stamp 0-constraints/outputs/.execstamp.constraints.tcl 0-constraints/outputs/.stamp.constraints.tcl

# debug

# ------------------------------------------------------------------------
# floorplan
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 1-floorplan/.prebuilt)","")
1-floorplan/.stamp:
	$(call cpdir-and-parameterize,1-floorplan,../design/floorplan,1-floorplan/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 1-floorplan/.prebuilt)","")

define 1_floorplan_commands_rule
	mkdir -p 1-floorplan/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t floorplan && chmod +x .mflowgen/1-floorplan/mflowgen-run && cp -f .mflowgen/1-floorplan/mflowgen-run 1-floorplan && if [[ -e .mflowgen/1-floorplan/mflowgen-debug ]]; then chmod +x .mflowgen/1-floorplan/mflowgen-debug && cp -f .mflowgen/1-floorplan/mflowgen-debug 1-floorplan; fi && if [[ -e .mflowgen/1-floorplan/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/1-floorplan/mflowgen-check-preconditions.py && cp -f .mflowgen/1-floorplan/mflowgen-check-preconditions.py 1-floorplan; fi && if [[ -e .mflowgen/1-floorplan/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/1-floorplan/mflowgen-check-postconditions.py && cp -f .mflowgen/1-floorplan/mflowgen-check-postconditions.py 1-floorplan; fi && cd 1-floorplan && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 1-floorplan/outputs/*
endef

1-floorplan/.execstamp: 1-floorplan/.stamp
	$(call 1_floorplan_commands_rule)
	touch $@

1-floorplan/outputs/.execstamp.floorplan.tcl: 1-floorplan/.execstamp
	touch $@

endif

# collect outputs

1-floorplan/outputs/.stamp.floorplan.tcl: 1-floorplan/.execstamp 1-floorplan/outputs/.execstamp.floorplan.tcl
	$(call stamp,1-floorplan/outputs/.stamp.floorplan.tcl)

# post-conditions

define 1_floorplan_post_conditions_commands_rule
	cd 1-floorplan && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

1-floorplan/.postconditions.stamp: 1-floorplan/.execstamp 1-floorplan/outputs/.execstamp.floorplan.tcl 1-floorplan/outputs/.stamp.floorplan.tcl
	$(call 1_floorplan_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: floorplan

floorplan: 1-floorplan/.execstamp 1-floorplan/.postconditions.stamp 1-floorplan/outputs/.execstamp.floorplan.tcl 1-floorplan/outputs/.stamp.floorplan.tcl

.PHONY: 1

1: 1-floorplan/.execstamp 1-floorplan/.postconditions.stamp 1-floorplan/outputs/.execstamp.floorplan.tcl 1-floorplan/outputs/.stamp.floorplan.tcl

# debug

# ------------------------------------------------------------------------
# info
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 2-info/.prebuilt)","")
2-info/.stamp:
	$(call cpdir-and-parameterize,2-info,../../mflowgen/steps/info,2-info/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 2-info/.prebuilt)","")

define 2_info_commands_rule
	mkdir -p 2-info/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t info && chmod +x .mflowgen/2-info/mflowgen-run && cp -f .mflowgen/2-info/mflowgen-run 2-info && if [[ -e .mflowgen/2-info/mflowgen-debug ]]; then chmod +x .mflowgen/2-info/mflowgen-debug && cp -f .mflowgen/2-info/mflowgen-debug 2-info; fi && if [[ -e .mflowgen/2-info/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/2-info/mflowgen-check-preconditions.py && cp -f .mflowgen/2-info/mflowgen-check-preconditions.py 2-info; fi && if [[ -e .mflowgen/2-info/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/2-info/mflowgen-check-postconditions.py && cp -f .mflowgen/2-info/mflowgen-check-postconditions.py 2-info; fi && cd 2-info && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 2-info/outputs/*
endef

2-info/.execstamp: 2-info/.stamp
	$(call 2_info_commands_rule)

2-info/.execstamp.execute-phony: 2-info/.execstamp
	touch $@

endif

# collect outputs

# post-conditions

define 2_info_post_conditions_commands_rule
	cd 2-info && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

2-info/.postconditions.stamp: 2-info/.execstamp 2-info/.execstamp.execute-phony
	$(call 2_info_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: info

info: 2-info/.execstamp 2-info/.execstamp.execute-phony 2-info/.postconditions.stamp

.PHONY: 2

2: 2-info/.execstamp 2-info/.execstamp.execute-phony 2-info/.postconditions.stamp

# debug

# ------------------------------------------------------------------------
# macros
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 3-macros/.prebuilt)","")
3-macros/.stamp:
	$(call cpdir-and-parameterize,3-macros,../design/macros,3-macros/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 3-macros/.prebuilt)","")

define 3_macros_commands_rule
	mkdir -p 3-macros/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t macros && chmod +x .mflowgen/3-macros/mflowgen-run && cp -f .mflowgen/3-macros/mflowgen-run 3-macros && if [[ -e .mflowgen/3-macros/mflowgen-debug ]]; then chmod +x .mflowgen/3-macros/mflowgen-debug && cp -f .mflowgen/3-macros/mflowgen-debug 3-macros; fi && if [[ -e .mflowgen/3-macros/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/3-macros/mflowgen-check-preconditions.py && cp -f .mflowgen/3-macros/mflowgen-check-preconditions.py 3-macros; fi && if [[ -e .mflowgen/3-macros/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/3-macros/mflowgen-check-postconditions.py && cp -f .mflowgen/3-macros/mflowgen-check-postconditions.py 3-macros; fi && cd 3-macros && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 3-macros/outputs/*
endef

3-macros/.execstamp: 3-macros/.stamp
	$(call 3_macros_commands_rule)
	touch $@

3-macros/outputs/.execstamp.dffram: 3-macros/.execstamp
	touch $@

endif

# collect outputs

3-macros/outputs/.stamp.dffram: 3-macros/.execstamp 3-macros/outputs/.execstamp.dffram
	$(call stamp,3-macros/outputs/.stamp.dffram)

# post-conditions

define 3_macros_post_conditions_commands_rule
	cd 3-macros && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

3-macros/.postconditions.stamp: 3-macros/.execstamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram
	$(call 3_macros_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: macros

macros: 3-macros/.execstamp 3-macros/.postconditions.stamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram

.PHONY: 3

3: 3-macros/.execstamp 3-macros/.postconditions.stamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram

# debug

# ------------------------------------------------------------------------
# pin-placement
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 4-pin-placement/.prebuilt)","")
4-pin-placement/.stamp:
	$(call cpdir-and-parameterize,4-pin-placement,../design/pin-placement,4-pin-placement/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 4-pin-placement/.prebuilt)","")

define 4_pin_placement_commands_rule
	mkdir -p 4-pin-placement/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t pin-placement && chmod +x .mflowgen/4-pin-placement/mflowgen-run && cp -f .mflowgen/4-pin-placement/mflowgen-run 4-pin-placement && if [[ -e .mflowgen/4-pin-placement/mflowgen-debug ]]; then chmod +x .mflowgen/4-pin-placement/mflowgen-debug && cp -f .mflowgen/4-pin-placement/mflowgen-debug 4-pin-placement; fi && if [[ -e .mflowgen/4-pin-placement/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/4-pin-placement/mflowgen-check-preconditions.py && cp -f .mflowgen/4-pin-placement/mflowgen-check-preconditions.py 4-pin-placement; fi && if [[ -e .mflowgen/4-pin-placement/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/4-pin-placement/mflowgen-check-postconditions.py && cp -f .mflowgen/4-pin-placement/mflowgen-check-postconditions.py 4-pin-placement; fi && cd 4-pin-placement && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 4-pin-placement/outputs/*
endef

4-pin-placement/.execstamp: 4-pin-placement/.stamp
	$(call 4_pin_placement_commands_rule)
	touch $@

4-pin-placement/outputs/.execstamp.pin-assignments.tcl: 4-pin-placement/.execstamp
	touch $@

endif

# collect outputs

4-pin-placement/outputs/.stamp.pin-assignments.tcl: 4-pin-placement/.execstamp 4-pin-placement/outputs/.execstamp.pin-assignments.tcl
	$(call stamp,4-pin-placement/outputs/.stamp.pin-assignments.tcl)

# post-conditions

define 4_pin_placement_post_conditions_commands_rule
	cd 4-pin-placement && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

4-pin-placement/.postconditions.stamp: 4-pin-placement/.execstamp 4-pin-placement/outputs/.execstamp.pin-assignments.tcl 4-pin-placement/outputs/.stamp.pin-assignments.tcl
	$(call 4_pin_placement_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: pin-placement

pin-placement: 4-pin-placement/.execstamp 4-pin-placement/.postconditions.stamp 4-pin-placement/outputs/.execstamp.pin-assignments.tcl 4-pin-placement/outputs/.stamp.pin-assignments.tcl

.PHONY: 4

4: 4-pin-placement/.execstamp 4-pin-placement/.postconditions.stamp 4-pin-placement/outputs/.execstamp.pin-assignments.tcl 4-pin-placement/outputs/.stamp.pin-assignments.tcl

# debug

# ------------------------------------------------------------------------
# plugin-dc
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 5-plugin-dc/.prebuilt)","")
5-plugin-dc/.stamp:
	$(call cpdir-and-parameterize,5-plugin-dc,../design/plugin-dc,5-plugin-dc/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 5-plugin-dc/.prebuilt)","")

define 5_plugin_dc_commands_rule
	mkdir -p 5-plugin-dc/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t plugin-dc && chmod +x .mflowgen/5-plugin-dc/mflowgen-run && cp -f .mflowgen/5-plugin-dc/mflowgen-run 5-plugin-dc && if [[ -e .mflowgen/5-plugin-dc/mflowgen-debug ]]; then chmod +x .mflowgen/5-plugin-dc/mflowgen-debug && cp -f .mflowgen/5-plugin-dc/mflowgen-debug 5-plugin-dc; fi && if [[ -e .mflowgen/5-plugin-dc/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/5-plugin-dc/mflowgen-check-preconditions.py && cp -f .mflowgen/5-plugin-dc/mflowgen-check-preconditions.py 5-plugin-dc; fi && if [[ -e .mflowgen/5-plugin-dc/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/5-plugin-dc/mflowgen-check-postconditions.py && cp -f .mflowgen/5-plugin-dc/mflowgen-check-postconditions.py 5-plugin-dc; fi && cd 5-plugin-dc && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 5-plugin-dc/outputs/*
endef

5-plugin-dc/.execstamp: 5-plugin-dc/.stamp
	$(call 5_plugin_dc_commands_rule)
	touch $@

5-plugin-dc/outputs/.execstamp.designer-interface.tcl: 5-plugin-dc/.execstamp
	touch $@

endif

# collect outputs

5-plugin-dc/outputs/.stamp.designer-interface.tcl: 5-plugin-dc/.execstamp 5-plugin-dc/outputs/.execstamp.designer-interface.tcl
	$(call stamp,5-plugin-dc/outputs/.stamp.designer-interface.tcl)

# post-conditions

define 5_plugin_dc_post_conditions_commands_rule
	cd 5-plugin-dc && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

5-plugin-dc/.postconditions.stamp: 5-plugin-dc/.execstamp 5-plugin-dc/outputs/.execstamp.designer-interface.tcl 5-plugin-dc/outputs/.stamp.designer-interface.tcl
	$(call 5_plugin_dc_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: plugin-dc

plugin-dc: 5-plugin-dc/.execstamp 5-plugin-dc/.postconditions.stamp 5-plugin-dc/outputs/.execstamp.designer-interface.tcl 5-plugin-dc/outputs/.stamp.designer-interface.tcl

.PHONY: 5

5: 5-plugin-dc/.execstamp 5-plugin-dc/.postconditions.stamp 5-plugin-dc/outputs/.execstamp.designer-interface.tcl 5-plugin-dc/outputs/.stamp.designer-interface.tcl

# debug

# ------------------------------------------------------------------------
# plugin-dc-retime
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 6-plugin-dc-retime/.prebuilt)","")
6-plugin-dc-retime/.stamp:
	$(call cpdir-and-parameterize,6-plugin-dc-retime,../design/plugin-dc-retime,6-plugin-dc-retime/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 6-plugin-dc-retime/.prebuilt)","")

define 6_plugin_dc_retime_commands_rule
	mkdir -p 6-plugin-dc-retime/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t plugin-dc-retime && chmod +x .mflowgen/6-plugin-dc-retime/mflowgen-run && cp -f .mflowgen/6-plugin-dc-retime/mflowgen-run 6-plugin-dc-retime && if [[ -e .mflowgen/6-plugin-dc-retime/mflowgen-debug ]]; then chmod +x .mflowgen/6-plugin-dc-retime/mflowgen-debug && cp -f .mflowgen/6-plugin-dc-retime/mflowgen-debug 6-plugin-dc-retime; fi && if [[ -e .mflowgen/6-plugin-dc-retime/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/6-plugin-dc-retime/mflowgen-check-preconditions.py && cp -f .mflowgen/6-plugin-dc-retime/mflowgen-check-preconditions.py 6-plugin-dc-retime; fi && if [[ -e .mflowgen/6-plugin-dc-retime/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/6-plugin-dc-retime/mflowgen-check-postconditions.py && cp -f .mflowgen/6-plugin-dc-retime/mflowgen-check-postconditions.py 6-plugin-dc-retime; fi && cd 6-plugin-dc-retime && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 6-plugin-dc-retime/outputs/*
endef

6-plugin-dc-retime/.execstamp: 6-plugin-dc-retime/.stamp
	$(call 6_plugin_dc_retime_commands_rule)
	touch $@

6-plugin-dc-retime/outputs/.execstamp.retime.tcl: 6-plugin-dc-retime/.execstamp
	touch $@

endif

# collect outputs

6-plugin-dc-retime/outputs/.stamp.retime.tcl: 6-plugin-dc-retime/.execstamp 6-plugin-dc-retime/outputs/.execstamp.retime.tcl
	$(call stamp,6-plugin-dc-retime/outputs/.stamp.retime.tcl)

# post-conditions

define 6_plugin_dc_retime_post_conditions_commands_rule
	cd 6-plugin-dc-retime && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

6-plugin-dc-retime/.postconditions.stamp: 6-plugin-dc-retime/.execstamp 6-plugin-dc-retime/outputs/.execstamp.retime.tcl 6-plugin-dc-retime/outputs/.stamp.retime.tcl
	$(call 6_plugin_dc_retime_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: plugin-dc-retime

plugin-dc-retime: 6-plugin-dc-retime/.execstamp 6-plugin-dc-retime/.postconditions.stamp 6-plugin-dc-retime/outputs/.execstamp.retime.tcl 6-plugin-dc-retime/outputs/.stamp.retime.tcl

.PHONY: 6

6: 6-plugin-dc-retime/.execstamp 6-plugin-dc-retime/.postconditions.stamp 6-plugin-dc-retime/outputs/.execstamp.retime.tcl 6-plugin-dc-retime/outputs/.stamp.retime.tcl

# debug

# ------------------------------------------------------------------------
# rtl
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 7-rtl/.prebuilt)","")
7-rtl/.stamp:
	$(call cpdir-and-parameterize,7-rtl,../design/rtl,7-rtl/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 7-rtl/.prebuilt)","")

define 7_rtl_commands_rule
	mkdir -p 7-rtl/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t rtl && chmod +x .mflowgen/7-rtl/mflowgen-run && cp -f .mflowgen/7-rtl/mflowgen-run 7-rtl && if [[ -e .mflowgen/7-rtl/mflowgen-debug ]]; then chmod +x .mflowgen/7-rtl/mflowgen-debug && cp -f .mflowgen/7-rtl/mflowgen-debug 7-rtl; fi && if [[ -e .mflowgen/7-rtl/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/7-rtl/mflowgen-check-preconditions.py && cp -f .mflowgen/7-rtl/mflowgen-check-preconditions.py 7-rtl; fi && if [[ -e .mflowgen/7-rtl/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/7-rtl/mflowgen-check-postconditions.py && cp -f .mflowgen/7-rtl/mflowgen-check-postconditions.py 7-rtl; fi && cd 7-rtl && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 7-rtl/outputs/*
endef

7-rtl/.execstamp: 7-rtl/.stamp
	$(call 7_rtl_commands_rule)
	touch $@

7-rtl/outputs/.execstamp.design.v: 7-rtl/.execstamp
	touch $@
7-rtl/outputs/.execstamp.design.vcs.v: 7-rtl/.execstamp
	touch $@

endif

# collect outputs

7-rtl/outputs/.stamp.design.v: 7-rtl/.execstamp 7-rtl/outputs/.execstamp.design.v 7-rtl/outputs/.execstamp.design.vcs.v
	$(call stamp,7-rtl/outputs/.stamp.design.v)

7-rtl/outputs/.stamp.design.vcs.v: 7-rtl/.execstamp 7-rtl/outputs/.execstamp.design.v 7-rtl/outputs/.execstamp.design.vcs.v
	$(call stamp,7-rtl/outputs/.stamp.design.vcs.v)

# post-conditions

define 7_rtl_post_conditions_commands_rule
	cd 7-rtl && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

7-rtl/.postconditions.stamp: 7-rtl/.execstamp 7-rtl/outputs/.execstamp.design.v 7-rtl/outputs/.execstamp.design.vcs.v 7-rtl/outputs/.stamp.design.v 7-rtl/outputs/.stamp.design.vcs.v
	$(call 7_rtl_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: rtl

rtl: 7-rtl/.execstamp 7-rtl/.postconditions.stamp 7-rtl/outputs/.execstamp.design.v 7-rtl/outputs/.execstamp.design.vcs.v 7-rtl/outputs/.stamp.design.v 7-rtl/outputs/.stamp.design.vcs.v

.PHONY: 7

7: 7-rtl/.execstamp 7-rtl/.postconditions.stamp 7-rtl/outputs/.execstamp.design.v 7-rtl/outputs/.execstamp.design.vcs.v 7-rtl/outputs/.stamp.design.v 7-rtl/outputs/.stamp.design.vcs.v

# debug

# ------------------------------------------------------------------------
# skywater-130nm
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 8-skywater-130nm/.prebuilt)","")
8-skywater-130nm/.stamp:
	$(call cpdir-and-parameterize,8-skywater-130nm,../../../../../afs/ir.stanford.edu/class/ee272/skywater-130nm-adk,8-skywater-130nm/.stamp)
endif

# collect inputs

# execute

ifeq ("$(wildcard 8-skywater-130nm/.prebuilt)","")

define 8_skywater_130nm_commands_rule
	mkdir -p 8-skywater-130nm/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t skywater-130nm && chmod +x .mflowgen/8-skywater-130nm/mflowgen-run && cp -f .mflowgen/8-skywater-130nm/mflowgen-run 8-skywater-130nm && if [[ -e .mflowgen/8-skywater-130nm/mflowgen-debug ]]; then chmod +x .mflowgen/8-skywater-130nm/mflowgen-debug && cp -f .mflowgen/8-skywater-130nm/mflowgen-debug 8-skywater-130nm; fi && if [[ -e .mflowgen/8-skywater-130nm/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/8-skywater-130nm/mflowgen-check-preconditions.py && cp -f .mflowgen/8-skywater-130nm/mflowgen-check-preconditions.py 8-skywater-130nm; fi && if [[ -e .mflowgen/8-skywater-130nm/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/8-skywater-130nm/mflowgen-check-postconditions.py && cp -f .mflowgen/8-skywater-130nm/mflowgen-check-postconditions.py 8-skywater-130nm; fi && cd 8-skywater-130nm && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 8-skywater-130nm/outputs/*
endef

8-skywater-130nm/.execstamp: 8-skywater-130nm/.stamp
	$(call 8_skywater_130nm_commands_rule)
	touch $@

8-skywater-130nm/outputs/.execstamp.adk: 8-skywater-130nm/.execstamp
	touch $@

endif

# collect outputs

8-skywater-130nm/outputs/.stamp.adk: 8-skywater-130nm/.execstamp 8-skywater-130nm/outputs/.execstamp.adk
	$(call stamp,8-skywater-130nm/outputs/.stamp.adk)

# post-conditions

define 8_skywater_130nm_post_conditions_commands_rule
	cd 8-skywater-130nm && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

8-skywater-130nm/.postconditions.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
	$(call 8_skywater_130nm_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: skywater-130nm

skywater-130nm: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk

.PHONY: 8

8: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk

# debug

# ------------------------------------------------------------------------
# synopsys-dc-synthesis
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/.stamp: 0-constraints/.execstamp 0-constraints/.postconditions.stamp 0-constraints/outputs/.execstamp.constraints.tcl 0-constraints/outputs/.stamp.constraints.tcl
9-synopsys-dc-synthesis/.stamp: 3-macros/.execstamp 3-macros/.postconditions.stamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram
9-synopsys-dc-synthesis/.stamp: 5-plugin-dc/.execstamp 5-plugin-dc/.postconditions.stamp 5-plugin-dc/outputs/.execstamp.designer-interface.tcl 5-plugin-dc/outputs/.stamp.designer-interface.tcl
9-synopsys-dc-synthesis/.stamp: 6-plugin-dc-retime/.execstamp 6-plugin-dc-retime/.postconditions.stamp 6-plugin-dc-retime/outputs/.execstamp.retime.tcl 6-plugin-dc-retime/outputs/.stamp.retime.tcl
9-synopsys-dc-synthesis/.stamp: 7-rtl/.execstamp 7-rtl/.postconditions.stamp 7-rtl/outputs/.execstamp.design.v 7-rtl/outputs/.execstamp.design.vcs.v 7-rtl/outputs/.stamp.design.v 7-rtl/outputs/.stamp.design.vcs.v
9-synopsys-dc-synthesis/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
9-synopsys-dc-synthesis/.stamp:
	$(call cpdir-and-parameterize,9-synopsys-dc-synthesis,../../mflowgen/steps/synopsys-dc-synthesis,9-synopsys-dc-synthesis/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.adk: 9-synopsys-dc-synthesis/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,9-synopsys-dc-synthesis/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.adk: 9-synopsys-dc-synthesis/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,9-synopsys-dc-synthesis/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.constraints.tcl: 9-synopsys-dc-synthesis/.stamp 0-constraints/outputs/.stamp.constraints.tcl
	$(call symlink,9-synopsys-dc-synthesis/inputs,constraints.tcl,../../0-constraints/outputs/constraints.tcl,.stamp.constraints.tcl)
endif

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.design.v: 9-synopsys-dc-synthesis/.stamp 7-rtl/outputs/.stamp.design.v
	$(call symlink,9-synopsys-dc-synthesis/inputs,design.v,../../7-rtl/outputs/design.v,.stamp.design.v)
endif

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.designer-interface.tcl: 9-synopsys-dc-synthesis/.stamp 5-plugin-dc/outputs/.stamp.designer-interface.tcl
	$(call symlink,9-synopsys-dc-synthesis/inputs,designer-interface.tcl,../../5-plugin-dc/outputs/designer-interface.tcl,.stamp.designer-interface.tcl)
endif

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.dffram: 9-synopsys-dc-synthesis/.stamp 3-macros/outputs/.stamp.dffram
	$(call symlink,9-synopsys-dc-synthesis/inputs,dffram,../../3-macros/outputs/dffram,.stamp.dffram)
endif

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")
9-synopsys-dc-synthesis/inputs/.stamp.retime.tcl: 9-synopsys-dc-synthesis/.stamp 6-plugin-dc-retime/outputs/.stamp.retime.tcl
	$(call symlink,9-synopsys-dc-synthesis/inputs,retime.tcl,../../6-plugin-dc-retime/outputs/retime.tcl,.stamp.retime.tcl)
endif

# execute

ifeq ("$(wildcard 9-synopsys-dc-synthesis/.prebuilt)","")

define 9_synopsys_dc_synthesis_commands_rule
	mkdir -p 9-synopsys-dc-synthesis/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t synopsys-dc-synthesis && chmod +x .mflowgen/9-synopsys-dc-synthesis/mflowgen-run && cp -f .mflowgen/9-synopsys-dc-synthesis/mflowgen-run 9-synopsys-dc-synthesis && if [[ -e .mflowgen/9-synopsys-dc-synthesis/mflowgen-debug ]]; then chmod +x .mflowgen/9-synopsys-dc-synthesis/mflowgen-debug && cp -f .mflowgen/9-synopsys-dc-synthesis/mflowgen-debug 9-synopsys-dc-synthesis; fi && if [[ -e .mflowgen/9-synopsys-dc-synthesis/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/9-synopsys-dc-synthesis/mflowgen-check-preconditions.py && cp -f .mflowgen/9-synopsys-dc-synthesis/mflowgen-check-preconditions.py 9-synopsys-dc-synthesis; fi && if [[ -e .mflowgen/9-synopsys-dc-synthesis/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/9-synopsys-dc-synthesis/mflowgen-check-postconditions.py && cp -f .mflowgen/9-synopsys-dc-synthesis/mflowgen-check-postconditions.py 9-synopsys-dc-synthesis; fi && cd 9-synopsys-dc-synthesis && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 9-synopsys-dc-synthesis/outputs/*
endef

9-synopsys-dc-synthesis/.execstamp: 9-synopsys-dc-synthesis/.stamp 9-synopsys-dc-synthesis/inputs/.stamp.adk 9-synopsys-dc-synthesis/inputs/.stamp.constraints.tcl 9-synopsys-dc-synthesis/inputs/.stamp.design.v 9-synopsys-dc-synthesis/inputs/.stamp.designer-interface.tcl 9-synopsys-dc-synthesis/inputs/.stamp.dffram 9-synopsys-dc-synthesis/inputs/.stamp.retime.tcl
	$(call 9_synopsys_dc_synthesis_commands_rule)
	touch $@

9-synopsys-dc-synthesis/outputs/.execstamp.design.v: 9-synopsys-dc-synthesis/.execstamp
	touch $@
9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc: 9-synopsys-dc-synthesis/.execstamp
	touch $@
9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap: 9-synopsys-dc-synthesis/.execstamp
	touch $@
9-synopsys-dc-synthesis/outputs/.execstamp.design.svf: 9-synopsys-dc-synthesis/.execstamp
	touch $@

endif

# collect outputs

9-synopsys-dc-synthesis/outputs/.stamp.design.v: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v
	$(call stamp,9-synopsys-dc-synthesis/outputs/.stamp.design.v)

9-synopsys-dc-synthesis/outputs/.stamp.design.sdc: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v
	$(call stamp,9-synopsys-dc-synthesis/outputs/.stamp.design.sdc)

9-synopsys-dc-synthesis/outputs/.stamp.design.namemap: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v
	$(call stamp,9-synopsys-dc-synthesis/outputs/.stamp.design.namemap)

9-synopsys-dc-synthesis/outputs/.stamp.design.svf: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v
	$(call stamp,9-synopsys-dc-synthesis/outputs/.stamp.design.svf)

# post-conditions

define 9_synopsys_dc_synthesis_post_conditions_commands_rule
	cd 9-synopsys-dc-synthesis && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

9-synopsys-dc-synthesis/.postconditions.stamp: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v
	$(call 9_synopsys_dc_synthesis_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: synopsys-dc-synthesis

synopsys-dc-synthesis: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v

.PHONY: 9

9: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v

# debug

define 9_debug_rule
	cd 9-synopsys-dc-synthesis && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-synopsys-dc-synthesis: 
	$(call 9_debug_rule)

.PHONY: debug-9

debug-9: debug-synopsys-dc-synthesis

# ------------------------------------------------------------------------
# cadence-innovus-flowsetup
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 10-cadence-innovus-flowsetup/.prebuilt)","")
10-cadence-innovus-flowsetup/.stamp: 3-macros/.execstamp 3-macros/.postconditions.stamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram
10-cadence-innovus-flowsetup/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
10-cadence-innovus-flowsetup/.stamp: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v
10-cadence-innovus-flowsetup/.stamp:
	$(call cpdir-and-parameterize,10-cadence-innovus-flowsetup,../design/iflow,10-cadence-innovus-flowsetup/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 10-cadence-innovus-flowsetup/.prebuilt)","")
10-cadence-innovus-flowsetup/inputs/.stamp.adk: 10-cadence-innovus-flowsetup/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,10-cadence-innovus-flowsetup/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 10-cadence-innovus-flowsetup/.prebuilt)","")
10-cadence-innovus-flowsetup/inputs/.stamp.design.sdc: 10-cadence-innovus-flowsetup/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc
	$(call symlink,10-cadence-innovus-flowsetup/inputs,design.sdc,../../9-synopsys-dc-synthesis/outputs/design.sdc,.stamp.design.sdc)
endif

ifeq ("$(wildcard 10-cadence-innovus-flowsetup/.prebuilt)","")
10-cadence-innovus-flowsetup/inputs/.stamp.design.v: 10-cadence-innovus-flowsetup/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.v
	$(call symlink,10-cadence-innovus-flowsetup/inputs,design.v,../../9-synopsys-dc-synthesis/outputs/design.v,.stamp.design.v)
endif

ifeq ("$(wildcard 10-cadence-innovus-flowsetup/.prebuilt)","")
10-cadence-innovus-flowsetup/inputs/.stamp.dffram: 10-cadence-innovus-flowsetup/.stamp 3-macros/outputs/.stamp.dffram
	$(call symlink,10-cadence-innovus-flowsetup/inputs,dffram,../../3-macros/outputs/dffram,.stamp.dffram)
endif

# execute

ifeq ("$(wildcard 10-cadence-innovus-flowsetup/.prebuilt)","")

define 10_cadence_innovus_flowsetup_commands_rule
	mkdir -p 10-cadence-innovus-flowsetup/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-flowsetup && chmod +x .mflowgen/10-cadence-innovus-flowsetup/mflowgen-run && cp -f .mflowgen/10-cadence-innovus-flowsetup/mflowgen-run 10-cadence-innovus-flowsetup && if [[ -e .mflowgen/10-cadence-innovus-flowsetup/mflowgen-debug ]]; then chmod +x .mflowgen/10-cadence-innovus-flowsetup/mflowgen-debug && cp -f .mflowgen/10-cadence-innovus-flowsetup/mflowgen-debug 10-cadence-innovus-flowsetup; fi && if [[ -e .mflowgen/10-cadence-innovus-flowsetup/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/10-cadence-innovus-flowsetup/mflowgen-check-preconditions.py && cp -f .mflowgen/10-cadence-innovus-flowsetup/mflowgen-check-preconditions.py 10-cadence-innovus-flowsetup; fi && if [[ -e .mflowgen/10-cadence-innovus-flowsetup/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/10-cadence-innovus-flowsetup/mflowgen-check-postconditions.py && cp -f .mflowgen/10-cadence-innovus-flowsetup/mflowgen-check-postconditions.py 10-cadence-innovus-flowsetup; fi && cd 10-cadence-innovus-flowsetup && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 10-cadence-innovus-flowsetup/outputs/*
endef

10-cadence-innovus-flowsetup/.execstamp: 10-cadence-innovus-flowsetup/.stamp 10-cadence-innovus-flowsetup/inputs/.stamp.adk 10-cadence-innovus-flowsetup/inputs/.stamp.design.sdc 10-cadence-innovus-flowsetup/inputs/.stamp.design.v 10-cadence-innovus-flowsetup/inputs/.stamp.dffram
	$(call 10_cadence_innovus_flowsetup_commands_rule)
	touch $@

10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow: 10-cadence-innovus-flowsetup/.execstamp
	touch $@

endif

# collect outputs

10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow
	$(call stamp,10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow)

# post-conditions

define 10_cadence_innovus_flowsetup_post_conditions_commands_rule
	cd 10-cadence-innovus-flowsetup && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

10-cadence-innovus-flowsetup/.postconditions.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call 10_cadence_innovus_flowsetup_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-flowsetup

cadence-innovus-flowsetup: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow

.PHONY: 10

10: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow

# debug

# ------------------------------------------------------------------------
# cadence-innovus-init
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/.stamp: 1-floorplan/.execstamp 1-floorplan/.postconditions.stamp 1-floorplan/outputs/.execstamp.floorplan.tcl 1-floorplan/outputs/.stamp.floorplan.tcl
11-cadence-innovus-init/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
11-cadence-innovus-init/.stamp: 3-macros/.execstamp 3-macros/.postconditions.stamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram
11-cadence-innovus-init/.stamp: 4-pin-placement/.execstamp 4-pin-placement/.postconditions.stamp 4-pin-placement/outputs/.execstamp.pin-assignments.tcl 4-pin-placement/outputs/.stamp.pin-assignments.tcl
11-cadence-innovus-init/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
11-cadence-innovus-init/.stamp: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v
11-cadence-innovus-init/.stamp:
	$(call cpdir-and-parameterize,11-cadence-innovus-init,../../mflowgen/steps/cadence-innovus-init,11-cadence-innovus-init/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.adk: 11-cadence-innovus-init/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,11-cadence-innovus-init/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.design.sdc: 11-cadence-innovus-init/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc
	$(call symlink,11-cadence-innovus-init/inputs,design.sdc,../../9-synopsys-dc-synthesis/outputs/design.sdc,.stamp.design.sdc)
endif

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.design.v: 11-cadence-innovus-init/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.v
	$(call symlink,11-cadence-innovus-init/inputs,design.v,../../9-synopsys-dc-synthesis/outputs/design.v,.stamp.design.v)
endif

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.dffram: 11-cadence-innovus-init/.stamp 3-macros/outputs/.stamp.dffram
	$(call symlink,11-cadence-innovus-init/inputs,dffram,../../3-macros/outputs/dffram,.stamp.dffram)
endif

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.floorplan.tcl: 11-cadence-innovus-init/.stamp 1-floorplan/outputs/.stamp.floorplan.tcl
	$(call symlink,11-cadence-innovus-init/inputs,floorplan.tcl,../../1-floorplan/outputs/floorplan.tcl,.stamp.floorplan.tcl)
endif

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.innovus-foundation-flow: 11-cadence-innovus-init/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,11-cadence-innovus-init/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")
11-cadence-innovus-init/inputs/.stamp.pin-assignments.tcl: 11-cadence-innovus-init/.stamp 4-pin-placement/outputs/.stamp.pin-assignments.tcl
	$(call symlink,11-cadence-innovus-init/inputs,pin-assignments.tcl,../../4-pin-placement/outputs/pin-assignments.tcl,.stamp.pin-assignments.tcl)
endif

# execute

ifeq ("$(wildcard 11-cadence-innovus-init/.prebuilt)","")

define 11_cadence_innovus_init_commands_rule
	mkdir -p 11-cadence-innovus-init/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-init && chmod +x .mflowgen/11-cadence-innovus-init/mflowgen-run && cp -f .mflowgen/11-cadence-innovus-init/mflowgen-run 11-cadence-innovus-init && if [[ -e .mflowgen/11-cadence-innovus-init/mflowgen-debug ]]; then chmod +x .mflowgen/11-cadence-innovus-init/mflowgen-debug && cp -f .mflowgen/11-cadence-innovus-init/mflowgen-debug 11-cadence-innovus-init; fi && if [[ -e .mflowgen/11-cadence-innovus-init/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/11-cadence-innovus-init/mflowgen-check-preconditions.py && cp -f .mflowgen/11-cadence-innovus-init/mflowgen-check-preconditions.py 11-cadence-innovus-init; fi && if [[ -e .mflowgen/11-cadence-innovus-init/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/11-cadence-innovus-init/mflowgen-check-postconditions.py && cp -f .mflowgen/11-cadence-innovus-init/mflowgen-check-postconditions.py 11-cadence-innovus-init; fi && cd 11-cadence-innovus-init && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 11-cadence-innovus-init/outputs/*
endef

11-cadence-innovus-init/.execstamp: 11-cadence-innovus-init/.stamp 11-cadence-innovus-init/inputs/.stamp.adk 11-cadence-innovus-init/inputs/.stamp.design.sdc 11-cadence-innovus-init/inputs/.stamp.design.v 11-cadence-innovus-init/inputs/.stamp.dffram 11-cadence-innovus-init/inputs/.stamp.floorplan.tcl 11-cadence-innovus-init/inputs/.stamp.innovus-foundation-flow 11-cadence-innovus-init/inputs/.stamp.pin-assignments.tcl
	$(call 11_cadence_innovus_init_commands_rule)
	touch $@

11-cadence-innovus-init/outputs/.execstamp.design.checkpoint: 11-cadence-innovus-init/.execstamp
	touch $@

endif

# collect outputs

11-cadence-innovus-init/outputs/.stamp.design.checkpoint: 11-cadence-innovus-init/.execstamp 11-cadence-innovus-init/outputs/.execstamp.design.checkpoint
	$(call stamp,11-cadence-innovus-init/outputs/.stamp.design.checkpoint)

# post-conditions

define 11_cadence_innovus_init_post_conditions_commands_rule
	cd 11-cadence-innovus-init && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

11-cadence-innovus-init/.postconditions.stamp: 11-cadence-innovus-init/.execstamp 11-cadence-innovus-init/outputs/.execstamp.design.checkpoint 11-cadence-innovus-init/outputs/.stamp.design.checkpoint
	$(call 11_cadence_innovus_init_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-init

cadence-innovus-init: 11-cadence-innovus-init/.execstamp 11-cadence-innovus-init/.postconditions.stamp 11-cadence-innovus-init/outputs/.execstamp.design.checkpoint 11-cadence-innovus-init/outputs/.stamp.design.checkpoint

.PHONY: 11

11: 11-cadence-innovus-init/.execstamp 11-cadence-innovus-init/.postconditions.stamp 11-cadence-innovus-init/outputs/.execstamp.design.checkpoint 11-cadence-innovus-init/outputs/.stamp.design.checkpoint

# debug

define 11_debug_rule
	cd 11-cadence-innovus-init && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-init: 
	$(call 11_debug_rule)

.PHONY: debug-11

debug-11: debug-cadence-innovus-init

# ------------------------------------------------------------------------
# cadence-innovus-power
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 12-cadence-innovus-power/.prebuilt)","")
12-cadence-innovus-power/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
12-cadence-innovus-power/.stamp: 11-cadence-innovus-init/.execstamp 11-cadence-innovus-init/.postconditions.stamp 11-cadence-innovus-init/outputs/.execstamp.design.checkpoint 11-cadence-innovus-init/outputs/.stamp.design.checkpoint
12-cadence-innovus-power/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
12-cadence-innovus-power/.stamp: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v
12-cadence-innovus-power/.stamp:
	$(call cpdir-and-parameterize,12-cadence-innovus-power,../design/cadence-innovus-power,12-cadence-innovus-power/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 12-cadence-innovus-power/.prebuilt)","")
12-cadence-innovus-power/inputs/.stamp.adk: 12-cadence-innovus-power/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,12-cadence-innovus-power/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 12-cadence-innovus-power/.prebuilt)","")
12-cadence-innovus-power/inputs/.stamp.design.checkpoint: 12-cadence-innovus-power/.stamp 11-cadence-innovus-init/outputs/.stamp.design.checkpoint
	$(call symlink,12-cadence-innovus-power/inputs,design.checkpoint,../../11-cadence-innovus-init/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 12-cadence-innovus-power/.prebuilt)","")
12-cadence-innovus-power/inputs/.stamp.design.sdc: 12-cadence-innovus-power/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc
	$(call symlink,12-cadence-innovus-power/inputs,design.sdc,../../9-synopsys-dc-synthesis/outputs/design.sdc,.stamp.design.sdc)
endif

ifeq ("$(wildcard 12-cadence-innovus-power/.prebuilt)","")
12-cadence-innovus-power/inputs/.stamp.innovus-foundation-flow: 12-cadence-innovus-power/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,12-cadence-innovus-power/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 12-cadence-innovus-power/.prebuilt)","")

define 12_cadence_innovus_power_commands_rule
	mkdir -p 12-cadence-innovus-power/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-power && chmod +x .mflowgen/12-cadence-innovus-power/mflowgen-run && cp -f .mflowgen/12-cadence-innovus-power/mflowgen-run 12-cadence-innovus-power && if [[ -e .mflowgen/12-cadence-innovus-power/mflowgen-debug ]]; then chmod +x .mflowgen/12-cadence-innovus-power/mflowgen-debug && cp -f .mflowgen/12-cadence-innovus-power/mflowgen-debug 12-cadence-innovus-power; fi && if [[ -e .mflowgen/12-cadence-innovus-power/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/12-cadence-innovus-power/mflowgen-check-preconditions.py && cp -f .mflowgen/12-cadence-innovus-power/mflowgen-check-preconditions.py 12-cadence-innovus-power; fi && if [[ -e .mflowgen/12-cadence-innovus-power/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/12-cadence-innovus-power/mflowgen-check-postconditions.py && cp -f .mflowgen/12-cadence-innovus-power/mflowgen-check-postconditions.py 12-cadence-innovus-power; fi && cd 12-cadence-innovus-power && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 12-cadence-innovus-power/outputs/*
endef

12-cadence-innovus-power/.execstamp: 12-cadence-innovus-power/.stamp 12-cadence-innovus-power/inputs/.stamp.adk 12-cadence-innovus-power/inputs/.stamp.design.checkpoint 12-cadence-innovus-power/inputs/.stamp.design.sdc 12-cadence-innovus-power/inputs/.stamp.innovus-foundation-flow
	$(call 12_cadence_innovus_power_commands_rule)
	touch $@

12-cadence-innovus-power/outputs/.execstamp.design.checkpoint: 12-cadence-innovus-power/.execstamp
	touch $@

endif

# collect outputs

12-cadence-innovus-power/outputs/.stamp.design.checkpoint: 12-cadence-innovus-power/.execstamp 12-cadence-innovus-power/outputs/.execstamp.design.checkpoint
	$(call stamp,12-cadence-innovus-power/outputs/.stamp.design.checkpoint)

# post-conditions

define 12_cadence_innovus_power_post_conditions_commands_rule
	cd 12-cadence-innovus-power && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

12-cadence-innovus-power/.postconditions.stamp: 12-cadence-innovus-power/.execstamp 12-cadence-innovus-power/outputs/.execstamp.design.checkpoint 12-cadence-innovus-power/outputs/.stamp.design.checkpoint
	$(call 12_cadence_innovus_power_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-power

cadence-innovus-power: 12-cadence-innovus-power/.execstamp 12-cadence-innovus-power/.postconditions.stamp 12-cadence-innovus-power/outputs/.execstamp.design.checkpoint 12-cadence-innovus-power/outputs/.stamp.design.checkpoint

.PHONY: 12

12: 12-cadence-innovus-power/.execstamp 12-cadence-innovus-power/.postconditions.stamp 12-cadence-innovus-power/outputs/.execstamp.design.checkpoint 12-cadence-innovus-power/outputs/.stamp.design.checkpoint

# debug

define 12_debug_rule
	cd 12-cadence-innovus-power && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-power: 
	$(call 12_debug_rule)

.PHONY: debug-12

debug-12: debug-cadence-innovus-power

# ------------------------------------------------------------------------
# cadence-innovus-place
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 13-cadence-innovus-place/.prebuilt)","")
13-cadence-innovus-place/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
13-cadence-innovus-place/.stamp: 12-cadence-innovus-power/.execstamp 12-cadence-innovus-power/.postconditions.stamp 12-cadence-innovus-power/outputs/.execstamp.design.checkpoint 12-cadence-innovus-power/outputs/.stamp.design.checkpoint
13-cadence-innovus-place/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
13-cadence-innovus-place/.stamp: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v
13-cadence-innovus-place/.stamp:
	$(call cpdir-and-parameterize,13-cadence-innovus-place,../../mflowgen/steps/cadence-innovus-place,13-cadence-innovus-place/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 13-cadence-innovus-place/.prebuilt)","")
13-cadence-innovus-place/inputs/.stamp.adk: 13-cadence-innovus-place/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,13-cadence-innovus-place/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 13-cadence-innovus-place/.prebuilt)","")
13-cadence-innovus-place/inputs/.stamp.design.checkpoint: 13-cadence-innovus-place/.stamp 12-cadence-innovus-power/outputs/.stamp.design.checkpoint
	$(call symlink,13-cadence-innovus-place/inputs,design.checkpoint,../../12-cadence-innovus-power/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 13-cadence-innovus-place/.prebuilt)","")
13-cadence-innovus-place/inputs/.stamp.design.sdc: 13-cadence-innovus-place/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc
	$(call symlink,13-cadence-innovus-place/inputs,design.sdc,../../9-synopsys-dc-synthesis/outputs/design.sdc,.stamp.design.sdc)
endif

ifeq ("$(wildcard 13-cadence-innovus-place/.prebuilt)","")
13-cadence-innovus-place/inputs/.stamp.innovus-foundation-flow: 13-cadence-innovus-place/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,13-cadence-innovus-place/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 13-cadence-innovus-place/.prebuilt)","")

define 13_cadence_innovus_place_commands_rule
	mkdir -p 13-cadence-innovus-place/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-place && chmod +x .mflowgen/13-cadence-innovus-place/mflowgen-run && cp -f .mflowgen/13-cadence-innovus-place/mflowgen-run 13-cadence-innovus-place && if [[ -e .mflowgen/13-cadence-innovus-place/mflowgen-debug ]]; then chmod +x .mflowgen/13-cadence-innovus-place/mflowgen-debug && cp -f .mflowgen/13-cadence-innovus-place/mflowgen-debug 13-cadence-innovus-place; fi && if [[ -e .mflowgen/13-cadence-innovus-place/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/13-cadence-innovus-place/mflowgen-check-preconditions.py && cp -f .mflowgen/13-cadence-innovus-place/mflowgen-check-preconditions.py 13-cadence-innovus-place; fi && if [[ -e .mflowgen/13-cadence-innovus-place/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/13-cadence-innovus-place/mflowgen-check-postconditions.py && cp -f .mflowgen/13-cadence-innovus-place/mflowgen-check-postconditions.py 13-cadence-innovus-place; fi && cd 13-cadence-innovus-place && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 13-cadence-innovus-place/outputs/*
endef

13-cadence-innovus-place/.execstamp: 13-cadence-innovus-place/.stamp 13-cadence-innovus-place/inputs/.stamp.adk 13-cadence-innovus-place/inputs/.stamp.design.checkpoint 13-cadence-innovus-place/inputs/.stamp.design.sdc 13-cadence-innovus-place/inputs/.stamp.innovus-foundation-flow
	$(call 13_cadence_innovus_place_commands_rule)
	touch $@

13-cadence-innovus-place/outputs/.execstamp.design.checkpoint: 13-cadence-innovus-place/.execstamp
	touch $@

endif

# collect outputs

13-cadence-innovus-place/outputs/.stamp.design.checkpoint: 13-cadence-innovus-place/.execstamp 13-cadence-innovus-place/outputs/.execstamp.design.checkpoint
	$(call stamp,13-cadence-innovus-place/outputs/.stamp.design.checkpoint)

# post-conditions

define 13_cadence_innovus_place_post_conditions_commands_rule
	cd 13-cadence-innovus-place && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

13-cadence-innovus-place/.postconditions.stamp: 13-cadence-innovus-place/.execstamp 13-cadence-innovus-place/outputs/.execstamp.design.checkpoint 13-cadence-innovus-place/outputs/.stamp.design.checkpoint
	$(call 13_cadence_innovus_place_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-place

cadence-innovus-place: 13-cadence-innovus-place/.execstamp 13-cadence-innovus-place/.postconditions.stamp 13-cadence-innovus-place/outputs/.execstamp.design.checkpoint 13-cadence-innovus-place/outputs/.stamp.design.checkpoint

.PHONY: 13

13: 13-cadence-innovus-place/.execstamp 13-cadence-innovus-place/.postconditions.stamp 13-cadence-innovus-place/outputs/.execstamp.design.checkpoint 13-cadence-innovus-place/outputs/.stamp.design.checkpoint

# debug

define 13_debug_rule
	cd 13-cadence-innovus-place && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-place: 
	$(call 13_debug_rule)

.PHONY: debug-13

debug-13: debug-cadence-innovus-place

# ------------------------------------------------------------------------
# cadence-innovus-cts
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 14-cadence-innovus-cts/.prebuilt)","")
14-cadence-innovus-cts/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
14-cadence-innovus-cts/.stamp: 13-cadence-innovus-place/.execstamp 13-cadence-innovus-place/.postconditions.stamp 13-cadence-innovus-place/outputs/.execstamp.design.checkpoint 13-cadence-innovus-place/outputs/.stamp.design.checkpoint
14-cadence-innovus-cts/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
14-cadence-innovus-cts/.stamp: 9-synopsys-dc-synthesis/.execstamp 9-synopsys-dc-synthesis/.postconditions.stamp 9-synopsys-dc-synthesis/outputs/.execstamp.design.namemap 9-synopsys-dc-synthesis/outputs/.execstamp.design.sdc 9-synopsys-dc-synthesis/outputs/.execstamp.design.svf 9-synopsys-dc-synthesis/outputs/.execstamp.design.v 9-synopsys-dc-synthesis/outputs/.stamp.design.namemap 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc 9-synopsys-dc-synthesis/outputs/.stamp.design.svf 9-synopsys-dc-synthesis/outputs/.stamp.design.v
14-cadence-innovus-cts/.stamp:
	$(call cpdir-and-parameterize,14-cadence-innovus-cts,../../mflowgen/steps/cadence-innovus-cts,14-cadence-innovus-cts/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 14-cadence-innovus-cts/.prebuilt)","")
14-cadence-innovus-cts/inputs/.stamp.adk: 14-cadence-innovus-cts/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,14-cadence-innovus-cts/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 14-cadence-innovus-cts/.prebuilt)","")
14-cadence-innovus-cts/inputs/.stamp.design.checkpoint: 14-cadence-innovus-cts/.stamp 13-cadence-innovus-place/outputs/.stamp.design.checkpoint
	$(call symlink,14-cadence-innovus-cts/inputs,design.checkpoint,../../13-cadence-innovus-place/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 14-cadence-innovus-cts/.prebuilt)","")
14-cadence-innovus-cts/inputs/.stamp.design.sdc: 14-cadence-innovus-cts/.stamp 9-synopsys-dc-synthesis/outputs/.stamp.design.sdc
	$(call symlink,14-cadence-innovus-cts/inputs,design.sdc,../../9-synopsys-dc-synthesis/outputs/design.sdc,.stamp.design.sdc)
endif

ifeq ("$(wildcard 14-cadence-innovus-cts/.prebuilt)","")
14-cadence-innovus-cts/inputs/.stamp.innovus-foundation-flow: 14-cadence-innovus-cts/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,14-cadence-innovus-cts/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 14-cadence-innovus-cts/.prebuilt)","")

define 14_cadence_innovus_cts_commands_rule
	mkdir -p 14-cadence-innovus-cts/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-cts && chmod +x .mflowgen/14-cadence-innovus-cts/mflowgen-run && cp -f .mflowgen/14-cadence-innovus-cts/mflowgen-run 14-cadence-innovus-cts && if [[ -e .mflowgen/14-cadence-innovus-cts/mflowgen-debug ]]; then chmod +x .mflowgen/14-cadence-innovus-cts/mflowgen-debug && cp -f .mflowgen/14-cadence-innovus-cts/mflowgen-debug 14-cadence-innovus-cts; fi && if [[ -e .mflowgen/14-cadence-innovus-cts/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/14-cadence-innovus-cts/mflowgen-check-preconditions.py && cp -f .mflowgen/14-cadence-innovus-cts/mflowgen-check-preconditions.py 14-cadence-innovus-cts; fi && if [[ -e .mflowgen/14-cadence-innovus-cts/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/14-cadence-innovus-cts/mflowgen-check-postconditions.py && cp -f .mflowgen/14-cadence-innovus-cts/mflowgen-check-postconditions.py 14-cadence-innovus-cts; fi && cd 14-cadence-innovus-cts && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 14-cadence-innovus-cts/outputs/*
endef

14-cadence-innovus-cts/.execstamp: 14-cadence-innovus-cts/.stamp 14-cadence-innovus-cts/inputs/.stamp.adk 14-cadence-innovus-cts/inputs/.stamp.design.checkpoint 14-cadence-innovus-cts/inputs/.stamp.design.sdc 14-cadence-innovus-cts/inputs/.stamp.innovus-foundation-flow
	$(call 14_cadence_innovus_cts_commands_rule)
	touch $@

14-cadence-innovus-cts/outputs/.execstamp.design.checkpoint: 14-cadence-innovus-cts/.execstamp
	touch $@

endif

# collect outputs

14-cadence-innovus-cts/outputs/.stamp.design.checkpoint: 14-cadence-innovus-cts/.execstamp 14-cadence-innovus-cts/outputs/.execstamp.design.checkpoint
	$(call stamp,14-cadence-innovus-cts/outputs/.stamp.design.checkpoint)

# post-conditions

define 14_cadence_innovus_cts_post_conditions_commands_rule
	cd 14-cadence-innovus-cts && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

14-cadence-innovus-cts/.postconditions.stamp: 14-cadence-innovus-cts/.execstamp 14-cadence-innovus-cts/outputs/.execstamp.design.checkpoint 14-cadence-innovus-cts/outputs/.stamp.design.checkpoint
	$(call 14_cadence_innovus_cts_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-cts

cadence-innovus-cts: 14-cadence-innovus-cts/.execstamp 14-cadence-innovus-cts/.postconditions.stamp 14-cadence-innovus-cts/outputs/.execstamp.design.checkpoint 14-cadence-innovus-cts/outputs/.stamp.design.checkpoint

.PHONY: 14

14: 14-cadence-innovus-cts/.execstamp 14-cadence-innovus-cts/.postconditions.stamp 14-cadence-innovus-cts/outputs/.execstamp.design.checkpoint 14-cadence-innovus-cts/outputs/.stamp.design.checkpoint

# debug

define 14_debug_rule
	cd 14-cadence-innovus-cts && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-cts: 
	$(call 14_debug_rule)

.PHONY: debug-14

debug-14: debug-cadence-innovus-cts

# ------------------------------------------------------------------------
# cadence-innovus-postcts_hold
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 15-cadence-innovus-postcts_hold/.prebuilt)","")
15-cadence-innovus-postcts_hold/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
15-cadence-innovus-postcts_hold/.stamp: 14-cadence-innovus-cts/.execstamp 14-cadence-innovus-cts/.postconditions.stamp 14-cadence-innovus-cts/outputs/.execstamp.design.checkpoint 14-cadence-innovus-cts/outputs/.stamp.design.checkpoint
15-cadence-innovus-postcts_hold/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
15-cadence-innovus-postcts_hold/.stamp:
	$(call cpdir-and-parameterize,15-cadence-innovus-postcts_hold,../../mflowgen/steps/cadence-innovus-postcts_hold,15-cadence-innovus-postcts_hold/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 15-cadence-innovus-postcts_hold/.prebuilt)","")
15-cadence-innovus-postcts_hold/inputs/.stamp.adk: 15-cadence-innovus-postcts_hold/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,15-cadence-innovus-postcts_hold/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 15-cadence-innovus-postcts_hold/.prebuilt)","")
15-cadence-innovus-postcts_hold/inputs/.stamp.design.checkpoint: 15-cadence-innovus-postcts_hold/.stamp 14-cadence-innovus-cts/outputs/.stamp.design.checkpoint
	$(call symlink,15-cadence-innovus-postcts_hold/inputs,design.checkpoint,../../14-cadence-innovus-cts/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 15-cadence-innovus-postcts_hold/.prebuilt)","")
15-cadence-innovus-postcts_hold/inputs/.stamp.innovus-foundation-flow: 15-cadence-innovus-postcts_hold/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,15-cadence-innovus-postcts_hold/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 15-cadence-innovus-postcts_hold/.prebuilt)","")

define 15_cadence_innovus_postcts_hold_commands_rule
	mkdir -p 15-cadence-innovus-postcts_hold/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-postcts_hold && chmod +x .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-run && cp -f .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-run 15-cadence-innovus-postcts_hold && if [[ -e .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-debug ]]; then chmod +x .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-debug && cp -f .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-debug 15-cadence-innovus-postcts_hold; fi && if [[ -e .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-check-preconditions.py && cp -f .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-check-preconditions.py 15-cadence-innovus-postcts_hold; fi && if [[ -e .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-check-postconditions.py && cp -f .mflowgen/15-cadence-innovus-postcts_hold/mflowgen-check-postconditions.py 15-cadence-innovus-postcts_hold; fi && cd 15-cadence-innovus-postcts_hold && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 15-cadence-innovus-postcts_hold/outputs/*
endef

15-cadence-innovus-postcts_hold/.execstamp: 15-cadence-innovus-postcts_hold/.stamp 15-cadence-innovus-postcts_hold/inputs/.stamp.adk 15-cadence-innovus-postcts_hold/inputs/.stamp.design.checkpoint 15-cadence-innovus-postcts_hold/inputs/.stamp.innovus-foundation-flow
	$(call 15_cadence_innovus_postcts_hold_commands_rule)
	touch $@

15-cadence-innovus-postcts_hold/outputs/.execstamp.design.checkpoint: 15-cadence-innovus-postcts_hold/.execstamp
	touch $@

endif

# collect outputs

15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint: 15-cadence-innovus-postcts_hold/.execstamp 15-cadence-innovus-postcts_hold/outputs/.execstamp.design.checkpoint
	$(call stamp,15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint)

# post-conditions

define 15_cadence_innovus_postcts_hold_post_conditions_commands_rule
	cd 15-cadence-innovus-postcts_hold && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

15-cadence-innovus-postcts_hold/.postconditions.stamp: 15-cadence-innovus-postcts_hold/.execstamp 15-cadence-innovus-postcts_hold/outputs/.execstamp.design.checkpoint 15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint
	$(call 15_cadence_innovus_postcts_hold_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-postcts_hold

cadence-innovus-postcts_hold: 15-cadence-innovus-postcts_hold/.execstamp 15-cadence-innovus-postcts_hold/.postconditions.stamp 15-cadence-innovus-postcts_hold/outputs/.execstamp.design.checkpoint 15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint

.PHONY: 15

15: 15-cadence-innovus-postcts_hold/.execstamp 15-cadence-innovus-postcts_hold/.postconditions.stamp 15-cadence-innovus-postcts_hold/outputs/.execstamp.design.checkpoint 15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint

# debug

define 15_debug_rule
	cd 15-cadence-innovus-postcts_hold && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-postcts_hold: 
	$(call 15_debug_rule)

.PHONY: debug-15

debug-15: debug-cadence-innovus-postcts_hold

# ------------------------------------------------------------------------
# cadence-innovus-route
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 16-cadence-innovus-route/.prebuilt)","")
16-cadence-innovus-route/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
16-cadence-innovus-route/.stamp: 15-cadence-innovus-postcts_hold/.execstamp 15-cadence-innovus-postcts_hold/.postconditions.stamp 15-cadence-innovus-postcts_hold/outputs/.execstamp.design.checkpoint 15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint
16-cadence-innovus-route/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
16-cadence-innovus-route/.stamp:
	$(call cpdir-and-parameterize,16-cadence-innovus-route,../../mflowgen/steps/cadence-innovus-route,16-cadence-innovus-route/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 16-cadence-innovus-route/.prebuilt)","")
16-cadence-innovus-route/inputs/.stamp.adk: 16-cadence-innovus-route/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,16-cadence-innovus-route/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 16-cadence-innovus-route/.prebuilt)","")
16-cadence-innovus-route/inputs/.stamp.design.checkpoint: 16-cadence-innovus-route/.stamp 15-cadence-innovus-postcts_hold/outputs/.stamp.design.checkpoint
	$(call symlink,16-cadence-innovus-route/inputs,design.checkpoint,../../15-cadence-innovus-postcts_hold/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 16-cadence-innovus-route/.prebuilt)","")
16-cadence-innovus-route/inputs/.stamp.innovus-foundation-flow: 16-cadence-innovus-route/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,16-cadence-innovus-route/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 16-cadence-innovus-route/.prebuilt)","")

define 16_cadence_innovus_route_commands_rule
	mkdir -p 16-cadence-innovus-route/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-route && chmod +x .mflowgen/16-cadence-innovus-route/mflowgen-run && cp -f .mflowgen/16-cadence-innovus-route/mflowgen-run 16-cadence-innovus-route && if [[ -e .mflowgen/16-cadence-innovus-route/mflowgen-debug ]]; then chmod +x .mflowgen/16-cadence-innovus-route/mflowgen-debug && cp -f .mflowgen/16-cadence-innovus-route/mflowgen-debug 16-cadence-innovus-route; fi && if [[ -e .mflowgen/16-cadence-innovus-route/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/16-cadence-innovus-route/mflowgen-check-preconditions.py && cp -f .mflowgen/16-cadence-innovus-route/mflowgen-check-preconditions.py 16-cadence-innovus-route; fi && if [[ -e .mflowgen/16-cadence-innovus-route/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/16-cadence-innovus-route/mflowgen-check-postconditions.py && cp -f .mflowgen/16-cadence-innovus-route/mflowgen-check-postconditions.py 16-cadence-innovus-route; fi && cd 16-cadence-innovus-route && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 16-cadence-innovus-route/outputs/*
endef

16-cadence-innovus-route/.execstamp: 16-cadence-innovus-route/.stamp 16-cadence-innovus-route/inputs/.stamp.adk 16-cadence-innovus-route/inputs/.stamp.design.checkpoint 16-cadence-innovus-route/inputs/.stamp.innovus-foundation-flow
	$(call 16_cadence_innovus_route_commands_rule)
	touch $@

16-cadence-innovus-route/outputs/.execstamp.design.checkpoint: 16-cadence-innovus-route/.execstamp
	touch $@

endif

# collect outputs

16-cadence-innovus-route/outputs/.stamp.design.checkpoint: 16-cadence-innovus-route/.execstamp 16-cadence-innovus-route/outputs/.execstamp.design.checkpoint
	$(call stamp,16-cadence-innovus-route/outputs/.stamp.design.checkpoint)

# post-conditions

define 16_cadence_innovus_route_post_conditions_commands_rule
	cd 16-cadence-innovus-route && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

16-cadence-innovus-route/.postconditions.stamp: 16-cadence-innovus-route/.execstamp 16-cadence-innovus-route/outputs/.execstamp.design.checkpoint 16-cadence-innovus-route/outputs/.stamp.design.checkpoint
	$(call 16_cadence_innovus_route_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-route

cadence-innovus-route: 16-cadence-innovus-route/.execstamp 16-cadence-innovus-route/.postconditions.stamp 16-cadence-innovus-route/outputs/.execstamp.design.checkpoint 16-cadence-innovus-route/outputs/.stamp.design.checkpoint

.PHONY: 16

16: 16-cadence-innovus-route/.execstamp 16-cadence-innovus-route/.postconditions.stamp 16-cadence-innovus-route/outputs/.execstamp.design.checkpoint 16-cadence-innovus-route/outputs/.stamp.design.checkpoint

# debug

define 16_debug_rule
	cd 16-cadence-innovus-route && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-route: 
	$(call 16_debug_rule)

.PHONY: debug-16

debug-16: debug-cadence-innovus-route

# ------------------------------------------------------------------------
# cadence-innovus-postroute
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 17-cadence-innovus-postroute/.prebuilt)","")
17-cadence-innovus-postroute/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
17-cadence-innovus-postroute/.stamp: 16-cadence-innovus-route/.execstamp 16-cadence-innovus-route/.postconditions.stamp 16-cadence-innovus-route/outputs/.execstamp.design.checkpoint 16-cadence-innovus-route/outputs/.stamp.design.checkpoint
17-cadence-innovus-postroute/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
17-cadence-innovus-postroute/.stamp:
	$(call cpdir-and-parameterize,17-cadence-innovus-postroute,../../mflowgen/steps/cadence-innovus-postroute,17-cadence-innovus-postroute/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 17-cadence-innovus-postroute/.prebuilt)","")
17-cadence-innovus-postroute/inputs/.stamp.adk: 17-cadence-innovus-postroute/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,17-cadence-innovus-postroute/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 17-cadence-innovus-postroute/.prebuilt)","")
17-cadence-innovus-postroute/inputs/.stamp.design.checkpoint: 17-cadence-innovus-postroute/.stamp 16-cadence-innovus-route/outputs/.stamp.design.checkpoint
	$(call symlink,17-cadence-innovus-postroute/inputs,design.checkpoint,../../16-cadence-innovus-route/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 17-cadence-innovus-postroute/.prebuilt)","")
17-cadence-innovus-postroute/inputs/.stamp.innovus-foundation-flow: 17-cadence-innovus-postroute/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,17-cadence-innovus-postroute/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 17-cadence-innovus-postroute/.prebuilt)","")

define 17_cadence_innovus_postroute_commands_rule
	mkdir -p 17-cadence-innovus-postroute/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-postroute && chmod +x .mflowgen/17-cadence-innovus-postroute/mflowgen-run && cp -f .mflowgen/17-cadence-innovus-postroute/mflowgen-run 17-cadence-innovus-postroute && if [[ -e .mflowgen/17-cadence-innovus-postroute/mflowgen-debug ]]; then chmod +x .mflowgen/17-cadence-innovus-postroute/mflowgen-debug && cp -f .mflowgen/17-cadence-innovus-postroute/mflowgen-debug 17-cadence-innovus-postroute; fi && if [[ -e .mflowgen/17-cadence-innovus-postroute/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/17-cadence-innovus-postroute/mflowgen-check-preconditions.py && cp -f .mflowgen/17-cadence-innovus-postroute/mflowgen-check-preconditions.py 17-cadence-innovus-postroute; fi && if [[ -e .mflowgen/17-cadence-innovus-postroute/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/17-cadence-innovus-postroute/mflowgen-check-postconditions.py && cp -f .mflowgen/17-cadence-innovus-postroute/mflowgen-check-postconditions.py 17-cadence-innovus-postroute; fi && cd 17-cadence-innovus-postroute && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 17-cadence-innovus-postroute/outputs/*
endef

17-cadence-innovus-postroute/.execstamp: 17-cadence-innovus-postroute/.stamp 17-cadence-innovus-postroute/inputs/.stamp.adk 17-cadence-innovus-postroute/inputs/.stamp.design.checkpoint 17-cadence-innovus-postroute/inputs/.stamp.innovus-foundation-flow
	$(call 17_cadence_innovus_postroute_commands_rule)
	touch $@

17-cadence-innovus-postroute/outputs/.execstamp.design.checkpoint: 17-cadence-innovus-postroute/.execstamp
	touch $@

endif

# collect outputs

17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint: 17-cadence-innovus-postroute/.execstamp 17-cadence-innovus-postroute/outputs/.execstamp.design.checkpoint
	$(call stamp,17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint)

# post-conditions

define 17_cadence_innovus_postroute_post_conditions_commands_rule
	cd 17-cadence-innovus-postroute && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

17-cadence-innovus-postroute/.postconditions.stamp: 17-cadence-innovus-postroute/.execstamp 17-cadence-innovus-postroute/outputs/.execstamp.design.checkpoint 17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint
	$(call 17_cadence_innovus_postroute_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-postroute

cadence-innovus-postroute: 17-cadence-innovus-postroute/.execstamp 17-cadence-innovus-postroute/.postconditions.stamp 17-cadence-innovus-postroute/outputs/.execstamp.design.checkpoint 17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint

.PHONY: 17

17: 17-cadence-innovus-postroute/.execstamp 17-cadence-innovus-postroute/.postconditions.stamp 17-cadence-innovus-postroute/outputs/.execstamp.design.checkpoint 17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint

# debug

define 17_debug_rule
	cd 17-cadence-innovus-postroute && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-postroute: 
	$(call 17_debug_rule)

.PHONY: debug-17

debug-17: debug-cadence-innovus-postroute

# ------------------------------------------------------------------------
# cadence-innovus-postroute_hold
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 18-cadence-innovus-postroute_hold/.prebuilt)","")
18-cadence-innovus-postroute_hold/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
18-cadence-innovus-postroute_hold/.stamp: 17-cadence-innovus-postroute/.execstamp 17-cadence-innovus-postroute/.postconditions.stamp 17-cadence-innovus-postroute/outputs/.execstamp.design.checkpoint 17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint
18-cadence-innovus-postroute_hold/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
18-cadence-innovus-postroute_hold/.stamp:
	$(call cpdir-and-parameterize,18-cadence-innovus-postroute_hold,../../mflowgen/steps/cadence-innovus-postroute_hold,18-cadence-innovus-postroute_hold/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 18-cadence-innovus-postroute_hold/.prebuilt)","")
18-cadence-innovus-postroute_hold/inputs/.stamp.adk: 18-cadence-innovus-postroute_hold/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,18-cadence-innovus-postroute_hold/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 18-cadence-innovus-postroute_hold/.prebuilt)","")
18-cadence-innovus-postroute_hold/inputs/.stamp.design.checkpoint: 18-cadence-innovus-postroute_hold/.stamp 17-cadence-innovus-postroute/outputs/.stamp.design.checkpoint
	$(call symlink,18-cadence-innovus-postroute_hold/inputs,design.checkpoint,../../17-cadence-innovus-postroute/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 18-cadence-innovus-postroute_hold/.prebuilt)","")
18-cadence-innovus-postroute_hold/inputs/.stamp.innovus-foundation-flow: 18-cadence-innovus-postroute_hold/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,18-cadence-innovus-postroute_hold/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 18-cadence-innovus-postroute_hold/.prebuilt)","")

define 18_cadence_innovus_postroute_hold_commands_rule
	mkdir -p 18-cadence-innovus-postroute_hold/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-postroute_hold && chmod +x .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-run && cp -f .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-run 18-cadence-innovus-postroute_hold && if [[ -e .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-debug ]]; then chmod +x .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-debug && cp -f .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-debug 18-cadence-innovus-postroute_hold; fi && if [[ -e .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-check-preconditions.py && cp -f .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-check-preconditions.py 18-cadence-innovus-postroute_hold; fi && if [[ -e .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-check-postconditions.py && cp -f .mflowgen/18-cadence-innovus-postroute_hold/mflowgen-check-postconditions.py 18-cadence-innovus-postroute_hold; fi && cd 18-cadence-innovus-postroute_hold && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 18-cadence-innovus-postroute_hold/outputs/*
endef

18-cadence-innovus-postroute_hold/.execstamp: 18-cadence-innovus-postroute_hold/.stamp 18-cadence-innovus-postroute_hold/inputs/.stamp.adk 18-cadence-innovus-postroute_hold/inputs/.stamp.design.checkpoint 18-cadence-innovus-postroute_hold/inputs/.stamp.innovus-foundation-flow
	$(call 18_cadence_innovus_postroute_hold_commands_rule)
	touch $@

18-cadence-innovus-postroute_hold/outputs/.execstamp.design.checkpoint: 18-cadence-innovus-postroute_hold/.execstamp
	touch $@

endif

# collect outputs

18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint: 18-cadence-innovus-postroute_hold/.execstamp 18-cadence-innovus-postroute_hold/outputs/.execstamp.design.checkpoint
	$(call stamp,18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint)

# post-conditions

define 18_cadence_innovus_postroute_hold_post_conditions_commands_rule
	cd 18-cadence-innovus-postroute_hold && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

18-cadence-innovus-postroute_hold/.postconditions.stamp: 18-cadence-innovus-postroute_hold/.execstamp 18-cadence-innovus-postroute_hold/outputs/.execstamp.design.checkpoint 18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint
	$(call 18_cadence_innovus_postroute_hold_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-postroute_hold

cadence-innovus-postroute_hold: 18-cadence-innovus-postroute_hold/.execstamp 18-cadence-innovus-postroute_hold/.postconditions.stamp 18-cadence-innovus-postroute_hold/outputs/.execstamp.design.checkpoint 18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint

.PHONY: 18

18: 18-cadence-innovus-postroute_hold/.execstamp 18-cadence-innovus-postroute_hold/.postconditions.stamp 18-cadence-innovus-postroute_hold/outputs/.execstamp.design.checkpoint 18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint

# debug

define 18_debug_rule
	cd 18-cadence-innovus-postroute_hold && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-postroute_hold: 
	$(call 18_debug_rule)

.PHONY: debug-18

debug-18: debug-cadence-innovus-postroute_hold

# ------------------------------------------------------------------------
# cadence-innovus-signoff
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 19-cadence-innovus-signoff/.prebuilt)","")
19-cadence-innovus-signoff/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
19-cadence-innovus-signoff/.stamp: 18-cadence-innovus-postroute_hold/.execstamp 18-cadence-innovus-postroute_hold/.postconditions.stamp 18-cadence-innovus-postroute_hold/outputs/.execstamp.design.checkpoint 18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint
19-cadence-innovus-signoff/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
19-cadence-innovus-signoff/.stamp:
	$(call cpdir-and-parameterize,19-cadence-innovus-signoff,../design/cadence-innovus-signoff,19-cadence-innovus-signoff/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 19-cadence-innovus-signoff/.prebuilt)","")
19-cadence-innovus-signoff/inputs/.stamp.adk: 19-cadence-innovus-signoff/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,19-cadence-innovus-signoff/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 19-cadence-innovus-signoff/.prebuilt)","")
19-cadence-innovus-signoff/inputs/.stamp.design.checkpoint: 19-cadence-innovus-signoff/.stamp 18-cadence-innovus-postroute_hold/outputs/.stamp.design.checkpoint
	$(call symlink,19-cadence-innovus-signoff/inputs,design.checkpoint,../../18-cadence-innovus-postroute_hold/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 19-cadence-innovus-signoff/.prebuilt)","")
19-cadence-innovus-signoff/inputs/.stamp.innovus-foundation-flow: 19-cadence-innovus-signoff/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,19-cadence-innovus-signoff/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 19-cadence-innovus-signoff/.prebuilt)","")

define 19_cadence_innovus_signoff_commands_rule
	mkdir -p 19-cadence-innovus-signoff/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-signoff && chmod +x .mflowgen/19-cadence-innovus-signoff/mflowgen-run && cp -f .mflowgen/19-cadence-innovus-signoff/mflowgen-run 19-cadence-innovus-signoff && if [[ -e .mflowgen/19-cadence-innovus-signoff/mflowgen-debug ]]; then chmod +x .mflowgen/19-cadence-innovus-signoff/mflowgen-debug && cp -f .mflowgen/19-cadence-innovus-signoff/mflowgen-debug 19-cadence-innovus-signoff; fi && if [[ -e .mflowgen/19-cadence-innovus-signoff/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/19-cadence-innovus-signoff/mflowgen-check-preconditions.py && cp -f .mflowgen/19-cadence-innovus-signoff/mflowgen-check-preconditions.py 19-cadence-innovus-signoff; fi && if [[ -e .mflowgen/19-cadence-innovus-signoff/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/19-cadence-innovus-signoff/mflowgen-check-postconditions.py && cp -f .mflowgen/19-cadence-innovus-signoff/mflowgen-check-postconditions.py 19-cadence-innovus-signoff; fi && cd 19-cadence-innovus-signoff && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 19-cadence-innovus-signoff/outputs/*
endef

19-cadence-innovus-signoff/.execstamp: 19-cadence-innovus-signoff/.stamp 19-cadence-innovus-signoff/inputs/.stamp.adk 19-cadence-innovus-signoff/inputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/inputs/.stamp.innovus-foundation-flow
	$(call 19_cadence_innovus_signoff_commands_rule)
	touch $@

19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.lef: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.sdf: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz: 19-cadence-innovus-signoff/.execstamp
	touch $@
19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz: 19-cadence-innovus-signoff/.execstamp
	touch $@

endif

# collect outputs

19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint)

19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz)

19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds)

19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v)

19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v)

19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v)

19-cadence-innovus-signoff/outputs/.stamp.design.lef: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.lef)

19-cadence-innovus-signoff/outputs/.stamp.design.def.gz: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.def.gz)

19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc)

19-cadence-innovus-signoff/outputs/.stamp.design.sdf: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.sdf)

19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v)

19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz)

19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v
	$(call stamp,19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz)

# post-conditions

define 19_cadence_innovus_signoff_post_conditions_commands_rule
	cd 19-cadence-innovus-signoff && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

19-cadence-innovus-signoff/.postconditions.stamp: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v
	$(call 19_cadence_innovus_signoff_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-signoff

cadence-innovus-signoff: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/.postconditions.stamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v

.PHONY: 19

19: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/.postconditions.stamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v

# debug

define 19_debug_rule
	cd 19-cadence-innovus-signoff && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-signoff: 
	$(call 19_debug_rule)

.PHONY: debug-19

debug-19: debug-cadence-innovus-signoff

# ------------------------------------------------------------------------
# mentor-calibre-gdsmerge
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 20-mentor-calibre-gdsmerge/.prebuilt)","")
20-mentor-calibre-gdsmerge/.stamp: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/.postconditions.stamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v
20-mentor-calibre-gdsmerge/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
20-mentor-calibre-gdsmerge/.stamp:
	$(call cpdir-and-parameterize,20-mentor-calibre-gdsmerge,../../mflowgen/steps/mentor-calibre-gdsmerge,20-mentor-calibre-gdsmerge/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 20-mentor-calibre-gdsmerge/.prebuilt)","")
20-mentor-calibre-gdsmerge/inputs/.stamp.adk: 20-mentor-calibre-gdsmerge/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,20-mentor-calibre-gdsmerge/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 20-mentor-calibre-gdsmerge/.prebuilt)","")
20-mentor-calibre-gdsmerge/inputs/.stamp.design.gds.gz: 20-mentor-calibre-gdsmerge/.stamp 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz
	$(call symlink,20-mentor-calibre-gdsmerge/inputs,design.gds.gz,../../19-cadence-innovus-signoff/outputs/design.gds.gz,.stamp.design.gds.gz)
endif

# execute

ifeq ("$(wildcard 20-mentor-calibre-gdsmerge/.prebuilt)","")

define 20_mentor_calibre_gdsmerge_commands_rule
	mkdir -p 20-mentor-calibre-gdsmerge/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t mentor-calibre-gdsmerge && chmod +x .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-run && cp -f .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-run 20-mentor-calibre-gdsmerge && if [[ -e .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-debug ]]; then chmod +x .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-debug && cp -f .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-debug 20-mentor-calibre-gdsmerge; fi && if [[ -e .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-check-preconditions.py && cp -f .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-check-preconditions.py 20-mentor-calibre-gdsmerge; fi && if [[ -e .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-check-postconditions.py && cp -f .mflowgen/20-mentor-calibre-gdsmerge/mflowgen-check-postconditions.py 20-mentor-calibre-gdsmerge; fi && cd 20-mentor-calibre-gdsmerge && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 20-mentor-calibre-gdsmerge/outputs/*
endef

20-mentor-calibre-gdsmerge/.execstamp: 20-mentor-calibre-gdsmerge/.stamp 20-mentor-calibre-gdsmerge/inputs/.stamp.adk 20-mentor-calibre-gdsmerge/inputs/.stamp.design.gds.gz
	$(call 20_mentor_calibre_gdsmerge_commands_rule)
	touch $@

20-mentor-calibre-gdsmerge/outputs/.execstamp.design_merged.gds: 20-mentor-calibre-gdsmerge/.execstamp
	touch $@

endif

# collect outputs

20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds: 20-mentor-calibre-gdsmerge/.execstamp 20-mentor-calibre-gdsmerge/outputs/.execstamp.design_merged.gds
	$(call stamp,20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds)

# post-conditions

define 20_mentor_calibre_gdsmerge_post_conditions_commands_rule
	cd 20-mentor-calibre-gdsmerge && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

20-mentor-calibre-gdsmerge/.postconditions.stamp: 20-mentor-calibre-gdsmerge/.execstamp 20-mentor-calibre-gdsmerge/outputs/.execstamp.design_merged.gds 20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds
	$(call 20_mentor_calibre_gdsmerge_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: mentor-calibre-gdsmerge

mentor-calibre-gdsmerge: 20-mentor-calibre-gdsmerge/.execstamp 20-mentor-calibre-gdsmerge/.postconditions.stamp 20-mentor-calibre-gdsmerge/outputs/.execstamp.design_merged.gds 20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds

.PHONY: 20

20: 20-mentor-calibre-gdsmerge/.execstamp 20-mentor-calibre-gdsmerge/.postconditions.stamp 20-mentor-calibre-gdsmerge/outputs/.execstamp.design_merged.gds 20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds

# debug

define 20_debug_rule
	cd 20-mentor-calibre-gdsmerge && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-mentor-calibre-gdsmerge: 
	$(call 20_debug_rule)

.PHONY: debug-20

debug-20: debug-mentor-calibre-gdsmerge

# ------------------------------------------------------------------------
# open-magic-def2spice
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 21-open-magic-def2spice/.prebuilt)","")
21-open-magic-def2spice/.stamp: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/.postconditions.stamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v
21-open-magic-def2spice/.stamp: 3-macros/.execstamp 3-macros/.postconditions.stamp 3-macros/outputs/.execstamp.dffram 3-macros/outputs/.stamp.dffram
21-open-magic-def2spice/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
21-open-magic-def2spice/.stamp:
	$(call cpdir-and-parameterize,21-open-magic-def2spice,../design/open-magic-def2spice,21-open-magic-def2spice/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 21-open-magic-def2spice/.prebuilt)","")
21-open-magic-def2spice/inputs/.stamp.adk: 21-open-magic-def2spice/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,21-open-magic-def2spice/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 21-open-magic-def2spice/.prebuilt)","")
21-open-magic-def2spice/inputs/.stamp.design.def.gz: 21-open-magic-def2spice/.stamp 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz
	$(call symlink,21-open-magic-def2spice/inputs,design.def.gz,../../19-cadence-innovus-signoff/outputs/design.def.gz,.stamp.design.def.gz)
endif

ifeq ("$(wildcard 21-open-magic-def2spice/.prebuilt)","")
21-open-magic-def2spice/inputs/.stamp.dffram: 21-open-magic-def2spice/.stamp 3-macros/outputs/.stamp.dffram
	$(call symlink,21-open-magic-def2spice/inputs,dffram,../../3-macros/outputs/dffram,.stamp.dffram)
endif

# execute

ifeq ("$(wildcard 21-open-magic-def2spice/.prebuilt)","")

define 21_open_magic_def2spice_commands_rule
	mkdir -p 21-open-magic-def2spice/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-magic-def2spice && chmod +x .mflowgen/21-open-magic-def2spice/mflowgen-run && cp -f .mflowgen/21-open-magic-def2spice/mflowgen-run 21-open-magic-def2spice && if [[ -e .mflowgen/21-open-magic-def2spice/mflowgen-debug ]]; then chmod +x .mflowgen/21-open-magic-def2spice/mflowgen-debug && cp -f .mflowgen/21-open-magic-def2spice/mflowgen-debug 21-open-magic-def2spice; fi && if [[ -e .mflowgen/21-open-magic-def2spice/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/21-open-magic-def2spice/mflowgen-check-preconditions.py && cp -f .mflowgen/21-open-magic-def2spice/mflowgen-check-preconditions.py 21-open-magic-def2spice; fi && if [[ -e .mflowgen/21-open-magic-def2spice/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/21-open-magic-def2spice/mflowgen-check-postconditions.py && cp -f .mflowgen/21-open-magic-def2spice/mflowgen-check-postconditions.py 21-open-magic-def2spice; fi && cd 21-open-magic-def2spice && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 21-open-magic-def2spice/outputs/*
endef

21-open-magic-def2spice/.execstamp: 21-open-magic-def2spice/.stamp 21-open-magic-def2spice/inputs/.stamp.adk 21-open-magic-def2spice/inputs/.stamp.design.def.gz 21-open-magic-def2spice/inputs/.stamp.dffram
	$(call 21_open_magic_def2spice_commands_rule)
	touch $@

21-open-magic-def2spice/outputs/.execstamp.design_extracted.spice: 21-open-magic-def2spice/.execstamp
	touch $@

endif

# collect outputs

21-open-magic-def2spice/outputs/.stamp.design_extracted.spice: 21-open-magic-def2spice/.execstamp 21-open-magic-def2spice/outputs/.execstamp.design_extracted.spice
	$(call stamp,21-open-magic-def2spice/outputs/.stamp.design_extracted.spice)

# post-conditions

define 21_open_magic_def2spice_post_conditions_commands_rule
	cd 21-open-magic-def2spice && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

21-open-magic-def2spice/.postconditions.stamp: 21-open-magic-def2spice/.execstamp 21-open-magic-def2spice/outputs/.execstamp.design_extracted.spice 21-open-magic-def2spice/outputs/.stamp.design_extracted.spice
	$(call 21_open_magic_def2spice_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: open-magic-def2spice

open-magic-def2spice: 21-open-magic-def2spice/.execstamp 21-open-magic-def2spice/.postconditions.stamp 21-open-magic-def2spice/outputs/.execstamp.design_extracted.spice 21-open-magic-def2spice/outputs/.stamp.design_extracted.spice

.PHONY: 21

21: 21-open-magic-def2spice/.execstamp 21-open-magic-def2spice/.postconditions.stamp 21-open-magic-def2spice/outputs/.execstamp.design_extracted.spice 21-open-magic-def2spice/outputs/.stamp.design_extracted.spice

# debug

# ------------------------------------------------------------------------
# synopsys-pt-eco
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 22-synopsys-pt-eco/.prebuilt)","")
22-synopsys-pt-eco/.stamp: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/.postconditions.stamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v
22-synopsys-pt-eco/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
22-synopsys-pt-eco/.stamp:
	$(call cpdir-and-parameterize,22-synopsys-pt-eco,../../mflowgen/steps/synopsys-pt-eco,22-synopsys-pt-eco/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 22-synopsys-pt-eco/.prebuilt)","")
22-synopsys-pt-eco/inputs/.stamp.adk: 22-synopsys-pt-eco/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,22-synopsys-pt-eco/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 22-synopsys-pt-eco/.prebuilt)","")
22-synopsys-pt-eco/inputs/.stamp.design.pt.sdc: 22-synopsys-pt-eco/.stamp 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc
	$(call symlink,22-synopsys-pt-eco/inputs,design.pt.sdc,../../19-cadence-innovus-signoff/outputs/design.pt.sdc,.stamp.design.pt.sdc)
endif

ifeq ("$(wildcard 22-synopsys-pt-eco/.prebuilt)","")
22-synopsys-pt-eco/inputs/.stamp.design.spef.gz: 22-synopsys-pt-eco/.stamp 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz
	$(call symlink,22-synopsys-pt-eco/inputs,design.spef.gz,../../19-cadence-innovus-signoff/outputs/design.spef.gz,.stamp.design.spef.gz)
endif

ifeq ("$(wildcard 22-synopsys-pt-eco/.prebuilt)","")
22-synopsys-pt-eco/inputs/.stamp.design.vcs.v: 22-synopsys-pt-eco/.stamp 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v
	$(call symlink,22-synopsys-pt-eco/inputs,design.vcs.v,../../19-cadence-innovus-signoff/outputs/design.vcs.v,.stamp.design.vcs.v)
endif

# execute

ifeq ("$(wildcard 22-synopsys-pt-eco/.prebuilt)","")

define 22_synopsys_pt_eco_commands_rule
	mkdir -p 22-synopsys-pt-eco/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t synopsys-pt-eco && chmod +x .mflowgen/22-synopsys-pt-eco/mflowgen-run && cp -f .mflowgen/22-synopsys-pt-eco/mflowgen-run 22-synopsys-pt-eco && if [[ -e .mflowgen/22-synopsys-pt-eco/mflowgen-debug ]]; then chmod +x .mflowgen/22-synopsys-pt-eco/mflowgen-debug && cp -f .mflowgen/22-synopsys-pt-eco/mflowgen-debug 22-synopsys-pt-eco; fi && if [[ -e .mflowgen/22-synopsys-pt-eco/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/22-synopsys-pt-eco/mflowgen-check-preconditions.py && cp -f .mflowgen/22-synopsys-pt-eco/mflowgen-check-preconditions.py 22-synopsys-pt-eco; fi && if [[ -e .mflowgen/22-synopsys-pt-eco/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/22-synopsys-pt-eco/mflowgen-check-postconditions.py && cp -f .mflowgen/22-synopsys-pt-eco/mflowgen-check-postconditions.py 22-synopsys-pt-eco; fi && cd 22-synopsys-pt-eco && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 22-synopsys-pt-eco/outputs/*
endef

22-synopsys-pt-eco/.execstamp: 22-synopsys-pt-eco/.stamp 22-synopsys-pt-eco/inputs/.stamp.adk 22-synopsys-pt-eco/inputs/.stamp.design.pt.sdc 22-synopsys-pt-eco/inputs/.stamp.design.spef.gz 22-synopsys-pt-eco/inputs/.stamp.design.vcs.v
	$(call 22_synopsys_pt_eco_commands_rule)
	touch $@

22-synopsys-pt-eco/outputs/.execstamp.icc_eco.tcl: 22-synopsys-pt-eco/.execstamp
	touch $@

endif

# collect outputs

22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl: 22-synopsys-pt-eco/.execstamp 22-synopsys-pt-eco/outputs/.execstamp.icc_eco.tcl
	$(call stamp,22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl)

# post-conditions

define 22_synopsys_pt_eco_post_conditions_commands_rule
	cd 22-synopsys-pt-eco && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

22-synopsys-pt-eco/.postconditions.stamp: 22-synopsys-pt-eco/.execstamp 22-synopsys-pt-eco/outputs/.execstamp.icc_eco.tcl 22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl
	$(call 22_synopsys_pt_eco_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: synopsys-pt-eco

synopsys-pt-eco: 22-synopsys-pt-eco/.execstamp 22-synopsys-pt-eco/.postconditions.stamp 22-synopsys-pt-eco/outputs/.execstamp.icc_eco.tcl 22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl

.PHONY: 22

22: 22-synopsys-pt-eco/.execstamp 22-synopsys-pt-eco/.postconditions.stamp 22-synopsys-pt-eco/outputs/.execstamp.icc_eco.tcl 22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl

# debug

# ------------------------------------------------------------------------
# open-icc2innovus
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 23-open-icc2innovus/.prebuilt)","")
23-open-icc2innovus/.stamp: 22-synopsys-pt-eco/.execstamp 22-synopsys-pt-eco/.postconditions.stamp 22-synopsys-pt-eco/outputs/.execstamp.icc_eco.tcl 22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl
23-open-icc2innovus/.stamp:
	$(call cpdir-and-parameterize,23-open-icc2innovus,../../mflowgen/steps/open-icc2innovus,23-open-icc2innovus/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 23-open-icc2innovus/.prebuilt)","")
23-open-icc2innovus/inputs/.stamp.icc_eco.tcl: 23-open-icc2innovus/.stamp 22-synopsys-pt-eco/outputs/.stamp.icc_eco.tcl
	$(call symlink,23-open-icc2innovus/inputs,icc_eco.tcl,../../22-synopsys-pt-eco/outputs/icc_eco.tcl,.stamp.icc_eco.tcl)
endif

# execute

ifeq ("$(wildcard 23-open-icc2innovus/.prebuilt)","")

define 23_open_icc2innovus_commands_rule
	mkdir -p 23-open-icc2innovus/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-icc2innovus && chmod +x .mflowgen/23-open-icc2innovus/mflowgen-run && cp -f .mflowgen/23-open-icc2innovus/mflowgen-run 23-open-icc2innovus && if [[ -e .mflowgen/23-open-icc2innovus/mflowgen-debug ]]; then chmod +x .mflowgen/23-open-icc2innovus/mflowgen-debug && cp -f .mflowgen/23-open-icc2innovus/mflowgen-debug 23-open-icc2innovus; fi && if [[ -e .mflowgen/23-open-icc2innovus/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/23-open-icc2innovus/mflowgen-check-preconditions.py && cp -f .mflowgen/23-open-icc2innovus/mflowgen-check-preconditions.py 23-open-icc2innovus; fi && if [[ -e .mflowgen/23-open-icc2innovus/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/23-open-icc2innovus/mflowgen-check-postconditions.py && cp -f .mflowgen/23-open-icc2innovus/mflowgen-check-postconditions.py 23-open-icc2innovus; fi && cd 23-open-icc2innovus && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 23-open-icc2innovus/outputs/*
endef

23-open-icc2innovus/.execstamp: 23-open-icc2innovus/.stamp 23-open-icc2innovus/inputs/.stamp.icc_eco.tcl
	$(call 23_open_icc2innovus_commands_rule)
	touch $@

23-open-icc2innovus/outputs/.execstamp.innovus_eco.tcl: 23-open-icc2innovus/.execstamp
	touch $@

endif

# collect outputs

23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl: 23-open-icc2innovus/.execstamp 23-open-icc2innovus/outputs/.execstamp.innovus_eco.tcl
	$(call stamp,23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl)

# post-conditions

define 23_open_icc2innovus_post_conditions_commands_rule
	cd 23-open-icc2innovus && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

23-open-icc2innovus/.postconditions.stamp: 23-open-icc2innovus/.execstamp 23-open-icc2innovus/outputs/.execstamp.innovus_eco.tcl 23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl
	$(call 23_open_icc2innovus_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: open-icc2innovus

open-icc2innovus: 23-open-icc2innovus/.execstamp 23-open-icc2innovus/.postconditions.stamp 23-open-icc2innovus/outputs/.execstamp.innovus_eco.tcl 23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl

.PHONY: 23

23: 23-open-icc2innovus/.execstamp 23-open-icc2innovus/.postconditions.stamp 23-open-icc2innovus/outputs/.execstamp.innovus_eco.tcl 23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl

# debug

# ------------------------------------------------------------------------
# open-magic-drc
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 24-open-magic-drc/.prebuilt)","")
24-open-magic-drc/.stamp: 20-mentor-calibre-gdsmerge/.execstamp 20-mentor-calibre-gdsmerge/.postconditions.stamp 20-mentor-calibre-gdsmerge/outputs/.execstamp.design_merged.gds 20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds
24-open-magic-drc/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
24-open-magic-drc/.stamp:
	$(call cpdir-and-parameterize,24-open-magic-drc,../design/open-magic-drc,24-open-magic-drc/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 24-open-magic-drc/.prebuilt)","")
24-open-magic-drc/inputs/.stamp.adk: 24-open-magic-drc/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,24-open-magic-drc/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 24-open-magic-drc/.prebuilt)","")
24-open-magic-drc/inputs/.stamp.design_merged.gds: 24-open-magic-drc/.stamp 20-mentor-calibre-gdsmerge/outputs/.stamp.design_merged.gds
	$(call symlink,24-open-magic-drc/inputs,design_merged.gds,../../20-mentor-calibre-gdsmerge/outputs/design_merged.gds,.stamp.design_merged.gds)
endif

# execute

ifeq ("$(wildcard 24-open-magic-drc/.prebuilt)","")

define 24_open_magic_drc_commands_rule
	mkdir -p 24-open-magic-drc/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-magic-drc && chmod +x .mflowgen/24-open-magic-drc/mflowgen-run && cp -f .mflowgen/24-open-magic-drc/mflowgen-run 24-open-magic-drc && if [[ -e .mflowgen/24-open-magic-drc/mflowgen-debug ]]; then chmod +x .mflowgen/24-open-magic-drc/mflowgen-debug && cp -f .mflowgen/24-open-magic-drc/mflowgen-debug 24-open-magic-drc; fi && if [[ -e .mflowgen/24-open-magic-drc/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/24-open-magic-drc/mflowgen-check-preconditions.py && cp -f .mflowgen/24-open-magic-drc/mflowgen-check-preconditions.py 24-open-magic-drc; fi && if [[ -e .mflowgen/24-open-magic-drc/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/24-open-magic-drc/mflowgen-check-postconditions.py && cp -f .mflowgen/24-open-magic-drc/mflowgen-check-postconditions.py 24-open-magic-drc; fi && cd 24-open-magic-drc && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 24-open-magic-drc/outputs/*
endef

24-open-magic-drc/.execstamp: 24-open-magic-drc/.stamp 24-open-magic-drc/inputs/.stamp.adk 24-open-magic-drc/inputs/.stamp.design_merged.gds
	$(call 24_open_magic_drc_commands_rule)
	touch $@

24-open-magic-drc/outputs/.execstamp.drc_results.log: 24-open-magic-drc/.execstamp
	touch $@

endif

# collect outputs

24-open-magic-drc/outputs/.stamp.drc_results.log: 24-open-magic-drc/.execstamp 24-open-magic-drc/outputs/.execstamp.drc_results.log
	$(call stamp,24-open-magic-drc/outputs/.stamp.drc_results.log)

# post-conditions

define 24_open_magic_drc_post_conditions_commands_rule
	cd 24-open-magic-drc && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

24-open-magic-drc/.postconditions.stamp: 24-open-magic-drc/.execstamp 24-open-magic-drc/outputs/.execstamp.drc_results.log 24-open-magic-drc/outputs/.stamp.drc_results.log
	$(call 24_open_magic_drc_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: open-magic-drc

open-magic-drc: 24-open-magic-drc/.execstamp 24-open-magic-drc/.postconditions.stamp 24-open-magic-drc/outputs/.execstamp.drc_results.log 24-open-magic-drc/outputs/.stamp.drc_results.log

.PHONY: 24

24: 24-open-magic-drc/.execstamp 24-open-magic-drc/.postconditions.stamp 24-open-magic-drc/outputs/.execstamp.drc_results.log 24-open-magic-drc/outputs/.stamp.drc_results.log

# debug

# ------------------------------------------------------------------------
# open-netgen-lvs
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 25-open-netgen-lvs/.prebuilt)","")
25-open-netgen-lvs/.stamp: 19-cadence-innovus-signoff/.execstamp 19-cadence-innovus-signoff/.postconditions.stamp 19-cadence-innovus-signoff/outputs/.execstamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.execstamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.execstamp.design.def.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.lef 19-cadence-innovus-signoff/outputs/.execstamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.execstamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.sdf 19-cadence-innovus-signoff/outputs/.execstamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.execstamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.execstamp.design.virtuoso.v 19-cadence-innovus-signoff/outputs/.stamp.design-merged.gds 19-cadence-innovus-signoff/outputs/.stamp.design.checkpoint 19-cadence-innovus-signoff/outputs/.stamp.design.def.gz 19-cadence-innovus-signoff/outputs/.stamp.design.gds.gz 19-cadence-innovus-signoff/outputs/.stamp.design.lef 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v 19-cadence-innovus-signoff/outputs/.stamp.design.pt.sdc 19-cadence-innovus-signoff/outputs/.stamp.design.rcbest.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.sdf 19-cadence-innovus-signoff/outputs/.stamp.design.spef.gz 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.pg.v 19-cadence-innovus-signoff/outputs/.stamp.design.vcs.v 19-cadence-innovus-signoff/outputs/.stamp.design.virtuoso.v
25-open-netgen-lvs/.stamp: 21-open-magic-def2spice/.execstamp 21-open-magic-def2spice/.postconditions.stamp 21-open-magic-def2spice/outputs/.execstamp.design_extracted.spice 21-open-magic-def2spice/outputs/.stamp.design_extracted.spice
25-open-netgen-lvs/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
25-open-netgen-lvs/.stamp:
	$(call cpdir-and-parameterize,25-open-netgen-lvs,../design/open-netgen-lvs,25-open-netgen-lvs/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 25-open-netgen-lvs/.prebuilt)","")
25-open-netgen-lvs/inputs/.stamp.adk: 25-open-netgen-lvs/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,25-open-netgen-lvs/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 25-open-netgen-lvs/.prebuilt)","")
25-open-netgen-lvs/inputs/.stamp.design.lvs.v: 25-open-netgen-lvs/.stamp 19-cadence-innovus-signoff/outputs/.stamp.design.lvs.v
	$(call symlink,25-open-netgen-lvs/inputs,design.lvs.v,../../19-cadence-innovus-signoff/outputs/design.lvs.v,.stamp.design.lvs.v)
endif

ifeq ("$(wildcard 25-open-netgen-lvs/.prebuilt)","")
25-open-netgen-lvs/inputs/.stamp.design_extracted.spice: 25-open-netgen-lvs/.stamp 21-open-magic-def2spice/outputs/.stamp.design_extracted.spice
	$(call symlink,25-open-netgen-lvs/inputs,design_extracted.spice,../../21-open-magic-def2spice/outputs/design_extracted.spice,.stamp.design_extracted.spice)
endif

# execute

ifeq ("$(wildcard 25-open-netgen-lvs/.prebuilt)","")

define 25_open_netgen_lvs_commands_rule
	mkdir -p 25-open-netgen-lvs/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-netgen-lvs && chmod +x .mflowgen/25-open-netgen-lvs/mflowgen-run && cp -f .mflowgen/25-open-netgen-lvs/mflowgen-run 25-open-netgen-lvs && if [[ -e .mflowgen/25-open-netgen-lvs/mflowgen-debug ]]; then chmod +x .mflowgen/25-open-netgen-lvs/mflowgen-debug && cp -f .mflowgen/25-open-netgen-lvs/mflowgen-debug 25-open-netgen-lvs; fi && if [[ -e .mflowgen/25-open-netgen-lvs/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/25-open-netgen-lvs/mflowgen-check-preconditions.py && cp -f .mflowgen/25-open-netgen-lvs/mflowgen-check-preconditions.py 25-open-netgen-lvs; fi && if [[ -e .mflowgen/25-open-netgen-lvs/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/25-open-netgen-lvs/mflowgen-check-postconditions.py && cp -f .mflowgen/25-open-netgen-lvs/mflowgen-check-postconditions.py 25-open-netgen-lvs; fi && cd 25-open-netgen-lvs && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 25-open-netgen-lvs/outputs/*
endef

25-open-netgen-lvs/.execstamp: 25-open-netgen-lvs/.stamp 25-open-netgen-lvs/inputs/.stamp.adk 25-open-netgen-lvs/inputs/.stamp.design.lvs.v 25-open-netgen-lvs/inputs/.stamp.design_extracted.spice
	$(call 25_open_netgen_lvs_commands_rule)
	touch $@

25-open-netgen-lvs/outputs/.execstamp.lvs_results.log: 25-open-netgen-lvs/.execstamp
	touch $@

endif

# collect outputs

25-open-netgen-lvs/outputs/.stamp.lvs_results.log: 25-open-netgen-lvs/.execstamp 25-open-netgen-lvs/outputs/.execstamp.lvs_results.log
	$(call stamp,25-open-netgen-lvs/outputs/.stamp.lvs_results.log)

# post-conditions

define 25_open_netgen_lvs_post_conditions_commands_rule
	cd 25-open-netgen-lvs && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

25-open-netgen-lvs/.postconditions.stamp: 25-open-netgen-lvs/.execstamp 25-open-netgen-lvs/outputs/.execstamp.lvs_results.log 25-open-netgen-lvs/outputs/.stamp.lvs_results.log
	$(call 25_open_netgen_lvs_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: open-netgen-lvs

open-netgen-lvs: 25-open-netgen-lvs/.execstamp 25-open-netgen-lvs/.postconditions.stamp 25-open-netgen-lvs/outputs/.execstamp.lvs_results.log 25-open-netgen-lvs/outputs/.stamp.lvs_results.log

.PHONY: 25

25: 25-open-netgen-lvs/.execstamp 25-open-netgen-lvs/.postconditions.stamp 25-open-netgen-lvs/outputs/.execstamp.lvs_results.log 25-open-netgen-lvs/outputs/.stamp.lvs_results.log

# debug

# ------------------------------------------------------------------------
# cadence-innovus-eco
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 26-cadence-innovus-eco/.prebuilt)","")
26-cadence-innovus-eco/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
26-cadence-innovus-eco/.stamp: 23-open-icc2innovus/.execstamp 23-open-icc2innovus/.postconditions.stamp 23-open-icc2innovus/outputs/.execstamp.innovus_eco.tcl 23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl
26-cadence-innovus-eco/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
26-cadence-innovus-eco/.stamp:
	$(call cpdir-and-parameterize,26-cadence-innovus-eco,../../mflowgen/steps/cadence-innovus-eco,26-cadence-innovus-eco/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 26-cadence-innovus-eco/.prebuilt)","")
26-cadence-innovus-eco/inputs/.stamp.adk: 26-cadence-innovus-eco/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,26-cadence-innovus-eco/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 26-cadence-innovus-eco/.prebuilt)","")
26-cadence-innovus-eco/inputs/.stamp.innovus-foundation-flow: 26-cadence-innovus-eco/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,26-cadence-innovus-eco/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

ifeq ("$(wildcard 26-cadence-innovus-eco/.prebuilt)","")
26-cadence-innovus-eco/inputs/.stamp.innovus_eco.tcl: 26-cadence-innovus-eco/.stamp 23-open-icc2innovus/outputs/.stamp.innovus_eco.tcl
	$(call symlink,26-cadence-innovus-eco/inputs,innovus_eco.tcl,../../23-open-icc2innovus/outputs/innovus_eco.tcl,.stamp.innovus_eco.tcl)
endif

# execute

ifeq ("$(wildcard 26-cadence-innovus-eco/.prebuilt)","")

define 26_cadence_innovus_eco_commands_rule
	mkdir -p 26-cadence-innovus-eco/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-eco && chmod +x .mflowgen/26-cadence-innovus-eco/mflowgen-run && cp -f .mflowgen/26-cadence-innovus-eco/mflowgen-run 26-cadence-innovus-eco && if [[ -e .mflowgen/26-cadence-innovus-eco/mflowgen-debug ]]; then chmod +x .mflowgen/26-cadence-innovus-eco/mflowgen-debug && cp -f .mflowgen/26-cadence-innovus-eco/mflowgen-debug 26-cadence-innovus-eco; fi && if [[ -e .mflowgen/26-cadence-innovus-eco/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/26-cadence-innovus-eco/mflowgen-check-preconditions.py && cp -f .mflowgen/26-cadence-innovus-eco/mflowgen-check-preconditions.py 26-cadence-innovus-eco; fi && if [[ -e .mflowgen/26-cadence-innovus-eco/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/26-cadence-innovus-eco/mflowgen-check-postconditions.py && cp -f .mflowgen/26-cadence-innovus-eco/mflowgen-check-postconditions.py 26-cadence-innovus-eco; fi && cd 26-cadence-innovus-eco && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 26-cadence-innovus-eco/outputs/*
endef

26-cadence-innovus-eco/.execstamp: 26-cadence-innovus-eco/.stamp 26-cadence-innovus-eco/inputs/.stamp.adk 26-cadence-innovus-eco/inputs/.stamp.innovus-foundation-flow 26-cadence-innovus-eco/inputs/.stamp.innovus_eco.tcl
	$(call 26_cadence_innovus_eco_commands_rule)
	touch $@

26-cadence-innovus-eco/outputs/.execstamp.design.checkpoint: 26-cadence-innovus-eco/.execstamp
	touch $@

endif

# collect outputs

26-cadence-innovus-eco/outputs/.stamp.design.checkpoint: 26-cadence-innovus-eco/.execstamp 26-cadence-innovus-eco/outputs/.execstamp.design.checkpoint
	$(call stamp,26-cadence-innovus-eco/outputs/.stamp.design.checkpoint)

# post-conditions

define 26_cadence_innovus_eco_post_conditions_commands_rule
	cd 26-cadence-innovus-eco && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

26-cadence-innovus-eco/.postconditions.stamp: 26-cadence-innovus-eco/.execstamp 26-cadence-innovus-eco/outputs/.execstamp.design.checkpoint 26-cadence-innovus-eco/outputs/.stamp.design.checkpoint
	$(call 26_cadence_innovus_eco_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: cadence-innovus-eco

cadence-innovus-eco: 26-cadence-innovus-eco/.execstamp 26-cadence-innovus-eco/.postconditions.stamp 26-cadence-innovus-eco/outputs/.execstamp.design.checkpoint 26-cadence-innovus-eco/outputs/.stamp.design.checkpoint

.PHONY: 26

26: 26-cadence-innovus-eco/.execstamp 26-cadence-innovus-eco/.postconditions.stamp 26-cadence-innovus-eco/outputs/.execstamp.design.checkpoint 26-cadence-innovus-eco/outputs/.stamp.design.checkpoint

# debug

define 26_debug_rule
	cd 26-cadence-innovus-eco && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-cadence-innovus-eco: 
	$(call 26_debug_rule)

.PHONY: debug-26

debug-26: debug-cadence-innovus-eco

# ------------------------------------------------------------------------
# signoff_posteco
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 27-signoff_posteco/.prebuilt)","")
27-signoff_posteco/.stamp: 10-cadence-innovus-flowsetup/.execstamp 10-cadence-innovus-flowsetup/.postconditions.stamp 10-cadence-innovus-flowsetup/outputs/.execstamp.innovus-foundation-flow 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
27-signoff_posteco/.stamp: 26-cadence-innovus-eco/.execstamp 26-cadence-innovus-eco/.postconditions.stamp 26-cadence-innovus-eco/outputs/.execstamp.design.checkpoint 26-cadence-innovus-eco/outputs/.stamp.design.checkpoint
27-signoff_posteco/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
27-signoff_posteco/.stamp:
	$(call cpdir-and-parameterize,27-signoff_posteco,../design/cadence-innovus-signoff,27-signoff_posteco/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 27-signoff_posteco/.prebuilt)","")
27-signoff_posteco/inputs/.stamp.adk: 27-signoff_posteco/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,27-signoff_posteco/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 27-signoff_posteco/.prebuilt)","")
27-signoff_posteco/inputs/.stamp.design.checkpoint: 27-signoff_posteco/.stamp 26-cadence-innovus-eco/outputs/.stamp.design.checkpoint
	$(call symlink,27-signoff_posteco/inputs,design.checkpoint,../../26-cadence-innovus-eco/outputs/design.checkpoint,.stamp.design.checkpoint)
endif

ifeq ("$(wildcard 27-signoff_posteco/.prebuilt)","")
27-signoff_posteco/inputs/.stamp.innovus-foundation-flow: 27-signoff_posteco/.stamp 10-cadence-innovus-flowsetup/outputs/.stamp.innovus-foundation-flow
	$(call symlink,27-signoff_posteco/inputs,innovus-foundation-flow,../../10-cadence-innovus-flowsetup/outputs/innovus-foundation-flow,.stamp.innovus-foundation-flow)
endif

# execute

ifeq ("$(wildcard 27-signoff_posteco/.prebuilt)","")

define 27_signoff_posteco_commands_rule
	mkdir -p 27-signoff_posteco/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t signoff_posteco && chmod +x .mflowgen/27-signoff_posteco/mflowgen-run && cp -f .mflowgen/27-signoff_posteco/mflowgen-run 27-signoff_posteco && if [[ -e .mflowgen/27-signoff_posteco/mflowgen-debug ]]; then chmod +x .mflowgen/27-signoff_posteco/mflowgen-debug && cp -f .mflowgen/27-signoff_posteco/mflowgen-debug 27-signoff_posteco; fi && if [[ -e .mflowgen/27-signoff_posteco/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/27-signoff_posteco/mflowgen-check-preconditions.py && cp -f .mflowgen/27-signoff_posteco/mflowgen-check-preconditions.py 27-signoff_posteco; fi && if [[ -e .mflowgen/27-signoff_posteco/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/27-signoff_posteco/mflowgen-check-postconditions.py && cp -f .mflowgen/27-signoff_posteco/mflowgen-check-postconditions.py 27-signoff_posteco; fi && cd 27-signoff_posteco && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 27-signoff_posteco/outputs/*
endef

27-signoff_posteco/.execstamp: 27-signoff_posteco/.stamp 27-signoff_posteco/inputs/.stamp.adk 27-signoff_posteco/inputs/.stamp.design.checkpoint 27-signoff_posteco/inputs/.stamp.innovus-foundation-flow
	$(call 27_signoff_posteco_commands_rule)
	touch $@

27-signoff_posteco/outputs/.execstamp.design.checkpoint: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.gds.gz: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design-merged.gds: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.lvs.v: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.vcs.v: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.lef: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.def.gz: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.pt.sdc: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.sdf: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.virtuoso.v: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.spef.gz: 27-signoff_posteco/.execstamp
	touch $@
27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz: 27-signoff_posteco/.execstamp
	touch $@

endif

# collect outputs

27-signoff_posteco/outputs/.stamp.design.checkpoint: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.checkpoint)

27-signoff_posteco/outputs/.stamp.design.gds.gz: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.gds.gz)

27-signoff_posteco/outputs/.stamp.design-merged.gds: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design-merged.gds)

27-signoff_posteco/outputs/.stamp.design.lvs.v: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.lvs.v)

27-signoff_posteco/outputs/.stamp.design.vcs.v: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.vcs.v)

27-signoff_posteco/outputs/.stamp.design.vcs.pg.v: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.vcs.pg.v)

27-signoff_posteco/outputs/.stamp.design.lef: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.lef)

27-signoff_posteco/outputs/.stamp.design.def.gz: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.def.gz)

27-signoff_posteco/outputs/.stamp.design.pt.sdc: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.pt.sdc)

27-signoff_posteco/outputs/.stamp.design.sdf: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.sdf)

27-signoff_posteco/outputs/.stamp.design.virtuoso.v: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.virtuoso.v)

27-signoff_posteco/outputs/.stamp.design.spef.gz: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.spef.gz)

27-signoff_posteco/outputs/.stamp.design.rcbest.spef.gz: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v
	$(call stamp,27-signoff_posteco/outputs/.stamp.design.rcbest.spef.gz)

# post-conditions

define 27_signoff_posteco_post_conditions_commands_rule
	cd 27-signoff_posteco && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

27-signoff_posteco/.postconditions.stamp: 27-signoff_posteco/.execstamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v 27-signoff_posteco/outputs/.stamp.design-merged.gds 27-signoff_posteco/outputs/.stamp.design.checkpoint 27-signoff_posteco/outputs/.stamp.design.def.gz 27-signoff_posteco/outputs/.stamp.design.gds.gz 27-signoff_posteco/outputs/.stamp.design.lef 27-signoff_posteco/outputs/.stamp.design.lvs.v 27-signoff_posteco/outputs/.stamp.design.pt.sdc 27-signoff_posteco/outputs/.stamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.stamp.design.sdf 27-signoff_posteco/outputs/.stamp.design.spef.gz 27-signoff_posteco/outputs/.stamp.design.vcs.pg.v 27-signoff_posteco/outputs/.stamp.design.vcs.v 27-signoff_posteco/outputs/.stamp.design.virtuoso.v
	$(call 27_signoff_posteco_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: signoff_posteco

signoff_posteco: 27-signoff_posteco/.execstamp 27-signoff_posteco/.postconditions.stamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v 27-signoff_posteco/outputs/.stamp.design-merged.gds 27-signoff_posteco/outputs/.stamp.design.checkpoint 27-signoff_posteco/outputs/.stamp.design.def.gz 27-signoff_posteco/outputs/.stamp.design.gds.gz 27-signoff_posteco/outputs/.stamp.design.lef 27-signoff_posteco/outputs/.stamp.design.lvs.v 27-signoff_posteco/outputs/.stamp.design.pt.sdc 27-signoff_posteco/outputs/.stamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.stamp.design.sdf 27-signoff_posteco/outputs/.stamp.design.spef.gz 27-signoff_posteco/outputs/.stamp.design.vcs.pg.v 27-signoff_posteco/outputs/.stamp.design.vcs.v 27-signoff_posteco/outputs/.stamp.design.virtuoso.v

.PHONY: 27

27: 27-signoff_posteco/.execstamp 27-signoff_posteco/.postconditions.stamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v 27-signoff_posteco/outputs/.stamp.design-merged.gds 27-signoff_posteco/outputs/.stamp.design.checkpoint 27-signoff_posteco/outputs/.stamp.design.def.gz 27-signoff_posteco/outputs/.stamp.design.gds.gz 27-signoff_posteco/outputs/.stamp.design.lef 27-signoff_posteco/outputs/.stamp.design.lvs.v 27-signoff_posteco/outputs/.stamp.design.pt.sdc 27-signoff_posteco/outputs/.stamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.stamp.design.sdf 27-signoff_posteco/outputs/.stamp.design.spef.gz 27-signoff_posteco/outputs/.stamp.design.vcs.pg.v 27-signoff_posteco/outputs/.stamp.design.vcs.v 27-signoff_posteco/outputs/.stamp.design.virtuoso.v

# debug

define 27_debug_rule
	cd 27-signoff_posteco && ./mflowgen-debug 2>&1 | tee mflowgen-debug.log
endef

debug-signoff_posteco: 
	$(call 27_debug_rule)

.PHONY: debug-27

debug-27: debug-signoff_posteco

# ------------------------------------------------------------------------
# synopsys-pt-timing-signoff
# ------------------------------------------------------------------------

# build dir

ifeq ("$(wildcard 28-synopsys-pt-timing-signoff/.prebuilt)","")
28-synopsys-pt-timing-signoff/.stamp: 27-signoff_posteco/.execstamp 27-signoff_posteco/.postconditions.stamp 27-signoff_posteco/outputs/.execstamp.design-merged.gds 27-signoff_posteco/outputs/.execstamp.design.checkpoint 27-signoff_posteco/outputs/.execstamp.design.def.gz 27-signoff_posteco/outputs/.execstamp.design.gds.gz 27-signoff_posteco/outputs/.execstamp.design.lef 27-signoff_posteco/outputs/.execstamp.design.lvs.v 27-signoff_posteco/outputs/.execstamp.design.pt.sdc 27-signoff_posteco/outputs/.execstamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.execstamp.design.sdf 27-signoff_posteco/outputs/.execstamp.design.spef.gz 27-signoff_posteco/outputs/.execstamp.design.vcs.pg.v 27-signoff_posteco/outputs/.execstamp.design.vcs.v 27-signoff_posteco/outputs/.execstamp.design.virtuoso.v 27-signoff_posteco/outputs/.stamp.design-merged.gds 27-signoff_posteco/outputs/.stamp.design.checkpoint 27-signoff_posteco/outputs/.stamp.design.def.gz 27-signoff_posteco/outputs/.stamp.design.gds.gz 27-signoff_posteco/outputs/.stamp.design.lef 27-signoff_posteco/outputs/.stamp.design.lvs.v 27-signoff_posteco/outputs/.stamp.design.pt.sdc 27-signoff_posteco/outputs/.stamp.design.rcbest.spef.gz 27-signoff_posteco/outputs/.stamp.design.sdf 27-signoff_posteco/outputs/.stamp.design.spef.gz 27-signoff_posteco/outputs/.stamp.design.vcs.pg.v 27-signoff_posteco/outputs/.stamp.design.vcs.v 27-signoff_posteco/outputs/.stamp.design.virtuoso.v
28-synopsys-pt-timing-signoff/.stamp: 8-skywater-130nm/.execstamp 8-skywater-130nm/.postconditions.stamp 8-skywater-130nm/outputs/.execstamp.adk 8-skywater-130nm/outputs/.stamp.adk
28-synopsys-pt-timing-signoff/.stamp:
	$(call cpdir-and-parameterize,28-synopsys-pt-timing-signoff,../../mflowgen/steps/synopsys-pt-timing-signoff,28-synopsys-pt-timing-signoff/.stamp)
endif

# collect inputs

ifeq ("$(wildcard 28-synopsys-pt-timing-signoff/.prebuilt)","")
28-synopsys-pt-timing-signoff/inputs/.stamp.adk: 28-synopsys-pt-timing-signoff/.stamp 8-skywater-130nm/outputs/.stamp.adk
	$(call symlink,28-synopsys-pt-timing-signoff/inputs,adk,../../8-skywater-130nm/outputs/adk,.stamp.adk)
endif

ifeq ("$(wildcard 28-synopsys-pt-timing-signoff/.prebuilt)","")
28-synopsys-pt-timing-signoff/inputs/.stamp.design.pt.sdc: 28-synopsys-pt-timing-signoff/.stamp 27-signoff_posteco/outputs/.stamp.design.pt.sdc
	$(call symlink,28-synopsys-pt-timing-signoff/inputs,design.pt.sdc,../../27-signoff_posteco/outputs/design.pt.sdc,.stamp.design.pt.sdc)
endif

ifeq ("$(wildcard 28-synopsys-pt-timing-signoff/.prebuilt)","")
28-synopsys-pt-timing-signoff/inputs/.stamp.design.spef.gz: 28-synopsys-pt-timing-signoff/.stamp 27-signoff_posteco/outputs/.stamp.design.spef.gz
	$(call symlink,28-synopsys-pt-timing-signoff/inputs,design.spef.gz,../../27-signoff_posteco/outputs/design.spef.gz,.stamp.design.spef.gz)
endif

ifeq ("$(wildcard 28-synopsys-pt-timing-signoff/.prebuilt)","")
28-synopsys-pt-timing-signoff/inputs/.stamp.design.vcs.v: 28-synopsys-pt-timing-signoff/.stamp 27-signoff_posteco/outputs/.stamp.design.vcs.v
	$(call symlink,28-synopsys-pt-timing-signoff/inputs,design.vcs.v,../../27-signoff_posteco/outputs/design.vcs.v,.stamp.design.vcs.v)
endif

# execute

ifeq ("$(wildcard 28-synopsys-pt-timing-signoff/.prebuilt)","")

define 28_synopsys_pt_timing_signoff_commands_rule
	mkdir -p 28-synopsys-pt-timing-signoff/outputs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t synopsys-pt-timing-signoff && chmod +x .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-run && cp -f .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-run 28-synopsys-pt-timing-signoff && if [[ -e .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-debug ]]; then chmod +x .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-debug && cp -f .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-debug 28-synopsys-pt-timing-signoff; fi && if [[ -e .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-check-preconditions.py ]]; then chmod +x .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-check-preconditions.py && cp -f .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-check-preconditions.py 28-synopsys-pt-timing-signoff; fi && if [[ -e .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-check-postconditions.py ]]; then chmod +x .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-check-postconditions.py && cp -f .mflowgen/28-synopsys-pt-timing-signoff/mflowgen-check-postconditions.py 28-synopsys-pt-timing-signoff; fi && cd 28-synopsys-pt-timing-signoff && if [[ -e mflowgen-check-preconditions.py ]]; then ./mflowgen-check-preconditions.py || exit 1; fi && ./mflowgen-run 2>&1 | tee mflowgen-run.log || exit 1 && cd .. && touch -c 28-synopsys-pt-timing-signoff/outputs/*
endef

28-synopsys-pt-timing-signoff/.execstamp: 28-synopsys-pt-timing-signoff/.stamp 28-synopsys-pt-timing-signoff/inputs/.stamp.adk 28-synopsys-pt-timing-signoff/inputs/.stamp.design.pt.sdc 28-synopsys-pt-timing-signoff/inputs/.stamp.design.spef.gz 28-synopsys-pt-timing-signoff/inputs/.stamp.design.vcs.v
	$(call 28_synopsys_pt_timing_signoff_commands_rule)
	touch $@

28-synopsys-pt-timing-signoff/outputs/.execstamp.design.sdf: 28-synopsys-pt-timing-signoff/.execstamp
	touch $@

endif

# collect outputs

28-synopsys-pt-timing-signoff/outputs/.stamp.design.sdf: 28-synopsys-pt-timing-signoff/.execstamp 28-synopsys-pt-timing-signoff/outputs/.execstamp.design.sdf
	$(call stamp,28-synopsys-pt-timing-signoff/outputs/.stamp.design.sdf)

# post-conditions

define 28_synopsys_pt_timing_signoff_post_conditions_commands_rule
	cd 28-synopsys-pt-timing-signoff && if [[ -e mflowgen-check-postconditions.py ]]; then ./mflowgen-check-postconditions.py || exit 1; fi && cd ..
endef

28-synopsys-pt-timing-signoff/.postconditions.stamp: 28-synopsys-pt-timing-signoff/.execstamp 28-synopsys-pt-timing-signoff/outputs/.execstamp.design.sdf 28-synopsys-pt-timing-signoff/outputs/.stamp.design.sdf
	$(call 28_synopsys_pt_timing_signoff_post_conditions_commands_rule)
	touch $@

# alias

.PHONY: synopsys-pt-timing-signoff

synopsys-pt-timing-signoff: 28-synopsys-pt-timing-signoff/.execstamp 28-synopsys-pt-timing-signoff/.postconditions.stamp 28-synopsys-pt-timing-signoff/outputs/.execstamp.design.sdf 28-synopsys-pt-timing-signoff/outputs/.stamp.design.sdf

.PHONY: 28

28: 28-synopsys-pt-timing-signoff/.execstamp 28-synopsys-pt-timing-signoff/.postconditions.stamp 28-synopsys-pt-timing-signoff/outputs/.execstamp.design.sdf 28-synopsys-pt-timing-signoff/outputs/.stamp.design.sdf

# debug

# ------------------------------------------------------------------------
# Misc
# ------------------------------------------------------------------------

# Clean

.PHONY: clean-all

clean-all:
	@find . -maxdepth 1 ! -name Makefile ! -name .mflowgen\* ! -name \. ! -name \.\. -exec rm -rf {} +

.PHONY: clean-0

clean-0:
	rm -rf ./0-constraints

.PHONY: clean-constraints

clean-constraints: clean-0

.PHONY: clean-1

clean-1:
	rm -rf ./1-floorplan

.PHONY: clean-floorplan

clean-floorplan: clean-1

.PHONY: clean-10

clean-10:
	rm -rf ./10-cadence-innovus-flowsetup

.PHONY: clean-cadence-innovus-flowsetup

clean-cadence-innovus-flowsetup: clean-10

.PHONY: clean-11

clean-11:
	rm -rf ./11-cadence-innovus-init

.PHONY: clean-cadence-innovus-init

clean-cadence-innovus-init: clean-11

.PHONY: clean-12

clean-12:
	rm -rf ./12-cadence-innovus-power

.PHONY: clean-cadence-innovus-power

clean-cadence-innovus-power: clean-12

.PHONY: clean-13

clean-13:
	rm -rf ./13-cadence-innovus-place

.PHONY: clean-cadence-innovus-place

clean-cadence-innovus-place: clean-13

.PHONY: clean-14

clean-14:
	rm -rf ./14-cadence-innovus-cts

.PHONY: clean-cadence-innovus-cts

clean-cadence-innovus-cts: clean-14

.PHONY: clean-15

clean-15:
	rm -rf ./15-cadence-innovus-postcts_hold

.PHONY: clean-cadence-innovus-postcts_hold

clean-cadence-innovus-postcts_hold: clean-15

.PHONY: clean-16

clean-16:
	rm -rf ./16-cadence-innovus-route

.PHONY: clean-cadence-innovus-route

clean-cadence-innovus-route: clean-16

.PHONY: clean-17

clean-17:
	rm -rf ./17-cadence-innovus-postroute

.PHONY: clean-cadence-innovus-postroute

clean-cadence-innovus-postroute: clean-17

.PHONY: clean-18

clean-18:
	rm -rf ./18-cadence-innovus-postroute_hold

.PHONY: clean-cadence-innovus-postroute_hold

clean-cadence-innovus-postroute_hold: clean-18

.PHONY: clean-19

clean-19:
	rm -rf ./19-cadence-innovus-signoff

.PHONY: clean-cadence-innovus-signoff

clean-cadence-innovus-signoff: clean-19

.PHONY: clean-2

clean-2:
	rm -rf ./2-info

.PHONY: clean-info

clean-info: clean-2

.PHONY: clean-20

clean-20:
	rm -rf ./20-mentor-calibre-gdsmerge

.PHONY: clean-mentor-calibre-gdsmerge

clean-mentor-calibre-gdsmerge: clean-20

.PHONY: clean-21

clean-21:
	rm -rf ./21-open-magic-def2spice

.PHONY: clean-open-magic-def2spice

clean-open-magic-def2spice: clean-21

.PHONY: clean-22

clean-22:
	rm -rf ./22-synopsys-pt-eco

.PHONY: clean-synopsys-pt-eco

clean-synopsys-pt-eco: clean-22

.PHONY: clean-23

clean-23:
	rm -rf ./23-open-icc2innovus

.PHONY: clean-open-icc2innovus

clean-open-icc2innovus: clean-23

.PHONY: clean-24

clean-24:
	rm -rf ./24-open-magic-drc

.PHONY: clean-open-magic-drc

clean-open-magic-drc: clean-24

.PHONY: clean-25

clean-25:
	rm -rf ./25-open-netgen-lvs

.PHONY: clean-open-netgen-lvs

clean-open-netgen-lvs: clean-25

.PHONY: clean-26

clean-26:
	rm -rf ./26-cadence-innovus-eco

.PHONY: clean-cadence-innovus-eco

clean-cadence-innovus-eco: clean-26

.PHONY: clean-27

clean-27:
	rm -rf ./27-signoff_posteco

.PHONY: clean-signoff_posteco

clean-signoff_posteco: clean-27

.PHONY: clean-28

clean-28:
	rm -rf ./28-synopsys-pt-timing-signoff

.PHONY: clean-synopsys-pt-timing-signoff

clean-synopsys-pt-timing-signoff: clean-28

.PHONY: clean-3

clean-3:
	rm -rf ./3-macros

.PHONY: clean-macros

clean-macros: clean-3

.PHONY: clean-4

clean-4:
	rm -rf ./4-pin-placement

.PHONY: clean-pin-placement

clean-pin-placement: clean-4

.PHONY: clean-5

clean-5:
	rm -rf ./5-plugin-dc

.PHONY: clean-plugin-dc

clean-plugin-dc: clean-5

.PHONY: clean-6

clean-6:
	rm -rf ./6-plugin-dc-retime

.PHONY: clean-plugin-dc-retime

clean-plugin-dc-retime: clean-6

.PHONY: clean-7

clean-7:
	rm -rf ./7-rtl

.PHONY: clean-rtl

clean-rtl: clean-7

.PHONY: clean-8

clean-8:
	rm -rf ./8-skywater-130nm

.PHONY: clean-skywater-130nm

clean-skywater-130nm: clean-8

.PHONY: clean-9

clean-9:
	rm -rf ./9-synopsys-dc-synthesis

.PHONY: clean-synopsys-dc-synthesis

clean-synopsys-dc-synthesis: clean-9

# Diff

.PHONY: diff-0

diff-0:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/constraints 0-constraints | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/constraints.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 0-constraints.*' && echo || true

.PHONY: diff-1

diff-1:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/floorplan 1-floorplan | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/floorplan.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 1-floorplan.*' && echo || true

.PHONY: diff-2

diff-2:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/info 2-info | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/info.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 2-info.*' && echo || true

.PHONY: diff-3

diff-3:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/macros 3-macros | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/macros.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 3-macros.*' && echo || true

.PHONY: diff-4

diff-4:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/pin-placement 4-pin-placement | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/pin-placement.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 4-pin-placement.*' && echo || true

.PHONY: diff-5

diff-5:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/plugin-dc 5-plugin-dc | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/plugin-dc.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 5-plugin-dc.*' && echo || true

.PHONY: diff-6

diff-6:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/plugin-dc-retime 6-plugin-dc-retime | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/plugin-dc-retime.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 6-plugin-dc-retime.*' && echo || true

.PHONY: diff-7

diff-7:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/rtl 7-rtl | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/rtl.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 7-rtl.*' && echo || true

.PHONY: diff-8

diff-8:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../../../../afs/ir.stanford.edu/class/ee272/skywater-130nm-adk 8-skywater-130nm | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../../../../afs/ir.stanford.edu/class/ee272/skywater-130nm-adk.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 8-skywater-130nm.*' && echo || true

.PHONY: diff-9

diff-9:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/synopsys-dc-synthesis 9-synopsys-dc-synthesis | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/synopsys-dc-synthesis.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 9-synopsys-dc-synthesis.*' && echo || true

.PHONY: diff-10

diff-10:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/iflow 10-cadence-innovus-flowsetup | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/iflow.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 10-cadence-innovus-flowsetup.*' && echo || true

.PHONY: diff-11

diff-11:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-init 11-cadence-innovus-init | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-init.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 11-cadence-innovus-init.*' && echo || true

.PHONY: diff-12

diff-12:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/cadence-innovus-power 12-cadence-innovus-power | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/cadence-innovus-power.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 12-cadence-innovus-power.*' && echo || true

.PHONY: diff-13

diff-13:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-place 13-cadence-innovus-place | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-place.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 13-cadence-innovus-place.*' && echo || true

.PHONY: diff-14

diff-14:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-cts 14-cadence-innovus-cts | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-cts.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 14-cadence-innovus-cts.*' && echo || true

.PHONY: diff-15

diff-15:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-postcts_hold 15-cadence-innovus-postcts_hold | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-postcts_hold.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 15-cadence-innovus-postcts_hold.*' && echo || true

.PHONY: diff-16

diff-16:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-route 16-cadence-innovus-route | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-route.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 16-cadence-innovus-route.*' && echo || true

.PHONY: diff-17

diff-17:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-postroute 17-cadence-innovus-postroute | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-postroute.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 17-cadence-innovus-postroute.*' && echo || true

.PHONY: diff-18

diff-18:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-postroute_hold 18-cadence-innovus-postroute_hold | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-postroute_hold.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 18-cadence-innovus-postroute_hold.*' && echo || true

.PHONY: diff-19

diff-19:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/cadence-innovus-signoff 19-cadence-innovus-signoff | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/cadence-innovus-signoff.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 19-cadence-innovus-signoff.*' && echo || true

.PHONY: diff-20

diff-20:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/mentor-calibre-gdsmerge 20-mentor-calibre-gdsmerge | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/mentor-calibre-gdsmerge.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 20-mentor-calibre-gdsmerge.*' && echo || true

.PHONY: diff-21

diff-21:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/open-magic-def2spice 21-open-magic-def2spice | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/open-magic-def2spice.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 21-open-magic-def2spice.*' && echo || true

.PHONY: diff-22

diff-22:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/synopsys-pt-eco 22-synopsys-pt-eco | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/synopsys-pt-eco.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 22-synopsys-pt-eco.*' && echo || true

.PHONY: diff-23

diff-23:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/open-icc2innovus 23-open-icc2innovus | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/open-icc2innovus.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 23-open-icc2innovus.*' && echo || true

.PHONY: diff-24

diff-24:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/open-magic-drc 24-open-magic-drc | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/open-magic-drc.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 24-open-magic-drc.*' && echo || true

.PHONY: diff-25

diff-25:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/open-netgen-lvs 25-open-netgen-lvs | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/open-netgen-lvs.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 25-open-netgen-lvs.*' && echo || true

.PHONY: diff-26

diff-26:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/cadence-innovus-eco 26-cadence-innovus-eco | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/cadence-innovus-eco.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 26-cadence-innovus-eco.*' && echo || true

.PHONY: diff-27

diff-27:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../design/cadence-innovus-signoff 27-signoff_posteco | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../design/cadence-innovus-signoff.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 27-signoff_posteco.*' && echo || true

.PHONY: diff-28

diff-28:
	@echo && diff -r -u --minimal --exclude={configure.yml,.time_end,.time_start,mflowgen-run*,mflowgen-debug,.stamp,inputs,outputs} ../../mflowgen/steps/synopsys-pt-timing-signoff 28-synopsys-pt-timing-signoff | grep --color=always -e '^-.*' -e '$$' -e 'Only in ../../mflowgen/steps/synopsys-pt-timing-signoff.*' | GREP_COLOR='01;32' grep --color=always -e '^+.*' -e '$$' -e 'Only in 28-synopsys-pt-timing-signoff.*' && echo || true

# Info

.PHONY: info-0

info-0:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t constraints && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/0-constraints/configure.yml

.PHONY: info-1

info-1:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t floorplan && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/1-floorplan/configure.yml

.PHONY: info-2

info-2:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t info && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/2-info/configure.yml

.PHONY: info-3

info-3:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t macros && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/3-macros/configure.yml

.PHONY: info-4

info-4:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t pin-placement && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/4-pin-placement/configure.yml

.PHONY: info-5

info-5:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t plugin-dc && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/5-plugin-dc/configure.yml

.PHONY: info-6

info-6:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t plugin-dc-retime && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/6-plugin-dc-retime/configure.yml

.PHONY: info-7

info-7:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t rtl && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/7-rtl/configure.yml

.PHONY: info-8

info-8:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t skywater-130nm && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/8-skywater-130nm/configure.yml

.PHONY: info-9

info-9:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t synopsys-dc-synthesis && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/9-synopsys-dc-synthesis/configure.yml

.PHONY: info-10

info-10:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-flowsetup && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/10-cadence-innovus-flowsetup/configure.yml

.PHONY: info-11

info-11:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-init && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/11-cadence-innovus-init/configure.yml

.PHONY: info-12

info-12:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-power && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/12-cadence-innovus-power/configure.yml

.PHONY: info-13

info-13:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-place && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/13-cadence-innovus-place/configure.yml

.PHONY: info-14

info-14:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-cts && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/14-cadence-innovus-cts/configure.yml

.PHONY: info-15

info-15:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-postcts_hold && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/15-cadence-innovus-postcts_hold/configure.yml

.PHONY: info-16

info-16:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-route && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/16-cadence-innovus-route/configure.yml

.PHONY: info-17

info-17:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-postroute && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/17-cadence-innovus-postroute/configure.yml

.PHONY: info-18

info-18:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-postroute_hold && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/18-cadence-innovus-postroute_hold/configure.yml

.PHONY: info-19

info-19:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-signoff && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/19-cadence-innovus-signoff/configure.yml

.PHONY: info-20

info-20:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t mentor-calibre-gdsmerge && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/20-mentor-calibre-gdsmerge/configure.yml

.PHONY: info-21

info-21:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-magic-def2spice && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/21-open-magic-def2spice/configure.yml

.PHONY: info-22

info-22:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t synopsys-pt-eco && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/22-synopsys-pt-eco/configure.yml

.PHONY: info-23

info-23:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-icc2innovus && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/23-open-icc2innovus/configure.yml

.PHONY: info-24

info-24:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-magic-drc && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/24-open-magic-drc/configure.yml

.PHONY: info-25

info-25:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t open-netgen-lvs && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/25-open-netgen-lvs/configure.yml

.PHONY: info-26

info-26:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t cadence-innovus-eco && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/26-cadence-innovus-eco/configure.yml

.PHONY: info-27

info-27:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t signoff_posteco && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/27-signoff_posteco/configure.yml

.PHONY: info-28

info-28:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-letters -c -t synopsys-pt-timing-signoff && /tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-info -y .mflowgen/28-synopsys-pt-timing-signoff/configure.yml

# Runtimes

.PHONY: runtimes

runtimes:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-runtimes

# List

.PHONY: list

list:
	@echo && echo Generic Targets: && echo && printf " - %s\\n" "list      -- List all steps" "status    -- Print build status for each step" "runtimes  -- Print runtimes for each step" "graph     -- Generate a PDF of the step dependency graph" "clean-all -- Remove all build directories" "clean-N   -- Clean target N" "info-N    -- Print configured info for step N" "diff-N    -- Diff target N" && echo && echo Targets: && echo && printf " - %s\\n" "  0 : constraints" "  1 : floorplan" "  2 : info" "  3 : macros" "  4 : pin-placement" "  5 : plugin-dc" "  6 : plugin-dc-retime" "  7 : rtl" "  8 : skywater-130nm" "  9 : synopsys-dc-synthesis" " 10 : cadence-innovus-flowsetup" " 11 : cadence-innovus-init" " 12 : cadence-innovus-power" " 13 : cadence-innovus-place" " 14 : cadence-innovus-cts" " 15 : cadence-innovus-postcts_hold" " 16 : cadence-innovus-route" " 17 : cadence-innovus-postroute" " 18 : cadence-innovus-postroute_hold" " 19 : cadence-innovus-signoff" " 20 : mentor-calibre-gdsmerge" " 21 : open-magic-def2spice" " 22 : synopsys-pt-eco" " 23 : open-icc2innovus" " 24 : open-magic-drc" " 25 : open-netgen-lvs" " 26 : cadence-innovus-eco" " 27 : signoff_posteco" " 28 : synopsys-pt-timing-signoff" && echo && echo Debug Targets: && echo && printf " - %s\\n" "debug-9  : debug-synopsys-dc-synthesis" "debug-11 : debug-cadence-innovus-init" "debug-12 : debug-cadence-innovus-power" "debug-13 : debug-cadence-innovus-place" "debug-14 : debug-cadence-innovus-cts" "debug-15 : debug-cadence-innovus-postcts_hold" "debug-16 : debug-cadence-innovus-route" "debug-17 : debug-cadence-innovus-postroute" "debug-18 : debug-cadence-innovus-postroute_hold" "debug-19 : debug-cadence-innovus-signoff" "debug-20 : debug-mentor-calibre-gdsmerge" "debug-26 : debug-cadence-innovus-eco" "debug-27 : debug-signoff_posteco" && echo

# Graph

.PHONY: graph

graph:
	dot -Tpdf .mflowgen/graph.dot > graph.pdf

# Status

.PHONY: status

status:
	@/tmp/ee272b/flow/mflowgen/mflowgen/scripts/mflowgen-status --backend make -s 0-constraints,1-floorplan,2-info,3-macros,4-pin-placement,5-plugin-dc,6-plugin-dc-retime,7-rtl,8-skywater-130nm,9-synopsys-dc-synthesis,10-cadence-innovus-flowsetup,11-cadence-innovus-init,12-cadence-innovus-power,13-cadence-innovus-place,14-cadence-innovus-cts,15-cadence-innovus-postcts_hold,16-cadence-innovus-route,17-cadence-innovus-postroute,18-cadence-innovus-postroute_hold,19-cadence-innovus-signoff,20-mentor-calibre-gdsmerge,21-open-magic-def2spice,22-synopsys-pt-eco,23-open-icc2innovus,24-open-magic-drc,25-open-netgen-lvs,26-cadence-innovus-eco,27-signoff_posteco,28-synopsys-pt-timing-signoff

