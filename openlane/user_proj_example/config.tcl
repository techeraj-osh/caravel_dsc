# SPDX-FileCopyrightText: 2020 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) user_proj_example

set ::env(VERILOG_FILES) "\
	$script_dir/../../caravel/verilog/rtl/defines.v \
        $script_dir/../../verilog/rtl/ptc/ptc_defines.v \
        $script_dir/../../verilog/rtl/ptc/ptc_top.v \
	$script_dir/../../verilog/rtl/user_proj_example.v"

        # $script_dir/../../verilog/rtl/pid/16x16bit_multiplier_pipelined.v \
        # $script_dir/../../verilog/rtl/pid/booth.v \
        # $script_dir/../../verilog/rtl/pid/CLA_fixed.v \
        # $script_dir/../../verilog/rtl/pid/PID.v 
	
set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_NET) ""
set ::env(CLOCK_PERIOD) "10"

# default 1
set ::env(DESIGN_IS_CORE) 0

# default 0
set ::env(FP_PDN_CORE_RING) 0

# default 6
set ::env(GLB_RT_MAXLAYER) 5

# Extra settings

#set ::env(FP_SIZING) absolute
#set ::env(DIE_AREA) "0 0 900 600"
#set ::env(DIE_AREA) [list 0.0 0.0 1748.0 1360.0]

set ::env(VDD_NETS) [list {vccd1} {vccd2} {vdda1} {vdda2}]
set ::env(GND_NETS) [list {vssd1} {vssd2} {vssa1} {vssa2}]

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg

set ::env(FP_CORE_UTIL) 25
set ::env(PL_TARGET_DENSITY) [ expr ($::env(FP_CORE_UTIL)+5) / 100.0 ]
set ::env(ROUTING_CORES) 12

# default 0
#set ::env(PL_BASIC_PLACEMENT) 1

# default 50
#set ::env(FP_CORE_UTIL) 20

# default 0.55
#set ::env(PL_TARGET_DENSITY) 0.05

# If you're going to use multiple power domains, then keep this disabled.
# default 1
set ::env(RUN_CVC) 0

# Routing Strategy 14 is more powerful than default
# default 0
#set ::env(ROUTING_STRATEGY) 14

# Synthesis parameters

# default 0
#set ::env(SYNTH_SIZING) 1

# default 1
#set ::env(SYNTH_BUFFERING) 1

# default AREA 0, try flow.tcl -synth_explore
set ::env(SYNTH_STRATEGY) "DELAY 1"

#set ::env(SYNTH_DRIVING_CELL) sky130_fd_sc_hc__inv_8

# default 1
#set ::env(GLB_RT_MAX_DIODE_INS_ITERS) 10
