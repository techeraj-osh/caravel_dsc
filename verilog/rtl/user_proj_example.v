// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output reg wbs_ack_o,
    output reg [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    wire pwm1_select , pwm2_select , pid_select ;
    wire pwm1_wbs_stb_i , pwm2_wbs_stb_i , pid_wbs_stb_i ;
    wire pwm1_wbs_ack_o , pwm2_wbs_ack_o , pid_wbs_ack_o ; 

    reg [15:0] pwm1_wbs_dat_o ;
    reg [15:0] pwm2_wbs_dat_o ;
    reg [31:0] pid_wbs_dat_o  ;

    wire pwm_out1 , pwm_out2 ;
    reg led1, led2, led3 ; 


    // IO
    assign io_out = {33'b0,pwm_out2,pwm_out1,led3,led2,led1};
    assign io_oeb = {33'b0,5'b11111};
    //assign io_out = {35'b0,led3,led2,led1};
    //assign io_oeb = {35'b0,3'b111};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    //assign la_data_out = {{(127-BITS){1'b0}}, count};
    assign la_data_out = 128'b0;
    // Assuming LA probes [63:32] are for controlling the count register  
    //assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    //assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    //assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;
   assign clk = wb_clk_i ;
   assign rst = wb_rst_i ;

   // Module Address Select Logic
   assign pwm1_select = (wbs_adr_i[31:12] == 20'h30001) ;
   assign pwm2_select = (wbs_adr_i[31:12] == 20'h30002) ;
   assign pid_select  = (wbs_adr_i[31:12] == 20'h30005) ;
  
   // Module STROBE Select based on Address Range
   assign pwm1_wbs_stb_i = (wbs_stb_i && pwm1_select) ;
   assign pwm2_wbs_stb_i = (wbs_stb_i && pwm2_select) ;
   assign pid_wbs_stb_i  = (wbs_stb_i && pid_select) ;

   // Led assigned from LA data in
   always @(posedge clk) begin
	   led1 <= la_data_in[0] && la_oenb[0] ; 
	   led2 <= la_data_in[1] && la_oenb[1] ; 
	   led3 <= la_data_in[2] && la_oenb[2] ; 
   end

   // Slave Acknowledge Response 
   always @(posedge clk)
	   wbs_ack_o <= (pwm1_wbs_ack_o || pwm2_wbs_ack_o || pid_wbs_ack_o) ;
	   //wbs_ack_o <=  pid_wbs_ack_o ;

   // Slave Return Data
   always @(posedge clk)
	   if (pwm1_wbs_ack_o)
		   wbs_dat_o <= {16'h0,pwm1_wbs_dat_o} ;
	   else if (pwm2_wbs_ack_o)
		   wbs_dat_o <= {16'h0,pwm2_wbs_dat_o} ;
	   else if (pid_wbs_ack_o)
		   wbs_dat_o <= pid_wbs_dat_o ;
	   else
		   wbs_dat_o <= 32'h0 ;

/*
   always @(posedge clk)
	   if (pid_wbs_ack_o)
		   wbs_dat_o <= pid_wbs_dat_o ;
	   else
		   wbs_dat_o <= 32'h0 ;
*/

	   
   // PWM1 Module instantiations 
   PWM pwm1 (
	   .i_wb_clk (clk),
	   .i_wb_rst (rst),
	   .i_wb_cyc (wbs_cyc_i),
	   .i_wb_stb (pwm1_wbs_stb_i),
	   .i_wb_we  (wbs_we_i),
	   .i_wb_adr ({8'h0,wbs_adr_i[7:0]}), // 16-bit address
	   .i_wb_data (wbs_dat_i[15:0]),
	   .o_wb_data (pwm1_wbs_dat_o),
	   .o_wb_ack (pwm1_wbs_ack_o),
	   .i_DC (16'h0),
	   .i_valid_DC (1'b0),
	   .o_pwm (pwm_out1)
   ); 
/*
   // PWM2 Module instantiations 
   PWM pwm2 (
	   .i_wb_clk (clk),
	   .i_wb_rst (rst),
	   .i_wb_cyc (wbs_cyc_i),
	   .i_wb_stb (pwm2_wbs_stb_i),
	   .i_wb_we  (wbs_we_i),
	   .i_wb_adr ({8'h0,wbs_adr_i[7:0]}), // 16-bit address
	   .i_wb_data (wbs_dat_i[15:0]),
	   .o_wb_data (pwm2_wbs_dat_o),
	   .o_wb_ack (pwm2_wbs_ack_o),
	   .i_DC (16'h0),
	   .i_valid_DC (1'b0),
	   .o_pwm (pwm_out2)
   );

  PID pid (
	  .i_clk (clk),
	  .i_rst (rst),
	  .i_wb_cyc (wbs_cyc_i),
	  .i_wb_stb (pid_wbs_stb_i),
	  .i_wb_we (wbs_we_i),
	  .i_wb_adr ({8'h0,wbs_adr_i[7:0]}), // 16-bit address
	  .i_wb_data (wbs_dat_i),
	  .o_wb_ack (pid_wbs_ack_o),
	  .o_wb_data (pid_wbs_dat_o),
	  .o_un (),
	  .o_valid ()
  ); 
*/
endmodule
`default_nettype wire
