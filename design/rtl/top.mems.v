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

`default_nettype wire

`ifndef __GLOBAL_DEFINE_H
// Global parameters
`define __GLOBAL_DEFINE_H

`define MPRJ_IO_PADS_1 19	/* number of user GPIO pads on user1 side */
`define MPRJ_IO_PADS_2 19	/* number of user GPIO pads on user2 side */
`define MPRJ_IO_PADS (`MPRJ_IO_PADS_1 + `MPRJ_IO_PADS_2)

`define MPRJ_PWR_PADS_1 2	/* vdda1, vccd1 enable/disable control */
`define MPRJ_PWR_PADS_2 2	/* vdda2, vccd2 enable/disable control */
`define MPRJ_PWR_PADS (`MPRJ_PWR_PADS_1 + `MPRJ_PWR_PADS_2)

// Analog pads are only used by the "caravan" module and associated
// modules such as user_analog_project_wrapper and chip_io_alt.

`define ANALOG_PADS_1 5
`define ANALOG_PADS_2 6

`define ANALOG_PADS (`ANALOG_PADS_1 + `ANALOG_PADS_2)

// Size of soc_mem_synth

// Type and size of soc_mem
// `define USE_OPENRAM
// `define USE_CUSTOM_DFFRAM
// don't change the following without double checking addr widths
`define MEM_WORDS 256

// Number of columns in the custom memory; takes one of three values:
// 1 column : 1 KB, 2 column: 2 KB, 4 column: 4KB
`define COLS 1

// not really parameterized but just to easily keep track of the number
// of ram_block across different modules
`define RAM_BLOCKS 2

// Clock divisor default value
`define CLK_DIV 3'b010

// GPIO conrol default mode and enable
`define DM_INIT 3'b110
`define OENB_INIT 1'b1

`endif // __GLOBAL_DEFINE_H
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

`default_nettype wire
`ifndef SYNTHESIS

module DFFRAM
#(
  parameter COLS = 1
)(
`ifdef USE_POWER_PINS
    input VPWR,
    input VGND,
`endif
    input CLK,
    input [3:0] WE,
    input EN,
    input [31:0] Di,
    output reg [31:0] Do,
    input [7+$clog2(COLS):0] A
);
  

reg [31:0] mem [0:`MEM_WORDS-1];

always @(posedge CLK) begin
    if (EN == 1'b1) begin
        Do <= mem[A];
        if (WE[0]) mem[A][ 7: 0] <= Di[ 7: 0];
        if (WE[1]) mem[A][15: 8] <= Di[15: 8];
        if (WE[2]) mem[A][23:16] <= Di[23:16];
        if (WE[3]) mem[A][31:24] <= Di[31:24];
    end
end
endmodule

`endif





module dffram_wrapper256
#(
  parameter DATA_WIDTH = 8,
  parameter ADDR_WIDTH = 7,
  parameter DEPTH = 128
)(
  input CLK,
  input [ ( (DATA_WIDTH + 8 - 1) / 8) - 1: 0] WE,
  input EN,
  input [ADDR_WIDTH - 1 : 0] A,
  input [DATA_WIDTH - 1 : 0] Di,
  output [DATA_WIDTH - 1 : 0] Do
);
  
  /*
  // synopsys translate_off
  reg [DATA_WIDTH - 1 : 0] rdata_reg;
  
  reg [DATA_WIDTH - 1 : 0] mem [DEPTH - 1 : 0];
  
  always @(posedge CLK) begin
    if (wen) begin
      mem[wadr] <= wdata; // write port
    end
    if (ren) begin
      rdata_reg <= mem[radr]; // read port
    end
  end
  assign Do = rdata_reg;
  
  // synopsys translate_on
  */
  
  genvar x, y; 
  
  generate
    if (DEPTH == 256) begin

      wire [DATA_WIDTH : 0] rdata_w;
      reg  [ADDR_WIDTH - 1 : 0] radr_r;

      always @ (posedge CLK) begin
        radr_r <= A;
      end

      for (x = 0; x < DATA_WIDTH/32; x = x + 1) begin: width_macro
          wire [3:0] bank_WE;
          assign bank_WE = WE[4*(x+1)-1 : 4*x];
          DFFRAM sram (
            .CLK(CLK),
            .WE(bank_WE),
            .EN(EN),
            .Di(Di[32*(x+1)-1 : 32*x]),
            .Do(rdata_w[32*(x+1)-1 : 32*x]),
            .A(A[ 7:0 ])
          );
      end

      assign Do = rdata_w;

    end else begin

      wire [DATA_WIDTH : 0] rdata_w [DEPTH/256 - 1 : 0];
      reg  [ADDR_WIDTH - 1 : 0] radr_r;

      always @ (posedge CLK) begin
        radr_r <= A;
      end

      for (x = 0; x < DATA_WIDTH/32; x = x + 1) begin: width_macro
        for (y = 0; y < DEPTH/256; y = y + 1) begin: depth_macro
          wire [ADDR_WIDTH - 9 : 0] bank;
          wire bank_match;
          assign bank = A[ ADDR_WIDTH-1 : 8 ];
          assign bank_match = bank == y;
          wire [3:0] bank_match_extended;
          assign bank_match_extended = {4{bank_match}};
          wire [3:0] bank_WE;
          assign bank_WE = WE[4*(x+1)-1 : 4*x];
          DFFRAM sram (
            .CLK(CLK),
            .WE(bank_WE & bank_match_extended),
            .EN(EN),
            .Di(Di[32*(x+1)-1 : 32*x]),
            .Do(rdata_w[y][32*(x+1)-1 : 32*x]),
            .A(A[ 7:0 ])
          );
        end
      end

      assign Do = rdata_w[radr_r[ADDR_WIDTH - 1 : 3]];

    end
  endgenerate
  
endmodule
/* 
	DFFRFile
	32x32 Register File with 2RW1W ports and clock gating for SKY130A 
	~ 3550 Cells
	< 2ns (no input or output delays)
*/
/*
    	Author: Mohamed Shalan (mshalan@aucegypt.edu)
*/

`timescale 1ns / 1ps
`default_nettype wire

module dffrf_2R1W_param 
#(
        parameter DATA_WIDTH = 8
)(
	input [2:0] 	RA, RB, RW,
	input [DATA_WIDTH-1:0] 	DW,
	output [DATA_WIDTH-1:0]	DA, DB,
	input CLK,
	input WE
);

	wire [7:0] sel1, sel2, selw;

	DEC3x8 DEC0 ( .A(RA), .SEL(sel1) );
	DEC3x8 DEC1 ( .A(RB), .SEL(sel2) );
	DEC3x8 DEC2 ( .A(RW), .SEL(selw) );
	
	RFWORD0 #(.DATA_WIDTH(DATA_WIDTH)) RFW0 ( .CLK(CLK), .WE(), .SEL1(sel1[0]), .SEL2(sel2[0]), .SELW(), .D1(DA), .D2(DB));	

	generate
		genvar e;
		for(e=1; e<8; e=e+1)
			RFWORD #(.DATA_WIDTH(DATA_WIDTH)) RFW ( .CLK(CLK), .WE(WE), .SEL1(sel1[e]), .SEL2(sel2[e]), .SELW(selw[e]), .D1(DA), .D2(DB), .DW(DW) );	
	endgenerate
endmodule

module RFWORD 
#(
        parameter DATA_WIDTH = 8
)(
    input   wire        CLK,
    input   wire        WE,
    input   wire        SEL1, 
    input   wire        SEL2, 
    input   wire        SELW,
    output  wire [DATA_WIDTH-1:0] D1, D2,
    input   wire [DATA_WIDTH-1:0] DW
);

    wire [DATA_WIDTH-1:0] q_wire;
    wire        we_wire;
    wire [3:0]  SEL1_B, SEL2_B;
    wire [3:0]  GCLK;

    sky130_fd_sc_hd__inv_4 INV1[3:0] (.Y(SEL1_B), .A(SEL1));
	sky130_fd_sc_hd__inv_4 INV2[3:0] (.Y(SEL2_B), .A(SEL2));

    sky130_fd_sc_hd__and2_1 CGAND ( .A(SELW), .B(WE), .X(we_wire) );
    sky130_fd_sc_hd__dlclkp_1 CG[3:0] ( .CLK(CLK), .GCLK(GCLK), .GATE(we_wire) );

    generate 
        genvar i;
        for(i=0; i<DATA_WIDTH; i=i+1) begin : BIT
            sky130_fd_sc_hd__dfxtp_1 FF ( .D(DW[i]), .Q(q_wire[i]), .CLK(GCLK[i/8]) );
            sky130_fd_sc_hd__ebufn_2 OBUF1 ( .A(q_wire[i]), .Z(D1[i]), .TE_B(SEL1_B[i/8]) );
			sky130_fd_sc_hd__ebufn_2 OBUF2 ( .A(q_wire[i]), .Z(D2[i]), .TE_B(SEL2_B[i/8]) );
        end
		
    endgenerate 
endmodule

module RFWORD0 
#(
        parameter DATA_WIDTH = 8
)(
    input CLK,
    input WE,
    input SEL1, SEL2, SELW,
    output [DATA_WIDTH-1:0] D1, D2
    //input [DATA_WIDTH-1:0] DW
);

    wire [DATA_WIDTH-1:0]  q_wire;
    wire   we_wire;
    wire [3:0]   SEL1_B, SEL2_B;
    wire [3:0]  GCLK;
	wire [7:0]	lo;

    sky130_fd_sc_hd__inv_4 INV1[3:0] (.Y(SEL1_B), .A(SEL1));
	sky130_fd_sc_hd__inv_4 INV2[3:0] (.Y(SEL2_B), .A(SEL2));

	sky130_fd_sc_hd__conb_1 TIE [7:0] (.LO(lo), .HI());

    generate 
        genvar i;
        for(i=0; i<DATA_WIDTH; i=i+1) begin : BIT
            sky130_fd_sc_hd__ebufn_2 OBUF1 ( .A(lo[i/8]), .Z(D1[i]), .TE_B(SEL1_B[i/8]) );
			sky130_fd_sc_hd__ebufn_2 OBUF2 ( .A(lo[4+1/8]), .Z(D2[i]), .TE_B(SEL2_B[i/8]) );
        end
    endgenerate 
endmodule

module DEC2x4 (
    input           EN,
    input   [1:0]   A,
    output  [3:0]   SEL
);
    sky130_fd_sc_hd__nor3b_4    AND0 ( .Y(SEL[0]), .A(A[0]),   .B(A[1]), .C_N(EN) );
    sky130_fd_sc_hd__and3b_4    AND1 ( .X(SEL[1]), .A_N(A[1]), .B(A[0]), .C(EN) );
    sky130_fd_sc_hd__and3b_4    AND2 ( .X(SEL[2]), .A_N(A[0]), .B(A[1]), .C(EN) );
    sky130_fd_sc_hd__and3_4     AND3 ( .X(SEL[3]), .A(A[1]),   .B(A[0]), .C(EN) );
    
endmodule

module DEC3x8 (
    input   wire        EN,
    input   wire [2:0]  A,
    output  wire [7:0]  SEL
);
    sky130_fd_sc_hd__nor4b_2    AND0 ( .Y(SEL[0])  , .A(A[0]), .B(A[1])  , .C(A[2]), .D_N(EN) ); // 000
    sky130_fd_sc_hd__and4bb_2   AND1 ( .X(SEL[1])  , .A_N(A[2]), .B_N(A[1]), .C(A[0])  , .D(EN) ); // 001
    sky130_fd_sc_hd__and4bb_2   AND2 ( .X(SEL[2])  , .A_N(A[2]), .B_N(A[0]), .C(A[1])  , .D(EN) ); // 010
    sky130_fd_sc_hd__and4b_2    AND3 ( .X(SEL[3])  , .A_N(A[2]), .B(A[1]), .C(A[0])  , .D(EN) );   // 011
    sky130_fd_sc_hd__and4bb_2   AND4 ( .X(SEL[4])  , .A_N(A[0]), .B_N(A[1]), .C(A[2])  , .D(EN) ); // 100
    sky130_fd_sc_hd__and4b_2    AND5 ( .X(SEL[5])  , .A_N(A[1]), .B(A[0]), .C(A[2])  , .D(EN) );   // 101
    sky130_fd_sc_hd__and4b_2    AND6 ( .X(SEL[6])  , .A_N(A[0]), .B(A[1]), .C(A[2])  , .D(EN) );   // 110
    sky130_fd_sc_hd__and4_2     AND7 ( .X(SEL[7])  , .A(A[0]), .B(A[1]), .C(A[2])  , .D(EN) ); // 111
endmodule

module DEC5x32 (
    input   [4:0]   A,
    output  [31:0]   SEL
);
	wire [3:0] EN;
	DEC3x8 D0 ( .A(A[2:0]), .SEL(SEL[7:0]), .EN(EN[0]) );
	DEC3x8 D1 ( .A(A[2:0]), .SEL(SEL[15:8]), .EN(EN[1]) );
	DEC3x8 D2 ( .A(A[2:0]), .SEL(SEL[23:16]), .EN(EN[2]) );
	DEC3x8 D3 ( .A(A[2:0]), .SEL(SEL[31:24]), .EN(EN[3]) );

	DEC2x4 D ( .A(A[4:3]), .SEL(EN), .EN(1'b1) );
endmodule

module dffram_2R1W_wrapper_param
#(
  parameter DATA_WIDTH = 32,
  parameter WRITE_WIDTH = 8,
  parameter ADDR_WIDTH = 7,
  parameter DEPTH = 128
)(
  input CLK,
  input [((DATA_WIDTH + WRITE_WIDTH - 1)/WRITE_WIDTH) - 1 : 0] WE,
  input [ADDR_WIDTH - 1 : 0] RW,
  input [DATA_WIDTH - 1 : 0] DW,
  input [ADDR_WIDTH - 1 : 0] RA,
  output [DATA_WIDTH - 1 : 0] DA
);
  
  /*
  // synopsys translate_off
  reg [DATA_WIDTH - 1 : 0] rdata_reg;
  
  reg [DATA_WIDTH - 1 : 0] mem [DEPTH - 1 : 0];
  
  always @(posedge CLK) begin
    if (wen) begin
      mem[wadr] <= wdata; // write port
    end
    if (ren) begin
      rdata_reg <= mem[radr]; // read port
    end
  end
  assign rdata = rdata_reg;
  
  // synopsys translate_on
  */
  
  genvar x, y; 
  
  generate

    if (DEPTH == 8) begin
      wire [DATA_WIDTH - 1 : 0] rdata_w;
      reg  [ADDR_WIDTH - 1 : 0] radr_r;

      always @ (posedge CLK) begin
              radr_r <= RA;
      end

      for (x = 0; x < DATA_WIDTH/WRITE_WIDTH; x = x + 1) begin: width_macro
        dffrf_2R1W_param #(
                .DATA_WIDTH(WRITE_WIDTH)
                )sram (
                .CLK(CLK),
                .WE(WE[x]),
                .RA(RA[ 2:0 ]),
                .RB(),
                .RW(RW[ 2:0 ]),
                .DW(DW[WRITE_WIDTH*(x+1)-1 : WRITE_WIDTH*x]),
                .DA(rdata_w[WRITE_WIDTH*(x+1)-1 : WRITE_WIDTH*x]),
                .DB()
        );
      end

      assign DA = rdata_w;

    end else begin
      wire [DATA_WIDTH - 1 : 0] rdata_w [DEPTH/8 - 1 : 0];
      reg  [ADDR_WIDTH - 1 : 0] radr_r;

      always @ (posedge CLK) begin
              radr_r <= RA;
      end

      for (x = 0; x < DATA_WIDTH/WRITE_WIDTH; x = x + 1) begin: width_macro
              for (y = 0; y < DEPTH/8; y = y + 1) begin: depth_macro
                      dffrf_2R1W_param #(
                              .DATA_WIDTH(WRITE_WIDTH)
                              )sram (
                              .CLK(CLK),
                              .WE(WE[x] & (RW[ ADDR_WIDTH-1 : 3 ] == y)),
                              .RA(RA[ 2:0 ]),
                              .RB(),
                              .RW(RW[ 2:0 ]),
                              .DW(DW[WRITE_WIDTH*(x+1)-1 : WRITE_WIDTH*x]),
                              .DA(rdata_w[y][WRITE_WIDTH*(x+1)-1 : WRITE_WIDTH*x]),
                              .DB()
                      );
              end
      end

      assign DA = rdata_w[radr_r[ADDR_WIDTH - 1 : 3]];

    end
  endgenerate
  
endmodule
module cc_dir_ext( // @[anonymous source 2:2]
  input  [1:0]  RW0_addr, // @[anonymous source 3:4]
  input         RW0_clk, // @[anonymous source 4:4]
  input  [71:0] RW0_wdata, // @[anonymous source 5:4]
  output [71:0] RW0_rdata, // @[anonymous source 6:4]
  input         RW0_en, // @[anonymous source 7:4]
  input         RW0_wmode, // @[anonymous source 8:4]
  input  [7:0]  RW0_wmask // @[anonymous source 9:4]
);
  wire [1:0] mem_0_0_RW0_addr; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 11:4]
  wire [8:0] mem_0_0_RW0_wdata; // @[anonymous source 11:4]
  wire [8:0] mem_0_0_RW0_rdata; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 11:4]
  wire [1:0] mem_0_1_RW0_addr; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_clk; // @[anonymous source 12:4]
  wire [8:0] mem_0_1_RW0_wdata; // @[anonymous source 12:4]
  wire [8:0] mem_0_1_RW0_rdata; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_en; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_wmode; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_wmask; // @[anonymous source 12:4]
  wire [1:0] mem_0_2_RW0_addr; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_clk; // @[anonymous source 13:4]
  wire [8:0] mem_0_2_RW0_wdata; // @[anonymous source 13:4]
  wire [8:0] mem_0_2_RW0_rdata; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_en; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_wmode; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_wmask; // @[anonymous source 13:4]
  wire [1:0] mem_0_3_RW0_addr; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_clk; // @[anonymous source 14:4]
  wire [8:0] mem_0_3_RW0_wdata; // @[anonymous source 14:4]
  wire [8:0] mem_0_3_RW0_rdata; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_en; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_wmode; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_wmask; // @[anonymous source 14:4]
  wire [1:0] mem_0_4_RW0_addr; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_clk; // @[anonymous source 15:4]
  wire [8:0] mem_0_4_RW0_wdata; // @[anonymous source 15:4]
  wire [8:0] mem_0_4_RW0_rdata; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_en; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_wmode; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_wmask; // @[anonymous source 15:4]
  wire [1:0] mem_0_5_RW0_addr; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_clk; // @[anonymous source 16:4]
  wire [8:0] mem_0_5_RW0_wdata; // @[anonymous source 16:4]
  wire [8:0] mem_0_5_RW0_rdata; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_en; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_wmode; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_wmask; // @[anonymous source 16:4]
  wire [1:0] mem_0_6_RW0_addr; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_clk; // @[anonymous source 17:4]
  wire [8:0] mem_0_6_RW0_wdata; // @[anonymous source 17:4]
  wire [8:0] mem_0_6_RW0_rdata; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_en; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_wmode; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_wmask; // @[anonymous source 17:4]
  wire [1:0] mem_0_7_RW0_addr; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_clk; // @[anonymous source 18:4]
  wire [8:0] mem_0_7_RW0_wdata; // @[anonymous source 18:4]
  wire [8:0] mem_0_7_RW0_rdata; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_en; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_wmode; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_wmask; // @[anonymous source 18:4]
  wire [8:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 21:4]
  wire [8:0] RW0_rdata_0_1 = mem_0_1_RW0_rdata; // @[anonymous source 28:4]
  wire [8:0] RW0_rdata_0_2 = mem_0_2_RW0_rdata; // @[anonymous source 35:4]
  wire [8:0] RW0_rdata_0_3 = mem_0_3_RW0_rdata; // @[anonymous source 42:4]
  wire [8:0] RW0_rdata_0_4 = mem_0_4_RW0_rdata; // @[anonymous source 49:4]
  wire [8:0] RW0_rdata_0_5 = mem_0_5_RW0_rdata; // @[anonymous source 56:4]
  wire [8:0] RW0_rdata_0_6 = mem_0_6_RW0_rdata; // @[anonymous source 63:4]
  wire [8:0] RW0_rdata_0_7 = mem_0_7_RW0_rdata; // @[anonymous source 70:4]
  wire [17:0] _GEN_0 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [26:0] _GEN_1 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [35:0] _GEN_2 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [44:0] _GEN_3 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [53:0] _GEN_4 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [62:0] _GEN_5 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [71:0] RW0_rdata_0 = {RW0_rdata_0_7,RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,
    RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [17:0] _GEN_6 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [26:0] _GEN_7 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [35:0] _GEN_8 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [44:0] _GEN_9 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [53:0] _GEN_10 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [62:0] _GEN_11 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 75:4]
  split_cc_dir_ext mem_0_0 ( // @[anonymous source 11:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  split_cc_dir_ext mem_0_1 ( // @[anonymous source 12:4]
    .RW0_addr(mem_0_1_RW0_addr),
    .RW0_clk(mem_0_1_RW0_clk),
    .RW0_wdata(mem_0_1_RW0_wdata),
    .RW0_rdata(mem_0_1_RW0_rdata),
    .RW0_en(mem_0_1_RW0_en),
    .RW0_wmode(mem_0_1_RW0_wmode),
    .RW0_wmask(mem_0_1_RW0_wmask)
  );
  split_cc_dir_ext mem_0_2 ( // @[anonymous source 13:4]
    .RW0_addr(mem_0_2_RW0_addr),
    .RW0_clk(mem_0_2_RW0_clk),
    .RW0_wdata(mem_0_2_RW0_wdata),
    .RW0_rdata(mem_0_2_RW0_rdata),
    .RW0_en(mem_0_2_RW0_en),
    .RW0_wmode(mem_0_2_RW0_wmode),
    .RW0_wmask(mem_0_2_RW0_wmask)
  );
  split_cc_dir_ext mem_0_3 ( // @[anonymous source 14:4]
    .RW0_addr(mem_0_3_RW0_addr),
    .RW0_clk(mem_0_3_RW0_clk),
    .RW0_wdata(mem_0_3_RW0_wdata),
    .RW0_rdata(mem_0_3_RW0_rdata),
    .RW0_en(mem_0_3_RW0_en),
    .RW0_wmode(mem_0_3_RW0_wmode),
    .RW0_wmask(mem_0_3_RW0_wmask)
  );
  split_cc_dir_ext mem_0_4 ( // @[anonymous source 15:4]
    .RW0_addr(mem_0_4_RW0_addr),
    .RW0_clk(mem_0_4_RW0_clk),
    .RW0_wdata(mem_0_4_RW0_wdata),
    .RW0_rdata(mem_0_4_RW0_rdata),
    .RW0_en(mem_0_4_RW0_en),
    .RW0_wmode(mem_0_4_RW0_wmode),
    .RW0_wmask(mem_0_4_RW0_wmask)
  );
  split_cc_dir_ext mem_0_5 ( // @[anonymous source 16:4]
    .RW0_addr(mem_0_5_RW0_addr),
    .RW0_clk(mem_0_5_RW0_clk),
    .RW0_wdata(mem_0_5_RW0_wdata),
    .RW0_rdata(mem_0_5_RW0_rdata),
    .RW0_en(mem_0_5_RW0_en),
    .RW0_wmode(mem_0_5_RW0_wmode),
    .RW0_wmask(mem_0_5_RW0_wmask)
  );
  split_cc_dir_ext mem_0_6 ( // @[anonymous source 17:4]
    .RW0_addr(mem_0_6_RW0_addr),
    .RW0_clk(mem_0_6_RW0_clk),
    .RW0_wdata(mem_0_6_RW0_wdata),
    .RW0_rdata(mem_0_6_RW0_rdata),
    .RW0_en(mem_0_6_RW0_en),
    .RW0_wmode(mem_0_6_RW0_wmode),
    .RW0_wmask(mem_0_6_RW0_wmask)
  );
  split_cc_dir_ext mem_0_7 ( // @[anonymous source 18:4]
    .RW0_addr(mem_0_7_RW0_addr),
    .RW0_clk(mem_0_7_RW0_clk),
    .RW0_wdata(mem_0_7_RW0_wdata),
    .RW0_rdata(mem_0_7_RW0_rdata),
    .RW0_en(mem_0_7_RW0_en),
    .RW0_wmode(mem_0_7_RW0_wmode),
    .RW0_wmask(mem_0_7_RW0_wmask)
  );
  assign RW0_rdata = {RW0_rdata_0_7,_GEN_5}; // @[anonymous source 75:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 20:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 19:4]
  assign mem_0_0_RW0_wdata = RW0_wdata[8:0]; // @[anonymous source 22:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 25:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 24:4]
  assign mem_0_0_RW0_wmask = RW0_wmask[0]; // @[anonymous source 23:4]
  assign mem_0_1_RW0_addr = RW0_addr; // @[anonymous source 27:4]
  assign mem_0_1_RW0_clk = RW0_clk; // @[anonymous source 26:4]
  assign mem_0_1_RW0_wdata = RW0_wdata[17:9]; // @[anonymous source 29:4]
  assign mem_0_1_RW0_en = RW0_en; // @[anonymous source 32:4]
  assign mem_0_1_RW0_wmode = RW0_wmode; // @[anonymous source 31:4]
  assign mem_0_1_RW0_wmask = RW0_wmask[1]; // @[anonymous source 30:4]
  assign mem_0_2_RW0_addr = RW0_addr; // @[anonymous source 34:4]
  assign mem_0_2_RW0_clk = RW0_clk; // @[anonymous source 33:4]
  assign mem_0_2_RW0_wdata = RW0_wdata[26:18]; // @[anonymous source 36:4]
  assign mem_0_2_RW0_en = RW0_en; // @[anonymous source 39:4]
  assign mem_0_2_RW0_wmode = RW0_wmode; // @[anonymous source 38:4]
  assign mem_0_2_RW0_wmask = RW0_wmask[2]; // @[anonymous source 37:4]
  assign mem_0_3_RW0_addr = RW0_addr; // @[anonymous source 41:4]
  assign mem_0_3_RW0_clk = RW0_clk; // @[anonymous source 40:4]
  assign mem_0_3_RW0_wdata = RW0_wdata[35:27]; // @[anonymous source 43:4]
  assign mem_0_3_RW0_en = RW0_en; // @[anonymous source 46:4]
  assign mem_0_3_RW0_wmode = RW0_wmode; // @[anonymous source 45:4]
  assign mem_0_3_RW0_wmask = RW0_wmask[3]; // @[anonymous source 44:4]
  assign mem_0_4_RW0_addr = RW0_addr; // @[anonymous source 48:4]
  assign mem_0_4_RW0_clk = RW0_clk; // @[anonymous source 47:4]
  assign mem_0_4_RW0_wdata = RW0_wdata[44:36]; // @[anonymous source 50:4]
  assign mem_0_4_RW0_en = RW0_en; // @[anonymous source 53:4]
  assign mem_0_4_RW0_wmode = RW0_wmode; // @[anonymous source 52:4]
  assign mem_0_4_RW0_wmask = RW0_wmask[4]; // @[anonymous source 51:4]
  assign mem_0_5_RW0_addr = RW0_addr; // @[anonymous source 55:4]
  assign mem_0_5_RW0_clk = RW0_clk; // @[anonymous source 54:4]
  assign mem_0_5_RW0_wdata = RW0_wdata[53:45]; // @[anonymous source 57:4]
  assign mem_0_5_RW0_en = RW0_en; // @[anonymous source 60:4]
  assign mem_0_5_RW0_wmode = RW0_wmode; // @[anonymous source 59:4]
  assign mem_0_5_RW0_wmask = RW0_wmask[5]; // @[anonymous source 58:4]
  assign mem_0_6_RW0_addr = RW0_addr; // @[anonymous source 62:4]
  assign mem_0_6_RW0_clk = RW0_clk; // @[anonymous source 61:4]
  assign mem_0_6_RW0_wdata = RW0_wdata[62:54]; // @[anonymous source 64:4]
  assign mem_0_6_RW0_en = RW0_en; // @[anonymous source 67:4]
  assign mem_0_6_RW0_wmode = RW0_wmode; // @[anonymous source 66:4]
  assign mem_0_6_RW0_wmask = RW0_wmask[6]; // @[anonymous source 65:4]
  assign mem_0_7_RW0_addr = RW0_addr; // @[anonymous source 69:4]
  assign mem_0_7_RW0_clk = RW0_clk; // @[anonymous source 68:4]
  assign mem_0_7_RW0_wdata = RW0_wdata[71:63]; // @[anonymous source 71:4]
  assign mem_0_7_RW0_en = RW0_en; // @[anonymous source 74:4]
  assign mem_0_7_RW0_wmode = RW0_wmode; // @[anonymous source 73:4]
  assign mem_0_7_RW0_wmask = RW0_wmask[7]; // @[anonymous source 72:4]
endmodule
module cc_banks_0_ext( // @[anonymous source 78:2]
  input  [5:0]  RW0_addr, // @[anonymous source 79:4]
  input         RW0_clk, // @[anonymous source 80:4]
  input  [63:0] RW0_wdata, // @[anonymous source 81:4]
  output [63:0] RW0_rdata, // @[anonymous source 82:4]
  input         RW0_en, // @[anonymous source 83:4]
  input         RW0_wmode // @[anonymous source 84:4]
);
  wire [5:0] mem_0_0_RW0_addr; // @[anonymous source 86:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 86:4]
  wire [63:0] mem_0_0_RW0_wdata; // @[anonymous source 86:4]
  wire [63:0] mem_0_0_RW0_rdata; // @[anonymous source 86:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 86:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 86:4]
  wire [63:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 89:4]
  wire [63:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 89:4]
  split_cc_banks_0_ext mem_0_0 ( // @[anonymous source 86:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 89:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 88:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 87:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 90:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 92:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 91:4]
endmodule
module tag_array_ext( // @[anonymous source 96:2]
  input  [1:0]  RW0_addr, // @[anonymous source 97:4]
  input         RW0_clk, // @[anonymous source 98:4]
  input  [25:0] RW0_wdata, // @[anonymous source 99:4]
  output [25:0] RW0_rdata, // @[anonymous source 100:4]
  input         RW0_en, // @[anonymous source 101:4]
  input         RW0_wmode, // @[anonymous source 102:4]
  input         RW0_wmask // @[anonymous source 103:4]
);
  wire [1:0] mem_0_0_RW0_addr; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 105:4]
  wire [25:0] mem_0_0_RW0_wdata; // @[anonymous source 105:4]
  wire [25:0] mem_0_0_RW0_rdata; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 105:4]
  wire [25:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 108:4]
  wire [25:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 108:4]
  split_tag_array_ext mem_0_0 ( // @[anonymous source 105:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 108:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 107:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 106:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 109:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 112:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 111:4]
  assign mem_0_0_RW0_wmask = RW0_wmask; // @[anonymous source 110:4]
endmodule
module dataArrayWay_0_ext( // @[anonymous source 116:2]
  input  [4:0]  RW0_addr, // @[anonymous source 117:4]
  input         RW0_clk, // @[anonymous source 118:4]
  input  [63:0] RW0_wdata, // @[anonymous source 119:4]
  output [63:0] RW0_rdata, // @[anonymous source 120:4]
  input         RW0_en, // @[anonymous source 121:4]
  input         RW0_wmode // @[anonymous source 122:4]
);
  wire [4:0] mem_0_0_RW0_addr; // @[anonymous source 124:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 124:4]
  wire [63:0] mem_0_0_RW0_wdata; // @[anonymous source 124:4]
  wire [63:0] mem_0_0_RW0_rdata; // @[anonymous source 124:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 124:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 124:4]
  wire [63:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 127:4]
  wire [63:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 127:4]
  split_dataArrayWay_0_ext mem_0_0 ( // @[anonymous source 124:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 127:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 126:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 125:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 128:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 130:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 129:4]
endmodule
module tag_array_0_ext( // @[anonymous source 134:2]
  input  [2:0]  RW0_addr, // @[anonymous source 135:4]
  input         RW0_clk, // @[anonymous source 136:4]
  input  [49:0] RW0_wdata, // @[anonymous source 137:4]
  output [49:0] RW0_rdata, // @[anonymous source 138:4]
  input         RW0_en, // @[anonymous source 139:4]
  input         RW0_wmode, // @[anonymous source 140:4]
  input  [1:0]  RW0_wmask // @[anonymous source 141:4]
);
  wire [2:0] mem_0_0_RW0_addr; // @[anonymous source 143:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 143:4]
  wire [24:0] mem_0_0_RW0_wdata; // @[anonymous source 143:4]
  wire [24:0] mem_0_0_RW0_rdata; // @[anonymous source 143:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 143:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 143:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 143:4]
  wire [2:0] mem_0_1_RW0_addr; // @[anonymous source 144:4]
  wire  mem_0_1_RW0_clk; // @[anonymous source 144:4]
  wire [24:0] mem_0_1_RW0_wdata; // @[anonymous source 144:4]
  wire [24:0] mem_0_1_RW0_rdata; // @[anonymous source 144:4]
  wire  mem_0_1_RW0_en; // @[anonymous source 144:4]
  wire  mem_0_1_RW0_wmode; // @[anonymous source 144:4]
  wire  mem_0_1_RW0_wmask; // @[anonymous source 144:4]
  wire [24:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 147:4]
  wire [24:0] RW0_rdata_0_1 = mem_0_1_RW0_rdata; // @[anonymous source 154:4]
  wire [49:0] RW0_rdata_0 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 159:4]
  split_tag_array_0_ext mem_0_0 ( // @[anonymous source 143:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  split_tag_array_0_ext mem_0_1 ( // @[anonymous source 144:4]
    .RW0_addr(mem_0_1_RW0_addr),
    .RW0_clk(mem_0_1_RW0_clk),
    .RW0_wdata(mem_0_1_RW0_wdata),
    .RW0_rdata(mem_0_1_RW0_rdata),
    .RW0_en(mem_0_1_RW0_en),
    .RW0_wmode(mem_0_1_RW0_wmode),
    .RW0_wmask(mem_0_1_RW0_wmask)
  );
  assign RW0_rdata = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 159:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 146:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 145:4]
  assign mem_0_0_RW0_wdata = RW0_wdata[24:0]; // @[anonymous source 148:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 151:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 150:4]
  assign mem_0_0_RW0_wmask = RW0_wmask[0]; // @[anonymous source 149:4]
  assign mem_0_1_RW0_addr = RW0_addr; // @[anonymous source 153:4]
  assign mem_0_1_RW0_clk = RW0_clk; // @[anonymous source 152:4]
  assign mem_0_1_RW0_wdata = RW0_wdata[49:25]; // @[anonymous source 155:4]
  assign mem_0_1_RW0_en = RW0_en; // @[anonymous source 158:4]
  assign mem_0_1_RW0_wmode = RW0_wmode; // @[anonymous source 157:4]
  assign mem_0_1_RW0_wmask = RW0_wmask[1]; // @[anonymous source 156:4]
endmodule
module array_0_0_ext( // @[anonymous source 162:2]
  input  [5:0]  W0_addr, // @[anonymous source 163:4]
  input         W0_clk, // @[anonymous source 164:4]
  input  [63:0] W0_data, // @[anonymous source 165:4]
  input         W0_en, // @[anonymous source 166:4]
  input         W0_mask, // @[anonymous source 167:4]
  input  [5:0]  R0_addr, // @[anonymous source 168:4]
  input         R0_clk, // @[anonymous source 169:4]
  output [63:0] R0_data, // @[anonymous source 170:4]
  input         R0_en // @[anonymous source 171:4]
);
dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(64),
	.WRITE_WIDTH(32),
	.ADDR_WIDTH(6),
	.DEPTH(64)
) dffram (
	.CLK(R0_clk),
	.WE({2{W0_en & W0_mask}}),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);
endmodule
module tag_array_1_ext( // @[anonymous source 186:2]
  input          W0_addr, // @[anonymous source 187:4]
  input          W0_clk, // @[anonymous source 188:4]
  input  [103:0] W0_data, // @[anonymous source 189:4]
  input          W0_en, // @[anonymous source 190:4]
  input  [3:0]   W0_mask, // @[anonymous source 191:4]
  input          R0_addr, // @[anonymous source 192:4]
  input          R0_clk, // @[anonymous source 193:4]
  output [103:0] R0_data, // @[anonymous source 194:4]
  input          R0_en // @[anonymous source 195:4]
);
  wire  mem_0_0_W0_addr; // @[anonymous source 197:4]
  wire  mem_0_0_W0_clk; // @[anonymous source 197:4]
  wire [25:0] mem_0_0_W0_data; // @[anonymous source 197:4]
  wire  mem_0_0_W0_en; // @[anonymous source 197:4]
  wire  mem_0_0_W0_mask; // @[anonymous source 197:4]
  wire  mem_0_0_R0_addr; // @[anonymous source 197:4]
  wire  mem_0_0_R0_clk; // @[anonymous source 197:4]
  wire [25:0] mem_0_0_R0_data; // @[anonymous source 197:4]
  wire  mem_0_0_R0_en; // @[anonymous source 197:4]
  wire  mem_0_1_W0_addr; // @[anonymous source 198:4]
  wire  mem_0_1_W0_clk; // @[anonymous source 198:4]
  wire [25:0] mem_0_1_W0_data; // @[anonymous source 198:4]
  wire  mem_0_1_W0_en; // @[anonymous source 198:4]
  wire  mem_0_1_W0_mask; // @[anonymous source 198:4]
  wire  mem_0_1_R0_addr; // @[anonymous source 198:4]
  wire  mem_0_1_R0_clk; // @[anonymous source 198:4]
  wire [25:0] mem_0_1_R0_data; // @[anonymous source 198:4]
  wire  mem_0_1_R0_en; // @[anonymous source 198:4]
  wire  mem_0_2_W0_addr; // @[anonymous source 199:4]
  wire  mem_0_2_W0_clk; // @[anonymous source 199:4]
  wire [25:0] mem_0_2_W0_data; // @[anonymous source 199:4]
  wire  mem_0_2_W0_en; // @[anonymous source 199:4]
  wire  mem_0_2_W0_mask; // @[anonymous source 199:4]
  wire  mem_0_2_R0_addr; // @[anonymous source 199:4]
  wire  mem_0_2_R0_clk; // @[anonymous source 199:4]
  wire [25:0] mem_0_2_R0_data; // @[anonymous source 199:4]
  wire  mem_0_2_R0_en; // @[anonymous source 199:4]
  wire  mem_0_3_W0_addr; // @[anonymous source 200:4]
  wire  mem_0_3_W0_clk; // @[anonymous source 200:4]
  wire [25:0] mem_0_3_W0_data; // @[anonymous source 200:4]
  wire  mem_0_3_W0_en; // @[anonymous source 200:4]
  wire  mem_0_3_W0_mask; // @[anonymous source 200:4]
  wire  mem_0_3_R0_addr; // @[anonymous source 200:4]
  wire  mem_0_3_R0_clk; // @[anonymous source 200:4]
  wire [25:0] mem_0_3_R0_data; // @[anonymous source 200:4]
  wire  mem_0_3_R0_en; // @[anonymous source 200:4]
  wire [25:0] R0_data_0_0 = mem_0_0_R0_data; // @[anonymous source 223:4]
  wire [25:0] R0_data_0_1 = mem_0_1_R0_data; // @[anonymous source 227:4]
  wire [25:0] R0_data_0_2 = mem_0_2_R0_data; // @[anonymous source 231:4]
  wire [25:0] R0_data_0_3 = mem_0_3_R0_data; // @[anonymous source 235:4]
  wire [51:0] _GEN_0 = {R0_data_0_1,R0_data_0_0}; // @[anonymous source 237:4]
  wire [77:0] _GEN_1 = {R0_data_0_2,R0_data_0_1,R0_data_0_0}; // @[anonymous source 237:4]
  wire [103:0] R0_data_0 = {R0_data_0_3,R0_data_0_2,R0_data_0_1,R0_data_0_0}; // @[anonymous source 237:4]
  wire [51:0] _GEN_2 = {R0_data_0_1,R0_data_0_0}; // @[anonymous source 237:4]
  wire [77:0] _GEN_3 = {R0_data_0_2,R0_data_0_1,R0_data_0_0}; // @[anonymous source 237:4]
  split_tag_array_1_ext mem_0_0 ( // @[anonymous source 197:4]
    .W0_addr(mem_0_0_W0_addr),
    .W0_clk(mem_0_0_W0_clk),
    .W0_data(mem_0_0_W0_data),
    .W0_en(mem_0_0_W0_en),
    .W0_mask(mem_0_0_W0_mask),
    .R0_addr(mem_0_0_R0_addr),
    .R0_clk(mem_0_0_R0_clk),
    .R0_data(mem_0_0_R0_data),
    .R0_en(mem_0_0_R0_en)
  );
  split_tag_array_1_ext mem_0_1 ( // @[anonymous source 198:4]
    .W0_addr(mem_0_1_W0_addr),
    .W0_clk(mem_0_1_W0_clk),
    .W0_data(mem_0_1_W0_data),
    .W0_en(mem_0_1_W0_en),
    .W0_mask(mem_0_1_W0_mask),
    .R0_addr(mem_0_1_R0_addr),
    .R0_clk(mem_0_1_R0_clk),
    .R0_data(mem_0_1_R0_data),
    .R0_en(mem_0_1_R0_en)
  );
  split_tag_array_1_ext mem_0_2 ( // @[anonymous source 199:4]
    .W0_addr(mem_0_2_W0_addr),
    .W0_clk(mem_0_2_W0_clk),
    .W0_data(mem_0_2_W0_data),
    .W0_en(mem_0_2_W0_en),
    .W0_mask(mem_0_2_W0_mask),
    .R0_addr(mem_0_2_R0_addr),
    .R0_clk(mem_0_2_R0_clk),
    .R0_data(mem_0_2_R0_data),
    .R0_en(mem_0_2_R0_en)
  );
  split_tag_array_1_ext mem_0_3 ( // @[anonymous source 200:4]
    .W0_addr(mem_0_3_W0_addr),
    .W0_clk(mem_0_3_W0_clk),
    .W0_data(mem_0_3_W0_data),
    .W0_en(mem_0_3_W0_en),
    .W0_mask(mem_0_3_W0_mask),
    .R0_addr(mem_0_3_R0_addr),
    .R0_clk(mem_0_3_R0_clk),
    .R0_data(mem_0_3_R0_data),
    .R0_en(mem_0_3_R0_en)
  );
  assign R0_data = {R0_data_0_3,_GEN_1}; // @[anonymous source 237:4]
  assign mem_0_0_W0_addr = W0_addr; // @[anonymous source 202:4]
  assign mem_0_0_W0_clk = W0_clk; // @[anonymous source 201:4]
  assign mem_0_0_W0_data = W0_data[25:0]; // @[anonymous source 203:4]
  assign mem_0_0_W0_en = W0_en; // @[anonymous source 205:4]
  assign mem_0_0_W0_mask = W0_mask[0]; // @[anonymous source 204:4]
  assign mem_0_0_R0_addr = R0_addr; // @[anonymous source 222:4]
  assign mem_0_0_R0_clk = R0_clk; // @[anonymous source 221:4]
  assign mem_0_0_R0_en = R0_en; // @[anonymous source 224:4]
  assign mem_0_1_W0_addr = W0_addr; // @[anonymous source 207:4]
  assign mem_0_1_W0_clk = W0_clk; // @[anonymous source 206:4]
  assign mem_0_1_W0_data = W0_data[51:26]; // @[anonymous source 208:4]
  assign mem_0_1_W0_en = W0_en; // @[anonymous source 210:4]
  assign mem_0_1_W0_mask = W0_mask[1]; // @[anonymous source 209:4]
  assign mem_0_1_R0_addr = R0_addr; // @[anonymous source 226:4]
  assign mem_0_1_R0_clk = R0_clk; // @[anonymous source 225:4]
  assign mem_0_1_R0_en = R0_en; // @[anonymous source 228:4]
  assign mem_0_2_W0_addr = W0_addr; // @[anonymous source 212:4]
  assign mem_0_2_W0_clk = W0_clk; // @[anonymous source 211:4]
  assign mem_0_2_W0_data = W0_data[77:52]; // @[anonymous source 213:4]
  assign mem_0_2_W0_en = W0_en; // @[anonymous source 215:4]
  assign mem_0_2_W0_mask = W0_mask[2]; // @[anonymous source 214:4]
  assign mem_0_2_R0_addr = R0_addr; // @[anonymous source 230:4]
  assign mem_0_2_R0_clk = R0_clk; // @[anonymous source 229:4]
  assign mem_0_2_R0_en = R0_en; // @[anonymous source 232:4]
  assign mem_0_3_W0_addr = W0_addr; // @[anonymous source 217:4]
  assign mem_0_3_W0_clk = W0_clk; // @[anonymous source 216:4]
  assign mem_0_3_W0_data = W0_data[103:78]; // @[anonymous source 218:4]
  assign mem_0_3_W0_en = W0_en; // @[anonymous source 220:4]
  assign mem_0_3_W0_mask = W0_mask[3]; // @[anonymous source 219:4]
  assign mem_0_3_R0_addr = R0_addr; // @[anonymous source 234:4]
  assign mem_0_3_R0_clk = R0_clk; // @[anonymous source 233:4]
  assign mem_0_3_R0_en = R0_en; // @[anonymous source 236:4]
endmodule
module dataArrayWay_0_0_ext( // @[anonymous source 240:2]
  input          RW0_addr, // @[anonymous source 241:4]
  input          RW0_clk, // @[anonymous source 242:4]
  input  [511:0] RW0_wdata, // @[anonymous source 243:4]
  output [511:0] RW0_rdata, // @[anonymous source 244:4]
  input          RW0_en, // @[anonymous source 245:4]
  input          RW0_wmode // @[anonymous source 246:4]
);
  wire  mem_0_0_RW0_addr; // @[anonymous source 248:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 248:4]
  wire [511:0] mem_0_0_RW0_wdata; // @[anonymous source 248:4]
  wire [511:0] mem_0_0_RW0_rdata; // @[anonymous source 248:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 248:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 248:4]
  wire [511:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 251:4]
  wire [511:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 251:4]
  split_dataArrayWay_0_0_ext mem_0_0 ( // @[anonymous source 248:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 251:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 250:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 249:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 252:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 254:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 253:4]
endmodule
module tag_array_2_ext( // @[anonymous source 258:2]
  input  [2:0]  W0_addr, // @[anonymous source 259:4]
  input         W0_clk, // @[anonymous source 260:4]
  input  [45:0] W0_data, // @[anonymous source 261:4]
  input         W0_en, // @[anonymous source 262:4]
  input  [1:0]  W0_mask, // @[anonymous source 263:4]
  input  [2:0]  R0_addr, // @[anonymous source 264:4]
  input         R0_clk, // @[anonymous source 265:4]
  output [45:0] R0_data, // @[anonymous source 266:4]
  input         R0_en // @[anonymous source 267:4]
);
dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(46),
	.WRITE_WIDTH(23),
	.ADDR_WIDTH(3),
	.DEPTH(8)
) dffram (
	.CLK(R0_clk),
	.WE({2{W0_en}} & W0_mask),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);
endmodule
module hi_us_ext( // @[anonymous source 292:2]
  input  [5:0] W0_addr, // @[anonymous source 293:4]
  input        W0_clk, // @[anonymous source 294:4]
  input  [3:0] W0_data, // @[anonymous source 295:4]
  input        W0_en, // @[anonymous source 296:4]
  input  [3:0] W0_mask, // @[anonymous source 297:4]
  input  [5:0] R0_addr, // @[anonymous source 298:4]
  input        R0_clk, // @[anonymous source 299:4]
  output [3:0] R0_data, // @[anonymous source 300:4]
  input        R0_en // @[anonymous source 301:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(4),
	.WRITE_WIDTH(1),
	.ADDR_WIDTH(6),
	.DEPTH(64)
) dffram (
	.CLK(R0_clk),
	.WE({4{W0_en}} & W0_mask),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module table_ext( // @[anonymous source 346:2]
  input  [5:0]  W0_addr, // @[anonymous source 347:4]
  input         W0_clk, // @[anonymous source 348:4]
  input  [47:0] W0_data, // @[anonymous source 349:4]
  input         W0_en, // @[anonymous source 350:4]
  input  [3:0]  W0_mask, // @[anonymous source 351:4]
  input  [5:0]  R0_addr, // @[anonymous source 352:4]
  input         R0_clk, // @[anonymous source 353:4]
  output [47:0] R0_data, // @[anonymous source 354:4]
  input         R0_en // @[anonymous source 355:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(48),
	.WRITE_WIDTH(12),
	.ADDR_WIDTH(6),
	.DEPTH(64)
) dffram (
	.CLK(R0_clk),
	.WE({4{W0_en}} & W0_mask),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module meta_0_ext( // @[anonymous source 400:2]
  input  [4:0]   W0_addr, // @[anonymous source 401:4]
  input          W0_clk, // @[anonymous source 402:4]
  input  [131:0] W0_data, // @[anonymous source 403:4]
  input          W0_en, // @[anonymous source 404:4]
  input  [3:0]   W0_mask, // @[anonymous source 405:4]
  input  [4:0]   R0_addr, // @[anonymous source 406:4]
  input          R0_clk, // @[anonymous source 407:4]
  output [131:0] R0_data, // @[anonymous source 408:4]
  input          R0_en // @[anonymous source 409:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(132),
	.WRITE_WIDTH(11),
	.ADDR_WIDTH(5),
	.DEPTH(32)
) dffram (
	.CLK(R0_clk),
	.WE({12{W0_en}} & {{3{W0_mask[3]}}, {3{W0_mask[2]}}, {3{W0_mask[1]}}, {3{W0_mask[0]}}}),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module btb_0_ext( // @[anonymous source 454:2]
  input  [4:0]  W0_addr, // @[anonymous source 455:4]
  input         W0_clk, // @[anonymous source 456:4]
  input  [55:0] W0_data, // @[anonymous source 457:4]
  input         W0_en, // @[anonymous source 458:4]
  input  [3:0]  W0_mask, // @[anonymous source 459:4]
  input  [4:0]  R0_addr, // @[anonymous source 460:4]
  input         R0_clk, // @[anonymous source 461:4]
  output [55:0] R0_data, // @[anonymous source 462:4]
  input         R0_en // @[anonymous source 463:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(56),
	.WRITE_WIDTH(14),
	.ADDR_WIDTH(5),
	.DEPTH(32)
) dffram (
	.CLK(R0_clk),
	.WE({4{W0_en}} & W0_mask),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module ebtb_ext( // @[anonymous source 508:2]
  input  [4:0]  W0_addr, // @[anonymous source 509:4]
  input         W0_clk, // @[anonymous source 510:4]
  input  [39:0] W0_data, // @[anonymous source 511:4]
  input         W0_en, // @[anonymous source 512:4]
  input  [4:0]  R0_addr, // @[anonymous source 513:4]
  input         R0_clk, // @[anonymous source 514:4]
  output [39:0] R0_data, // @[anonymous source 515:4]
  input         R0_en // @[anonymous source 516:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(40),
	.WRITE_WIDTH(20),
	.ADDR_WIDTH(5),
	.DEPTH(32)
) dffram (
	.CLK(R0_clk),
	.WE({2{W0_en}}),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module data_ext( // @[anonymous source 530:2]
  input  [6:0] W0_addr, // @[anonymous source 531:4]
  input        W0_clk, // @[anonymous source 532:4]
  input  [7:0] W0_data, // @[anonymous source 533:4]
  input        W0_en, // @[anonymous source 534:4]
  input  [3:0] W0_mask, // @[anonymous source 535:4]
  input  [6:0] R0_addr, // @[anonymous source 536:4]
  input        R0_clk, // @[anonymous source 537:4]
  output [7:0] R0_data, // @[anonymous source 538:4]
  input        R0_en // @[anonymous source 539:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(8),
	.WRITE_WIDTH(2),
	.ADDR_WIDTH(7),
	.DEPTH(128)
) dffram (
	.CLK(R0_clk),
	.WE({4{W0_en}} & W0_mask),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module meta_ext( // @[anonymous source 584:2]
  input  [3:0]  W0_addr, // @[anonymous source 585:4]
  input         W0_clk, // @[anonymous source 586:4]
  input  [44:0] W0_data, // @[anonymous source 587:4]
  input         W0_en, // @[anonymous source 588:4]
  input  [3:0]  R0_addr, // @[anonymous source 589:4]
  input         R0_clk, // @[anonymous source 590:4]
  output [44:0] R0_data, // @[anonymous source 591:4]
  input         R0_en // @[anonymous source 592:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(45),
	.WRITE_WIDTH(15),
	.ADDR_WIDTH(4),
	.DEPTH(16)
) dffram (
	.CLK(R0_clk),
	.WE({3{W0_en}}),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module ghist_0_ext( // @[anonymous source 606:2]
  input  [3:0]  W0_addr, // @[anonymous source 607:4]
  input         W0_clk, // @[anonymous source 608:4]
  input  [23:0] W0_data, // @[anonymous source 609:4]
  input         W0_en, // @[anonymous source 610:4]
  input  [3:0]  R0_addr, // @[anonymous source 611:4]
  input         R0_clk, // @[anonymous source 612:4]
  output [23:0] R0_data, // @[anonymous source 613:4]
  input         R0_en // @[anonymous source 614:4]
);

dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(24),
	.WRITE_WIDTH(24),
	.ADDR_WIDTH(4),
	.DEPTH(16)
) dffram (
	.CLK(R0_clk),
	.WE(W0_en),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module rob_debug_inst_mem_ext( // @[anonymous source 628:2]
  input  [4:0]  W0_addr, // @[anonymous source 629:4]
  input         W0_clk, // @[anonymous source 630:4]
  input  [31:0] W0_data, // @[anonymous source 631:4]
  input         W0_en, // @[anonymous source 632:4]
  input         W0_mask, // @[anonymous source 633:4]
  input  [4:0]  R0_addr, // @[anonymous source 634:4]
  input         R0_clk, // @[anonymous source 635:4]
  output [31:0] R0_data, // @[anonymous source 636:4]
  input         R0_en // @[anonymous source 637:4]
);


dffram_2R1W_wrapper_param
#(
	.DATA_WIDTH(32),
	.WRITE_WIDTH(32),
	.ADDR_WIDTH(5),
	.DEPTH(32)
) dffram (
	.CLK(R0_clk),
	.WE(W0_en & W0_mask),
	.RW(W0_addr),
	.DW(W0_data),
	.RA(R0_addr),
	.DA(R0_data)
);

endmodule
module l2_tlb_ram_ext( // @[anonymous source 652:2]
  input  [5:0]  RW0_addr, // @[anonymous source 653:4]
  input         RW0_clk, // @[anonymous source 654:4]
  input  [47:0] RW0_wdata, // @[anonymous source 655:4]
  output [47:0] RW0_rdata, // @[anonymous source 656:4]
  input         RW0_en, // @[anonymous source 657:4]
  input         RW0_wmode // @[anonymous source 658:4]
);
  wire [5:0] mem_0_0_RW0_addr; // @[anonymous source 660:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 660:4]
  wire [47:0] mem_0_0_RW0_wdata; // @[anonymous source 660:4]
  wire [47:0] mem_0_0_RW0_rdata; // @[anonymous source 660:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 660:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 660:4]
  wire [47:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 663:4]
  wire [47:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 663:4]
  split_l2_tlb_ram_ext mem_0_0 ( // @[anonymous source 660:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 663:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 662:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 661:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 664:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 666:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 665:4]
endmodule
module mem_ext( // @[anonymous source 670:2]
  input  [9:0]  RW0_addr, // @[anonymous source 671:4]
  input         RW0_clk, // @[anonymous source 672:4]
  input  [63:0] RW0_wdata, // @[anonymous source 673:4]
  output [63:0] RW0_rdata, // @[anonymous source 674:4]
  input         RW0_en, // @[anonymous source 675:4]
  input         RW0_wmode, // @[anonymous source 676:4]
  input  [7:0]  RW0_wmask // @[anonymous source 677:4]
);

dffram_wrapper256
#(
	.DATA_WIDTH(64),
	.ADDR_WIDTH(10),
	.DEPTH(1024)
) dffram (
	.CLK(RW0_clk),
	.WE({8{RW0_wmode}} & RW0_wmask),
	.EN(RW0_en),
	.A(RW0_addr),
	.Di(RW0_wdata),
	.Do(RW0_rdata)
);

endmodule
module split_cc_dir_ext( // @[anonymous source 746:2]
  input  [1:0] RW0_addr, // @[anonymous source 747:4]
  input        RW0_clk, // @[anonymous source 748:4]
  input  [8:0] RW0_wdata, // @[anonymous source 749:4]
  output [8:0] RW0_rdata, // @[anonymous source 750:4]
  input        RW0_en, // @[anonymous source 751:4]
  input        RW0_wmode, // @[anonymous source 752:4]
  input        RW0_wmask // @[anonymous source 753:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [8:0] ram [0:3]; // @[anonymous source 755:4]
  wire [8:0] ram_RW_0_r_data; // @[anonymous source 755:4]
  wire [1:0] ram_RW_0_r_addr; // @[anonymous source 755:4]
  wire [8:0] ram_RW_0_w_data; // @[anonymous source 755:4]
  wire [1:0] ram_RW_0_w_addr; // @[anonymous source 755:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 755:4]
  wire  ram_RW_0_w_en; // @[anonymous source 755:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [1:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 755:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 767:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 755:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 4; initvar = initvar+1)
    ram[initvar] = _RAND_0[8:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[1:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_cc_banks_0_ext( // @[anonymous source 770:2]
  input  [5:0]  RW0_addr, // @[anonymous source 771:4]
  input         RW0_clk, // @[anonymous source 772:4]
  input  [63:0] RW0_wdata, // @[anonymous source 773:4]
  output [63:0] RW0_rdata, // @[anonymous source 774:4]
  input         RW0_en, // @[anonymous source 775:4]
  input         RW0_wmode // @[anonymous source 776:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [63:0] ram [0:63]; // @[anonymous source 778:4]
  wire [63:0] ram_RW_0_r_data; // @[anonymous source 778:4]
  wire [5:0] ram_RW_0_r_addr; // @[anonymous source 778:4]
  wire [63:0] ram_RW_0_w_data; // @[anonymous source 778:4]
  wire [5:0] ram_RW_0_w_addr; // @[anonymous source 778:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 778:4]
  wire  ram_RW_0_w_en; // @[anonymous source 778:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [5:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 778:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 790:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 778:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[63:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[5:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_tag_array_ext( // @[anonymous source 793:2]
  input  [1:0]  RW0_addr, // @[anonymous source 794:4]
  input         RW0_clk, // @[anonymous source 795:4]
  input  [25:0] RW0_wdata, // @[anonymous source 796:4]
  output [25:0] RW0_rdata, // @[anonymous source 797:4]
  input         RW0_en, // @[anonymous source 798:4]
  input         RW0_wmode, // @[anonymous source 799:4]
  input         RW0_wmask // @[anonymous source 800:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [25:0] ram [0:3]; // @[anonymous source 802:4]
  wire [25:0] ram_RW_0_r_data; // @[anonymous source 802:4]
  wire [1:0] ram_RW_0_r_addr; // @[anonymous source 802:4]
  wire [25:0] ram_RW_0_w_data; // @[anonymous source 802:4]
  wire [1:0] ram_RW_0_w_addr; // @[anonymous source 802:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 802:4]
  wire  ram_RW_0_w_en; // @[anonymous source 802:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [1:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 802:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 814:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 802:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 4; initvar = initvar+1)
    ram[initvar] = _RAND_0[25:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[1:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_dataArrayWay_0_ext( // @[anonymous source 817:2]
  input  [4:0]  RW0_addr, // @[anonymous source 818:4]
  input         RW0_clk, // @[anonymous source 819:4]
  input  [63:0] RW0_wdata, // @[anonymous source 820:4]
  output [63:0] RW0_rdata, // @[anonymous source 821:4]
  input         RW0_en, // @[anonymous source 822:4]
  input         RW0_wmode // @[anonymous source 823:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [63:0] ram [0:31]; // @[anonymous source 825:4]
  wire [63:0] ram_RW_0_r_data; // @[anonymous source 825:4]
  wire [4:0] ram_RW_0_r_addr; // @[anonymous source 825:4]
  wire [63:0] ram_RW_0_w_data; // @[anonymous source 825:4]
  wire [4:0] ram_RW_0_w_addr; // @[anonymous source 825:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 825:4]
  wire  ram_RW_0_w_en; // @[anonymous source 825:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [4:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 825:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 837:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 825:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 32; initvar = initvar+1)
    ram[initvar] = _RAND_0[63:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[4:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_tag_array_0_ext( // @[anonymous source 840:2]
  input  [2:0]  RW0_addr, // @[anonymous source 841:4]
  input         RW0_clk, // @[anonymous source 842:4]
  input  [24:0] RW0_wdata, // @[anonymous source 843:4]
  output [24:0] RW0_rdata, // @[anonymous source 844:4]
  input         RW0_en, // @[anonymous source 845:4]
  input         RW0_wmode, // @[anonymous source 846:4]
  input         RW0_wmask // @[anonymous source 847:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [24:0] ram [0:7]; // @[anonymous source 849:4]
  wire [24:0] ram_RW_0_r_data; // @[anonymous source 849:4]
  wire [2:0] ram_RW_0_r_addr; // @[anonymous source 849:4]
  wire [24:0] ram_RW_0_w_data; // @[anonymous source 849:4]
  wire [2:0] ram_RW_0_w_addr; // @[anonymous source 849:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 849:4]
  wire  ram_RW_0_w_en; // @[anonymous source 849:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [2:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 849:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 861:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 849:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 8; initvar = initvar+1)
    ram[initvar] = _RAND_0[24:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[2:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_array_0_0_ext( // @[anonymous source 864:2]
  input  [5:0]  W0_addr, // @[anonymous source 865:4]
  input         W0_clk, // @[anonymous source 866:4]
  input  [63:0] W0_data, // @[anonymous source 867:4]
  input         W0_en, // @[anonymous source 868:4]
  input         W0_mask, // @[anonymous source 869:4]
  input  [5:0]  R0_addr, // @[anonymous source 870:4]
  input         R0_clk, // @[anonymous source 871:4]
  output [63:0] R0_data, // @[anonymous source 872:4]
  input         R0_en // @[anonymous source 873:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [63:0] ram [0:63]; // @[anonymous source 875:4]
  wire [63:0] ram_R_0_data; // @[anonymous source 875:4]
  wire [5:0] ram_R_0_addr; // @[anonymous source 875:4]
  wire [63:0] ram_W_0_data; // @[anonymous source 875:4]
  wire [5:0] ram_W_0_addr; // @[anonymous source 875:4]
  wire  ram_W_0_mask; // @[anonymous source 875:4]
  wire  ram_W_0_en; // @[anonymous source 875:4]
  reg  ram_R_0_en_pipe_0;
  reg [5:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 875:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 886:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 875:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[63:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[5:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_tag_array_1_ext( // @[anonymous source 893:2]
  input         W0_addr, // @[anonymous source 894:4]
  input         W0_clk, // @[anonymous source 895:4]
  input  [25:0] W0_data, // @[anonymous source 896:4]
  input         W0_en, // @[anonymous source 897:4]
  input         W0_mask, // @[anonymous source 898:4]
  input         R0_addr, // @[anonymous source 899:4]
  input         R0_clk, // @[anonymous source 900:4]
  output [25:0] R0_data, // @[anonymous source 901:4]
  input         R0_en // @[anonymous source 902:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [25:0] ram [0:0]; // @[anonymous source 904:4]
  wire [25:0] ram_R_0_data; // @[anonymous source 904:4]
  wire  ram_R_0_addr; // @[anonymous source 904:4]
  wire [25:0] ram_W_0_data; // @[anonymous source 904:4]
  wire  ram_W_0_addr; // @[anonymous source 904:4]
  wire  ram_W_0_mask; // @[anonymous source 904:4]
  wire  ram_W_0_en; // @[anonymous source 904:4]
  reg  ram_R_0_en_pipe_0;
  reg  ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 904:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 915:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 904:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 1; initvar = initvar+1)
    ram[initvar] = _RAND_0[25:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_dataArrayWay_0_0_ext( // @[anonymous source 922:2]
  input          RW0_addr, // @[anonymous source 923:4]
  input          RW0_clk, // @[anonymous source 924:4]
  input  [511:0] RW0_wdata, // @[anonymous source 925:4]
  output [511:0] RW0_rdata, // @[anonymous source 926:4]
  input          RW0_en, // @[anonymous source 927:4]
  input          RW0_wmode // @[anonymous source 928:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [511:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [511:0] ram [0:0]; // @[anonymous source 930:4]
  wire [511:0] ram_RW_0_r_data; // @[anonymous source 930:4]
  wire  ram_RW_0_r_addr; // @[anonymous source 930:4]
  wire [511:0] ram_RW_0_w_data; // @[anonymous source 930:4]
  wire  ram_RW_0_w_addr; // @[anonymous source 930:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 930:4]
  wire  ram_RW_0_w_en; // @[anonymous source 930:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg  ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 930:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 942:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 930:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {16{`RANDOM}};
  for (initvar = 0; initvar < 1; initvar = initvar+1)
    ram[initvar] = _RAND_0[511:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_tag_array_2_ext( // @[anonymous source 945:2]
  input  [2:0]  W0_addr, // @[anonymous source 946:4]
  input         W0_clk, // @[anonymous source 947:4]
  input  [22:0] W0_data, // @[anonymous source 948:4]
  input         W0_en, // @[anonymous source 949:4]
  input         W0_mask, // @[anonymous source 950:4]
  input  [2:0]  R0_addr, // @[anonymous source 951:4]
  input         R0_clk, // @[anonymous source 952:4]
  output [22:0] R0_data, // @[anonymous source 953:4]
  input         R0_en // @[anonymous source 954:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [22:0] ram [0:7]; // @[anonymous source 956:4]
  wire [22:0] ram_R_0_data; // @[anonymous source 956:4]
  wire [2:0] ram_R_0_addr; // @[anonymous source 956:4]
  wire [22:0] ram_W_0_data; // @[anonymous source 956:4]
  wire [2:0] ram_W_0_addr; // @[anonymous source 956:4]
  wire  ram_W_0_mask; // @[anonymous source 956:4]
  wire  ram_W_0_en; // @[anonymous source 956:4]
  reg  ram_R_0_en_pipe_0;
  reg [2:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 956:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 967:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 956:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 8; initvar = initvar+1)
    ram[initvar] = _RAND_0[22:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[2:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_hi_us_ext( // @[anonymous source 974:2]
  input  [5:0] W0_addr, // @[anonymous source 975:4]
  input        W0_clk, // @[anonymous source 976:4]
  input        W0_data, // @[anonymous source 977:4]
  input        W0_en, // @[anonymous source 978:4]
  input        W0_mask, // @[anonymous source 979:4]
  input  [5:0] R0_addr, // @[anonymous source 980:4]
  input        R0_clk, // @[anonymous source 981:4]
  output       R0_data, // @[anonymous source 982:4]
  input        R0_en // @[anonymous source 983:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg  ram [0:63]; // @[anonymous source 985:4]
  wire  ram_R_0_data; // @[anonymous source 985:4]
  wire [5:0] ram_R_0_addr; // @[anonymous source 985:4]
  wire  ram_W_0_data; // @[anonymous source 985:4]
  wire [5:0] ram_W_0_addr; // @[anonymous source 985:4]
  wire  ram_W_0_mask; // @[anonymous source 985:4]
  wire  ram_W_0_en; // @[anonymous source 985:4]
  reg  ram_R_0_en_pipe_0;
  reg [5:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 985:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 996:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 985:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[0:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[5:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_table_ext( // @[anonymous source 1003:2]
  input  [5:0]  W0_addr, // @[anonymous source 1004:4]
  input         W0_clk, // @[anonymous source 1005:4]
  input  [11:0] W0_data, // @[anonymous source 1006:4]
  input         W0_en, // @[anonymous source 1007:4]
  input         W0_mask, // @[anonymous source 1008:4]
  input  [5:0]  R0_addr, // @[anonymous source 1009:4]
  input         R0_clk, // @[anonymous source 1010:4]
  output [11:0] R0_data, // @[anonymous source 1011:4]
  input         R0_en // @[anonymous source 1012:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [11:0] ram [0:63]; // @[anonymous source 1014:4]
  wire [11:0] ram_R_0_data; // @[anonymous source 1014:4]
  wire [5:0] ram_R_0_addr; // @[anonymous source 1014:4]
  wire [11:0] ram_W_0_data; // @[anonymous source 1014:4]
  wire [5:0] ram_W_0_addr; // @[anonymous source 1014:4]
  wire  ram_W_0_mask; // @[anonymous source 1014:4]
  wire  ram_W_0_en; // @[anonymous source 1014:4]
  reg  ram_R_0_en_pipe_0;
  reg [5:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1014:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1025:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1014:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[11:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[5:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_meta_0_ext( // @[anonymous source 1032:2]
  input  [4:0]  W0_addr, // @[anonymous source 1033:4]
  input         W0_clk, // @[anonymous source 1034:4]
  input  [32:0] W0_data, // @[anonymous source 1035:4]
  input         W0_en, // @[anonymous source 1036:4]
  input         W0_mask, // @[anonymous source 1037:4]
  input  [4:0]  R0_addr, // @[anonymous source 1038:4]
  input         R0_clk, // @[anonymous source 1039:4]
  output [32:0] R0_data, // @[anonymous source 1040:4]
  input         R0_en // @[anonymous source 1041:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [32:0] ram [0:31]; // @[anonymous source 1043:4]
  wire [32:0] ram_R_0_data; // @[anonymous source 1043:4]
  wire [4:0] ram_R_0_addr; // @[anonymous source 1043:4]
  wire [32:0] ram_W_0_data; // @[anonymous source 1043:4]
  wire [4:0] ram_W_0_addr; // @[anonymous source 1043:4]
  wire  ram_W_0_mask; // @[anonymous source 1043:4]
  wire  ram_W_0_en; // @[anonymous source 1043:4]
  reg  ram_R_0_en_pipe_0;
  reg [4:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1043:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1054:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1043:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 32; initvar = initvar+1)
    ram[initvar] = _RAND_0[32:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[4:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_btb_0_ext( // @[anonymous source 1061:2]
  input  [4:0]  W0_addr, // @[anonymous source 1062:4]
  input         W0_clk, // @[anonymous source 1063:4]
  input  [13:0] W0_data, // @[anonymous source 1064:4]
  input         W0_en, // @[anonymous source 1065:4]
  input         W0_mask, // @[anonymous source 1066:4]
  input  [4:0]  R0_addr, // @[anonymous source 1067:4]
  input         R0_clk, // @[anonymous source 1068:4]
  output [13:0] R0_data, // @[anonymous source 1069:4]
  input         R0_en // @[anonymous source 1070:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [13:0] ram [0:31]; // @[anonymous source 1072:4]
  wire [13:0] ram_R_0_data; // @[anonymous source 1072:4]
  wire [4:0] ram_R_0_addr; // @[anonymous source 1072:4]
  wire [13:0] ram_W_0_data; // @[anonymous source 1072:4]
  wire [4:0] ram_W_0_addr; // @[anonymous source 1072:4]
  wire  ram_W_0_mask; // @[anonymous source 1072:4]
  wire  ram_W_0_en; // @[anonymous source 1072:4]
  reg  ram_R_0_en_pipe_0;
  reg [4:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1072:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1083:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1072:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 32; initvar = initvar+1)
    ram[initvar] = _RAND_0[13:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[4:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_ebtb_ext( // @[anonymous source 1090:2]
  input  [4:0]  W0_addr, // @[anonymous source 1091:4]
  input         W0_clk, // @[anonymous source 1092:4]
  input  [39:0] W0_data, // @[anonymous source 1093:4]
  input         W0_en, // @[anonymous source 1094:4]
  input  [4:0]  R0_addr, // @[anonymous source 1095:4]
  input         R0_clk, // @[anonymous source 1096:4]
  output [39:0] R0_data, // @[anonymous source 1097:4]
  input         R0_en // @[anonymous source 1098:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [39:0] ram [0:31]; // @[anonymous source 1100:4]
  wire [39:0] ram_R_0_data; // @[anonymous source 1100:4]
  wire [4:0] ram_R_0_addr; // @[anonymous source 1100:4]
  wire [39:0] ram_W_0_data; // @[anonymous source 1100:4]
  wire [4:0] ram_W_0_addr; // @[anonymous source 1100:4]
  wire  ram_W_0_mask; // @[anonymous source 1100:4]
  wire  ram_W_0_en; // @[anonymous source 1100:4]
  reg  ram_R_0_en_pipe_0;
  reg [4:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1100:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = 1'h1;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1111:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1100:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 32; initvar = initvar+1)
    ram[initvar] = _RAND_0[39:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[4:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_data_ext( // @[anonymous source 1118:2]
  input  [6:0] W0_addr, // @[anonymous source 1119:4]
  input        W0_clk, // @[anonymous source 1120:4]
  input  [1:0] W0_data, // @[anonymous source 1121:4]
  input        W0_en, // @[anonymous source 1122:4]
  input        W0_mask, // @[anonymous source 1123:4]
  input  [6:0] R0_addr, // @[anonymous source 1124:4]
  input        R0_clk, // @[anonymous source 1125:4]
  output [1:0] R0_data, // @[anonymous source 1126:4]
  input        R0_en // @[anonymous source 1127:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [1:0] ram [0:127]; // @[anonymous source 1129:4]
  wire [1:0] ram_R_0_data; // @[anonymous source 1129:4]
  wire [6:0] ram_R_0_addr; // @[anonymous source 1129:4]
  wire [1:0] ram_W_0_data; // @[anonymous source 1129:4]
  wire [6:0] ram_W_0_addr; // @[anonymous source 1129:4]
  wire  ram_W_0_mask; // @[anonymous source 1129:4]
  wire  ram_W_0_en; // @[anonymous source 1129:4]
  reg  ram_R_0_en_pipe_0;
  reg [6:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1129:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1140:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1129:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 128; initvar = initvar+1)
    ram[initvar] = _RAND_0[1:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[6:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_meta_ext( // @[anonymous source 1147:2]
  input  [3:0]  W0_addr, // @[anonymous source 1148:4]
  input         W0_clk, // @[anonymous source 1149:4]
  input  [44:0] W0_data, // @[anonymous source 1150:4]
  input         W0_en, // @[anonymous source 1151:4]
  input  [3:0]  R0_addr, // @[anonymous source 1152:4]
  input         R0_clk, // @[anonymous source 1153:4]
  output [44:0] R0_data, // @[anonymous source 1154:4]
  input         R0_en // @[anonymous source 1155:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [44:0] ram [0:15]; // @[anonymous source 1157:4]
  wire [44:0] ram_R_0_data; // @[anonymous source 1157:4]
  wire [3:0] ram_R_0_addr; // @[anonymous source 1157:4]
  wire [44:0] ram_W_0_data; // @[anonymous source 1157:4]
  wire [3:0] ram_W_0_addr; // @[anonymous source 1157:4]
  wire  ram_W_0_mask; // @[anonymous source 1157:4]
  wire  ram_W_0_en; // @[anonymous source 1157:4]
  reg  ram_R_0_en_pipe_0;
  reg [3:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1157:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = 1'h1;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1168:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1157:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 16; initvar = initvar+1)
    ram[initvar] = _RAND_0[44:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[3:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_ghist_0_ext( // @[anonymous source 1175:2]
  input  [3:0]  W0_addr, // @[anonymous source 1176:4]
  input         W0_clk, // @[anonymous source 1177:4]
  input  [23:0] W0_data, // @[anonymous source 1178:4]
  input         W0_en, // @[anonymous source 1179:4]
  input  [3:0]  R0_addr, // @[anonymous source 1180:4]
  input         R0_clk, // @[anonymous source 1181:4]
  output [23:0] R0_data, // @[anonymous source 1182:4]
  input         R0_en // @[anonymous source 1183:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [23:0] ram [0:15]; // @[anonymous source 1185:4]
  wire [23:0] ram_R_0_data; // @[anonymous source 1185:4]
  wire [3:0] ram_R_0_addr; // @[anonymous source 1185:4]
  wire [23:0] ram_W_0_data; // @[anonymous source 1185:4]
  wire [3:0] ram_W_0_addr; // @[anonymous source 1185:4]
  wire  ram_W_0_mask; // @[anonymous source 1185:4]
  wire  ram_W_0_en; // @[anonymous source 1185:4]
  reg  ram_R_0_en_pipe_0;
  reg [3:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1185:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = 1'h1;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1196:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1185:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 16; initvar = initvar+1)
    ram[initvar] = _RAND_0[23:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[3:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_rob_debug_inst_mem_ext( // @[anonymous source 1203:2]
  input  [4:0]  W0_addr, // @[anonymous source 1204:4]
  input         W0_clk, // @[anonymous source 1205:4]
  input  [31:0] W0_data, // @[anonymous source 1206:4]
  input         W0_en, // @[anonymous source 1207:4]
  input         W0_mask, // @[anonymous source 1208:4]
  input  [4:0]  R0_addr, // @[anonymous source 1209:4]
  input         R0_clk, // @[anonymous source 1210:4]
  output [31:0] R0_data, // @[anonymous source 1211:4]
  input         R0_en // @[anonymous source 1212:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [31:0] ram [0:31]; // @[anonymous source 1214:4]
  wire [31:0] ram_R_0_data; // @[anonymous source 1214:4]
  wire [4:0] ram_R_0_addr; // @[anonymous source 1214:4]
  wire [31:0] ram_W_0_data; // @[anonymous source 1214:4]
  wire [4:0] ram_W_0_addr; // @[anonymous source 1214:4]
  wire  ram_W_0_mask; // @[anonymous source 1214:4]
  wire  ram_W_0_en; // @[anonymous source 1214:4]
  reg  ram_R_0_en_pipe_0;
  reg [4:0] ram_R_0_addr_pipe_0;
  assign ram_R_0_addr = ram_R_0_addr_pipe_0;
  assign ram_R_0_data = ram[ram_R_0_addr]; // @[anonymous source 1214:4]
  assign ram_W_0_data = W0_data;
  assign ram_W_0_addr = W0_addr;
  assign ram_W_0_mask = W0_mask;
  assign ram_W_0_en = W0_en;
  assign R0_data = ram_R_0_data; // @[anonymous source 1225:4]
  always @(posedge W0_clk) begin
    if(ram_W_0_en & ram_W_0_mask) begin
      ram[ram_W_0_addr] <= ram_W_0_data; // @[anonymous source 1214:4]
    end
  end
  always @(posedge R0_clk) begin
    ram_R_0_en_pipe_0 <= R0_en;
    if (R0_en) begin
      ram_R_0_addr_pipe_0 <= R0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 32; initvar = initvar+1)
    ram[initvar] = _RAND_0[31:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_R_0_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_R_0_addr_pipe_0 = _RAND_2[4:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_l2_tlb_ram_ext( // @[anonymous source 1232:2]
  input  [5:0]  RW0_addr, // @[anonymous source 1233:4]
  input         RW0_clk, // @[anonymous source 1234:4]
  input  [47:0] RW0_wdata, // @[anonymous source 1235:4]
  output [47:0] RW0_rdata, // @[anonymous source 1236:4]
  input         RW0_en, // @[anonymous source 1237:4]
  input         RW0_wmode // @[anonymous source 1238:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [47:0] ram [0:63]; // @[anonymous source 1240:4]
  wire [47:0] ram_RW_0_r_data; // @[anonymous source 1240:4]
  wire [5:0] ram_RW_0_r_addr; // @[anonymous source 1240:4]
  wire [47:0] ram_RW_0_w_data; // @[anonymous source 1240:4]
  wire [5:0] ram_RW_0_w_addr; // @[anonymous source 1240:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 1240:4]
  wire  ram_RW_0_w_en; // @[anonymous source 1240:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [5:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 1240:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 1252:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 1240:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[47:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[5:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module split_mem_ext( // @[anonymous source 1255:2]
  input  [9:0] RW0_addr, // @[anonymous source 1256:4]
  input        RW0_clk, // @[anonymous source 1257:4]
  input  [7:0] RW0_wdata, // @[anonymous source 1258:4]
  output [7:0] RW0_rdata, // @[anonymous source 1259:4]
  input        RW0_en, // @[anonymous source 1260:4]
  input        RW0_wmode, // @[anonymous source 1261:4]
  input        RW0_wmask // @[anonymous source 1262:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [7:0] ram [0:1023]; // @[anonymous source 1264:4]
  wire [7:0] ram_RW_0_r_data; // @[anonymous source 1264:4]
  wire [9:0] ram_RW_0_r_addr; // @[anonymous source 1264:4]
  wire [7:0] ram_RW_0_w_data; // @[anonymous source 1264:4]
  wire [9:0] ram_RW_0_w_addr; // @[anonymous source 1264:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 1264:4]
  wire  ram_RW_0_w_en; // @[anonymous source 1264:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [9:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 1264:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 1276:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 1264:4]
    end
    ram_RW_0_r_en_pipe_0 <= RW0_en & ~RW0_wmode;
    if (RW0_en & ~RW0_wmode) begin
      ram_RW_0_r_addr_pipe_0 <= RW0_addr;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 1024; initvar = initvar+1)
    ram[initvar] = _RAND_0[7:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[9:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
