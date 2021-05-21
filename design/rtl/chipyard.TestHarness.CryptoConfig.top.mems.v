module cc_dir_ext( // @[anonymous source 2:2]
  input  [1:0]  RW0_addr, // @[anonymous source 3:4]
  input         RW0_clk, // @[anonymous source 4:4]
  input  [63:0] RW0_wdata, // @[anonymous source 5:4]
  output [63:0] RW0_rdata, // @[anonymous source 6:4]
  input         RW0_en, // @[anonymous source 7:4]
  input         RW0_wmode, // @[anonymous source 8:4]
  input  [7:0]  RW0_wmask // @[anonymous source 9:4]
);
  wire [1:0] mem_0_0_RW0_addr; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 11:4]
  wire [7:0] mem_0_0_RW0_wdata; // @[anonymous source 11:4]
  wire [7:0] mem_0_0_RW0_rdata; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 11:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 11:4]
  wire [1:0] mem_0_1_RW0_addr; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_clk; // @[anonymous source 12:4]
  wire [7:0] mem_0_1_RW0_wdata; // @[anonymous source 12:4]
  wire [7:0] mem_0_1_RW0_rdata; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_en; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_wmode; // @[anonymous source 12:4]
  wire  mem_0_1_RW0_wmask; // @[anonymous source 12:4]
  wire [1:0] mem_0_2_RW0_addr; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_clk; // @[anonymous source 13:4]
  wire [7:0] mem_0_2_RW0_wdata; // @[anonymous source 13:4]
  wire [7:0] mem_0_2_RW0_rdata; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_en; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_wmode; // @[anonymous source 13:4]
  wire  mem_0_2_RW0_wmask; // @[anonymous source 13:4]
  wire [1:0] mem_0_3_RW0_addr; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_clk; // @[anonymous source 14:4]
  wire [7:0] mem_0_3_RW0_wdata; // @[anonymous source 14:4]
  wire [7:0] mem_0_3_RW0_rdata; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_en; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_wmode; // @[anonymous source 14:4]
  wire  mem_0_3_RW0_wmask; // @[anonymous source 14:4]
  wire [1:0] mem_0_4_RW0_addr; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_clk; // @[anonymous source 15:4]
  wire [7:0] mem_0_4_RW0_wdata; // @[anonymous source 15:4]
  wire [7:0] mem_0_4_RW0_rdata; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_en; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_wmode; // @[anonymous source 15:4]
  wire  mem_0_4_RW0_wmask; // @[anonymous source 15:4]
  wire [1:0] mem_0_5_RW0_addr; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_clk; // @[anonymous source 16:4]
  wire [7:0] mem_0_5_RW0_wdata; // @[anonymous source 16:4]
  wire [7:0] mem_0_5_RW0_rdata; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_en; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_wmode; // @[anonymous source 16:4]
  wire  mem_0_5_RW0_wmask; // @[anonymous source 16:4]
  wire [1:0] mem_0_6_RW0_addr; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_clk; // @[anonymous source 17:4]
  wire [7:0] mem_0_6_RW0_wdata; // @[anonymous source 17:4]
  wire [7:0] mem_0_6_RW0_rdata; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_en; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_wmode; // @[anonymous source 17:4]
  wire  mem_0_6_RW0_wmask; // @[anonymous source 17:4]
  wire [1:0] mem_0_7_RW0_addr; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_clk; // @[anonymous source 18:4]
  wire [7:0] mem_0_7_RW0_wdata; // @[anonymous source 18:4]
  wire [7:0] mem_0_7_RW0_rdata; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_en; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_wmode; // @[anonymous source 18:4]
  wire  mem_0_7_RW0_wmask; // @[anonymous source 18:4]
  wire [7:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 21:4]
  wire [7:0] RW0_rdata_0_1 = mem_0_1_RW0_rdata; // @[anonymous source 28:4]
  wire [7:0] RW0_rdata_0_2 = mem_0_2_RW0_rdata; // @[anonymous source 35:4]
  wire [7:0] RW0_rdata_0_3 = mem_0_3_RW0_rdata; // @[anonymous source 42:4]
  wire [7:0] RW0_rdata_0_4 = mem_0_4_RW0_rdata; // @[anonymous source 49:4]
  wire [7:0] RW0_rdata_0_5 = mem_0_5_RW0_rdata; // @[anonymous source 56:4]
  wire [7:0] RW0_rdata_0_6 = mem_0_6_RW0_rdata; // @[anonymous source 63:4]
  wire [7:0] RW0_rdata_0_7 = mem_0_7_RW0_rdata; // @[anonymous source 70:4]
  wire [15:0] _GEN_0 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [23:0] _GEN_1 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [31:0] _GEN_2 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [39:0] _GEN_3 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [47:0] _GEN_4 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [55:0] _GEN_5 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [63:0] RW0_rdata_0 = {RW0_rdata_0_7,RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,
    RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [15:0] _GEN_6 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [23:0] _GEN_7 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [31:0] _GEN_8 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [39:0] _GEN_9 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [47:0] _GEN_10 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 75:4]
  wire [55:0] _GEN_11 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
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
  assign mem_0_0_RW0_wdata = RW0_wdata[7:0]; // @[anonymous source 22:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 25:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 24:4]
  assign mem_0_0_RW0_wmask = RW0_wmask[0]; // @[anonymous source 23:4]
  assign mem_0_1_RW0_addr = RW0_addr; // @[anonymous source 27:4]
  assign mem_0_1_RW0_clk = RW0_clk; // @[anonymous source 26:4]
  assign mem_0_1_RW0_wdata = RW0_wdata[15:8]; // @[anonymous source 29:4]
  assign mem_0_1_RW0_en = RW0_en; // @[anonymous source 32:4]
  assign mem_0_1_RW0_wmode = RW0_wmode; // @[anonymous source 31:4]
  assign mem_0_1_RW0_wmask = RW0_wmask[1]; // @[anonymous source 30:4]
  assign mem_0_2_RW0_addr = RW0_addr; // @[anonymous source 34:4]
  assign mem_0_2_RW0_clk = RW0_clk; // @[anonymous source 33:4]
  assign mem_0_2_RW0_wdata = RW0_wdata[23:16]; // @[anonymous source 36:4]
  assign mem_0_2_RW0_en = RW0_en; // @[anonymous source 39:4]
  assign mem_0_2_RW0_wmode = RW0_wmode; // @[anonymous source 38:4]
  assign mem_0_2_RW0_wmask = RW0_wmask[2]; // @[anonymous source 37:4]
  assign mem_0_3_RW0_addr = RW0_addr; // @[anonymous source 41:4]
  assign mem_0_3_RW0_clk = RW0_clk; // @[anonymous source 40:4]
  assign mem_0_3_RW0_wdata = RW0_wdata[31:24]; // @[anonymous source 43:4]
  assign mem_0_3_RW0_en = RW0_en; // @[anonymous source 46:4]
  assign mem_0_3_RW0_wmode = RW0_wmode; // @[anonymous source 45:4]
  assign mem_0_3_RW0_wmask = RW0_wmask[3]; // @[anonymous source 44:4]
  assign mem_0_4_RW0_addr = RW0_addr; // @[anonymous source 48:4]
  assign mem_0_4_RW0_clk = RW0_clk; // @[anonymous source 47:4]
  assign mem_0_4_RW0_wdata = RW0_wdata[39:32]; // @[anonymous source 50:4]
  assign mem_0_4_RW0_en = RW0_en; // @[anonymous source 53:4]
  assign mem_0_4_RW0_wmode = RW0_wmode; // @[anonymous source 52:4]
  assign mem_0_4_RW0_wmask = RW0_wmask[4]; // @[anonymous source 51:4]
  assign mem_0_5_RW0_addr = RW0_addr; // @[anonymous source 55:4]
  assign mem_0_5_RW0_clk = RW0_clk; // @[anonymous source 54:4]
  assign mem_0_5_RW0_wdata = RW0_wdata[47:40]; // @[anonymous source 57:4]
  assign mem_0_5_RW0_en = RW0_en; // @[anonymous source 60:4]
  assign mem_0_5_RW0_wmode = RW0_wmode; // @[anonymous source 59:4]
  assign mem_0_5_RW0_wmask = RW0_wmask[5]; // @[anonymous source 58:4]
  assign mem_0_6_RW0_addr = RW0_addr; // @[anonymous source 62:4]
  assign mem_0_6_RW0_clk = RW0_clk; // @[anonymous source 61:4]
  assign mem_0_6_RW0_wdata = RW0_wdata[55:48]; // @[anonymous source 64:4]
  assign mem_0_6_RW0_en = RW0_en; // @[anonymous source 67:4]
  assign mem_0_6_RW0_wmode = RW0_wmode; // @[anonymous source 66:4]
  assign mem_0_6_RW0_wmask = RW0_wmask[6]; // @[anonymous source 65:4]
  assign mem_0_7_RW0_addr = RW0_addr; // @[anonymous source 69:4]
  assign mem_0_7_RW0_clk = RW0_clk; // @[anonymous source 68:4]
  assign mem_0_7_RW0_wdata = RW0_wdata[63:56]; // @[anonymous source 71:4]
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
module data_arrays_0_ext( // @[anonymous source 96:2]
  input  [5:0]  RW0_addr, // @[anonymous source 97:4]
  input         RW0_clk, // @[anonymous source 98:4]
  input  [63:0] RW0_wdata, // @[anonymous source 99:4]
  output [63:0] RW0_rdata, // @[anonymous source 100:4]
  input         RW0_en, // @[anonymous source 101:4]
  input         RW0_wmode, // @[anonymous source 102:4]
  input  [7:0]  RW0_wmask // @[anonymous source 103:4]
);
  wire [5:0] mem_0_0_RW0_addr; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 105:4]
  wire [7:0] mem_0_0_RW0_wdata; // @[anonymous source 105:4]
  wire [7:0] mem_0_0_RW0_rdata; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 105:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 105:4]
  wire [5:0] mem_0_1_RW0_addr; // @[anonymous source 106:4]
  wire  mem_0_1_RW0_clk; // @[anonymous source 106:4]
  wire [7:0] mem_0_1_RW0_wdata; // @[anonymous source 106:4]
  wire [7:0] mem_0_1_RW0_rdata; // @[anonymous source 106:4]
  wire  mem_0_1_RW0_en; // @[anonymous source 106:4]
  wire  mem_0_1_RW0_wmode; // @[anonymous source 106:4]
  wire  mem_0_1_RW0_wmask; // @[anonymous source 106:4]
  wire [5:0] mem_0_2_RW0_addr; // @[anonymous source 107:4]
  wire  mem_0_2_RW0_clk; // @[anonymous source 107:4]
  wire [7:0] mem_0_2_RW0_wdata; // @[anonymous source 107:4]
  wire [7:0] mem_0_2_RW0_rdata; // @[anonymous source 107:4]
  wire  mem_0_2_RW0_en; // @[anonymous source 107:4]
  wire  mem_0_2_RW0_wmode; // @[anonymous source 107:4]
  wire  mem_0_2_RW0_wmask; // @[anonymous source 107:4]
  wire [5:0] mem_0_3_RW0_addr; // @[anonymous source 108:4]
  wire  mem_0_3_RW0_clk; // @[anonymous source 108:4]
  wire [7:0] mem_0_3_RW0_wdata; // @[anonymous source 108:4]
  wire [7:0] mem_0_3_RW0_rdata; // @[anonymous source 108:4]
  wire  mem_0_3_RW0_en; // @[anonymous source 108:4]
  wire  mem_0_3_RW0_wmode; // @[anonymous source 108:4]
  wire  mem_0_3_RW0_wmask; // @[anonymous source 108:4]
  wire [5:0] mem_0_4_RW0_addr; // @[anonymous source 109:4]
  wire  mem_0_4_RW0_clk; // @[anonymous source 109:4]
  wire [7:0] mem_0_4_RW0_wdata; // @[anonymous source 109:4]
  wire [7:0] mem_0_4_RW0_rdata; // @[anonymous source 109:4]
  wire  mem_0_4_RW0_en; // @[anonymous source 109:4]
  wire  mem_0_4_RW0_wmode; // @[anonymous source 109:4]
  wire  mem_0_4_RW0_wmask; // @[anonymous source 109:4]
  wire [5:0] mem_0_5_RW0_addr; // @[anonymous source 110:4]
  wire  mem_0_5_RW0_clk; // @[anonymous source 110:4]
  wire [7:0] mem_0_5_RW0_wdata; // @[anonymous source 110:4]
  wire [7:0] mem_0_5_RW0_rdata; // @[anonymous source 110:4]
  wire  mem_0_5_RW0_en; // @[anonymous source 110:4]
  wire  mem_0_5_RW0_wmode; // @[anonymous source 110:4]
  wire  mem_0_5_RW0_wmask; // @[anonymous source 110:4]
  wire [5:0] mem_0_6_RW0_addr; // @[anonymous source 111:4]
  wire  mem_0_6_RW0_clk; // @[anonymous source 111:4]
  wire [7:0] mem_0_6_RW0_wdata; // @[anonymous source 111:4]
  wire [7:0] mem_0_6_RW0_rdata; // @[anonymous source 111:4]
  wire  mem_0_6_RW0_en; // @[anonymous source 111:4]
  wire  mem_0_6_RW0_wmode; // @[anonymous source 111:4]
  wire  mem_0_6_RW0_wmask; // @[anonymous source 111:4]
  wire [5:0] mem_0_7_RW0_addr; // @[anonymous source 112:4]
  wire  mem_0_7_RW0_clk; // @[anonymous source 112:4]
  wire [7:0] mem_0_7_RW0_wdata; // @[anonymous source 112:4]
  wire [7:0] mem_0_7_RW0_rdata; // @[anonymous source 112:4]
  wire  mem_0_7_RW0_en; // @[anonymous source 112:4]
  wire  mem_0_7_RW0_wmode; // @[anonymous source 112:4]
  wire  mem_0_7_RW0_wmask; // @[anonymous source 112:4]
  wire [7:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 115:4]
  wire [7:0] RW0_rdata_0_1 = mem_0_1_RW0_rdata; // @[anonymous source 122:4]
  wire [7:0] RW0_rdata_0_2 = mem_0_2_RW0_rdata; // @[anonymous source 129:4]
  wire [7:0] RW0_rdata_0_3 = mem_0_3_RW0_rdata; // @[anonymous source 136:4]
  wire [7:0] RW0_rdata_0_4 = mem_0_4_RW0_rdata; // @[anonymous source 143:4]
  wire [7:0] RW0_rdata_0_5 = mem_0_5_RW0_rdata; // @[anonymous source 150:4]
  wire [7:0] RW0_rdata_0_6 = mem_0_6_RW0_rdata; // @[anonymous source 157:4]
  wire [7:0] RW0_rdata_0_7 = mem_0_7_RW0_rdata; // @[anonymous source 164:4]
  wire [15:0] _GEN_0 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [23:0] _GEN_1 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [31:0] _GEN_2 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [39:0] _GEN_3 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [47:0] _GEN_4 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [55:0] _GEN_5 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [63:0] RW0_rdata_0 = {RW0_rdata_0_7,RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,
    RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [15:0] _GEN_6 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [23:0] _GEN_7 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [31:0] _GEN_8 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [39:0] _GEN_9 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [47:0] _GEN_10 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 169:4]
  wire [55:0] _GEN_11 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 169:4]
  split_data_arrays_0_ext mem_0_0 ( // @[anonymous source 105:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_1 ( // @[anonymous source 106:4]
    .RW0_addr(mem_0_1_RW0_addr),
    .RW0_clk(mem_0_1_RW0_clk),
    .RW0_wdata(mem_0_1_RW0_wdata),
    .RW0_rdata(mem_0_1_RW0_rdata),
    .RW0_en(mem_0_1_RW0_en),
    .RW0_wmode(mem_0_1_RW0_wmode),
    .RW0_wmask(mem_0_1_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_2 ( // @[anonymous source 107:4]
    .RW0_addr(mem_0_2_RW0_addr),
    .RW0_clk(mem_0_2_RW0_clk),
    .RW0_wdata(mem_0_2_RW0_wdata),
    .RW0_rdata(mem_0_2_RW0_rdata),
    .RW0_en(mem_0_2_RW0_en),
    .RW0_wmode(mem_0_2_RW0_wmode),
    .RW0_wmask(mem_0_2_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_3 ( // @[anonymous source 108:4]
    .RW0_addr(mem_0_3_RW0_addr),
    .RW0_clk(mem_0_3_RW0_clk),
    .RW0_wdata(mem_0_3_RW0_wdata),
    .RW0_rdata(mem_0_3_RW0_rdata),
    .RW0_en(mem_0_3_RW0_en),
    .RW0_wmode(mem_0_3_RW0_wmode),
    .RW0_wmask(mem_0_3_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_4 ( // @[anonymous source 109:4]
    .RW0_addr(mem_0_4_RW0_addr),
    .RW0_clk(mem_0_4_RW0_clk),
    .RW0_wdata(mem_0_4_RW0_wdata),
    .RW0_rdata(mem_0_4_RW0_rdata),
    .RW0_en(mem_0_4_RW0_en),
    .RW0_wmode(mem_0_4_RW0_wmode),
    .RW0_wmask(mem_0_4_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_5 ( // @[anonymous source 110:4]
    .RW0_addr(mem_0_5_RW0_addr),
    .RW0_clk(mem_0_5_RW0_clk),
    .RW0_wdata(mem_0_5_RW0_wdata),
    .RW0_rdata(mem_0_5_RW0_rdata),
    .RW0_en(mem_0_5_RW0_en),
    .RW0_wmode(mem_0_5_RW0_wmode),
    .RW0_wmask(mem_0_5_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_6 ( // @[anonymous source 111:4]
    .RW0_addr(mem_0_6_RW0_addr),
    .RW0_clk(mem_0_6_RW0_clk),
    .RW0_wdata(mem_0_6_RW0_wdata),
    .RW0_rdata(mem_0_6_RW0_rdata),
    .RW0_en(mem_0_6_RW0_en),
    .RW0_wmode(mem_0_6_RW0_wmode),
    .RW0_wmask(mem_0_6_RW0_wmask)
  );
  split_data_arrays_0_ext mem_0_7 ( // @[anonymous source 112:4]
    .RW0_addr(mem_0_7_RW0_addr),
    .RW0_clk(mem_0_7_RW0_clk),
    .RW0_wdata(mem_0_7_RW0_wdata),
    .RW0_rdata(mem_0_7_RW0_rdata),
    .RW0_en(mem_0_7_RW0_en),
    .RW0_wmode(mem_0_7_RW0_wmode),
    .RW0_wmask(mem_0_7_RW0_wmask)
  );
  assign RW0_rdata = {RW0_rdata_0_7,_GEN_5}; // @[anonymous source 169:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 114:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 113:4]
  assign mem_0_0_RW0_wdata = RW0_wdata[7:0]; // @[anonymous source 116:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 119:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 118:4]
  assign mem_0_0_RW0_wmask = RW0_wmask[0]; // @[anonymous source 117:4]
  assign mem_0_1_RW0_addr = RW0_addr; // @[anonymous source 121:4]
  assign mem_0_1_RW0_clk = RW0_clk; // @[anonymous source 120:4]
  assign mem_0_1_RW0_wdata = RW0_wdata[15:8]; // @[anonymous source 123:4]
  assign mem_0_1_RW0_en = RW0_en; // @[anonymous source 126:4]
  assign mem_0_1_RW0_wmode = RW0_wmode; // @[anonymous source 125:4]
  assign mem_0_1_RW0_wmask = RW0_wmask[1]; // @[anonymous source 124:4]
  assign mem_0_2_RW0_addr = RW0_addr; // @[anonymous source 128:4]
  assign mem_0_2_RW0_clk = RW0_clk; // @[anonymous source 127:4]
  assign mem_0_2_RW0_wdata = RW0_wdata[23:16]; // @[anonymous source 130:4]
  assign mem_0_2_RW0_en = RW0_en; // @[anonymous source 133:4]
  assign mem_0_2_RW0_wmode = RW0_wmode; // @[anonymous source 132:4]
  assign mem_0_2_RW0_wmask = RW0_wmask[2]; // @[anonymous source 131:4]
  assign mem_0_3_RW0_addr = RW0_addr; // @[anonymous source 135:4]
  assign mem_0_3_RW0_clk = RW0_clk; // @[anonymous source 134:4]
  assign mem_0_3_RW0_wdata = RW0_wdata[31:24]; // @[anonymous source 137:4]
  assign mem_0_3_RW0_en = RW0_en; // @[anonymous source 140:4]
  assign mem_0_3_RW0_wmode = RW0_wmode; // @[anonymous source 139:4]
  assign mem_0_3_RW0_wmask = RW0_wmask[3]; // @[anonymous source 138:4]
  assign mem_0_4_RW0_addr = RW0_addr; // @[anonymous source 142:4]
  assign mem_0_4_RW0_clk = RW0_clk; // @[anonymous source 141:4]
  assign mem_0_4_RW0_wdata = RW0_wdata[39:32]; // @[anonymous source 144:4]
  assign mem_0_4_RW0_en = RW0_en; // @[anonymous source 147:4]
  assign mem_0_4_RW0_wmode = RW0_wmode; // @[anonymous source 146:4]
  assign mem_0_4_RW0_wmask = RW0_wmask[4]; // @[anonymous source 145:4]
  assign mem_0_5_RW0_addr = RW0_addr; // @[anonymous source 149:4]
  assign mem_0_5_RW0_clk = RW0_clk; // @[anonymous source 148:4]
  assign mem_0_5_RW0_wdata = RW0_wdata[47:40]; // @[anonymous source 151:4]
  assign mem_0_5_RW0_en = RW0_en; // @[anonymous source 154:4]
  assign mem_0_5_RW0_wmode = RW0_wmode; // @[anonymous source 153:4]
  assign mem_0_5_RW0_wmask = RW0_wmask[5]; // @[anonymous source 152:4]
  assign mem_0_6_RW0_addr = RW0_addr; // @[anonymous source 156:4]
  assign mem_0_6_RW0_clk = RW0_clk; // @[anonymous source 155:4]
  assign mem_0_6_RW0_wdata = RW0_wdata[55:48]; // @[anonymous source 158:4]
  assign mem_0_6_RW0_en = RW0_en; // @[anonymous source 161:4]
  assign mem_0_6_RW0_wmode = RW0_wmode; // @[anonymous source 160:4]
  assign mem_0_6_RW0_wmask = RW0_wmask[6]; // @[anonymous source 159:4]
  assign mem_0_7_RW0_addr = RW0_addr; // @[anonymous source 163:4]
  assign mem_0_7_RW0_clk = RW0_clk; // @[anonymous source 162:4]
  assign mem_0_7_RW0_wdata = RW0_wdata[63:56]; // @[anonymous source 165:4]
  assign mem_0_7_RW0_en = RW0_en; // @[anonymous source 168:4]
  assign mem_0_7_RW0_wmode = RW0_wmode; // @[anonymous source 167:4]
  assign mem_0_7_RW0_wmask = RW0_wmask[7]; // @[anonymous source 166:4]
endmodule
module tag_array_ext( // @[anonymous source 172:2]
  input  [2:0]  RW0_addr, // @[anonymous source 173:4]
  input         RW0_clk, // @[anonymous source 174:4]
  input  [24:0] RW0_wdata, // @[anonymous source 175:4]
  output [24:0] RW0_rdata, // @[anonymous source 176:4]
  input         RW0_en, // @[anonymous source 177:4]
  input         RW0_wmode // @[anonymous source 178:4]
);
  wire [2:0] mem_0_0_RW0_addr; // @[anonymous source 180:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 180:4]
  wire [24:0] mem_0_0_RW0_wdata; // @[anonymous source 180:4]
  wire [24:0] mem_0_0_RW0_rdata; // @[anonymous source 180:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 180:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 180:4]
  wire [24:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 183:4]
  wire [24:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 183:4]
  split_tag_array_ext mem_0_0 ( // @[anonymous source 180:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 183:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 182:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 181:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 184:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 186:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 185:4]
endmodule
module tag_array_0_ext( // @[anonymous source 190:2]
  input  [2:0]  RW0_addr, // @[anonymous source 191:4]
  input         RW0_clk, // @[anonymous source 192:4]
  input  [23:0] RW0_wdata, // @[anonymous source 193:4]
  output [23:0] RW0_rdata, // @[anonymous source 194:4]
  input         RW0_en, // @[anonymous source 195:4]
  input         RW0_wmode, // @[anonymous source 196:4]
  input         RW0_wmask // @[anonymous source 197:4]
);
  wire [2:0] mem_0_0_RW0_addr; // @[anonymous source 199:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 199:4]
  wire [23:0] mem_0_0_RW0_wdata; // @[anonymous source 199:4]
  wire [23:0] mem_0_0_RW0_rdata; // @[anonymous source 199:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 199:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 199:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 199:4]
  wire [23:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 202:4]
  wire [23:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 202:4]
  split_tag_array_0_ext mem_0_0 ( // @[anonymous source 199:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 202:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 201:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 200:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 203:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 206:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 205:4]
  assign mem_0_0_RW0_wmask = RW0_wmask; // @[anonymous source 204:4]
endmodule
module data_arrays_0_0_ext( // @[anonymous source 210:2]
  input  [5:0]  RW0_addr, // @[anonymous source 211:4]
  input         RW0_clk, // @[anonymous source 212:4]
  input  [31:0] RW0_wdata, // @[anonymous source 213:4]
  output [31:0] RW0_rdata, // @[anonymous source 214:4]
  input         RW0_en, // @[anonymous source 215:4]
  input         RW0_wmode, // @[anonymous source 216:4]
  input         RW0_wmask // @[anonymous source 217:4]
);
  wire [5:0] mem_0_0_RW0_addr; // @[anonymous source 219:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 219:4]
  wire [31:0] mem_0_0_RW0_wdata; // @[anonymous source 219:4]
  wire [31:0] mem_0_0_RW0_rdata; // @[anonymous source 219:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 219:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 219:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 219:4]
  wire [31:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 222:4]
  wire [31:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 222:4]
  split_data_arrays_0_0_ext mem_0_0 ( // @[anonymous source 219:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 222:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 221:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 220:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 223:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 226:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 225:4]
  assign mem_0_0_RW0_wmask = RW0_wmask; // @[anonymous source 224:4]
endmodule
module l2_tlb_ram_ext( // @[anonymous source 230:2]
  input  [5:0]  RW0_addr, // @[anonymous source 231:4]
  input         RW0_clk, // @[anonymous source 232:4]
  input  [47:0] RW0_wdata, // @[anonymous source 233:4]
  output [47:0] RW0_rdata, // @[anonymous source 234:4]
  input         RW0_en, // @[anonymous source 235:4]
  input         RW0_wmode // @[anonymous source 236:4]
);
  wire [5:0] mem_0_0_RW0_addr; // @[anonymous source 238:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 238:4]
  wire [47:0] mem_0_0_RW0_wdata; // @[anonymous source 238:4]
  wire [47:0] mem_0_0_RW0_rdata; // @[anonymous source 238:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 238:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 238:4]
  wire [47:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 241:4]
  wire [47:0] RW0_rdata_0 = RW0_rdata_0_0; // @[anonymous source 241:4]
  split_l2_tlb_ram_ext mem_0_0 ( // @[anonymous source 238:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode)
  );
  assign RW0_rdata = mem_0_0_RW0_rdata; // @[anonymous source 241:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 240:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 239:4]
  assign mem_0_0_RW0_wdata = RW0_wdata; // @[anonymous source 242:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 244:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 243:4]
endmodule
module mem_ext( // @[anonymous source 248:2]
  input  [8:0]  RW0_addr, // @[anonymous source 249:4]
  input         RW0_clk, // @[anonymous source 250:4]
  input  [63:0] RW0_wdata, // @[anonymous source 251:4]
  output [63:0] RW0_rdata, // @[anonymous source 252:4]
  input         RW0_en, // @[anonymous source 253:4]
  input         RW0_wmode, // @[anonymous source 254:4]
  input  [7:0]  RW0_wmask // @[anonymous source 255:4]
);
  wire [8:0] mem_0_0_RW0_addr; // @[anonymous source 257:4]
  wire  mem_0_0_RW0_clk; // @[anonymous source 257:4]
  wire [7:0] mem_0_0_RW0_wdata; // @[anonymous source 257:4]
  wire [7:0] mem_0_0_RW0_rdata; // @[anonymous source 257:4]
  wire  mem_0_0_RW0_en; // @[anonymous source 257:4]
  wire  mem_0_0_RW0_wmode; // @[anonymous source 257:4]
  wire  mem_0_0_RW0_wmask; // @[anonymous source 257:4]
  wire [8:0] mem_0_1_RW0_addr; // @[anonymous source 258:4]
  wire  mem_0_1_RW0_clk; // @[anonymous source 258:4]
  wire [7:0] mem_0_1_RW0_wdata; // @[anonymous source 258:4]
  wire [7:0] mem_0_1_RW0_rdata; // @[anonymous source 258:4]
  wire  mem_0_1_RW0_en; // @[anonymous source 258:4]
  wire  mem_0_1_RW0_wmode; // @[anonymous source 258:4]
  wire  mem_0_1_RW0_wmask; // @[anonymous source 258:4]
  wire [8:0] mem_0_2_RW0_addr; // @[anonymous source 259:4]
  wire  mem_0_2_RW0_clk; // @[anonymous source 259:4]
  wire [7:0] mem_0_2_RW0_wdata; // @[anonymous source 259:4]
  wire [7:0] mem_0_2_RW0_rdata; // @[anonymous source 259:4]
  wire  mem_0_2_RW0_en; // @[anonymous source 259:4]
  wire  mem_0_2_RW0_wmode; // @[anonymous source 259:4]
  wire  mem_0_2_RW0_wmask; // @[anonymous source 259:4]
  wire [8:0] mem_0_3_RW0_addr; // @[anonymous source 260:4]
  wire  mem_0_3_RW0_clk; // @[anonymous source 260:4]
  wire [7:0] mem_0_3_RW0_wdata; // @[anonymous source 260:4]
  wire [7:0] mem_0_3_RW0_rdata; // @[anonymous source 260:4]
  wire  mem_0_3_RW0_en; // @[anonymous source 260:4]
  wire  mem_0_3_RW0_wmode; // @[anonymous source 260:4]
  wire  mem_0_3_RW0_wmask; // @[anonymous source 260:4]
  wire [8:0] mem_0_4_RW0_addr; // @[anonymous source 261:4]
  wire  mem_0_4_RW0_clk; // @[anonymous source 261:4]
  wire [7:0] mem_0_4_RW0_wdata; // @[anonymous source 261:4]
  wire [7:0] mem_0_4_RW0_rdata; // @[anonymous source 261:4]
  wire  mem_0_4_RW0_en; // @[anonymous source 261:4]
  wire  mem_0_4_RW0_wmode; // @[anonymous source 261:4]
  wire  mem_0_4_RW0_wmask; // @[anonymous source 261:4]
  wire [8:0] mem_0_5_RW0_addr; // @[anonymous source 262:4]
  wire  mem_0_5_RW0_clk; // @[anonymous source 262:4]
  wire [7:0] mem_0_5_RW0_wdata; // @[anonymous source 262:4]
  wire [7:0] mem_0_5_RW0_rdata; // @[anonymous source 262:4]
  wire  mem_0_5_RW0_en; // @[anonymous source 262:4]
  wire  mem_0_5_RW0_wmode; // @[anonymous source 262:4]
  wire  mem_0_5_RW0_wmask; // @[anonymous source 262:4]
  wire [8:0] mem_0_6_RW0_addr; // @[anonymous source 263:4]
  wire  mem_0_6_RW0_clk; // @[anonymous source 263:4]
  wire [7:0] mem_0_6_RW0_wdata; // @[anonymous source 263:4]
  wire [7:0] mem_0_6_RW0_rdata; // @[anonymous source 263:4]
  wire  mem_0_6_RW0_en; // @[anonymous source 263:4]
  wire  mem_0_6_RW0_wmode; // @[anonymous source 263:4]
  wire  mem_0_6_RW0_wmask; // @[anonymous source 263:4]
  wire [8:0] mem_0_7_RW0_addr; // @[anonymous source 264:4]
  wire  mem_0_7_RW0_clk; // @[anonymous source 264:4]
  wire [7:0] mem_0_7_RW0_wdata; // @[anonymous source 264:4]
  wire [7:0] mem_0_7_RW0_rdata; // @[anonymous source 264:4]
  wire  mem_0_7_RW0_en; // @[anonymous source 264:4]
  wire  mem_0_7_RW0_wmode; // @[anonymous source 264:4]
  wire  mem_0_7_RW0_wmask; // @[anonymous source 264:4]
  wire [7:0] RW0_rdata_0_0 = mem_0_0_RW0_rdata; // @[anonymous source 267:4]
  wire [7:0] RW0_rdata_0_1 = mem_0_1_RW0_rdata; // @[anonymous source 274:4]
  wire [7:0] RW0_rdata_0_2 = mem_0_2_RW0_rdata; // @[anonymous source 281:4]
  wire [7:0] RW0_rdata_0_3 = mem_0_3_RW0_rdata; // @[anonymous source 288:4]
  wire [7:0] RW0_rdata_0_4 = mem_0_4_RW0_rdata; // @[anonymous source 295:4]
  wire [7:0] RW0_rdata_0_5 = mem_0_5_RW0_rdata; // @[anonymous source 302:4]
  wire [7:0] RW0_rdata_0_6 = mem_0_6_RW0_rdata; // @[anonymous source 309:4]
  wire [7:0] RW0_rdata_0_7 = mem_0_7_RW0_rdata; // @[anonymous source 316:4]
  wire [15:0] _GEN_0 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [23:0] _GEN_1 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [31:0] _GEN_2 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [39:0] _GEN_3 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [47:0] _GEN_4 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [55:0] _GEN_5 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [63:0] RW0_rdata_0 = {RW0_rdata_0_7,RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,
    RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [15:0] _GEN_6 = {RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [23:0] _GEN_7 = {RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [31:0] _GEN_8 = {RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [39:0] _GEN_9 = {RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [47:0] _GEN_10 = {RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,RW0_rdata_0_0}; // @[anonymous source 321:4]
  wire [55:0] _GEN_11 = {RW0_rdata_0_6,RW0_rdata_0_5,RW0_rdata_0_4,RW0_rdata_0_3,RW0_rdata_0_2,RW0_rdata_0_1,
    RW0_rdata_0_0}; // @[anonymous source 321:4]
  split_mem_ext mem_0_0 ( // @[anonymous source 257:4]
    .RW0_addr(mem_0_0_RW0_addr),
    .RW0_clk(mem_0_0_RW0_clk),
    .RW0_wdata(mem_0_0_RW0_wdata),
    .RW0_rdata(mem_0_0_RW0_rdata),
    .RW0_en(mem_0_0_RW0_en),
    .RW0_wmode(mem_0_0_RW0_wmode),
    .RW0_wmask(mem_0_0_RW0_wmask)
  );
  split_mem_ext mem_0_1 ( // @[anonymous source 258:4]
    .RW0_addr(mem_0_1_RW0_addr),
    .RW0_clk(mem_0_1_RW0_clk),
    .RW0_wdata(mem_0_1_RW0_wdata),
    .RW0_rdata(mem_0_1_RW0_rdata),
    .RW0_en(mem_0_1_RW0_en),
    .RW0_wmode(mem_0_1_RW0_wmode),
    .RW0_wmask(mem_0_1_RW0_wmask)
  );
  split_mem_ext mem_0_2 ( // @[anonymous source 259:4]
    .RW0_addr(mem_0_2_RW0_addr),
    .RW0_clk(mem_0_2_RW0_clk),
    .RW0_wdata(mem_0_2_RW0_wdata),
    .RW0_rdata(mem_0_2_RW0_rdata),
    .RW0_en(mem_0_2_RW0_en),
    .RW0_wmode(mem_0_2_RW0_wmode),
    .RW0_wmask(mem_0_2_RW0_wmask)
  );
  split_mem_ext mem_0_3 ( // @[anonymous source 260:4]
    .RW0_addr(mem_0_3_RW0_addr),
    .RW0_clk(mem_0_3_RW0_clk),
    .RW0_wdata(mem_0_3_RW0_wdata),
    .RW0_rdata(mem_0_3_RW0_rdata),
    .RW0_en(mem_0_3_RW0_en),
    .RW0_wmode(mem_0_3_RW0_wmode),
    .RW0_wmask(mem_0_3_RW0_wmask)
  );
  split_mem_ext mem_0_4 ( // @[anonymous source 261:4]
    .RW0_addr(mem_0_4_RW0_addr),
    .RW0_clk(mem_0_4_RW0_clk),
    .RW0_wdata(mem_0_4_RW0_wdata),
    .RW0_rdata(mem_0_4_RW0_rdata),
    .RW0_en(mem_0_4_RW0_en),
    .RW0_wmode(mem_0_4_RW0_wmode),
    .RW0_wmask(mem_0_4_RW0_wmask)
  );
  split_mem_ext mem_0_5 ( // @[anonymous source 262:4]
    .RW0_addr(mem_0_5_RW0_addr),
    .RW0_clk(mem_0_5_RW0_clk),
    .RW0_wdata(mem_0_5_RW0_wdata),
    .RW0_rdata(mem_0_5_RW0_rdata),
    .RW0_en(mem_0_5_RW0_en),
    .RW0_wmode(mem_0_5_RW0_wmode),
    .RW0_wmask(mem_0_5_RW0_wmask)
  );
  split_mem_ext mem_0_6 ( // @[anonymous source 263:4]
    .RW0_addr(mem_0_6_RW0_addr),
    .RW0_clk(mem_0_6_RW0_clk),
    .RW0_wdata(mem_0_6_RW0_wdata),
    .RW0_rdata(mem_0_6_RW0_rdata),
    .RW0_en(mem_0_6_RW0_en),
    .RW0_wmode(mem_0_6_RW0_wmode),
    .RW0_wmask(mem_0_6_RW0_wmask)
  );
  split_mem_ext mem_0_7 ( // @[anonymous source 264:4]
    .RW0_addr(mem_0_7_RW0_addr),
    .RW0_clk(mem_0_7_RW0_clk),
    .RW0_wdata(mem_0_7_RW0_wdata),
    .RW0_rdata(mem_0_7_RW0_rdata),
    .RW0_en(mem_0_7_RW0_en),
    .RW0_wmode(mem_0_7_RW0_wmode),
    .RW0_wmask(mem_0_7_RW0_wmask)
  );
  assign RW0_rdata = {RW0_rdata_0_7,_GEN_5}; // @[anonymous source 321:4]
  assign mem_0_0_RW0_addr = RW0_addr; // @[anonymous source 266:4]
  assign mem_0_0_RW0_clk = RW0_clk; // @[anonymous source 265:4]
  assign mem_0_0_RW0_wdata = RW0_wdata[7:0]; // @[anonymous source 268:4]
  assign mem_0_0_RW0_en = RW0_en; // @[anonymous source 271:4]
  assign mem_0_0_RW0_wmode = RW0_wmode; // @[anonymous source 270:4]
  assign mem_0_0_RW0_wmask = RW0_wmask[0]; // @[anonymous source 269:4]
  assign mem_0_1_RW0_addr = RW0_addr; // @[anonymous source 273:4]
  assign mem_0_1_RW0_clk = RW0_clk; // @[anonymous source 272:4]
  assign mem_0_1_RW0_wdata = RW0_wdata[15:8]; // @[anonymous source 275:4]
  assign mem_0_1_RW0_en = RW0_en; // @[anonymous source 278:4]
  assign mem_0_1_RW0_wmode = RW0_wmode; // @[anonymous source 277:4]
  assign mem_0_1_RW0_wmask = RW0_wmask[1]; // @[anonymous source 276:4]
  assign mem_0_2_RW0_addr = RW0_addr; // @[anonymous source 280:4]
  assign mem_0_2_RW0_clk = RW0_clk; // @[anonymous source 279:4]
  assign mem_0_2_RW0_wdata = RW0_wdata[23:16]; // @[anonymous source 282:4]
  assign mem_0_2_RW0_en = RW0_en; // @[anonymous source 285:4]
  assign mem_0_2_RW0_wmode = RW0_wmode; // @[anonymous source 284:4]
  assign mem_0_2_RW0_wmask = RW0_wmask[2]; // @[anonymous source 283:4]
  assign mem_0_3_RW0_addr = RW0_addr; // @[anonymous source 287:4]
  assign mem_0_3_RW0_clk = RW0_clk; // @[anonymous source 286:4]
  assign mem_0_3_RW0_wdata = RW0_wdata[31:24]; // @[anonymous source 289:4]
  assign mem_0_3_RW0_en = RW0_en; // @[anonymous source 292:4]
  assign mem_0_3_RW0_wmode = RW0_wmode; // @[anonymous source 291:4]
  assign mem_0_3_RW0_wmask = RW0_wmask[3]; // @[anonymous source 290:4]
  assign mem_0_4_RW0_addr = RW0_addr; // @[anonymous source 294:4]
  assign mem_0_4_RW0_clk = RW0_clk; // @[anonymous source 293:4]
  assign mem_0_4_RW0_wdata = RW0_wdata[39:32]; // @[anonymous source 296:4]
  assign mem_0_4_RW0_en = RW0_en; // @[anonymous source 299:4]
  assign mem_0_4_RW0_wmode = RW0_wmode; // @[anonymous source 298:4]
  assign mem_0_4_RW0_wmask = RW0_wmask[4]; // @[anonymous source 297:4]
  assign mem_0_5_RW0_addr = RW0_addr; // @[anonymous source 301:4]
  assign mem_0_5_RW0_clk = RW0_clk; // @[anonymous source 300:4]
  assign mem_0_5_RW0_wdata = RW0_wdata[47:40]; // @[anonymous source 303:4]
  assign mem_0_5_RW0_en = RW0_en; // @[anonymous source 306:4]
  assign mem_0_5_RW0_wmode = RW0_wmode; // @[anonymous source 305:4]
  assign mem_0_5_RW0_wmask = RW0_wmask[5]; // @[anonymous source 304:4]
  assign mem_0_6_RW0_addr = RW0_addr; // @[anonymous source 308:4]
  assign mem_0_6_RW0_clk = RW0_clk; // @[anonymous source 307:4]
  assign mem_0_6_RW0_wdata = RW0_wdata[55:48]; // @[anonymous source 310:4]
  assign mem_0_6_RW0_en = RW0_en; // @[anonymous source 313:4]
  assign mem_0_6_RW0_wmode = RW0_wmode; // @[anonymous source 312:4]
  assign mem_0_6_RW0_wmask = RW0_wmask[6]; // @[anonymous source 311:4]
  assign mem_0_7_RW0_addr = RW0_addr; // @[anonymous source 315:4]
  assign mem_0_7_RW0_clk = RW0_clk; // @[anonymous source 314:4]
  assign mem_0_7_RW0_wdata = RW0_wdata[63:56]; // @[anonymous source 317:4]
  assign mem_0_7_RW0_en = RW0_en; // @[anonymous source 320:4]
  assign mem_0_7_RW0_wmode = RW0_wmode; // @[anonymous source 319:4]
  assign mem_0_7_RW0_wmask = RW0_wmask[7]; // @[anonymous source 318:4]
endmodule
module split_cc_dir_ext( // @[anonymous source 324:2]
  input  [1:0] RW0_addr, // @[anonymous source 325:4]
  input        RW0_clk, // @[anonymous source 326:4]
  input  [7:0] RW0_wdata, // @[anonymous source 327:4]
  output [7:0] RW0_rdata, // @[anonymous source 328:4]
  input        RW0_en, // @[anonymous source 329:4]
  input        RW0_wmode, // @[anonymous source 330:4]
  input        RW0_wmask // @[anonymous source 331:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [7:0] ram [0:3]; // @[anonymous source 333:4]
  wire [7:0] ram_RW_0_r_data; // @[anonymous source 333:4]
  wire [1:0] ram_RW_0_r_addr; // @[anonymous source 333:4]
  wire [7:0] ram_RW_0_w_data; // @[anonymous source 333:4]
  wire [1:0] ram_RW_0_w_addr; // @[anonymous source 333:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 333:4]
  wire  ram_RW_0_w_en; // @[anonymous source 333:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [1:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 333:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 345:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 333:4]
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
    ram[initvar] = _RAND_0[7:0];
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
module split_cc_banks_0_ext( // @[anonymous source 348:2]
  input  [5:0]  RW0_addr, // @[anonymous source 349:4]
  input         RW0_clk, // @[anonymous source 350:4]
  input  [63:0] RW0_wdata, // @[anonymous source 351:4]
  output [63:0] RW0_rdata, // @[anonymous source 352:4]
  input         RW0_en, // @[anonymous source 353:4]
  input         RW0_wmode // @[anonymous source 354:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [63:0] ram [0:63]; // @[anonymous source 356:4]
  wire [63:0] ram_RW_0_r_data; // @[anonymous source 356:4]
  wire [5:0] ram_RW_0_r_addr; // @[anonymous source 356:4]
  wire [63:0] ram_RW_0_w_data; // @[anonymous source 356:4]
  wire [5:0] ram_RW_0_w_addr; // @[anonymous source 356:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 356:4]
  wire  ram_RW_0_w_en; // @[anonymous source 356:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [5:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 356:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 368:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 356:4]
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
module split_data_arrays_0_ext( // @[anonymous source 371:2]
  input  [5:0] RW0_addr, // @[anonymous source 372:4]
  input        RW0_clk, // @[anonymous source 373:4]
  input  [7:0] RW0_wdata, // @[anonymous source 374:4]
  output [7:0] RW0_rdata, // @[anonymous source 375:4]
  input        RW0_en, // @[anonymous source 376:4]
  input        RW0_wmode, // @[anonymous source 377:4]
  input        RW0_wmask // @[anonymous source 378:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [7:0] ram [0:63]; // @[anonymous source 380:4]
  wire [7:0] ram_RW_0_r_data; // @[anonymous source 380:4]
  wire [5:0] ram_RW_0_r_addr; // @[anonymous source 380:4]
  wire [7:0] ram_RW_0_w_data; // @[anonymous source 380:4]
  wire [5:0] ram_RW_0_w_addr; // @[anonymous source 380:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 380:4]
  wire  ram_RW_0_w_en; // @[anonymous source 380:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [5:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 380:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 392:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 380:4]
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
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[7:0];
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
module split_tag_array_ext( // @[anonymous source 395:2]
  input  [2:0]  RW0_addr, // @[anonymous source 396:4]
  input         RW0_clk, // @[anonymous source 397:4]
  input  [24:0] RW0_wdata, // @[anonymous source 398:4]
  output [24:0] RW0_rdata, // @[anonymous source 399:4]
  input         RW0_en, // @[anonymous source 400:4]
  input         RW0_wmode // @[anonymous source 401:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [24:0] ram [0:7]; // @[anonymous source 403:4]
  wire [24:0] ram_RW_0_r_data; // @[anonymous source 403:4]
  wire [2:0] ram_RW_0_r_addr; // @[anonymous source 403:4]
  wire [24:0] ram_RW_0_w_data; // @[anonymous source 403:4]
  wire [2:0] ram_RW_0_w_addr; // @[anonymous source 403:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 403:4]
  wire  ram_RW_0_w_en; // @[anonymous source 403:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [2:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 403:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 415:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 403:4]
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
module split_tag_array_0_ext( // @[anonymous source 418:2]
  input  [2:0]  RW0_addr, // @[anonymous source 419:4]
  input         RW0_clk, // @[anonymous source 420:4]
  input  [23:0] RW0_wdata, // @[anonymous source 421:4]
  output [23:0] RW0_rdata, // @[anonymous source 422:4]
  input         RW0_en, // @[anonymous source 423:4]
  input         RW0_wmode, // @[anonymous source 424:4]
  input         RW0_wmask // @[anonymous source 425:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [23:0] ram [0:7]; // @[anonymous source 427:4]
  wire [23:0] ram_RW_0_r_data; // @[anonymous source 427:4]
  wire [2:0] ram_RW_0_r_addr; // @[anonymous source 427:4]
  wire [23:0] ram_RW_0_w_data; // @[anonymous source 427:4]
  wire [2:0] ram_RW_0_w_addr; // @[anonymous source 427:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 427:4]
  wire  ram_RW_0_w_en; // @[anonymous source 427:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [2:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 427:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 439:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 427:4]
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
    ram[initvar] = _RAND_0[23:0];
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
module split_data_arrays_0_0_ext( // @[anonymous source 442:2]
  input  [5:0]  RW0_addr, // @[anonymous source 443:4]
  input         RW0_clk, // @[anonymous source 444:4]
  input  [31:0] RW0_wdata, // @[anonymous source 445:4]
  output [31:0] RW0_rdata, // @[anonymous source 446:4]
  input         RW0_en, // @[anonymous source 447:4]
  input         RW0_wmode, // @[anonymous source 448:4]
  input         RW0_wmask // @[anonymous source 449:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [31:0] ram [0:63]; // @[anonymous source 451:4]
  wire [31:0] ram_RW_0_r_data; // @[anonymous source 451:4]
  wire [5:0] ram_RW_0_r_addr; // @[anonymous source 451:4]
  wire [31:0] ram_RW_0_w_data; // @[anonymous source 451:4]
  wire [5:0] ram_RW_0_w_addr; // @[anonymous source 451:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 451:4]
  wire  ram_RW_0_w_en; // @[anonymous source 451:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [5:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 451:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 463:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 451:4]
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
  for (initvar = 0; initvar < 64; initvar = initvar+1)
    ram[initvar] = _RAND_0[31:0];
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
module split_l2_tlb_ram_ext( // @[anonymous source 466:2]
  input  [5:0]  RW0_addr, // @[anonymous source 467:4]
  input         RW0_clk, // @[anonymous source 468:4]
  input  [47:0] RW0_wdata, // @[anonymous source 469:4]
  output [47:0] RW0_rdata, // @[anonymous source 470:4]
  input         RW0_en, // @[anonymous source 471:4]
  input         RW0_wmode // @[anonymous source 472:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [47:0] ram [0:63]; // @[anonymous source 474:4]
  wire [47:0] ram_RW_0_r_data; // @[anonymous source 474:4]
  wire [5:0] ram_RW_0_r_addr; // @[anonymous source 474:4]
  wire [47:0] ram_RW_0_w_data; // @[anonymous source 474:4]
  wire [5:0] ram_RW_0_w_addr; // @[anonymous source 474:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 474:4]
  wire  ram_RW_0_w_en; // @[anonymous source 474:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [5:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 474:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = 1'h1;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 486:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 474:4]
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
module split_mem_ext( // @[anonymous source 489:2]
  input  [8:0] RW0_addr, // @[anonymous source 490:4]
  input        RW0_clk, // @[anonymous source 491:4]
  input  [7:0] RW0_wdata, // @[anonymous source 492:4]
  output [7:0] RW0_rdata, // @[anonymous source 493:4]
  input        RW0_en, // @[anonymous source 494:4]
  input        RW0_wmode, // @[anonymous source 495:4]
  input        RW0_wmask // @[anonymous source 496:4]
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [7:0] ram [0:511]; // @[anonymous source 498:4]
  wire [7:0] ram_RW_0_r_data; // @[anonymous source 498:4]
  wire [8:0] ram_RW_0_r_addr; // @[anonymous source 498:4]
  wire [7:0] ram_RW_0_w_data; // @[anonymous source 498:4]
  wire [8:0] ram_RW_0_w_addr; // @[anonymous source 498:4]
  wire  ram_RW_0_w_mask; // @[anonymous source 498:4]
  wire  ram_RW_0_w_en; // @[anonymous source 498:4]
  reg  ram_RW_0_r_en_pipe_0;
  reg [8:0] ram_RW_0_r_addr_pipe_0;
  wire  _GEN_0 = ~RW0_wmode;
  wire  _GEN_1 = ~RW0_wmode;
  assign ram_RW_0_r_addr = ram_RW_0_r_addr_pipe_0;
  assign ram_RW_0_r_data = ram[ram_RW_0_r_addr]; // @[anonymous source 498:4]
  assign ram_RW_0_w_data = RW0_wdata;
  assign ram_RW_0_w_addr = RW0_addr;
  assign ram_RW_0_w_mask = RW0_wmask;
  assign ram_RW_0_w_en = RW0_en & RW0_wmode;
  assign RW0_rdata = ram_RW_0_r_data; // @[anonymous source 510:4]
  always @(posedge RW0_clk) begin
    if(ram_RW_0_w_en & ram_RW_0_w_mask) begin
      ram[ram_RW_0_w_addr] <= ram_RW_0_w_data; // @[anonymous source 498:4]
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
  for (initvar = 0; initvar < 512; initvar = initvar+1)
    ram[initvar] = _RAND_0[7:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  ram_RW_0_r_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  ram_RW_0_r_addr_pipe_0 = _RAND_2[8:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
