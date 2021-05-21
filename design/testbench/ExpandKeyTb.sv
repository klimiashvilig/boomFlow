`define CLK_PERIOD 20
`define NUM_TESTS 2
`define ASSIGNMENT_DELAY 5
`define FINISH_TIME 2000000

module ExpandKeyTb;
 
  reg clk;
  reg rst;
  reg  [255 : 0] key;
  wire [127 : 0] key_out [14:0];
  reg key_ready;

  wire key_valid;

  always #(`CLK_PERIOD/2) clk =~clk;
  ExpandKey ExpandKey_inst
    (
      .clk(clk),
      .rst(rst),
      .ready(key_ready),
      .key_in(key),
      .key_out(key_out),
      .valid(key_valid)
    );
//   ExpandKey ExpandKey
// (
//   .key_in(key),
//   .key_out(key_out)
// );

  initial begin
    clk <= 0;
    rst <= 0;
    #20 rst <= 1;
    #20 rst <= 0;
    key_ready<= 1;
    key <= 256'h1212121269696969343434343434343456565656565656567878787878787878;
    #20 key_ready <= 0;
    #20 
    #20
    #20
    #20
    if(key_out[8] != 128'h981ccf1e5edf501fb532e109b536b9cf) begin
      $display("FAIL");
    end else begin 
      $display("PASS");
    end


    #3000
    $finish;
  end


  initial begin
    $vcdplusfile("dump.vcd");
    $vcdplusmemon();
    $vcdpluson(0, ExpandKeyTb);
    #(`FINISH_TIME);
    #20000000;
    $finish(2);
  end

endmodule 
