`define CLK_PERIOD 20
`define NUM_TESTS 2
`define ASSIGNMENT_DELAY 5
`define FINISH_TIME 2000000

module MixColumnsTb;
 
  reg clk;
  reg rst;
  reg  [127 : 0] data_in;
  wire [127 : 0] data_out;

  always #(`CLK_PERIOD/2) clk =~clk;
  MixColumns MixColumns
(
  .in(data_in),
  .out(data_out)
);
  
  initial begin
    clk <= 0;
    rst <= 0;
    #20 
    data_in <= 128'h634cdb994cdb9963db99634c99634cdb; 
    #20 
    if(data_out != 128'h5014321b14321b50321b50141b501432) begin
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
    $vcdpluson(0, MixColumnsTb);
    #(`FINISH_TIME);
    #20000000;
    $finish(2);
  end

endmodule 
