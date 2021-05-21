`define CLK_PERIOD 20
`define NUM_TESTS 2
`define ASSIGNMENT_DELAY 5
`define FINISH_TIME 2000000

module AddRoundKeyTb;
 
  reg clk;
  reg rst;
  reg  [127 : 0] data_in;
  reg  [127 : 0] key;
  wire [127 : 0] data_out;

  always #(`CLK_PERIOD/2) clk =~clk;

  AddRoundKey AddRoundKey(.in(data_in), .key(key), .out(data_out));
  
  initial begin
    clk <= 0;
    rst <= 0;
    #20 
    data_in <= 128'h5014321b14321b50321b50141b501432; 
    key <= 256'h56565656565656567878787878787878;
    #20 
    if(data_out != 128'h0642644d42644d064a63286c63286c4a) begin
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
    $vcdpluson(0, AddRoundKeyTb);
    #(`FINISH_TIME);
    #20000000;
    $finish(2);
  end

endmodule 
