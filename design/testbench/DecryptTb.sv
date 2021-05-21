`define CLK_PERIOD 50
`define ASSIGNMENT_DELAY 5
`define FINISH_TIME 2000000
`define NUM_TEST_VECTORS 100

module DecryptTb;
 
  reg clk;
  reg [127 : 0] data_in;
  wire [127 : 0] data_out;
  reg [255 : 0] key;
  wire [  127:0] key_out [14:0];

  always #(`CLK_PERIOD/2) clk =~clk;
  
  AESDecrypt AESDecrypt
  (
    .data_in(data_in),
    .key(key_out),
    .data_out(data_out)
  );

  ExpandKey ExpandKey
  (
    .key_in(key),
    .key_out(key_out)
  );

  initial begin
    clk <= 0;
    #20 
    data_in <= 128'ha52422117500d3e82c96d0dafc491931;
    key <= 256'h1212121269696969343434343434343456565656565656567878787878787878;
    #20 
    if(data_out != 128'h1212121234343434ababababcdcdcdcd) begin
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
    $vcdpluson(0, DecryptTb);
    #(`FINISH_TIME);
    #20000000;
    $finish(2);
  end

endmodule 
