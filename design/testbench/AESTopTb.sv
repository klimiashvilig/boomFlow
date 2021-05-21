`define CLK_PERIOD 20
`define NUM_TESTS 2
`define ASSIGNMENT_DELAY 5
`define FINISH_TIME 2000000

module AESTopTb;
 
  reg clk;
  reg rst;
  reg output_ready , input_valid;
  wire input_ready , output_valid;
  wire  [255 : 0] data_in;
  wire [127 : 0] data_out;
  reg [127 : 0]  saved_out;
  wire busy;
  reg  [6:0] opcode; 

  always #(`CLK_PERIOD/2) clk =~clk;
  
  AESTop AESTop_inst
  (
    .clk(clk),
    .rst(rst),
    .input_valid(input_valid),
    .output_ready(output_ready),
    .opcode(opcode),
    .data_in(data_in), //end of input 
    .data_out(data_out),
    .input_ready(input_ready),
    .output_valid(output_valid),
    .busy(busy)
  );

    reg [127:0] encrypt_mem [`NUM_TESTS - 1:0];
    reg [127:0] decrypt_mem [`NUM_TESTS - 1:0];
    reg [255:0] key_mem [`NUM_TESTS - 1:0];
    reg [$clog2(`NUM_TESTS) + 1:0] data_idx;
    reg [$clog2(`NUM_TESTS):0] key_idx;
    reg [$clog2(`NUM_TESTS):0] num_incorrect;
    integer  f;
    
    initial begin
        $readmemh("testbench/inputs.mem", encrypt_mem);
    end

    initial begin
        $readmemh("testbench/key.mem", key_mem);
    end

  assign data_in = (opcode == 0) ? key_mem[key_idx] : ((opcode == 1) ? {encrypt_mem[data_idx], 128'd0} : {decrypt_mem[data_idx], 128'd0}); 

  always_ff @(posedge clk, posedge rst) begin
      if(rst) begin
        output_ready <= 1; 
        opcode = 0; 
        data_idx <= 0;
        key_idx <= 0;
        input_valid <= 1'b0; 
        num_incorrect <= 0;
      end
      else begin
        input_valid <= 1'b1; 
          if(output_valid && data_idx < `NUM_TESTS) begin
            if(opcode == 0) begin
              opcode <= 1; 
              key_idx <= key_idx + 1;
            end else if(opcode == 1) begin 
              decrypt_mem[data_idx] <= data_out;
              //$display("%h%h%h%h", data_out[127:96], data_out[95:64], data_out[63:32], data_out[31:0]); 
              $fwrite(f,"%h%h%h%h\n", data_out[127:96], data_out[95:64], data_out[63:32], data_out[31:0]); 
              opcode <= 2;  
            end else if(opcode == 2) begin 
              opcode <= 0;
              if (encrypt_mem[data_idx] != data_out) begin
                num_incorrect<= num_incorrect + 1;
              end
              data_idx <= data_idx + 1; 
            end
          end
          if (data_idx == `NUM_TESTS) begin 
            if (num_incorrect > 0) begin
              $display("FAIL");
            end else begin
              $display("PASS");
            end
            data_idx <= data_idx + 1;
          end
      end
  end
  initial begin
    f = $fopen("output.txt");
    clk <= 0;
    rst <= 0;
    #20 rst <= 1;
    #20 rst <= 0; 

    #30000
    $finish;
  end


  initial begin
    $vcdplusfile("dump.vcd");
    $vcdplusmemon();
    $vcdpluson(0, AESTopTb);
    #(`FINISH_TIME);
    #20000000;
    $finish(2);
  end

endmodule 
