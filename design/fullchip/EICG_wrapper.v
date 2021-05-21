/* verilator lint_off UNOPTFLAT */

module EICG_wrapper(
  output out,
  input en,
  input test_en,
  input in
);
  
  /*
  reg en_latched // verilator clock_enable;

  always @(*) begin
     if (!in) begin
        en_latched = en || test_en;
     end
  end

  assign out = en_latched && in;
  */
  sky130_fd_sc_hd__dlclkp_1 clk_gate (.GCLK(out), .GATE(en), .CLK(in));

endmodule
