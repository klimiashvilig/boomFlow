check: compile
	./simv
	./a.out

compile:
	vcs -full64 -sverilog -timescale=1ns/1ps -debug_access+pp testbench/AESTopTb.sv rtl/AESTopFinal.v > compile_tb.log
	gcc -std=c99 -Wall testbench/aes256/gold_test.c testbench/aes256/aes256.c > compile_c.log

clean:
	rm -rf ./simv
	rm -rf output.txt
	rm -rf a.out