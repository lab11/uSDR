module testbench();

reg		clk, resetn;
reg		[15:0] data_a, data_b;
reg		start;
reg		debug;
reg		[2:0] command;
reg		[2:0] addr;
wire	ready;
wire	ldac;
wire	clr;
wire	din;
wire	sclk;
wire	sync;

audio_dac_write dac0( clk, resetn, data_a, data_b, start, debug, command, addr, ready, ldac, clr, din, sclk, sync);

initial
begin
	clk = 0;
	resetn = 1;
	data_a = 0;
	data_b = 0;
	start = 0;
	debug = 0;
	command = 0;
	addr = 0;

	@ (posedge clk)
	@ (posedge clk)
	@ (posedge clk)
		#1 resetn = 0;

	@ (posedge clk)
	@ (posedge clk)
	@ (posedge clk)
		#1 resetn = 1;
		data_a = 16'habcd;
		data_b = 16'h1234;
	@ (posedge clk)
		#1 start = 1;
	@ (negedge ready)
		start = 0;
	
	@ (posedge ready)
		#100 
		data_a = 16'hfa5f;
		addr = 3'b110;
		command = 3'b010;
		debug = 1;
	@ (posedge clk)
		#1 start = 1;
	@ (negedge ready)
		start = 0;
	
	@ (posedge ready)
		#100 $stop;

	
		
	
	
end

always #5 clk = ~clk;
endmodule
