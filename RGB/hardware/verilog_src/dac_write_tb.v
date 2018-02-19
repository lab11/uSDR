module testbench();


reg				clk, resetn;
reg		[7:0]	ch0, ch1, ch2, ch3;
reg				start, pwr, clear;
wire			ready;

wire			CS, WR, GAIN, LDAC, CLR, PD;
wire	[7:0]	DATA;
wire			A0, A1;

dac_write dac0(clk, resetn, ch0, ch1, ch2, ch3, start, pwr, ready, clear,
					CS, WR, DATA, GAIN, LDAC, CLR, PD, A0, A1);

initial
begin
	clk = 0;
	resetn = 1;
	ch0 = 0;
	ch1 = 0;
	ch2 = 0;
	ch3 = 0;
	start = 0;
	pwr = 0;
	clear = 0;

	#3
	resetn = 0;

	@(negedge clk)
	@(negedge clk)
	@(negedge clk)
	resetn = 1;

	@(posedge clk)
	#3
	pwr = 1;

	@(posedge ready)
	#3
	ch0 = 123;
	ch1 = 234;
	ch2 =  12;
	ch3 = 125;
	start = 1;

	@(negedge ready)
	start = 0;

	@(posedge ready)
	#3
	ch0 = 23;
	ch1 = 24;
	ch2 = 112;
	ch3 = 12;
	start = 1;

	@(negedge ready)
	start = 0;

	@(posedge ready)
	$stop;
end

always #5 clk = ~clk;

endmodule
