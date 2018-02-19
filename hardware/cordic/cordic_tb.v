module tb();

reg	clk, resetn, start;
reg	signed [7:0] x, y;
wire signed [15:0] angle;
reg [3:0] counter;
reg [8:0] trial;
reg [31:0] input_rand;
wire done, ready;
integer seed;

cordic2 c0(clk, resetn, start, x, y, angle, done, ready);
`define SD 2

initial begin
	clk = 0;
	resetn = 0;
	start = 0;
	counter = 0;
	trial = 0;
	input_rand = 0;
	seed = 128;
	#20
		resetn = 1;
end

always #5 clk = ~clk;

always @ (posedge clk)
begin
	if (counter==11)
	begin
		if (ready)
		begin
			input_rand = $random(seed);
			x <= #`SD input_rand[7:0];
			y <= #`SD input_rand[15:8];
			start <= #`SD 1;
			trial <= #`SD trial + 1;
			if (trial==30)
				$finish;
		end
		counter <= #`SD 0;
	end
	else
		counter <= #`SD counter + 1;

	if (start==1)
		start <= #`SD 0;
	
end

endmodule
