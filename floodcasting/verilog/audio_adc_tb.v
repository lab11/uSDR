module testbench();

reg clk, resetn, din, start_conv;
wire cs, sclk, ready, dat_valid;
wire [15:0] dout;

audio_adc_read adc0(.clk(clk), .resetn(resetn), .cs(cs), .sclk(sclk), .SDATA(din), .data_out(dout), .ready(ready), .dat_valid(dat_valid), .start_conv(start_conv));

reg [1:0] state;
parameter TASK0 = 0;
parameter WAIT = 3;

initial
begin
	clk = 0;
	resetn = 0;
	din = 0;
	@ (posedge clk)
	@ (posedge clk)
	@ (posedge clk)
	@ (posedge clk)
	#1 resetn = 1;
	@ (posedge clk)
	@ (posedge clk)
	@ (posedge clk)
	@ (posedge clk)
	#1 state = TASK0;
	@ (posedge dat_valid)
	#1000
	@ (posedge clk)
	#1 state = TASK0;
	@ (posedge dat_valid)
	$stop;
end

always #5 clk = ~clk;

always @ (posedge clk)
begin
	if (~resetn)
	begin
		start_conv <= 0;
	end
	else
	begin
		case (state)
			TASK0:
			begin
				if (ready)
				begin
					start_conv <= 1;
					state <= WAIT;
				end
			end

			WAIT:
			begin
				if (~ready)
					start_conv <= 0;
			end
		endcase
	end
end

always @ (negedge sclk)
begin
	din <= $random;
end

endmodule
