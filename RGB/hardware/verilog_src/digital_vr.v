module vr(	addr, ud, clk, resetn, ready, start,
			LED_R_GAIN_CS, LED_G_GAIN_CS, LED_B_GAIN_CS,
			LED_R_REF_CS, LED_G_REF_CS, LED_B_REF_CS,
			UD, CLKOUT);
input	[2:0]	addr;
input			ud, start;
input			clk, resetn;
output			ready;

output			LED_R_GAIN_CS, LED_G_GAIN_CS, LED_B_GAIN_CS;
output			LED_R_REF_CS, LED_G_REF_CS, LED_B_REF_CS;
output			UD, CLKOUT;

reg		[2:0]	state, next_state;
reg				CLKOUT, next_CLKOUT;
reg				CS, next_CS;
reg				UD, next_UD;
reg				ready, next_ready;

assign LED_R_GAIN_CS = (addr==3'b000)? CS : 1;
assign LED_G_GAIN_CS = (addr==3'b001)? CS : 1;
assign LED_B_GAIN_CS = (addr==3'b010)? CS : 1;
assign LED_R_REF_CS  = (addr==3'b011)? CS : 1;
assign LED_G_REF_CS  = (addr==3'b100)? CS : 1;
assign LED_B_REF_CS  = (addr==3'b101)? CS : 1;

`define SD #1

always @ (posedge clk)
begin
	if (~resetn)
	begin
		state <= `SD 0;
		CLKOUT <= `SD 1;
		CS <= `SD 1;
		UD <= `SD 0;
		ready <= `SD 0;
	end
	else
	begin
		state <= `SD next_state;
		CLKOUT <= `SD next_CLKOUT;
		CS <= `SD next_CS;
		UD <= `SD next_UD;
		ready <= `SD next_ready;
	end
end

always @ *
begin
	next_state = state;
	next_CLKOUT = CLKOUT;
	next_CS = CS;
	next_UD = UD;
	next_ready = ready;
	case (state)
	0:
	begin
		next_ready = 1;
		next_CLKOUT = 1;
		if (start)
		begin
			next_state = 1;
			next_CS = 0;
			next_UD = ud;
			next_ready = 0;
		end
	end

	1:
	begin
		next_CLKOUT = 0;
		next_state = 2;
	end

	2:
	begin
		next_CS = 1;
		next_state = 3;
	end

	3:
	begin
		next_CLKOUT = 1;
		next_state = 0;
	end

	endcase
end
endmodule
