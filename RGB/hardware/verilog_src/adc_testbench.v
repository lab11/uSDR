
module testbench(clk, resetn, Data_out, EOC, A0, A1, CONVST, PD, RD, CS);

input	clk, resetn;
input	A0, A1, CONVST, PD, RD, CS;
output	EOC;
output	[7:0] Data_out;

reg		EOC, next_EOC;
reg		[7:0] Data_out, next_Data_out;

reg		[1:0] adc_addr, next_adc_addr;
reg		[2:0] state, next_state;
reg		[7:0] sample_rom;
reg		[7:0] sample, next_sample;

// clk = 40MHz
always @ (posedge clk)
begin
	if (~resetn)
	begin
		adc_addr <= 0;
		state <= 0;
		Data_out <= 0;
		EOC <= 1;
		sample <= 0;
	end
	else
	begin
		adc_addr <= next_adc_addr;
		state <= next_state;
		Data_out <= next_Data_out;
		EOC <= next_EOC;
		sample <= next_sample;
	end
end

always @ *
begin
	next_adc_addr = adc_addr;
	next_state = state;
	next_Data_out = Data_out;
	next_EOC = EOC;
	next_sample = sample;

	case (state)
		0:
		begin
			if ((~CONVST) & (PD))
			begin
				next_state = 1;
				next_sample = sample_rom;
			end
		end

		1:
		begin
			next_state = 2;
		end

		2:
		begin
			next_EOC = 0;
			next_state = 3;
		end

		3:
		begin
			if (~(RD|CS))
			begin
				next_adc_addr = {A1, A0};
				next_state = 4;
				next_Data_out = sample;
			end
		end

		4:
		begin
			if (RD)
				next_state = 5;
		end

		5:
		begin
			next_EOC = 1;
			next_state = 6;
		end

		6:
		begin
			next_state = 0;
		end
	endcase
end

always @ *
begin
	sample_rom = 0;
	case (adc_addr)
		0: begin sample_rom = 8'h12; end
		1: begin sample_rom = 8'h34; end
		2: begin sample_rom = 8'h56; end
		default: begin sample_rom = 8'hff; end
	endcase
end

endmodule
