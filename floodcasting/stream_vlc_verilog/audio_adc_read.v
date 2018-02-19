
module audio_adc_read(
	input clk,
	input resetn,
	// serial ADC
	output reg cs,
	output reg sclk,
	input SDATA,
	//
	output reg [15:0] data_out,
	output reg ready,
	output reg dat_valid,
	input  start_conv
);

reg	[2:0] state, next_state;
reg	next_sclk, next_cs;
reg [4:0] bit_cnt, next_bit_cnt;
reg [15:0] next_data_out;
reg next_ready, next_dat_valid;
reg [1:0] clk_dividor;


always @ (posedge clk)
begin
	if (~resetn)
	begin
		cs <= 1;
		sclk <= 1;
		bit_cnt <= 0;
		data_out <= 0;
		ready <= 1;
		dat_valid <= 0;
		clk_dividor <= 0;
		state <=0;
	end
	else
	begin
		if (clk_dividor==3)
		begin
			cs <= next_cs;
			sclk <= next_sclk;
			bit_cnt <= next_bit_cnt;
			data_out <= next_data_out;
			ready <= next_ready;
			dat_valid <= next_dat_valid;
			state <= next_state;
			clk_dividor <= 0;
		end
		else
			clk_dividor <= clk_dividor + 1;
	end
end

always @ *
begin
	next_cs = cs;
	next_sclk = sclk;
	next_bit_cnt = bit_cnt;
	next_data_out = data_out;
	next_ready = ready;
	next_dat_valid = dat_valid;
	next_state = state;

	case (state)
		0:
		begin
			next_bit_cnt = 0;
			next_sclk = 1;
			if (start_conv)
			begin
				next_ready = 0;
				next_state = 1;
				next_cs = 0;
				next_dat_valid = 0;
			end
		end

		1:
		begin
			next_sclk = ~sclk;
			if (~sclk)
				next_bit_cnt = bit_cnt + 1;
			if (bit_cnt==3)
				next_state = 2;
		end

		2:
		begin
			next_sclk = ~sclk;
			if (~sclk)
				next_bit_cnt = bit_cnt + 1;
			else
				next_data_out = {data_out[14:0], SDATA};

			if (bit_cnt==19)
				next_state = 3;
		end

		3:
		begin
			if (bit_cnt==24)
				next_state = 4;
			else
			begin
				next_sclk = ~sclk;
				if (~sclk)
					next_bit_cnt = bit_cnt + 1;
			end
		end

		4:
		begin
			next_cs = 1;
			next_dat_valid = 1;
			next_state = 5;
		end

		5:
		begin
			next_ready = 1;
			next_state = 0;
		end
		
	endcase
end
endmodule
