
module dac_write(clk, resetn, ch0, ch1, ch2, ch3, start, pwr, ready, clear, num_of_channels,
					CS, WR, DATA, GAIN, LDAC, CLR, PD, A0, A1);

// This module take 4 channels at the same time, asynchronous updates the output
// clk cannot faster than 50 MHz
// num_of_channel = 3 -> 4 channels, 0 = 1 channel

input			clk, resetn;
input	[7:0]	ch0, ch1, ch2, ch3;
input			start, pwr, clear;
input	[1:0]	num_of_channels;
output			ready;

output			CS, WR, GAIN, LDAC, CLR, PD;
output	[7:0]	DATA;
output			A0, A1;


reg		[7:0]	ch0_reg, next_ch0_reg, ch1_reg, next_ch1_reg, ch2_reg, next_ch2_reg, ch3_reg, next_ch3_reg;
reg				ready, next_ready;
reg				CS, next_CS, WR, next_WR, LDAC, next_LDAC, CLR, next_CLR, PD, next_PD;
reg		[7:0]	DATA, next_DATA;
reg		[3:0]	state, next_state;
reg		[1:0]	ch_counter, next_ch_counter;

assign A0 = ch_counter[0];
assign A1 = ch_counter[1];

assign GAIN = 0;	// output range from 0~Vref

parameter OFF = 0;
parameter CLEAR = 1;
parameter INIT = 2;
parameter READY = 3;
parameter TX_INIT0 = 5;
parameter TX_INIT1 = 6;
parameter TX_INIT2 = 7;
parameter WRITE_TO_INPUT_REG = 8;
parameter PREPARE_NEXT = 9;
parameter CHECK_UPDATE = 10;
parameter BACK_TO_READY = 13;

`define SD #1

always @ (posedge clk)
begin
	if (~resetn)
	begin
		ch0_reg <= 0;
		ch1_reg <= 0;
		ch2_reg <= 0;
		ch3_reg <= 0;
		ready <= 0;
		CS <= 1;
		WR <= 1;
		LDAC <= 1;
		CLR <= 0;		// clear all registers
		PD <= 0;		// default off
		DATA <= 0;
		state <= OFF;
		ch_counter <= 0;
	end
	else
	begin
		ch0_reg <= `SD next_ch0_reg;
		ch1_reg <= `SD next_ch1_reg;
		ch2_reg <= `SD next_ch2_reg;
		ch3_reg <= `SD next_ch3_reg;
		ready	<= `SD next_ready;
		CS 		<= `SD next_CS;
		WR 		<= `SD next_WR;
		LDAC 	<= `SD next_LDAC;
		CLR 	<= `SD next_CLR;
		PD 		<= `SD next_PD;
		DATA	<= `SD next_DATA;
		state	<= `SD next_state;
		ch_counter <= `SD next_ch_counter;
	end
end

always @ *
begin
	next_ch0_reg = ch0_reg;
	next_ch1_reg = ch1_reg;
	next_ch2_reg = ch2_reg;
	next_ch3_reg = ch3_reg;
	next_ready = ready;
	next_CS = CS;
	next_WR = WR;
	next_LDAC = LDAC;
	next_CLR = CLR;
	next_PD = PD;
	next_DATA = DATA;
	next_state = state;
	next_ch_counter = ch_counter;

	case (state)
		OFF:
		begin
			next_CLR = 0;
			next_PD = 0;
			if (pwr)
				next_state = INIT;
		end

		CLEAR:
		begin
			next_CLR = 0;
			next_state = READY;
		end

		INIT:
		begin
			next_CLR = 1;	// exit CLR
			next_PD = 1;	// power up
			next_state = READY;
		end

		READY:
		begin
			next_ready = 1;
			if (start)
			begin
				next_ready = 0;
				next_ch0_reg = ch0;
				next_ch1_reg = ch1;
				next_ch2_reg = ch2;
				next_ch3_reg = ch3;
				next_state = TX_INIT0;
				next_ch_counter = 0;
			end
			else if (~pwr)
				next_state = OFF;
			else if (clear)
				next_state = CLEAR;
		end

		TX_INIT0:
		begin
			next_CS = 0;
			next_state = TX_INIT1;
		end

		TX_INIT1:
		begin
			next_WR = 0;
			next_state = TX_INIT2;
		end

		TX_INIT2:
		begin
			case (ch_counter)
				0: 	begin next_DATA = ch0_reg; 	end
				1: 	begin next_DATA = ch1_reg; 	end
				2: 	begin next_DATA = ch2_reg; 	end
				3: 	begin next_DATA = ch3_reg; 	end
			endcase
			next_state = WRITE_TO_INPUT_REG;
		end

		WRITE_TO_INPUT_REG:
		begin
			next_WR = 1;
			next_state = PREPARE_NEXT;
		end

		PREPARE_NEXT:
		begin
			next_CS = 1;
			next_state = CHECK_UPDATE;
		end

		CHECK_UPDATE:
		begin
			if (ch_counter==num_of_channels)
			begin
				next_LDAC = 0;
				next_state = BACK_TO_READY;
			end
			else
			begin
				next_ch_counter = ch_counter + 1;
				next_state = TX_INIT0;
			end
		end

		BACK_TO_READY:
		begin
			next_LDAC = 1;
			next_state = READY;
		end

	endcase
end

endmodule
