
`timescale 1ns/1ps

module adc_read(clk, resetn, pwr, Data_in, EOC, start,
				A0, A1, CONVST, PD, RD, CS, ready,
				ch0_data, ch1_data, ch2_data);

input			clk, resetn;
input			pwr;
input	[7:0]	Data_in;
input			EOC;

input			start;

output			A0, A1;
output			CONVST;
output			PD;
output			RD, CS;

output			ready;
output	[7:0]	ch0_data, ch1_data, ch2_data;

reg				CONVST, next_CONVST;
reg				PD, next_PD;
reg				RD, next_RD, CS, next_CS;
reg		[10:0]	counter, next_counter;
reg		[3:0]	state, next_state;
reg				ready, next_ready;
reg		[1:0]	addr, next_addr;
reg		[7:0]	ch0_data, ch1_data, ch2_data;
reg		[7:0]	next_ch0_data, next_ch1_data, next_ch2_data;

parameter	CLK_FREQ = 50;			// cannot go faster than 50MHz
parameter	PD_DELAY = CLK_FREQ*25;	// 25 us for power up

assign A0 = addr[0];
assign A1 = addr[1];

`define SD #1

parameter OFF = 0;
parameter BOOT = 1;
parameter READY_TO_SAMPLE = 2;
parameter SAMPLING = 3;
parameter WAITING = 4;
parameter ADDR_INC = 5;
parameter PREPARE_READ = 6;
parameter PREPARE_READ1 = 7;
parameter READ = 8;
parameter DONE_READING = 9;
parameter CHECK_ADDR = 10;

always @ (posedge clk)
begin
	if (~resetn)
	begin
		CONVST	<= 1;
		PD		<= 0;	// turns off by default
		RD		<= 1;
		CS		<= 1;
		state	<= OFF;
		counter	<= 0;
		ready	<= 0;
		addr	<= 0;
		ch0_data<= 0;
		ch1_data<= 0;
		ch2_data<= 0;
	end
	else
	begin
		CONVST	<= `SD next_CONVST;
		PD		<= `SD next_PD;
		RD		<= `SD next_RD;
		CS		<= `SD next_CS;
		state	<= `SD next_state;
		counter	<= `SD next_counter;
		ready	<= `SD next_ready;
		addr	<= `SD next_addr;
		ch0_data<= `SD next_ch0_data;
		ch1_data<= `SD next_ch1_data;
		ch2_data<= `SD next_ch2_data;
	end
end

always @ *
begin
	next_CONVST = CONVST;
	next_PD = PD;
	next_RD = RD;
	next_CS = CS;
	next_state = state;
	next_counter = counter;
	next_ready = ready;
	next_addr = addr;
	next_ch0_data = ch0_data;
	next_ch1_data = ch1_data;
	next_ch2_data = ch2_data;

	case (state)
		OFF:
		begin
			next_counter = 0;
			next_PD = 0;
			if (pwr)
				next_state = BOOT;
		end

		BOOT:
		begin
			next_counter = counter + 1;
			next_PD = 1;
			next_CONVST = 1;
			if (counter==PD_DELAY)
				next_state = READY_TO_SAMPLE;
		end

		READY_TO_SAMPLE:
		begin
			next_ready = 1;
			if (start)
				next_state = SAMPLING;
			else if (~pwr)
				next_state = OFF;
		end

		SAMPLING:
		begin
			next_ready = 0;
			next_CONVST = 0;
			next_state = WAITING;
		end

		WAITING:
		begin
			next_CONVST = 1;
			if (~EOC)
				next_state = ADDR_INC;
		end

		ADDR_INC:
		begin
			next_CS = 0;
			if (addr==2)
				next_addr = 0;
			else
				next_addr = addr + 1;
			next_state = PREPARE_READ;
		end

		PREPARE_READ:
		begin
			next_RD = 0;
			next_state = PREPARE_READ1;
		end

		PREPARE_READ1:
		begin
			next_state = READ;
		end

		READ:
		begin
			next_RD = 1;
			case(addr)
				0: begin next_ch2_data = Data_in; end
				1: begin next_ch0_data = Data_in; end
				2: begin next_ch1_data = Data_in; end
			endcase
			next_state = DONE_READING;
		end

		DONE_READING:
		begin
			next_CS = 1;
			next_state = CHECK_ADDR;
		end

		CHECK_ADDR:
		begin
			if (addr==0)
				next_state = READY_TO_SAMPLE;
			else
				next_state = SAMPLING;
		end

	endcase
end

endmodule
