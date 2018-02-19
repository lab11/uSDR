/*
 * "Copyright (c) 2010-2011 The Regents of the University of Michigan.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @author Ye-Sheng Kuo <samkuo@eecs.umich.edu>
 * Last modify: 9/19 '12
 * Note: only work for uSDRv2 board, with parallel gain control
 */

module agc(
	input			adc_clock, 
	input			HRESETn,
	input			rx_agc_en,
	input			sdata,
	input	[7:0]	input_pwr,
	output			sclk, cs,
	output	reg [6:0] B,
	output	[15:0]	test_points,
	output	[7:0]	RSSI_OUT
);

localparam NUM_OF_RSSI_SAMPLE = 4;		// average over 8 samples
localparam RSSI_CNT_MSB = 2;		// 2^3 = 8 
localparam RSSI_ACCUM_MSB = (RSSI_CNT_MSB+8);

reg		[3:0]	state, next_state;
reg		[(RSSI_ACCUM_MSB-1):0]	RSSI_accum, next_RSSI_accum, signal_accum, next_signal_accum;
reg		[(RSSI_CNT_MSB-1):0]	RSSI_cnt, next_RSSI_cnt;

// AGC table
reg		[7:0]	table_rssi_in, next_table_rssi_in, table_pwr_in, next_table_pwr_in;
wire	[6:0]	rx_gain;
reg				agc_table_en, next_agc_table_en;

// serial ADC control
reg				adc_en, next_adc_en;
reg				adc_shdn, next_adc_shdn;
wire			data_rdy, adc_rdy;
wire	[7:0]	adc_data_out;

reg		[6:0]	next_B;

assign test_points = {table_pwr_in, 1'b0, B};
assign RSSI_OUT = table_rssi_in;

agc_table2 tab1 (.clk(adc_clock), .resetn(HRESETn), .en(agc_table_en), 
				.rssi_in(table_rssi_in), .pwr_in(table_pwr_in), .gain_out(rx_gain));

serialADC adc0 (.clk(adc_clock), .resetn(HRESETn), .en(adc_en), .shdn(adc_shdn), .data_rdy(data_rdy),
				.adc_rdy(adc_rdy), .data_out(adc_data_out), .sclk(sclk), .cs(cs), .sdata(sdata));

always @ (posedge adc_clock or negedge HRESETn)
begin
	if (~HRESETn)
	begin
		state		<= 0;
		RSSI_accum	<= 0;
		signal_accum <= 0;
		RSSI_cnt	<= 0;
		table_rssi_in <= 8'hff;
		table_pwr_in <= 8'hff;
		adc_en		<= 0;
		adc_shdn	<= 0;
		agc_table_en <= 0;
		B <= 7'b1111111;	// max gain;
	end
	else
	begin
		state		<= next_state;
		RSSI_accum	<= next_RSSI_accum;
		signal_accum <= next_signal_accum;
		RSSI_cnt	<= next_RSSI_cnt;
		table_rssi_in <= next_table_rssi_in;
		table_pwr_in <= next_table_pwr_in;
		adc_en		<= next_adc_en;
		adc_shdn	<= next_adc_shdn;
		agc_table_en <= next_agc_table_en;
		B <= next_B;
	end
end

always @ *
begin
	next_B = B;
	next_state = state;
	next_RSSI_accum = RSSI_accum;
	next_signal_accum = signal_accum;
	next_RSSI_cnt = RSSI_cnt;
	next_table_rssi_in = table_rssi_in;
	next_table_pwr_in = table_pwr_in;
	next_adc_en = adc_en;
	next_adc_shdn = adc_shdn;
	next_agc_table_en = 0;
	case (state)
		4'b0000:
		begin
			if (rx_agc_en)
				next_state = 4'b0001;
			// enter shutdown mode
			else
				next_state = 4'b1000;
			next_RSSI_cnt = 0;
			next_RSSI_accum = 0;
			next_signal_accum = 0;
		end

		// start collecting RSSI value
		4'b0001:
		begin
			if (adc_rdy)
			begin
				next_state = 4'b0010;
				next_adc_en = 1;
			end
		end

		4'b0010:
		begin
			if (~adc_rdy)
				next_state = 4'b0011;
		end

		4'b0011:
		begin
			next_adc_en = 0;
			if (data_rdy)
			begin
				next_RSSI_accum = RSSI_accum + adc_data_out;
				next_signal_accum = signal_accum + input_pwr;
				if (RSSI_cnt==(NUM_OF_RSSI_SAMPLE-1))
				begin
					next_state = 4'b0100;
					next_table_rssi_in = RSSI_accum[(RSSI_ACCUM_MSB-1):(RSSI_ACCUM_MSB-8)];
					next_table_pwr_in = signal_accum[(RSSI_ACCUM_MSB-1):(RSSI_ACCUM_MSB-8)];
					next_agc_table_en = 1;
				end
				else
				begin
					next_state = 4'b0001;
					next_RSSI_cnt = RSSI_cnt + 1;
				end
			end
		end

		// idle state
		// waiting for agc table to process
		4'b0100:
		begin
			next_state = 4'b0101;
		end

		4'b0101:
		begin
			next_B = rx_gain;
			next_state = 4'b0111;
		end

		4'b0111:
		begin
			next_state = 4'b0000;
		end

		// shutdown ADC
		4'b1000:
		begin
			if (adc_rdy)
			begin
				next_adc_en = 1;
				next_adc_shdn = 1;
				next_state = 4'b1001;
			end
		end

		4'b1001:
		begin
			if (~adc_rdy)
			begin
				next_adc_en = 0;
				next_state = 4'b1010;
			end
		end

		4'b1010:
		begin
			if (rx_agc_en)
				// leave shutdown mode
				next_state = 4'b1011;
		end

		4'b1011:
		begin
			next_adc_shdn = 0;
			next_state = 4'b0111;
		end

	endcase
end

endmodule
