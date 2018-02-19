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
 */

module spi_tx(clk, resetn, data_in, start, data_out, clk_out, en_out);

input	clk, resetn, start;
input	[17:0]	data_in;
output	data_out, clk_out, en_out;


reg		data_out_reg, clk_out_reg, en_out_reg;
reg		next_data_out, next_clk_out, next_en_out;

assign	data_out 	= data_out_reg;
assign	clk_out		= clk_out_reg;
assign	en_out		= en_out_reg;


reg		[4:0]	bit_pos, next_bit_pos;
reg		[17:0]	data_in_reg, next_data_in;
reg		[1:0]	state, next_state;
reg		[1:0]	dat_state, next_dat_state;
reg				lastbit, next_lastbit;
wire 	data_bit_ex  = ((data_in_reg & (1<<bit_pos))>0)? 1 : 0;

`define WAIT_FOR_START 	2'b00
`define	START_TX		2'b01
`define END_TX			2'b10
`define LATCH_INPUT		2'b11


always @ (posedge clk)
begin
	if (~resetn)
	begin
		en_out_reg		<= 1;
		data_out_reg	<= 0;
		clk_out_reg		<= 0;
		bit_pos			<= 17;
		state			<= `WAIT_FOR_START;
		data_in_reg		<= 0;
		dat_state		<= 0;
		lastbit			<= 0;
	end
	else
	begin
		en_out_reg		<= next_en_out;
		clk_out_reg		<= next_clk_out;
		data_out_reg	<= next_data_out;
		bit_pos			<= next_bit_pos;
		state			<= next_state;
		data_in_reg		<= next_data_in;
		dat_state		<= next_dat_state;
		lastbit			<= next_lastbit;
	end
end

always @ (*)
begin
	next_data_in	= data_in_reg;
	next_en_out		= en_out_reg;
	next_clk_out	= clk_out_reg;
	next_data_out	= data_out_reg;
	next_bit_pos	= bit_pos;
	next_state		= state;
	next_dat_state	= dat_state;
	next_lastbit	= lastbit;

	case (state)
		`WAIT_FOR_START:
		begin
			if (start)
			begin
				next_state = `LATCH_INPUT;
				next_data_in = data_in;
			end
		end

		`LATCH_INPUT:
		begin
			next_state = `START_TX;
			next_en_out = 0;
			next_dat_state = 0;
		end

		`START_TX:
		begin
			case (dat_state)
				2'b00:
				begin
					next_data_out = data_bit_ex;
					next_dat_state = 2'b01;
					if (bit_pos>0)
						next_bit_pos = bit_pos - 1;
					else
						next_lastbit = 1;
					
				end

				2'b01:
				begin
					next_dat_state = 2'b10;
					next_clk_out = 1;
				end

				2'b10:
				begin
					next_dat_state = 2'b00;
					next_clk_out = 0;
					if (lastbit)
						next_state = `END_TX;
				end

				default:
				begin
					next_dat_state = 2'b00;
				end
			endcase
		end

		`END_TX:
		begin
			next_en_out = 1;
			next_bit_pos = 17;
			next_clk_out = 0;
			next_data_out = 0;
			next_state = `WAIT_FOR_START;
			next_lastbit = 0;
		end

	endcase
end

endmodule
