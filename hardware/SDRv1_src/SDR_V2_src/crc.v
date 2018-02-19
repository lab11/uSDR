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

module crc(clk, nreset, clear, data_in, crc_out, start, out_ready);

input 	clk, nreset, start, clear;
input	[7:0]	data_in;
output	[15:0]	crc_out;
output	out_ready;

reg		[15:0]	crc_reg;
reg		out_rd_reg;
reg		[2:0]	bit_cnt, next_bit_cnt;

reg		[15:0]	next_crc_reg;
reg		next_out_ready;
reg		[1:0]	state, next_state;
reg		[7:0]	data_in_reg, next_data_in_reg;

assign	crc_out = crc_reg;
assign	out_ready = out_rd_reg;


wire	bit_extract	= (data_in_reg & (1<<bit_cnt))>0? 1 : 0;

wire	in_bit_xor 	= bit_extract ^ crc_reg[0];
wire	bit4_xor 	= in_bit_xor ^ crc_reg[4];
wire	bit10_xor 	= in_bit_xor ^ crc_reg[11];

`define	RESET	2'b00
`define WAIT_ST	2'b01
`define	START	2'b10
`define	DONE	2'b11


always @ (posedge clk)
begin
	if (~nreset)
	begin
		out_rd_reg	<= 0;
		crc_reg		<= 0;
		bit_cnt		<= 0;
		state		<= `RESET;
		data_in_reg	<= 0;
	end
	else
	begin
		out_rd_reg	<= next_out_ready;
		crc_reg		<= next_crc_reg;
		bit_cnt		<= next_bit_cnt;
		state		<= next_state;
		data_in_reg <= next_data_in_reg;
	end
end

always @ *
begin
	next_crc_reg = crc_reg;
	next_out_ready = out_rd_reg;
	next_state = state;
	next_bit_cnt = bit_cnt;
	next_data_in_reg = data_in_reg;
	case (state)
		`RESET:
		begin
			next_crc_reg = 0;
			next_out_ready = 0;
			next_bit_cnt = 0;
			next_state = `WAIT_ST;
			next_data_in_reg = 0;
		end

		`WAIT_ST:
		begin
			if (clear)
				next_state = `RESET;
			else if (start==1)
			begin
				next_state = `START;
				next_data_in_reg = data_in;
			end
		end

		`START:
		begin
			next_out_ready = 0;
			if (bit_cnt<7)
				next_bit_cnt = bit_cnt + 1;
			else
				next_state = `DONE;

			next_crc_reg = {in_bit_xor, crc_reg[15:12], bit10_xor, 
							crc_reg[10:5], bit4_xor, crc_reg[3:1]};
		end
		
		`DONE:
		begin
			next_out_ready = 1;
			next_bit_cnt = 0;
			next_state = `WAIT_ST;
		end


	endcase
end

endmodule
