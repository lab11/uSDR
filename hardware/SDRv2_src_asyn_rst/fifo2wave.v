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

module fifo2wave(
	input			HCLK, 
	input			HRESETn,
	input			fire,
	input	[3:0]	fifo_din,
	input			fifo_empty,
	output	reg		fifo_RE,
	output			tx_cpl,
	output	reg [7:0]	data_out,
	output	reg		half_clk_out,
	output	reg		stop_cnt
);

// HCLK is 48 MHz 
// Chip perios is 1us
parameter SMP_PER_CHIP = 48;
// Q channel delayed by half cycle of chip, 48/2 = 24
parameter HALF_CYCLE_DELAY = 24;

// idx controller


// idx controller 
reg		[5:0]	counter, next_counter; 
wire	[31:0]	chip_seq;
reg		[4:0]	bit_pos, next_bit_pos;
wire	[7:0]	next_data_out;
reg				half_clk_out;
reg				q_buf, next_q_buf;
reg		[5:0]	i_idx, q_idx, next_i_idx, next_q_idx;
wire	[7:0]	i_out, q_out;
wire			i_bit, q_bit;
reg				next_stop_cnt;
reg				tx_complete, next_tx_cpl;
reg				ready_to_go;

//fifo
reg				next_fifo_RE;


// idx_control
assign next_data_out = (stop_cnt==1)? 8'h80 : (counter[0]==0)? q_out : i_out;
assign i_bit = ((chip_seq & (1<<bit_pos))>0)? 1 : 0;
assign q_bit = ((chip_seq & (1<<(bit_pos-1)))>0)? 1 : 0;
assign tx_cpl = tx_complete;

always @ (posedge HCLK or negedge HRESETn)
begin
	if (~HRESETn)
	begin
		fifo_RE			<= 0;
		counter			<= 0;
		bit_pos			<= 31;
		half_clk_out<= 0;
		q_buf			<= 0;
		i_idx			<= 0;
		q_idx			<= 0;
		data_out	<= 0;
		stop_cnt		<= 1;
	end
	else
	begin
		fifo_RE			<= next_fifo_RE;
		counter			<= next_counter;
		bit_pos			<= next_bit_pos;
		half_clk_out<= ~half_clk_out;
		q_buf			<= next_q_buf;
		i_idx			<= next_i_idx;
		q_idx			<= next_q_idx;
		data_out	<= next_data_out;
		stop_cnt		<= next_stop_cnt;
	end
end

always @ (posedge HCLK or negedge HRESETn)
begin
	if (~HRESETn)
	begin
		ready_to_go		<= 0;
		tx_complete		<= 0;
	end
	else
	begin
		tx_complete		<= next_tx_cpl;
		if (fire)
			ready_to_go		<= 1;
		else
			ready_to_go		<= 0;
	end
end


always @ (*)
begin
	next_bit_pos	= bit_pos;
	next_fifo_RE	= 0;
	next_q_buf		= q_buf;
	next_i_idx		= i_idx;
	next_q_idx		= q_idx;
	next_counter	= counter;
	next_stop_cnt	= stop_cnt;
	next_tx_cpl		= 0;

	// if fifo is empty, last bit has been transimitted, stop counter
	if ((fifo_empty==1)&&(bit_pos==1)&&(counter==(SMP_PER_CHIP-1)))
	begin
		next_stop_cnt = 1;
		next_tx_cpl = 1;
	end

	// if fifo has data, prepare to load the data out and starts the counter
	if ((stop_cnt==1)&&(fifo_empty==0)&&(half_clk_out==0)&&(ready_to_go==1))
	begin
		next_counter = (SMP_PER_CHIP - 3);
		next_stop_cnt = 0;
	end

	if (stop_cnt==0)
	begin
		if (counter<(SMP_PER_CHIP-1))
			next_counter = counter + 1;
		else
			next_counter = 0;
	end
	

	if (counter==(SMP_PER_CHIP-3))
	begin
		if ((bit_pos==1) && (fifo_empty==0))
		begin
			next_fifo_RE = 1;
			next_bit_pos = 31;
		end
		else
			next_bit_pos = bit_pos - 2;
	end
	
	if (counter==(HALF_CYCLE_DELAY-1))
		next_q_buf = q_bit;

	if (counter==HALF_CYCLE_DELAY)
	begin
		if (q_buf==1)
			next_q_idx = 0;
		else
			next_q_idx = HALF_CYCLE_DELAY;
	end
	else if (counter==(SMP_PER_CHIP-1))
	begin
		if (i_bit==1)
			next_i_idx = 0;
		else
			next_i_idx = HALF_CYCLE_DELAY;
	end
	else
	begin
		// data goes to I
		if (counter[0]==0)
			next_q_idx = q_idx + 1;
		// data goes to Q
		else
			next_i_idx = i_idx + 1;
	end
		
end

sine_table	t0(.idx_in(i_idx), .out(i_out));
sine_table	t1(.idx_in(q_idx), .out(q_out));
chip_table	seq0(.in(fifo_din), .out(chip_seq));

endmodule

