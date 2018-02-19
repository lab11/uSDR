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
/* Functional description:
* This module finding "zero" in the coming sequence to "synchronize" the
* packet. */
module zero_finding (adc_clock, resetn, input_seq, zero_found, bit_counter, 
					 in_seq0, in_seq1, in_seq2, in_seq3, in_seq4, in_seq5,
					 in_seq6, in_seq7);

input			adc_clock, resetn;
input			input_seq;
output			zero_found;
output	[7:0]	bit_counter;
output	[30:0]	in_seq0, in_seq1, in_seq2, in_seq3, in_seq4, in_seq5, in_seq6, in_seq7;
reg				zero_found, next_zero_found;
reg		[7:0]	bit_counter, next_bit_counter;
reg		[30:0]	in_seq0, in_seq1, in_seq2, in_seq3, in_seq4, in_seq5, in_seq6, in_seq7;

parameter ZERO_THRESHOLD = 5;

wire	[30:0]	zero_seq = 31'b1100000011101111010111001101100;
wire	[30:0]	seq0_xor = in_seq0 ^ zero_seq;
wire	[30:0]	seq1_xor = in_seq1 ^ zero_seq;
wire	[30:0]	seq2_xor = in_seq2 ^ zero_seq;
wire	[30:0]	seq3_xor = in_seq3 ^ zero_seq;
wire	[30:0]	seq4_xor = in_seq4 ^ zero_seq;
wire	[30:0]	seq5_xor = in_seq5 ^ zero_seq;
wire	[30:0]	seq6_xor = in_seq6 ^ zero_seq;
wire	[30:0]	seq7_xor = in_seq7 ^ zero_seq;

wire	[4:0]	seq0_cal, seq1_cal, seq2_cal, seq3_cal, seq4_cal, seq5_cal, seq6_cal, seq7_cal;
bit_accumlator b0(seq0_xor, seq0_cal);
bit_accumlator b1(seq1_xor, seq1_cal);
bit_accumlator b2(seq2_xor, seq2_cal);
bit_accumlator b3(seq3_xor, seq3_cal);
bit_accumlator b4(seq4_xor, seq4_cal);
bit_accumlator b5(seq5_xor, seq5_cal);
bit_accumlator b6(seq6_xor, seq6_cal);
bit_accumlator b7(seq7_xor, seq7_cal);

wire	seq01_mim = (seq0_cal < seq1_cal)? 0 : 1;
wire	seq23_mim = (seq2_cal < seq3_cal)? 0 : 1;
wire	seq45_mim = (seq4_cal < seq5_cal)? 0 : 1;
wire	seq67_mim = (seq6_cal < seq7_cal)? 0 : 1;
wire	[4:0] seq01_mim_val = (seq01_mim)? seq0_cal : seq1_cal;
wire	[4:0] seq23_mim_val = (seq23_mim)? seq2_cal : seq3_cal;
wire	[4:0] seq45_mim_val = (seq45_mim)? seq4_cal : seq5_cal;
wire	[4:0] seq67_mim_val = (seq67_mim)? seq6_cal : seq7_cal;

wire	seq0123_mim = (seq01_mim_val < seq23_mim_val)? 0 : 1;
wire	seq4567_mim = (seq45_mim_val < seq67_mim_val)? 0 : 1;
wire	[4:0] seq0123_mim_val = (seq0123_mim)? seq01_mim_val : seq23_mim_val;
wire	[4:0] seq4567_mim_val = (seq4567_mim)? seq45_mim_val : seq67_mim_val;

wire	[4:0] seq_tot_mim_val = (seq0123_mim_val < seq4567_mim_val)? seq0123_mim_val : seq4567_mim_val;


always @ *
begin
	next_zero_found = zero_found;
	next_bit_counter = bit_counter + 1;
	if ((seq_tot_mim_val < ZERO_THRESHOLD)&&(~zero_found))
	begin
		next_zero_found = 1;
		next_bit_counter = 0;
	end
end


always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		zero_found <= 0;
		bit_counter <= 0;
		in_seq0	<= 0;
    	in_seq1	<= 0;
    	in_seq2	<= 0;
    	in_seq3	<= 0;
    	in_seq4	<= 0;
    	in_seq5	<= 0;
		in_seq6	<= 0;	
    	in_seq7	<= 0;
	end
	else
	begin
		zero_found <= next_zero_found;
		bit_counter <= next_bit_counter;
		case (bit_counter[2:0])
			3'b000: begin in_seq0 <= {in_seq0[29:0], input_seq}; end
			3'b001: begin in_seq1 <= {in_seq1[29:0], input_seq}; end
			3'b010: begin in_seq2 <= {in_seq2[29:0], input_seq}; end
			3'b011: begin in_seq3 <= {in_seq3[29:0], input_seq}; end
			3'b100: begin in_seq4 <= {in_seq4[29:0], input_seq}; end
			3'b101: begin in_seq5 <= {in_seq5[29:0], input_seq}; end
			3'b110: begin in_seq6 <= {in_seq6[29:0], input_seq}; end
			3'b111: begin in_seq7 <= {in_seq7[29:0], input_seq}; end
		endcase
	end
end


endmodule
