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
* convert the bits to symbol, compare all the 8 samples sequences and locate
* the minimum distance as index.*/

module sequence_compare(adc_clock, resetn, zero_found, output_idx, idx_found, seq_counter,
						seq0_in, seq1_in, seq2_in, seq3_in, seq4_in, seq5_in, seq6_in, seq7_in);

input			adc_clock, resetn, zero_found;
input	[30:0]	seq0_in, seq1_in, seq2_in, seq3_in, seq4_in, seq5_in, seq6_in, seq7_in;
output	[3:0]	output_idx;
output			idx_found;

reg		[3:0]	output_idx, next_output_idx;
reg				idx_found, next_idx_found;
reg		[30:0]	seq0, seq1, seq2, seq3, seq4, seq5, seq6, seq7;
reg		[30:0]	next_seq0, next_seq1, next_seq2, next_seq3, next_seq4, next_seq5, next_seq6, next_seq7;
reg		[2:0]	cmp_idx, next_cmp_idx;
input	[7:0]	seq_counter;

// IOs for comparator
reg		[30:0]	cmp_sequence, next_cmp_sequence;
reg				cmp_start, next_cmp_start;
reg		[2:0]	state, next_state;
reg		[4:0]	chip_mim_val, next_chip_mim_val;
reg		[3:0]	chip_mim_idx, next_chip_mim_idx;
wire	[3:0]	sequence_idx;
wire	[4:0]	chip_val;
wire			chip_locked;

hs_match m1(.adc_clock(adc_clock), .resetn(resetn), .in_seq(cmp_sequence), .start(cmp_start), .done(chip_locked), .output_idx(sequence_idx), .output_val(chip_val));

`define FINDING_ZERO		3'b000
`define COLLECTING_SAMPLES	3'b001
`define COMPARE				3'b010
`define INTER_STATE			3'b011
`define WAIT_FOR_COMPARE	3'b100
`define UPDATE_RESULT		3'b101

always @ *
begin
	next_state = state;
	next_cmp_sequence = cmp_sequence;
	next_cmp_start = cmp_start;
	next_cmp_idx = cmp_idx;
	next_chip_mim_val = chip_mim_val;
	next_chip_mim_idx = chip_mim_idx;
	next_seq0 = seq0;
	next_seq1 = seq1;
	next_seq2 = seq2;
	next_seq3 = seq3;
	next_seq4 = seq4;
	next_seq5 = seq5;
	next_seq6 = seq6;
	next_seq7 = seq7;
	next_idx_found = idx_found;
	next_output_idx = output_idx;
	case (state)
		`FINDING_ZERO:
		begin
			next_idx_found = 0;
			if (zero_found)
				next_state = `COLLECTING_SAMPLES;
		end

		`COLLECTING_SAMPLES:
		begin
			if (~zero_found)
				next_state = `FINDING_ZERO;

			if (seq_counter==255)
			begin
				next_idx_found = 0;
				next_state = `COMPARE;
				next_seq0 = seq0_in;
				next_seq1 = seq1_in;
				next_seq2 = seq2_in;
				next_seq3 = seq3_in;
				next_seq4 = seq4_in;
				next_seq5 = seq5_in;
				next_seq6 = seq6_in;
				next_seq7 = seq7_in;
			end
			next_cmp_idx = 0;
			next_chip_mim_val = 31;
			next_chip_mim_idx = 0;
		end

		`COMPARE:
		begin
			case (cmp_idx)
				0: begin next_cmp_sequence = seq0; end
				1: begin next_cmp_sequence = seq1; end
				2: begin next_cmp_sequence = seq2; end
				3: begin next_cmp_sequence = seq3; end
				4: begin next_cmp_sequence = seq4; end
				5: begin next_cmp_sequence = seq5; end
				6: begin next_cmp_sequence = seq6; end
				7: begin next_cmp_sequence = seq7; end
			endcase
			next_cmp_start = 1;
			next_state = `INTER_STATE;
		end

		`INTER_STATE:
		begin
			next_cmp_start = 0;
			next_state = `WAIT_FOR_COMPARE;
		end

		`WAIT_FOR_COMPARE:
		begin
			if (chip_locked)
			begin
				if (chip_val < chip_mim_val)
				begin
					next_chip_mim_val = chip_val;
					next_chip_mim_idx = sequence_idx;
				end

				if (cmp_idx<7)
				begin
					next_state = `COMPARE;
					next_cmp_idx = cmp_idx + 1;
				end
				// all compare done
				else
				begin
					next_state = `UPDATE_RESULT;
				end
			end
		end

		`UPDATE_RESULT:
		begin
			next_output_idx = chip_mim_idx;
			next_idx_found = 1;
			next_state = `COLLECTING_SAMPLES;
		end
	endcase
end


always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		cmp_sequence <= 0;
		cmp_start <= 0;
		chip_mim_idx <= 0;
		chip_mim_val <= 0;
		cmp_idx <= 0;
		state <= 0;
		seq0 <= 0;
		seq1 <= 0;
		seq2 <= 0;
		seq3 <= 0;
		seq4 <= 0;
		seq5 <= 0;
		seq6 <= 0;
		seq7 <= 0;
	end
	else
	begin
		chip_mim_idx <= next_chip_mim_idx;
		chip_mim_val <= next_chip_mim_val;
		cmp_sequence <= next_cmp_sequence;
		cmp_start <= next_cmp_start;
		cmp_idx <= next_cmp_idx;
		state <= next_state;
		seq0 <= next_seq0;
		seq1 <= next_seq1;
		seq2 <= next_seq2;
		seq3 <= next_seq3;
		seq4 <= next_seq4;
		seq5 <= next_seq5;
		seq6 <= next_seq6;
		seq7 <= next_seq7;
	end
end

always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		output_idx <= 0;
		idx_found <= 0;
	end
	else
	begin
		output_idx <= next_output_idx;
		idx_found <= next_idx_found;
	end
end


endmodule
