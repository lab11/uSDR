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
* This module is a high speed minimum distance finder. */
module hs_match(adc_clock, resetn, in_seq, start, done, output_idx, output_val);

input			adc_clock, resetn, start;
input	[30:0]	in_seq;
output	[3:0]	output_idx;
output	[4:0]	output_val;
output			done;

reg		[3:0]	output_idx, next_output_idx;
reg		[4:0]	output_val, next_output_val;
reg				done, next_done;
reg		[1:0] 	state, next_state;
reg		[4:0]	chip00_reg, chip01_reg, chip02_reg, chip03_reg, chip04_reg,
				chip05_reg, chip06_reg, chip07_reg, chip08_reg, chip09_reg,
				chip10_reg, chip11_reg, chip12_reg, chip13_reg, chip14_reg,
				chip15_reg;

wire	[30:0]	chip_00_seq = 	31'b1100000011101111010111001101100;
wire	[30:0]	chip_01_seq = 	31'b1001110000001110111101011100110;
wire	[30:0]	chip_02_seq = 	31'b1101100111000000111011110101110;
wire	[30:0]	chip_03_seq = 	31'b1100110110011100000011101111010;
wire	[30:0]	chip_04_seq = 	31'b0101110011011001110000001110111;
wire	[30:0]	chip_05_seq = 	31'b1111010111001101100111000000111;
wire	[30:0]	chip_06_seq = 	31'b1110111101011100110110011100000;
wire	[30:0]	chip_07_seq = 	31'b0000111011110101110011011001110;
wire	[30:0]	chip_08_seq = 	31'b0011111100010000101000110010011;
wire	[30:0]	chip_09_seq = 	31'b0110001111110001000010100011001;
wire	[30:0]	chip_10_seq = 	31'b0010011000111111000100001010001;
wire	[30:0]	chip_11_seq = 	31'b0011001001100011111100010000101;
wire	[30:0]	chip_12_seq = 	31'b1010001100100110001111110001000;
wire	[30:0]	chip_13_seq = 	31'b0000101000110010011000111111000;
wire	[30:0]	chip_14_seq = 	31'b0001000010100011001001100011111;
wire	[30:0]	chip_15_seq = 	31'b1111000100001010001100100110001;

wire	[30:0]	chip00_xor =	in_seq ^ chip_00_seq;	
wire	[30:0]	chip01_xor =	in_seq ^ chip_01_seq;	
wire	[30:0]	chip02_xor =	in_seq ^ chip_02_seq;	
wire	[30:0]	chip03_xor =	in_seq ^ chip_03_seq;	
wire	[30:0]	chip04_xor =	in_seq ^ chip_04_seq;	
wire	[30:0]	chip05_xor =	in_seq ^ chip_05_seq;	
wire	[30:0]	chip06_xor =	in_seq ^ chip_06_seq;	
wire	[30:0]	chip07_xor =	in_seq ^ chip_07_seq;	
wire	[30:0]	chip08_xor =	in_seq ^ chip_08_seq;	
wire	[30:0]	chip09_xor =	in_seq ^ chip_09_seq;	
wire	[30:0]	chip10_xor =	in_seq ^ chip_10_seq;	
wire	[30:0]	chip11_xor =	in_seq ^ chip_11_seq;	
wire	[30:0]	chip12_xor =	in_seq ^ chip_12_seq;	
wire	[30:0]	chip13_xor =	in_seq ^ chip_13_seq;	
wire	[30:0]	chip14_xor =	in_seq ^ chip_14_seq;	
wire	[30:0]	chip15_xor =	in_seq ^ chip_15_seq;	

wire	[4:0]	chip00_cal, chip01_cal, chip02_cal, chip03_cal, chip04_cal, chip05_cal, chip06_cal, chip07_cal,
				chip08_cal, chip09_cal, chip10_cal, chip11_cal, chip12_cal, chip13_cal, chip14_cal, chip15_cal;
bit_accumlator b00(chip00_xor, chip00_cal);
bit_accumlator b01(chip01_xor, chip01_cal);
bit_accumlator b02(chip02_xor, chip02_cal);
bit_accumlator b03(chip03_xor, chip03_cal);
bit_accumlator b04(chip04_xor, chip04_cal);
bit_accumlator b05(chip05_xor, chip05_cal);
bit_accumlator b06(chip06_xor, chip06_cal);
bit_accumlator b07(chip07_xor, chip07_cal);
bit_accumlator b08(chip08_xor, chip08_cal);
bit_accumlator b09(chip09_xor, chip09_cal);
bit_accumlator b10(chip10_xor, chip10_cal);
bit_accumlator b11(chip11_xor, chip11_cal);
bit_accumlator b12(chip12_xor, chip12_cal);
bit_accumlator b13(chip13_xor, chip13_cal);
bit_accumlator b14(chip14_xor, chip14_cal);
bit_accumlator b15(chip15_xor, chip15_cal);

wire	chip0001mim = (chip00_reg < chip01_reg)? 1 : 0;
wire	chip0203mim = (chip02_reg < chip03_reg)? 1 : 0;
wire	chip0405mim = (chip04_reg < chip05_reg)? 1 : 0;
wire	chip0607mim = (chip06_reg < chip07_reg)? 1 : 0;
wire	chip0809mim = (chip08_reg < chip09_reg)? 1 : 0;
wire	chip1011mim = (chip10_reg < chip11_reg)? 1 : 0;
wire	chip1213mim = (chip12_reg < chip13_reg)? 1 : 0;
wire	chip1415mim = (chip14_reg < chip15_reg)? 1 : 0;
wire	[4:0]	chip0001mim_val = (chip0001mim)? chip00_reg : chip01_reg;
wire	[4:0]	chip0203mim_val = (chip0203mim)? chip02_reg : chip03_reg;
wire	[4:0]	chip0405mim_val = (chip0405mim)? chip04_reg : chip05_reg;
wire	[4:0]	chip0607mim_val = (chip0607mim)? chip06_reg : chip07_reg;
wire	[4:0]	chip0809mim_val = (chip0809mim)? chip08_reg : chip09_reg;
wire	[4:0]	chip1011mim_val = (chip1011mim)? chip10_reg : chip11_reg;
wire	[4:0]	chip1213mim_val = (chip1213mim)? chip12_reg : chip13_reg;
wire	[4:0]	chip1415mim_val = (chip1415mim)? chip14_reg : chip15_reg;

wire	chip00010203mim = (chip0001mim_val < chip0203mim_val)? 1 : 0;
wire	chip04050607mim = (chip0405mim_val < chip0607mim_val)? 1 : 0;
wire	chip08091011mim = (chip0809mim_val < chip1011mim_val)? 1 : 0;
wire	chip12131415mim = (chip1213mim_val < chip1415mim_val)? 1 : 0;
wire	[4:0]	chip00010203mim_val = (chip00010203mim)? chip0001mim_val : chip0203mim_val;
wire	[4:0]	chip04050607mim_val = (chip04050607mim)? chip0405mim_val : chip0607mim_val;
wire	[4:0]	chip08091011mim_val = (chip08091011mim)? chip0809mim_val : chip1011mim_val;
wire	[4:0]	chip12131415mim_val = (chip12131415mim)? chip1213mim_val : chip1415mim_val;

wire	chip00to07mim = (chip00010203mim_val < chip04050607mim_val)? 1 : 0;
wire	chip08to15mim = (chip08091011mim_val < chip12131415mim_val)? 1 : 0;
wire	[4:0]	chip00to07mim_val = chip00to07mim? chip00010203mim_val : chip04050607mim_val;
wire	[4:0]	chip08to15mim_val = chip08to15mim? chip08091011mim_val : chip12131415mim_val;


always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		output_idx	<= 0;
		output_val	<= 31;
		done		<= 0;
		state <= 0;
	end
	else
	begin
		output_idx <= next_output_idx;
		output_val <= next_output_val;
		done <= next_done;
		state <= next_state;
	end
end

always @ *
begin
	next_output_idx = output_idx;
	next_output_val = output_val;
	next_done = done;
	next_state = state;
	case (state)
		0:
		begin
			if (start)
			begin
				next_state = 1;
				next_done = 0;
			end
		end

		1:
		begin
			next_state = 2;
		end

		2:
		begin
			next_state = 3;
		end

		3:
		begin
			next_done = 1;
			next_state = 0;
			if (chip00to07mim_val < chip08to15mim_val)
			begin
				next_output_val = chip00to07mim_val;
				if (chip00to07mim)
				begin
					if (chip00010203mim)
					begin
						if (chip0001mim)
							next_output_idx = 0;
						else
							next_output_idx = 1;
					end
					else
					begin
						if (chip0203mim)
							next_output_idx = 2;
						else
							next_output_idx = 3;
					end
				end
				else
				begin
					if (chip04050607mim)
					begin
						if (chip0405mim)
							next_output_idx = 4;
						else
							next_output_idx = 5;
					end
					else
					begin
						if (chip0607mim)
							next_output_idx = 6;
						else
							next_output_idx = 7;
					end
				end
			end
			else
			begin
				next_output_val = chip08to15mim_val;
				if (chip08to15mim)
				begin
					if (chip08091011mim)
					begin
						if (chip0809mim)
							next_output_idx = 8;
						else
							next_output_idx = 9;
					end
					else
					begin
						if (chip1011mim)
							next_output_idx = 10;
						else
							next_output_idx = 11;
					end
				end
				else
				begin
					if (chip12131415mim)
					begin
						if (chip1213mim)
							next_output_idx = 12;
						else
							next_output_idx = 13;
					end
					else
					begin
						if (chip1415mim)
							next_output_idx = 14;
						else
							next_output_idx = 15;
					end
				end
			end
			
		end
	endcase
end

always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		chip00_reg <= 0;
		chip01_reg <= 0;
		chip02_reg <= 0;
		chip03_reg <= 0;
		chip04_reg <= 0;
		chip05_reg <= 0;
		chip06_reg <= 0;
		chip07_reg <= 0;
		chip08_reg <= 0;
		chip09_reg <= 0;
		chip10_reg <= 0;
		chip11_reg <= 0;
		chip12_reg <= 0;
		chip13_reg <= 0;
		chip14_reg <= 0;
		chip15_reg <= 0;
	end
	else
	begin
		if (start)
		begin
			chip00_reg <= chip00_cal;
			chip01_reg <= chip01_cal;
			chip02_reg <= chip02_cal;
			chip03_reg <= chip03_cal;
			chip04_reg <= chip04_cal;
			chip05_reg <= chip05_cal;
			chip06_reg <= chip06_cal;
			chip07_reg <= chip07_cal;
			chip08_reg <= chip08_cal;
			chip09_reg <= chip09_cal;
			chip10_reg <= chip10_cal;
			chip11_reg <= chip11_cal;
			chip12_reg <= chip12_cal;
			chip13_reg <= chip13_cal;
			chip14_reg <= chip14_cal;
			chip15_reg <= chip15_cal;
		end
	end
end

endmodule
