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

module agc_table(clk, resetn, auto_agc_en, RSSI, PWR, modify, value);

input			clk, resetn, auto_agc_en;
input	[7:0]	RSSI, PWR;
output			modify;
output	[6:0]	value;
reg				modify, next_modify;
reg		[3:0]	gain_level, next_gain_level;
reg		[1:0]	lna_gain, next_lna_gain;
reg		[4:0]	vga_gain, next_vga_gain;

assign value = {lna_gain, vga_gain};

`define MIN_LNA_GAIN	2'b01
`define MID_LNA_GAIN	2'b10
`define MAX_LNA_GAIN	2'b11
`define PWR_LOWER_THRESHOLD	8'd30
`define PWR_UPPER_THRESHOLD	8'd150

always @ (posedge clk)
begin
	if (~resetn)
	begin
		lna_gain <= `MAX_LNA_GAIN;
		vga_gain <= 5'b11111;
		modify	 <= 0;
		gain_level <= 13;
	end
	else
	begin
		if (auto_agc_en)
		begin
			lna_gain <= next_lna_gain;
			vga_gain <= next_vga_gain;
			modify	 <= next_modify;
			gain_level <= next_gain_level;
		end
	end
end

always @ *
begin
	next_gain_level = gain_level;
	next_modify = 0;
	case (gain_level)
		0:
		// vga = 13
		begin
			if ((RSSI<120)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		1:
		// lna = 1
		// vga = 16
		begin
			if ((RSSI>130)||(PWR>`PWR_UPPER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<110)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		2:
		// vga = 7
		begin
			if ((RSSI>146)||(PWR>`PWR_UPPER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<126)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		3:
		// vga = 10
		begin
			if ((RSSI>130)||(PWR>`PWR_UPPER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<110)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		4:
		// vga = 13
		// lna = 2
		begin
			if ((RSSI>115)||(PWR>`PWR_UPPER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<95)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		5:
		// vga = 7
		begin
			if ((RSSI>140)||(PWR>`PWR_UPPER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<120)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		6:
		// vga = 10
		begin
			if ((RSSI>129)||(PWR>`PWR_UPPER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<109)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		7:
		// vga = 13
		begin
			if ((RSSI>117)||(PWR>`PWR_UPPER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<87)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		8:
		// vga = 16
		begin
			if ((RSSI>105)||(PWR>`PWR_UPPER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<85)||(PWR<`PWR_LOWER_THRESHOLD))	
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		9:
		// vga = 19
		begin
			if ((RSSI>88)||(PWR>`PWR_UPPER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<68)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		10:
		// vga = 22
		begin
			if ((RSSI>75)||(PWR>`PWR_UPPER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<55)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		11:
		// vga = 25
		begin
			if ((RSSI>62)||(PWR>`PWR_UPPER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<42)||(PWR<`PWR_LOWER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		12:
		// vga = 28
		begin
			if ((RSSI>50)||(PWR>`PWR_UPPER_THRESHOLD))
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
			else if ((RSSI<45)||(PWR<`PWR_LOWER_THRESHOLD))	// rssi = 0.7V
			begin
				next_modify = 1;
				next_gain_level = gain_level + 1;
			end
		end

		13:
		// maximum gain, lna = 2'd3,
		// vga = 31
		begin
			if (RSSI>45)							// rssi = 0.8V
			begin
				next_modify = 1;
				next_gain_level = gain_level - 1;
			end
		end
	endcase
end

always @ *
begin
	next_lna_gain = lna_gain;
	next_vga_gain = vga_gain;
	case (next_gain_level)
		0:
		begin
			next_lna_gain = 1;
			next_vga_gain = 13;
		end

		1:
		begin
			next_lna_gain = 1;
			next_vga_gain = 16;
		end

		2:
		begin
			next_lna_gain = 2;
			next_vga_gain = 7;
		end

		3:	//0x44B
		begin
			next_lna_gain = 2;
			next_vga_gain = 10;
		end

		4:	//0x48B
		begin
			next_lna_gain = 2;
			next_vga_gain = 13;
		end

		5:	//0x4CB
		begin
			next_lna_gain = 3;
			next_vga_gain = 7;
		end

		6:
		begin
			next_lna_gain = 3;
			next_vga_gain = 10;
		end

		7:
		begin
			next_lna_gain = 3;
			next_vga_gain = 13;
		end

		8:
		begin
			next_lna_gain = 3;
			next_vga_gain = 16;
		end

		9:
		begin
			next_lna_gain = 3;
			next_vga_gain = 19;
		end

		10:
		begin
			next_lna_gain = 3;
			next_vga_gain = 22;
		end

		11:
		begin
			next_lna_gain = 3;
			next_vga_gain = 25;
		end

		12:
		begin
			next_lna_gain = 3;
			next_vga_gain = 28;
		end

		13:
		begin
			next_lna_gain = 3;
			next_vga_gain = 31;
		end

		default:
		begin
			next_lna_gain = 3;
			next_vga_gain = 5'd31;
		end

	endcase
end

endmodule
