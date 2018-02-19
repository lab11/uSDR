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

// sin_table.v
module sine_table(idx_in, out);

input	[5:0]	idx_in;
output	[7:0]	out;

reg		[7:0]	out_reg;
assign	out = out_reg;

always @ *
begin
	case (idx_in)
		6'd0: out_reg = 8'h7f;
        6'd1: out_reg = 8'h90;
        6'd2: out_reg = 8'ha0;
        6'd3: out_reg = 8'hb0;
        6'd4: out_reg = 8'hbf;
        6'd5: out_reg = 8'hcc;
        6'd6: out_reg = 8'hd9;
        6'd7: out_reg = 8'he4;
        6'd8: out_reg = 8'hed;
        6'd9: out_reg = 8'hf4;
        6'd10: out_reg = 8'hfa;
        6'd11: out_reg = 8'hfd;
        6'd12: out_reg = 8'hfe;
        6'd13: out_reg = 8'hfd;
        6'd14: out_reg = 8'hfa;
        6'd15: out_reg = 8'hf4;
        6'd16: out_reg = 8'hed;
        6'd17: out_reg = 8'he4;
        6'd18: out_reg = 8'hd9;
        6'd19: out_reg = 8'hcc;
        6'd20: out_reg = 8'hbe;
        6'd21: out_reg = 8'hb0;
        6'd22: out_reg = 8'ha0;
		6'd23: out_reg = 8'h90;
        6'd24: out_reg = 8'h7f;
        6'd25: out_reg = 8'h6e;
        6'd26: out_reg = 8'h5e;
        6'd27: out_reg = 8'h4e;
        6'd28: out_reg = 8'h3f;
        6'd29: out_reg = 8'h32;
        6'd30: out_reg = 8'h25;
        6'd31: out_reg = 8'h1a;
        6'd32: out_reg = 8'h11;
        6'd33: out_reg = 8'ha;
        6'd34: out_reg = 8'h4;
        6'd35: out_reg = 8'h1;
        6'd36: out_reg = 8'h0;
        6'd37: out_reg = 8'h1;
        6'd38: out_reg = 8'h4;
        6'd39: out_reg = 8'ha;
        6'd40: out_reg = 8'h11;
        6'd41: out_reg = 8'h1a;
        6'd42: out_reg = 8'h25;
        6'd43: out_reg = 8'h32;
        6'd44: out_reg = 8'h40;
        6'd45: out_reg = 8'h4e;
		6'd46: out_reg = 8'h5e;
        6'd47: out_reg = 8'h6e;
		default: out_reg = 8'h0;
	endcase
end

endmodule
