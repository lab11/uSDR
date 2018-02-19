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

module chip_table(in, out);

input   [3:0]   in;
output  [31:0]  out;

reg     [31:0]  seq;
assign  out = seq;

always @ (*)
begin
	case (in)
		4'd0: seq = 32'd3653456430;
		4'd1: seq = 32'd3986437410;
		4'd2: seq = 32'd786023250;
		4'd3: seq = 32'd585997365;
		4'd4: seq = 32'd1378802115;
		4'd5: seq = 32'd891481500;
		4'd6: seq = 32'd3276943065;
		4'd7: seq = 32'd2620728045;
		4'd8: seq = 32'd2358642555;
		4'd9: seq = 32'd3100205175;
		4'd10: seq = 32'd2072811015;
		4'd11: seq = 32'd2008598880;
		4'd12: seq = 32'd125537430;
		4'd13: seq = 32'd1618458825;
		4'd14: seq = 32'd2517072780;
		4'd15: seq = 32'd3378542520;
	endcase
end

endmodule
