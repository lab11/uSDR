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

module frame_ctl(
	input	[15:0]	FCF,
	input	[159:0]	address_field,
	
	output	reg [15:0]	src_pan, 
	output	reg [15:0]	dest_pan,
	output	reg [63:0]	dest_addr, 
	output	reg [63:0]	src_addr
);

wire			PAN_ID_comp = FCF[6];
wire	[1:0]	dest_addr_mode = FCF[11:10];
wire	[1:0]	src_addr_mode = FCF[15:14];

always @ *
begin
	src_pan = 0;
	dest_pan = 0;
	src_addr = 0;
	dest_addr = 0;
	// Dest, Src address "MUST" present
	if (PAN_ID_comp)
	begin
		dest_pan = {address_field[151:144], address_field[159:152]};
		src_pan = {address_field[151:144], address_field[159:152]};
		// dest addr mode = 11, 8 bytes for address
		if (dest_addr_mode[0])
		begin
			dest_addr = {address_field[87:80], address_field[95:88], address_field[103:96], address_field[111:104],
						address_field[119:112], address_field[127:120], address_field[135:128], address_field[143:136]};
			// src addr mode = 11, 8 bytes for address
			if (src_addr_mode[0])
				src_addr = {address_field[23:16], address_field[31:24], address_field[39:32], address_field[47:40],
							address_field[55:48], address_field[63:56], address_field[71:64], address_field[79:72]};
			// src addr mode = 10, 2 bytes for address
			else
				src_addr = {address_field[71:64], address_field[79:72]};
		end
		else
		// dest addr mode = 10, 2 bytes for address
		begin
			dest_addr = {address_field[135:128], address_field[143:136]};
			// src addr mode = 11, 8 bytes for address
			if (src_addr_mode[0])
				src_addr = {address_field[71:64], address_field[79:72], address_field[87:80], address_field[95:88],
							address_field[103:96], address_field[111:104], address_field[119:112], address_field[127:120]};
			// src addr mode = 10, 2 bytes for address
			else
				src_addr = {address_field[119:112], address_field[127:120]};
		end
	end
	else
	begin
		case (dest_addr_mode)
			// no dest addr present
			2'b00:
			begin
				dest_pan = 0;
				dest_addr = 0;
				case (src_addr_mode)
					2'b00:
					begin
						src_pan = 0;
						src_addr = 0;
					end

					2'b10:
					begin
						src_pan = {address_field[151:144], address_field[159:152]};
						src_addr = {address_field[135:128], address_field[143:136]};
					end

					2'b11:
					begin
						src_pan = {address_field[151:144], address_field[159:152]};
						src_addr = {address_field[87:80], address_field[95:88], address_field[103:96], address_field[111:104],
									address_field[119:112], address_field[127:120], address_field[135:128], address_field[143:136]};
					end

					default:
					begin
						src_pan = 0;
						src_addr = 0;
					end
				endcase
			end
			// 2 bytes dest address, 2 bytes dest pan
			2'b10:
			begin
				dest_pan = {address_field[151:144], address_field[159:152]};
				dest_addr = {address_field[135:128], address_field[143:136]};
				case (src_addr_mode)
					// no src address present, no src pan
					2'b00:
					begin
						src_pan = 0;
						src_addr = 0;
					end

					// 2 bytes for src pan, 2 bytes for src address
					2'b10:
					begin
						src_pan = {address_field[119:112], address_field[127:120]};
						src_addr = {address_field[103:96], address_field[111:104]};
					end

					// 2 bytes for src pan, 8 bytes for src address
					2'b11:
					begin
						src_pan = {address_field[119:112], address_field[127:120]};
						src_addr = {address_field[55:48], address_field[63:56], address_field[71:64], address_field[79:72],
									address_field[87:80], address_field[95:88], address_field[103:96], address_field[111:104]};
					end

					default:
					begin
						src_pan = 0;
						src_addr = 0;
					end
				endcase
			end
			// 2 bytes dest address, 2 bytes dest pan
			2'b11:
			begin
				dest_pan = {address_field[151:144], address_field[159:152]};
				dest_addr = {address_field[87:80], address_field[95:88], address_field[103:96], address_field[111:104],
							 address_field[119:112], address_field[127:120], address_field[135:128], address_field[143:136]};
				case (src_addr_mode)
					2'b00:
					begin
						src_pan = 0;
						src_addr = 0;
					end

					2'b10:
					begin
						src_pan = {address_field[71:64],address_field[79:72]};
						src_addr = {address_field[55:48],address_field[63:56]};
					end

					2'b11:
					begin
						src_pan = {address_field[71:64],address_field[79:72]};
						src_addr = {address_field[7:0], address_field[15:8], address_field[23:16], address_field[31:24],
									address_field[39:32], address_field[47:40], address_field[55:48], address_field[63:56]};
					end

					default:
					begin
						src_pan = 0;
						src_addr = 0;
					end
				endcase
			end

			default:
			begin
				src_pan = 0;
				dest_pan = 0;
				src_addr = 0;
				dest_addr = 0;
			end
		endcase

	end
end

endmodule
