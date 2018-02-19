/*
 * "Copyright (c) 2010-2013 The Regents of the University of Michigan.
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
 * Note:
 * Last modify data: 2/3/2013,
 * Last modify content: first create
 */

module sfd_sync(
	input resetn,
	input clk,
	input clear,
	input [3:0] input_hb,
	input zero_locked,
	input chip_locked,
	output reg [7:0] idx_out,
	output reg SFD_LOCKED,
	output reg update
);

reg	[2:0] state, next_state;
reg	next_update;
reg [7:0] next_idx_out, idx_buf, next_idx_buf;
reg	next_sfd_locked;
reg	hb, next_hb;

parameter SFD=8'ha7;

always @ (posedge clk)
begin
	if (~(resetn&clear))
	begin
		state <= 0;
		update <= 0;
		idx_out <= 0;
		idx_buf <= 0;
		SFD_LOCKED <= 0;
		hb <= 1;
	end
	else
	begin
		state <= next_state;
		update <= next_update;
		idx_out <= next_idx_out;
		idx_buf <= next_idx_buf;
		SFD_LOCKED <= next_sfd_locked;
		hb <= next_hb;
	end
end

always @ *
begin
	next_state = state;
	next_update = 0;
	next_idx_out = idx_out;
	next_idx_buf = idx_buf;
	next_sfd_locked = SFD_LOCKED;
	next_hb = hb;

	case (state)
		3'b000:
		begin
			if (zero_locked)
				next_state = 3'b001;
		end

		3'b001:
		begin
			if (chip_locked)
			begin
				next_idx_buf = {input_hb, idx_buf[7:4]};
				next_state = 3'b010;
			end
		end

		3'b010:
		begin
			next_hb = 1;
			next_state = 3'b011;
			if (idx_buf==SFD)
			begin
				next_sfd_locked = 1;
				next_idx_out = idx_buf;
			end
		end

		3'b011:
		begin
			if (~chip_locked)
			begin
				if (SFD_LOCKED)
					next_state = 3'b100;
				else
					next_state = 3'b001;
			end
		end

		3'b100:
		begin
			if (chip_locked)
			begin
				next_idx_buf = {input_hb, idx_buf[7:4]};
				next_state = 3'b101;
				next_hb = ~hb;
			end
		end

		3'b101:
		begin
			next_state = 3'b011;
			if (hb)
			begin
				next_update = 1;
				next_idx_out = idx_buf;
			end
		end
	endcase
end


endmodule
