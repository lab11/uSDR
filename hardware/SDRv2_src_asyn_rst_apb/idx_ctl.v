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
 * Last update: prevent assert HREADYOUT if the packet is fully loaded
 * @author Ye-Sheng Kuo <samkuo@eecs.umich.edu>
 */

// idx_ctl.v
module idx_control(
	// APB interface
	input	PCLK,
	input	PRESETn, 
	input	[31:0] PADDR, 
	input	PSELx, 
	input	PENABLE,
	input	PWRITE, 
	input	[31:0] PWDATA, 
	output	reg PREADY, 
	output	[31:0] PRDATA,
	output	PSLVERR,

	// to fifo controller
	output	reg [7:0]	tx_fifo_dout,
	output	reg	tx_fifo_we,
	// from fifo controller
	input	tx_fifo_full,
	input	tx_fifo_empty,
	input	[1:0] current_owner
);

assign PSLVERR = 0;
assign PRDATA		= 0;

// AHB registers
reg		[2:0]	fsm;

// SFD, sync, CRC control
reg		[2:0]	zero_cnt;
reg		[1:0]	pkt_state;
reg		[7:0]	pkt_len, pkt_cnt;
reg		[1:0]	pkt_end_state;
reg				crc_clear, crc_start;
reg		[1:0]	crc_state;
reg		[1:0]	crc_cnt;
wire	[15:0]	crc_result;
wire			crc_ready;


localparam RESET =		2'b00;
localparam LOAD_SFD =	2'b01;
localparam AHB_TRANS =	2'b10;
localparam CRC_ADD =	2'b11;

localparam CRC_LSB =	2'b00;
localparam CRC_MSB =	2'b01;
localparam PKT_DONE =	2'b10;

localparam CRC_START =	2'b00;
localparam CRC_WAIT =	2'b01;
localparam CRC_IDLE =	2'b10;
localparam CRC_DISABLE =2'b11;

always @ (posedge PCLK or negedge PRESETn)
begin
    if (~PRESETn)
    begin
		fsm			<= 0;
		tx_fifo_dout    <= 0;
		tx_fifo_we	    <= 0;
		PREADY <= 0;
		zero_cnt	<= 0;
		pkt_state	<= RESET;
		pkt_len		<= 0;
		pkt_cnt		<= 0;
		pkt_end_state <= CRC_LSB;
		crc_start	<= 0;
		crc_clear	<= 0;
		crc_state	<= CRC_IDLE;
		crc_cnt		<= 0;
    end
    else
    begin
		case (fsm)
		    3'b000:
		    begin
				case (pkt_state)
					RESET:
					begin
						if ((current_owner ^ 2'b10) && (tx_fifo_empty))
							pkt_state <= LOAD_SFD;
						zero_cnt <= 0;
						pkt_end_state <= CRC_LSB;
						crc_clear <= 1;
					end

					LOAD_SFD:
					begin
						zero_cnt <= zero_cnt + 1;
						crc_clear <= 0;
						if (zero_cnt<4)
						begin
							tx_fifo_we  <= 1;
							tx_fifo_dout <= 0;
						end
						else if (zero_cnt==4)
						begin
							tx_fifo_we  <= 1;
							tx_fifo_dout <= 8'ha7;
						end
						else
						begin
							pkt_state <= AHB_TRANS;
							tx_fifo_we  <= 0;
						end
					end

					AHB_TRANS:
					begin
						if (PSELx & PENABLE)
							fsm	<= 3'b001;
					end

					CRC_ADD:
					begin
						//test_pt_reg <= 3'b100;
						case (pkt_end_state)
							CRC_LSB:
							begin
								tx_fifo_we <= 1;
								tx_fifo_dout <= crc_result[7:0];
								pkt_end_state <= CRC_MSB;
							end

							CRC_MSB:
							begin
								tx_fifo_we <= 1;
								tx_fifo_dout <= crc_result[15:8];
								pkt_end_state <= PKT_DONE;
								crc_clear <= 1;
							end

							PKT_DONE:
							begin
								crc_clear <= 0;
								tx_fifo_we	<= 0;
								pkt_state <= RESET;
								pkt_cnt	<= 0;
								pkt_len	<= 0;
							end
						endcase
					end

					default:
					begin
						pkt_state <= RESET;
						tx_fifo_we	<= 0;
					end
				endcase
		    end

			3'b001:
			begin
				fsm	<= 3'b010;
				crc_cnt <= 0;
				if (PWRITE)
				begin
					if (~tx_fifo_full)
					begin
						if (pkt_cnt==0)
						begin
							pkt_len <= PWDATA[7:0];
							crc_state <= CRC_IDLE;
						end
						else
							crc_state <= CRC_START;
						pkt_cnt <= pkt_cnt + 1;
						tx_fifo_we <= 1;
						tx_fifo_dout <= PWDATA[7:0];
					end
				end
				else
					tx_fifo_we	<= 0;
			end
		    
		    3'b010:
		    begin
				tx_fifo_we <= 0;
				case (crc_state)
					CRC_START:
					begin
						crc_start <= 1;
						crc_state <= CRC_DISABLE;
					end

					CRC_DISABLE:
					begin
						crc_start <= 0;
						crc_cnt <= crc_cnt + 1;
						if (crc_cnt==2'b10)
							crc_state <= CRC_WAIT;
					end

					CRC_WAIT:
					begin
						if (crc_ready)
						begin
							fsm <= 3'b011;
							crc_state <= CRC_IDLE;
						end

					end

					CRC_IDLE:
					begin
						fsm	<= 3'b011;
					end
				endcase
			end

			3'b011:
			begin
				PREADY <= 1;
				fsm <= 3'b100;
				zero_cnt <= 0;
				if (((pkt_len-1)==pkt_cnt) && (pkt_len>0))
					pkt_state <= CRC_ADD;
			end

			3'b100:
			begin
				if (~PSELx)
				begin
					fsm <= 0;
					PREADY <= 0;
				end
			end
		endcase
    end
end


crc			tx_crc(	.clk(PCLK), .resetn(PRESETn), .clear(crc_clear), .data_in(tx_fifo_dout), 
					.crc_out(crc_result), .start(crc_start), .out_ready(crc_ready));

endmodule

