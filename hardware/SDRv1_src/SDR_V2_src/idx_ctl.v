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

// idx_ctl.v
module idx_control(	tx_fifo_dout, tx_fifo_we, tx_fifo_full, tx_fifo_empty, current_owner, 
					HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA);

//AHB interface
input HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input [31:0] HADDR, HWDATA;
input [2:0] HSIZE, HBURST;
input [3:0] HPROT;
input [1:0] HTRANS;

output HREADYOUT;
output [1:0] HRESP;
output [31:0] HRDATA;

// idx controller
input			tx_fifo_full, tx_fifo_empty;
output	[7:0]	tx_fifo_dout;
output			tx_fifo_we;

// test points
//output	[12:0]	test_pt;

// AHB registers
reg		[1:0]	fsm;
reg		[7:0]	data_reg;
reg				hwrite_reg;
wire			HREADYOUT;


// idx controller
input	[1:0]	current_owner;

//fifo
reg				fifo_WE;
wire			tx_fifo_full, tx_fifo_empty;

wire			tx_fifo_we		= fifo_WE;
wire	[7:0]	tx_fifo_dout	= data_reg;

// SFD, sync, CRC control
reg				hready_reg;
reg		[2:0]	zero_cnt;
reg		[1:0]	pkt_state;
reg		[7:0]	pkt_len, pkt_cnt;
reg		[1:0]	pkt_end_state;
reg				crc_clear, crc_start;
reg		[1:0]	crc_state;
reg		[1:0]	crc_cnt;
wire	[15:0]	crc_result;
wire			crc_ready;


`define	RESET		2'b00
`define	LOAD_SFD	2'b01
`define AHB_TRANS	2'b10
`define CRC_ADD		2'b11

`define CRC_LSB		2'b00
`define CRC_MSB		2'b01
`define PKT_DONE	2'b10

`define CRC_START	2'b00
`define CRC_WAIT	2'b01
`define	CRC_IDLE	2'b10
`define CRC_DISABLE 2'b11

// AHB
assign HRESP		= 2'b00;
assign HREADYOUT	= (~tx_fifo_full) & hready_reg;
assign HRDATA		= 0;


always @ (posedge HCLK)
begin
    if (~HRESETn)
    begin
		fsm			<= 2'b00;
		data_reg    <= 0;
		hwrite_reg  <= 0;
		fifo_WE	    <= 0;
		hready_reg	<= 0;
		zero_cnt	<= 0;
		pkt_state	<= `RESET;
		pkt_len		<= 0;
		pkt_cnt		<= 0;
		pkt_end_state <= `CRC_LSB;
		crc_start	<= 0;
		crc_clear	<= 0;
		crc_state	<= `CRC_IDLE;
		crc_cnt		<= 0;
    end
    else
    begin
		case (fsm)
		    2'b00:
		    begin
				case (pkt_state)
					`RESET:
					begin
						if ((current_owner ^ 2'b10) && (tx_fifo_empty))
							pkt_state <= `LOAD_SFD;
						zero_cnt <= 0;
						pkt_end_state <= `CRC_LSB;
						crc_clear <= 1;
					end

					`LOAD_SFD:
					begin
						zero_cnt <= zero_cnt + 1;
						crc_clear <= 0;
						if (zero_cnt<4)
						begin
							fifo_WE  <= 1;
							data_reg <= 0;
						end
						else if (zero_cnt==4)
						begin
							fifo_WE  <= 1;
							data_reg <= 8'ha7;
						end
						else
						begin
							pkt_state <= `AHB_TRANS;
							hready_reg <= 1;
							fifo_WE  <= 0;
						end
					end

					`AHB_TRANS:
					begin
						if (((pkt_len-1)==pkt_cnt) && (pkt_len>0))
							pkt_state <= `CRC_ADD;
						else
						begin
							if (HSEL && HREADY && HTRANS[1] && HSIZE==3'b010)
							begin
								hwrite_reg	<= HWRITE;
								fsm			<= 2'b01;
								hready_reg	<= 0;
							end
						end
					end

					`CRC_ADD:
					begin
						hready_reg <= 0;
						//test_pt_reg <= 3'b100;
						case (pkt_end_state)
							`CRC_LSB:
							begin
								fifo_WE <= 1;
								data_reg <= crc_result[7:0];
								pkt_end_state <= `CRC_MSB;
							end

							`CRC_MSB:
							begin
								fifo_WE <= 1;
								data_reg <= crc_result[15:8];
								pkt_end_state <= `PKT_DONE;
								crc_clear <= 1;
							end

							`PKT_DONE:
							begin
								crc_clear <= 0;
								fifo_WE	<= 0;
								pkt_state <= `RESET;
								pkt_cnt	<= 0;
								pkt_len	<= 0;
							end
						endcase
					end

					default:
					begin
						pkt_state <= `RESET;
						fifo_WE	<= 0;
					end
				endcase
		    end

			2'b01:
			begin
				//test_pt_reg <= 3'b011;
				fsm	<= 2'b10;
				crc_cnt <= 0;
				if (hwrite_reg)
				begin
					if (pkt_cnt==0)
					begin
						pkt_len <= HWDATA[7:0];
						crc_state <= `CRC_IDLE;
					end
					else
						crc_state <= `CRC_START;
					pkt_cnt <= pkt_cnt + 1;
					fifo_WE <= 1;
					data_reg <= HWDATA[7:0];
				end
				else
					fifo_WE	<= 0;
			end
		    
		    2'b10:
		    begin
				fifo_WE <= 0;
				case (crc_state)
					`CRC_START:
					begin
						crc_start <= 1;
						crc_state <= `CRC_DISABLE;
					end

					`CRC_DISABLE:
					begin
						crc_start <= 0;
						crc_cnt <= crc_cnt + 1;
						if (crc_cnt==2'b10)
							crc_state <= `CRC_WAIT;
					end

					`CRC_WAIT:
					begin
						if (crc_ready)
						begin
							fsm <= 2'b11;
							crc_state <= `CRC_IDLE;
						end

					end

					`CRC_IDLE:
					begin
						fsm	<= 2'b11;
					end
				endcase
			end

			2'b11:
			begin
				hready_reg <= 1;
				fsm <= 2'b00;
				zero_cnt <= 0;
			end
		endcase
    end
end


crc			tx_crc(	.clk(HCLK), .nreset(HRESETn), .clear(crc_clear), .data_in(data_reg), 
					.crc_out(crc_result), .start(crc_start), .out_ready(crc_ready));

endmodule

