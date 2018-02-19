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

module fifo_ctl(clk, nreset, data_out, half_clk_out, 
				tx_fire, tfdat, tfWE, 
				tx_complete, tffull, tfempty, 
				DSN, DSN_sel, ack_ready, owner, glossy_ack_complete,
				rx_data, rx_WE, isGlossy, rx_done, sw_tx, ack_flush,
				glossy_ready);

input			clk, nreset;
output	[7:0]	data_out;
output			half_clk_out;
output	[1:0]	owner;
output			glossy_ack_complete;
output			sw_tx;
output			glossy_ready;
input			ack_flush;			// Flush ACK if address not match
reg				sw_tx, next_sw_tx;	// MAX2831 TX, RX line


//	Transmit Fifo
input	[7:0]	tfdat;
input			tfWE;
input			tx_fire;
output			tffull, tfempty, tx_complete;
wire	[7:0]	tfdat;
wire	[3:0]	tfdout;
wire			tffull, tfempty;
wire			tfWE, tfRE;
wire			tx_complete;

// Acknowledge Fifo
input	[7:0]	DSN;
input			DSN_sel;
output			ack_ready;
reg		[7:0]	afdat, next_afdat;
reg				afWE, next_afWE;
wire			afRE;
wire	[3:0]	afdout;
wire			affull, afempty;
wire			ack_complete;
reg				afrst, next_afrst, tfrst, next_tfrst;

//glossy fifo
input	[7:0]	rx_data;
input			rx_WE;
input			isGlossy;
input			rx_done;
wire			glossy_complete;
reg		[7:0]	gfdat, next_gfdat;
reg				gfWE, next_gfWE;
wire	[3:0]	gfdout;
wire			gfRE;
reg		[1:0]	gfcnt, next_gfcnt;
reg				gfrst, next_gfrst; 
wire			gfempty;
reg				glossy_ready, next_glossy_ready;
reg				glossy_avail, next_glossy_avail;
reg				glossy_tx_cpl, next_glossy_tx_cpl;
reg				glossy_req_owner, next_glossy_req_owner;
reg		[2:0]	glossy_state, next_glossy_state;



// crc control
reg				crc_clear, crc_start, next_crc_clear, next_crc_start;
reg		[7:0]	crc_data_in, next_crc_data_in;
wire	[15:0]	crc_result;
wire			crc_ready;

// fifo2wave
reg				fire, next_fire;
wire	[3:0]	fifo_din;
wire			tx_cpl;
wire			fifo_RE, fifo_empty;

reg		[1:0]	owner, next_owner;
reg		[1:0]	own_cnt, next_own_cnt;
reg		[7:0]	DSN_reg, next_DSN_reg;
reg				DSN_avail, next_DSN_avail;
reg		[13:0]	DSN_counter, next_DSN_counter;
reg				ack_req_owner, next_ack_req_owner;
reg				tx_req_owner, next_tx_req_owner;
reg		[1:0]	own_fsm, next_own_fsm;
reg				ack_ready, next_ack_ready;
reg				ack_tx_cpl, next_ack_tx_cpl;

reg		[2:0]	ack_state, next_ack_state;
reg		[1:0]	ack_zero_cnt, next_ack_zero_cnt;

reg		[1:0]	crc_zero_cnt, next_crc_zero_cnt;
reg				crc_dsn_done, next_crc_dsn_done;
reg				crc_fsm, next_crc_fsm;
reg		[2:0]	crc_state, next_crc_state;


`define	TX		2'b10
`define RX		2'b01
`define GLOSSY	2'b11
`define NONE	2'b00

`define	ACK_RST			3'b000
`define ZERO_ADD		3'b001
`define FRAME_ADD		3'b010
`define WAIT_DSN		3'b011
`define WAIT_DSN_CRC0	3'b100
`define WAIT_DSN_CRC1	3'b101
`define ACK_READY		3'b110

`define CRC_RST			3'b000
`define CRC_PRELOAD		3'b001
`define CRC_INTER_STATE	3'b010
`define CRC_DSN			3'b011
`define CRC_DSN_START	3'b100
`define CRC_DONE		3'b101

`define GLOSSY_RST		3'b000
`define GLOSSY_INIT		3'b001
`define GLOSSY_SFD		3'b010
`define GLOSSY_COPY		3'b011
`define GLOSSY_COPY2	3'b100
`define GLOSSY_WAIT		3'b101

`define DEFAULT_ACK_VALUE 		14'd8886 // 192 us
`define MAX2831_TXRX_TIME		96    // 2 us

assign fifo_din = (owner==`GLOSSY)? gfdout : (owner==`TX)? tfdout : (owner==`RX)? afdout : 0;
assign tfRE = (owner==`TX)? fifo_RE : 0;
assign afRE = (owner==`RX)? fifo_RE : 0;
assign gfRE = (owner==`GLOSSY)? fifo_RE : 0;
assign fifo_empty = (owner==`TX)? tfempty : (owner==`RX)? afempty : (owner==`GLOSSY)? gfempty : 1;
assign tx_complete = (owner==`TX)? tx_cpl : 0;
assign ack_complete = (owner==`RX)? tx_cpl : 0;
assign glossy_complete = (owner==`GLOSSY)? tx_cpl : 0;
assign glossy_ack_complete = ack_complete | glossy_complete;


always @ (posedge clk)
begin
	if (~nreset)
	begin
		owner		<= `NONE;
		fire		<= 0;
		DSN_reg		<= 0;
		DSN_avail	<= 0;
		DSN_counter	<= 0;
		ack_req_owner <= 0;
		tx_req_owner <= 0;
		own_fsm		<= 0;
		ack_ready	<= 1;
		ack_tx_cpl	<= 0;
		tfrst		<= 1;
		own_cnt		<= 0;
		glossy_req_owner <= 0;
		glossy_avail <= 0;
		glossy_tx_cpl <= 0;
		glossy_ready <= 1;
		sw_tx		<= 0;
	end
	else
	begin
		owner		<= next_owner;
		fire		<= next_fire;
		DSN_reg		<= next_DSN_reg;
		DSN_avail	<= next_DSN_avail;
		DSN_counter	<= next_DSN_counter;
		ack_req_owner <= next_ack_req_owner;
		tx_req_owner <= next_tx_req_owner;
		own_fsm		<= next_own_fsm;
		ack_ready	<= next_ack_ready;
		ack_tx_cpl	<= next_ack_tx_cpl;
		tfrst		<= next_tfrst;
		own_cnt		<= next_own_cnt;
		glossy_req_owner <= next_glossy_req_owner;
		glossy_avail <= next_glossy_avail;
		glossy_tx_cpl <= next_glossy_tx_cpl;
		glossy_ready <= next_glossy_ready;
		sw_tx		<= next_sw_tx;
	end
end


always @ *
begin
	next_owner = owner;
	next_fire = fire;
	next_DSN_reg = DSN_reg;
	next_DSN_avail = DSN_avail;
	next_DSN_counter = DSN_counter;
	next_ack_req_owner = ack_req_owner;
	next_tx_req_owner = tx_req_owner;
	next_own_fsm = own_fsm;
	next_ack_ready = ack_ready;
	next_ack_tx_cpl = ack_tx_cpl;
	next_tfrst = tfrst;
	next_own_cnt = own_cnt;
	next_glossy_req_owner = glossy_req_owner;
	next_glossy_avail = glossy_avail;
	next_glossy_tx_cpl = glossy_tx_cpl;
	next_glossy_ready = glossy_ready;
	next_sw_tx = sw_tx;
	
	if (ack_flush)
	begin
		next_DSN_avail = 0;
		next_ack_ready = 1;
		next_sw_tx = 0;
	end

	if (glossy_avail)
	begin
		if (DSN_counter==0)
			next_glossy_req_owner = 1;
		else
			next_DSN_counter = DSN_counter - 1;
	end

	// attention
	if (rx_done & isGlossy & glossy_ready)
	begin
		next_DSN_counter = `DEFAULT_ACK_VALUE;
		next_glossy_ready = 0;
		next_glossy_avail = 1;
	end

	if (DSN_counter==`MAX2831_TXRX_TIME)
		next_sw_tx = 1;

	if (DSN_avail)
	begin
		if (DSN_counter==0)
			next_ack_req_owner = 1;
		else
			next_DSN_counter = DSN_counter - 1;
	end

	if (DSN_sel & ack_ready)
	begin
		next_ack_ready = 0;	// if ack_ready = 1, receive ack pkt, else, processing ack pkt
		next_DSN_reg = DSN;
		next_DSN_avail = 1;
		next_DSN_counter = `DEFAULT_ACK_VALUE;
	end

	if (tx_fire)
	begin
		next_tx_req_owner = 1;
	end

	case (owner)
		`TX:
		begin
			next_tx_req_owner = 0;
			case (own_fsm)
				2'b00:
				begin
					next_own_cnt = own_cnt + 1;
					if (own_cnt==1)
						next_fire = 1;
					else if (own_cnt==3)
						next_own_fsm = 2'b01;
				end

				2'b01:
				begin
					next_fire = 0;
					if (tx_cpl)
					begin
						next_own_fsm = 2'b10;
						next_tfrst = 0;
					end
				end

				2'b10:
				begin
					next_own_fsm = 2'b11;
				end

				2'b11:
				begin
					next_tfrst = 1;
					next_owner = `NONE;
				end
			endcase
		end

		`RX:
		begin
			next_ack_req_owner = 0;
			case (own_fsm)
				2'b00:
				begin
					next_own_cnt = own_cnt + 1;
					if (own_cnt==1)
						next_fire = 1;
					else if (own_cnt==3)
						next_own_fsm = 2'b01;
				end

				2'b01:
				begin
					next_fire = 0;
					if (tx_cpl)
					begin
						next_own_fsm = 2'b10;
					end
				end

				2'b10:
				begin
					next_own_fsm = 2'b11;
				end

				2'b11:
				begin
					next_ack_tx_cpl = 1;
					next_ack_ready = 1;
					next_owner = `NONE;
					next_sw_tx = 0;
				end
			endcase
			
		end

		`GLOSSY:
		begin
			next_glossy_req_owner = 0;
			case (own_fsm)
				2'b00:
				begin
					next_own_cnt = own_cnt + 1;
					if (own_cnt==1)
						next_fire = 1;
					else if (own_cnt==3)
						next_own_fsm = 2'b01;
				end

				2'b01:
				begin
					next_fire = 0;
					if (tx_cpl)
					begin
						next_own_fsm = 2'b10;
					end
				end

				2'b10:
				begin
					next_own_fsm = 2'b11;
				end

				2'b11:
				begin
					next_glossy_tx_cpl = 1;
					next_glossy_ready = 1;
					next_owner = `NONE;
					next_sw_tx = 0;
				end
			endcase
		end

		`NONE:
		begin
			next_own_fsm = 2'b00;
			next_ack_tx_cpl = 0;
			next_glossy_tx_cpl = 0;
			next_own_cnt = 0;
			if ((glossy_req_owner) & (~gfempty))
			begin
				next_glossy_avail = 0;
				next_owner = `GLOSSY;
			end
			else if ((ack_req_owner) & (~afempty))
			begin
				next_DSN_avail = 0;
				next_owner = `RX;
			end
			else if ((tx_req_owner) & (~tfempty))
			begin
				next_owner = `TX;
			end
		end

	endcase


end

always @ (posedge clk)
begin
	if (~nreset)
	begin
		glossy_state <= `GLOSSY_RST;
		gfdat <= 0;
		gfWE <= 0;
		gfcnt <= 0;
		gfrst <= 1;
	end
	else
	begin
		glossy_state <= next_glossy_state;
		gfdat <= next_gfdat;
		gfWE <= next_gfWE;
		gfcnt <= next_gfcnt;
		gfrst <= next_gfrst;
	end
end

always @ *
begin
	next_glossy_state = glossy_state;
	next_gfdat = gfdat;
	next_gfWE = 0;
	next_gfcnt = gfcnt;
	next_gfrst = 1;

	case (glossy_state)
		`GLOSSY_RST:
		begin
			next_glossy_state = `GLOSSY_INIT;
			next_gfcnt = 0;
			next_gfrst = 0;
		end

		`GLOSSY_INIT:
		begin
			next_gfdat = 0;
			next_gfWE = 1;
			next_gfcnt = gfcnt + 1;
			if (gfcnt==3)
				next_glossy_state = `GLOSSY_SFD;
		end

		`GLOSSY_SFD:
		begin
			next_gfdat = 8'ha7;
			next_gfWE = 1;
			next_glossy_state = `GLOSSY_COPY;
		end

		`GLOSSY_COPY:
		begin
			next_gfdat = rx_data;
			if (rx_WE)
			begin
				next_gfWE = 1;
				next_glossy_state = `GLOSSY_COPY2;
			end
			else if (rx_done)
			begin
				if (isGlossy)
					next_glossy_state = `GLOSSY_WAIT;
				else
					next_glossy_state = `GLOSSY_RST;
			end
		end

		`GLOSSY_COPY2:
		begin
			if (~rx_WE)
				next_glossy_state = `GLOSSY_COPY;
		end

		`GLOSSY_WAIT:
		begin
			if (glossy_tx_cpl)
				next_glossy_state = `GLOSSY_RST;
		end

		default:
		begin
			next_glossy_state = `GLOSSY_RST;
		end
	endcase
end

always @ (posedge clk)
begin
	if (~nreset)
	begin
		ack_state		<= `ACK_RST;
		afdat			<= 0;
		afWE			<= 0;
		ack_zero_cnt	<= 0;
		afrst			<= 1;
	end
	else
	begin
		ack_state		<= next_ack_state; 
		afdat			<= next_afdat;
		afWE			<= next_afWE;
		ack_zero_cnt	<= next_ack_zero_cnt;
		afrst			<= next_afrst;
	end
end

always @ *
begin
	next_ack_state = ack_state;
	next_afdat = afdat;
	next_afWE = afWE;
	next_ack_zero_cnt = ack_zero_cnt;
	next_afrst = 1;

	case (ack_state)
		`ACK_RST:
		begin
			next_afWE = 0;
			next_ack_zero_cnt = 0;
			next_afdat = 0;
			next_ack_state = `ZERO_ADD;
			next_afrst = 0;
		end

		`ZERO_ADD:
		begin
			next_afdat = 0;
			next_afWE = 1;
			next_ack_zero_cnt = ack_zero_cnt + 1;
			if (ack_zero_cnt==2'b11)
			begin
				next_ack_state = `FRAME_ADD;
				next_ack_zero_cnt = 0;
			end
		end

		`FRAME_ADD:
		begin
			next_afWE = 1;
			next_ack_zero_cnt = ack_zero_cnt + 1;
			case (ack_zero_cnt)
				2'b00:
				begin
					next_afdat = 8'ha7;
				end

				2'b01:
				begin
					next_afdat = 8'h05;
				end

				2'b10:
				begin
					next_afdat = 8'h02;
				end

				2'b11:
				begin
					next_afdat = 8'h00;
					next_ack_state = `WAIT_DSN;
				end
			endcase
		end

		`WAIT_DSN:
		begin
			if (DSN_avail)
			begin
				next_afdat = DSN_reg;
				next_afWE = 1;
				next_ack_state = `WAIT_DSN_CRC0;
			end
			else
				next_afWE = 0;
		end

		`WAIT_DSN_CRC0:
		begin
			if (crc_dsn_done)
			begin
				next_afWE = 1;
				next_afdat = crc_result[7:0];
				next_ack_state = `WAIT_DSN_CRC1;
			end
			else
				next_afWE = 0;
		end

		`WAIT_DSN_CRC1:
		begin
			next_afWE = 1;
			next_afdat = crc_result[15:8];
			next_ack_state = `ACK_READY;
			next_ack_zero_cnt = 0;
		end
		
		`ACK_READY:
		begin
			next_afWE = 0;
			if (ack_tx_cpl | ack_flush)
				next_ack_state = `ACK_RST;
		end


		default:
		begin
			next_ack_state = `ACK_RST;
		end
	endcase
end

always @ (posedge clk)
begin
	if (~nreset)
	begin
		crc_data_in		<= 0;
		crc_start		<= 0;
		crc_zero_cnt	<= 0;
		crc_clear		<= 0;
		crc_dsn_done	<= 0;
		crc_fsm			<= 0;
		crc_state		<= `CRC_RST;
	end
	else
	begin
		crc_data_in		<= next_crc_data_in;
		crc_start		<= next_crc_start;
		crc_zero_cnt	<= next_crc_zero_cnt;
		crc_clear		<= next_crc_clear;
		crc_dsn_done	<= next_crc_dsn_done;
		crc_fsm			<= next_crc_fsm;
		crc_state		<= next_crc_state;
	end
end

always @ *
begin
	next_crc_data_in = crc_data_in;
	next_crc_start = crc_start;
	next_crc_zero_cnt = crc_zero_cnt;
	next_crc_clear = crc_clear;
	next_crc_dsn_done = crc_dsn_done;
	next_crc_fsm = crc_fsm;
	next_crc_state = crc_state;
	case (crc_state)
		`CRC_RST:
		begin
			next_crc_clear = 0;
			next_crc_state = `CRC_PRELOAD;
			next_crc_dsn_done = 0;
			next_crc_fsm = 0;
		end

		`CRC_PRELOAD:
		begin
			next_crc_start = 1;
			next_crc_zero_cnt = 0;
			case (crc_fsm)
				0:
				begin
					next_crc_data_in = 8'h02;
					next_crc_state = `CRC_INTER_STATE;
					next_crc_fsm = 1;
				end

				1:
				begin
					next_crc_data_in = 8'h00;
					next_crc_state = `CRC_DSN;
					next_crc_fsm = 0;
				end

			endcase
		end

		`CRC_INTER_STATE:
		begin
			next_crc_start = 0;
			if (crc_zero_cnt<3)
				next_crc_zero_cnt = crc_zero_cnt + 1;
			else if (crc_ready)
				next_crc_state = `CRC_PRELOAD;
		end

		`CRC_DSN:
		begin
			next_crc_start = 0;
			if (crc_zero_cnt<3)
				next_crc_zero_cnt = crc_zero_cnt + 1;
			else if (crc_ready & DSN_avail)
			begin
				next_crc_state = `CRC_DSN_START;
				next_crc_start = 1;
				next_crc_data_in = DSN_reg;
				next_crc_zero_cnt = 0;
			end
		end

		`CRC_DSN_START:
		begin
			next_crc_start = 0;
			if (crc_zero_cnt<3)
				next_crc_zero_cnt = crc_zero_cnt + 1;
			else if (crc_ready)
			begin
				next_crc_state = `CRC_DONE;
				next_crc_dsn_done = 1;
			end
		end

		`CRC_DONE:
		begin
			if (ack_tx_cpl | ack_flush)
			begin
				next_crc_clear = 1;
				next_crc_state = `CRC_RST;
			end
		end

	endcase
end

fifo12x8 ackfifo(
    .DATA(afdat),
    .Q(afdout),
    .WE(afWE),
    .RE(afRE),
    .CLK(clk),
    .FULL(affull),
    .EMPTY(afempty),
    .RESET(nreset & afrst)
);

fifo128x8 txfifo(
    .DATA(tfdat),
    .Q(tfdout),
    .WE(tfWE),
    .RE(tfRE),
    .CLK(clk),
    .FULL(tffull),
    .EMPTY(tfempty),
    .RESET(nreset & tfrst)
);

fifo128x8 glossyfifo(
    .DATA(gfdat),
    .Q(gfdout),
    .WE(gfWE),
    .RE(gfRE),
    .CLK(clk),
    .FULL(),
    .EMPTY(gfempty),
    .RESET(nreset & gfrst)
);

fifo2wave	f2w0(	.HCLK(clk), .HRESETn(nreset),
					.data_out(data_out), .half_clk_out(half_clk_out), .fire(fire), .tx_cpl(tx_cpl), 
					.fifo_din(fifo_din), .fifo_RE(fifo_RE), .fifo_empty(fifo_empty), .stop_cnt());

crc			tx_crc(	.clk(clk), .nreset(nreset), .clear(crc_clear), .data_in(crc_data_in), 
					.crc_out(crc_result), .start(crc_start), .out_ready(crc_ready));

endmodule
