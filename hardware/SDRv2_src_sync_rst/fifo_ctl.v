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

module fifo_ctl(
	input 	clk, 
	input	nreset, 
	// to parallel DAC
	output	[7:0] data_out, 
	output	half_clk_out, 

	input	tx_fire, 
	input	[7:0] tfdat,
	input	tfWE, 
	
	output	tx_complete,
	output	tffull, 
	output	tfempty, 
	// from correlator
	input	[7:0] DSN,
	input	DSN_sel, 
	// to radio controller
	output	fifo_ready, 
	output	reg [1:0] owner,
	output	fifo_tx_cpl,

	input	[7:0] rx_data, 
	input	rx_WE, 
	input	isGlossy, 
	input	length_int,
	// to raido controll
	output	reg sw_tx, 
	input	ack_flush, 
	output	reg [2:0] ack_gl_state, 
	output	[7:0] fifo_ctl_tp
);

reg				next_sw_tx;	// MAX2831 TX, RX line

//	Transmit Fifo
wire	[3:0]	tfdout;
wire			tfRE;

// Acknowledge Fifo
reg		[7:0]	afdat, next_afdat;
reg				afWE, next_afWE;
wire			afRE;
wire	[3:0]	afdout;
wire			affull, afempty;
reg				afrst, next_afrst, tfrst, next_tfrst;

//glossy fifo
reg		[7:0]	gfdat, next_gfdat;
reg				gfWE, next_gfWE;
wire	[3:0]	gfdout;
wire			gfRE;
reg		[1:0]	gfcnt, next_gfcnt;
reg				gfrst, next_gfrst; 
wire			gfempty;
reg				glossy_tx_cpl, next_glossy_tx_cpl;
reg		[2:0]	glossy_state, next_glossy_state;

reg				ack_fifo_ready, next_ack_fifo_ready, glossy_fifo_ready, next_glossy_fifo_ready;
reg				length_int_reg, length_int_reg2;

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

reg		[1:0]	next_owner;
reg		[1:0]	own_cnt, next_own_cnt;
reg		[7:0]	DSN_reg, next_DSN_reg;
reg				DSN_avail, next_DSN_avail;
reg		[13:0]	txrx_turn_cnt, next_txrx_turn_cnt;
reg		[1:0]	own_fsm, next_own_fsm;
reg				ack_tx_cpl, next_ack_tx_cpl;
reg		[2:0]	next_ack_gl_state;

reg		[3:0]	ack_state, next_ack_state;
reg		[2:0]	ack_zero_cnt, next_ack_zero_cnt;

reg		[1:0]	crc_zero_cnt, next_crc_zero_cnt;
reg				crc_dsn_done, next_crc_dsn_done;
reg				crc_fsm, next_crc_fsm;
reg		[2:0]	crc_state, next_crc_state;


parameter TX =				2'b10;
parameter RX =				2'b01;
parameter GLOSSY =			2'b11;
parameter NONE =			2'b00;

parameter ACK_RST =			4'b0000;
parameter ZERO_ADD =		4'b0001;
parameter FRAME_ADD =		4'b0010;
parameter WAIT_DSN =		4'b0011;
parameter WRITE_DSN =		4'b0100;
parameter WAIT_DSN_CRC0 =	4'b0101;
parameter WRITE_DSN_CRC0 =	4'b0110;
parameter WAIT_DSN_CRC1 =	4'b0111;
parameter WRITE_DSN_CRC1 =	4'b1000;
parameter ACK_READY =		4'b1001;
parameter FIFO_WE_DEASSERT =4'b1010;
parameter ACK_WAIT_FIFO =	4'b1111;

parameter CRC_RST =			3'b000;
parameter CRC_PRELOAD =		3'b001;
parameter CRC_INTER_STATE =	3'b010;
parameter CRC_DSN =			3'b011;
parameter CRC_DSN_START =	3'b100;
parameter CRC_DONE =		3'b101;

parameter GLOSSY_RST =		3'b000;
parameter GLOSSY_INIT =		3'b001;
parameter GLOSSY_SFD =		3'b010;
parameter GLOSSY_COPY =		3'b011;
parameter GLOSSY_COPY2 =	3'b100;
parameter GLOSSY_WAIT =		3'b101;
parameter GLOSSY_WRFIFO =	3'b110;

parameter WAITING =			3'b000;
parameter GLOSSY_CNT =		3'b001;
parameter ACK_CNT =			3'b010;
parameter TXING	=			3'b011;
parameter GLOSSYING =		3'b100;
parameter ACKING =			3'b101;
parameter COLLECTING_DATA =	3'b110;

parameter HCLK_FREQ = 48;		// 48MHz
parameter PKT_DONE_LAT = 5;		// 5us
parameter MAX2831_TXRX_TIME	= 2*(HCLK_FREQ);		// HCLK is 48MHz, MAX2831 requires 2us to turnaround, 2/(1/48) = 96 ticks
parameter DEFAULT_TURN_TIME	= (192-PKT_DONE_LAT)*(HCLK_FREQ);		// 60us for warmup

assign fifo_din = (owner==GLOSSY)? gfdout : (owner==TX)? tfdout : (owner==RX)? afdout : 0;
assign tfRE = (owner==TX)? fifo_RE : 0;
assign afRE = (owner==RX)? fifo_RE : 0;
assign gfRE = (owner==GLOSSY)? fifo_RE : 0;
assign fifo_empty = (owner==TX)? tfempty : (owner==RX)? afempty : (owner==GLOSSY)? gfempty : 1;
assign tx_complete = (owner==TX)? tx_cpl : 0;	// goes to processor only
assign fifo_tx_cpl = tx_cpl;
assign fifo_ready = (ack_gl_state==WAITING)? 1 : 0;
assign fifo_ctl_tp = {ack_state, afWE, afRE, DSN_sel, crc_dsn_done};


always @ (posedge clk)
begin
	if (~nreset)
	begin
		owner		<= NONE;
		fire		<= 0;
		DSN_reg		<= 0;
		DSN_avail	<= 0;
		txrx_turn_cnt <= DEFAULT_TURN_TIME;
		own_fsm		<= 0;
		ack_tx_cpl	<= 0;
		tfrst		<= 1;
		own_cnt		<= 0;
		glossy_tx_cpl <= 0;
		sw_tx		<= 0;
		ack_gl_state <= WAITING;
	end
	else
	begin
		owner		<= next_owner;
		fire		<= next_fire;
		DSN_reg		<= next_DSN_reg;
		DSN_avail	<= next_DSN_avail;
		txrx_turn_cnt <= next_txrx_turn_cnt;
		own_fsm		<= next_own_fsm;
		tfrst		<= next_tfrst;
		own_cnt		<= next_own_cnt;
		sw_tx		<= next_sw_tx;
		glossy_tx_cpl <= next_glossy_tx_cpl;
		ack_tx_cpl	<= next_ack_tx_cpl;
		ack_gl_state <= next_ack_gl_state;
	end
end


always @ *
begin
	next_owner = owner;
	next_fire = fire;
	next_DSN_reg = DSN_reg;
	next_DSN_avail = DSN_avail;
	next_txrx_turn_cnt = txrx_turn_cnt;
	next_own_fsm = own_fsm;
	next_tfrst = tfrst;
	next_own_cnt = own_cnt;
	next_sw_tx = sw_tx;
	next_ack_tx_cpl = ack_tx_cpl;
	next_glossy_tx_cpl = glossy_tx_cpl;

	next_ack_gl_state = ack_gl_state;

	case (ack_gl_state)
		WAITING:
		begin
			next_owner = NONE;
			next_fire = 0;
			next_DSN_reg = 0;
			next_DSN_avail = 0;
			next_txrx_turn_cnt = DEFAULT_TURN_TIME;
			next_own_fsm = 0;
			next_tfrst = 1;
			next_own_cnt = 0;
			next_sw_tx = 0;
			next_ack_tx_cpl = 0;
			next_glossy_tx_cpl = 0;

			if (tx_fire)
			begin
				next_ack_gl_state = TXING;
				next_owner = TX;
			end
			else if (length_int_reg)
				next_ack_gl_state = COLLECTING_DATA;
		end

		COLLECTING_DATA:
		begin
			if (~length_int_reg)
			begin
				if (isGlossy)
					next_ack_gl_state = GLOSSY_CNT;
				else if (DSN_sel)
				begin
					next_ack_gl_state = ACK_CNT;
					next_DSN_reg = DSN;
					next_DSN_avail = 1;
				end
				else
					next_ack_gl_state = WAITING;
			end
		end

		GLOSSY_CNT:
		begin
			next_txrx_turn_cnt = txrx_turn_cnt - 1;
			if (txrx_turn_cnt==0)
			begin
				if (glossy_fifo_ready)
				begin
					next_ack_gl_state = GLOSSYING;
					next_owner = GLOSSY;
				end
				else
					next_ack_gl_state = WAITING;
			end
			else if (txrx_turn_cnt==MAX2831_TXRX_TIME)
				next_sw_tx = 1;
		end

		ACK_CNT:
		begin
			next_txrx_turn_cnt = txrx_turn_cnt - 1;
			if (txrx_turn_cnt==0)
			begin
				if (ack_fifo_ready)
				begin
					next_ack_gl_state = ACKING;
					next_owner = RX;
				end
				else
					next_ack_gl_state = WAITING;
			end
			else if (txrx_turn_cnt==MAX2831_TXRX_TIME)
				next_sw_tx = 1;
			
			if (ack_flush)
				next_ack_gl_state = WAITING;
		end

		TXING:
		begin
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
					next_ack_gl_state = WAITING;
				end
			endcase
		end

		GLOSSYING:
		begin
			next_sw_tx = 0;
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
					next_ack_gl_state = WAITING;
				end
			endcase
		end

		ACKING:
		begin
			next_sw_tx = 0;
			next_DSN_avail = 0;
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
					next_ack_gl_state = WAITING;
				end
			endcase
		end

	endcase
end

always @ (posedge clk)
begin
	if (~nreset)
	begin
		glossy_state <= GLOSSY_RST;
		gfdat <= 0;
		gfWE <= 0;
		gfcnt <= 0;
		gfrst <= 1;
		length_int_reg <= 0;
		length_int_reg2 <= 0;
		glossy_fifo_ready <= 0;
	end
	else
	begin
		glossy_state <= next_glossy_state;
		gfdat <= next_gfdat;
		gfWE <= next_gfWE;
		gfcnt <= next_gfcnt;
		gfrst <= next_gfrst;
		length_int_reg <= length_int;
		length_int_reg2 <= length_int_reg;
		glossy_fifo_ready <= next_glossy_fifo_ready;
	end
end

always @ *
begin
	next_glossy_state = glossy_state;
	next_gfdat = gfdat;
	next_gfWE = gfWE;
	next_gfcnt = gfcnt;
	next_gfrst = 1;
	next_glossy_fifo_ready = glossy_fifo_ready;

	case (glossy_state)
		GLOSSY_RST:
		begin
			next_glossy_state = GLOSSY_INIT;
			next_glossy_fifo_ready = 0;
			next_gfcnt = 0;
			next_gfrst = 0;
		end

		GLOSSY_INIT:
		begin
			next_gfdat = 0;
			next_gfWE = 1;
			next_gfcnt = gfcnt + 1;
			if (gfcnt==3)
				next_glossy_state = GLOSSY_SFD;
		end

		GLOSSY_SFD:
		begin
			next_gfdat = 8'ha7;
			next_gfWE = 1;
			next_glossy_state = GLOSSY_COPY;
		end

		GLOSSY_COPY:
		begin
			next_gfWE = 0;
			if (rx_WE)
				next_glossy_state = GLOSSY_WRFIFO;
			else if (length_int_reg2 & (~length_int_reg)) // negative edge of length_int = packet done
			begin
				if (isGlossy)
					next_glossy_state = GLOSSY_WAIT;
				else
					next_glossy_state = GLOSSY_RST;
			end
		end

		GLOSSY_WRFIFO:
		begin
			next_gfWE = 1;
			next_gfdat = rx_data;
			next_glossy_state = GLOSSY_COPY2;
		end

		GLOSSY_COPY2:
		begin
			next_gfWE = 0;
			if (~rx_WE)
				next_glossy_state = GLOSSY_COPY;
		end

		GLOSSY_WAIT:
		begin
			next_glossy_fifo_ready = 1;
			if (glossy_tx_cpl)
				next_glossy_state = GLOSSY_RST;
		end

		default:
		begin
			next_glossy_state = GLOSSY_RST;
		end
	endcase
end

always @ (posedge clk)
begin
	if (~nreset)
	begin
		ack_state		<= ACK_RST;
		afdat			<= 0;
		afWE			<= 0;
		ack_zero_cnt	<= 0;
		afrst			<= 1;
		ack_fifo_ready	<= 0;
	end
	else
	begin
		ack_state		<= next_ack_state; 
		afdat			<= next_afdat;
		afWE			<= next_afWE;
		ack_zero_cnt	<= next_ack_zero_cnt;
		afrst			<= next_afrst;
		ack_fifo_ready	<= next_ack_fifo_ready;
	end
end

always @ *
begin
	next_ack_state = ack_state;
	next_afdat = afdat;
	next_afWE = 0;
	next_ack_zero_cnt = ack_zero_cnt;
	next_afrst = 1;
	next_ack_fifo_ready = ack_fifo_ready;

	case (ack_state)
		ACK_RST:
		begin
			next_ack_zero_cnt = 0;
			next_afdat = 0;
			next_ack_state = ACK_WAIT_FIFO;
			next_afrst = 0;
			next_ack_fifo_ready = 0;
		end

		ACK_WAIT_FIFO:
		begin
			next_ack_state = FRAME_ADD;
		end

		FIFO_WE_DEASSERT:
		begin
			next_ack_zero_cnt = ack_zero_cnt + 1;
			if (ack_zero_cnt==7)
				next_ack_state = WAIT_DSN;
			else
				next_ack_state = FRAME_ADD;
		end

		FRAME_ADD:
		begin
			next_afWE = 1;
			next_ack_state = FIFO_WE_DEASSERT;
			case (ack_zero_cnt)
				0:
				begin
					next_afdat = 8'h00;
				end

				1:
				begin
					next_afdat = 8'h00;
				end

				2:
				begin
					next_afdat = 8'h00;
				end

				3:
				begin
					next_afdat = 8'h00;
				end

				4:
				begin
					next_afdat = 8'ha7;
				end

				5:
				begin
					next_afdat = 8'h05;
				end

				6:
				begin
					next_afdat = 8'h02;
				end

				7:
				begin
					next_afdat = 8'h00;
				end
			endcase
		end

		WAIT_DSN:
		begin
			if (DSN_avail)
				next_ack_state = WRITE_DSN;
		end

		WRITE_DSN:
		begin
			next_afWE = 1;
			next_afdat = DSN_reg;
			next_ack_state = WAIT_DSN_CRC0;
		end

		WAIT_DSN_CRC0:
		begin
			if (crc_dsn_done)
				next_ack_state = WRITE_DSN_CRC0;
		end

		WRITE_DSN_CRC0:
		begin
			next_afWE = 1;
			next_afdat = crc_result[7:0];
			next_ack_state = WAIT_DSN_CRC1;
		end

		WAIT_DSN_CRC1:
		begin
			next_ack_state = WRITE_DSN_CRC1;
		end

		WRITE_DSN_CRC1:
		begin
			next_afWE = 1;
			next_afdat = crc_result[15:8];
			next_ack_state = ACK_READY;
		end
		
		ACK_READY:
		begin
			next_ack_fifo_ready = 1;
			if (ack_tx_cpl | ack_flush)
				next_ack_state = ACK_RST;
		end


		default:
		begin
			next_ack_state = ACK_RST;
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
		crc_state		<= CRC_RST;
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
		CRC_RST:
		begin
			next_crc_clear = 0;
			next_crc_state = CRC_PRELOAD;
			next_crc_dsn_done = 0;
			next_crc_fsm = 0;
		end

		CRC_PRELOAD:
		begin
			next_crc_start = 1;
			next_crc_zero_cnt = 0;
			case (crc_fsm)
				0:
				begin
					next_crc_data_in = 8'h02;
					next_crc_state = CRC_INTER_STATE;
					next_crc_fsm = 1;
				end

				1:
				begin
					next_crc_data_in = 8'h00;
					next_crc_state = CRC_DSN;
					next_crc_fsm = 0;
				end

			endcase
		end

		CRC_INTER_STATE:
		begin
			next_crc_start = 0;
			if (crc_zero_cnt<3)
				next_crc_zero_cnt = crc_zero_cnt + 1;
			else if (crc_ready)
				next_crc_state = CRC_PRELOAD;
		end

		CRC_DSN:
		begin
			next_crc_start = 0;
			if (crc_zero_cnt<3)
				next_crc_zero_cnt = crc_zero_cnt + 1;
			else if (crc_ready & DSN_avail)
			begin
				next_crc_state = CRC_DSN_START;
				next_crc_start = 1;
				next_crc_data_in = DSN_reg;
				next_crc_zero_cnt = 0;
			end
		end

		CRC_DSN_START:
		begin
			next_crc_start = 0;
			if (crc_zero_cnt<3)
				next_crc_zero_cnt = crc_zero_cnt + 1;
			else if (crc_ready)
			begin
				next_crc_state = CRC_DONE;
				next_crc_dsn_done = 1;
			end
		end

		CRC_DONE:
		begin
			if (ack_tx_cpl | ack_flush)
			begin
				next_crc_clear = 1;
				next_crc_state = CRC_RST;
			end
		end

	endcase
end

fifo12x8 ackfifo(
//fifo128x8 ackfifo(
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
