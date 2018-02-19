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
 * Last modify data: 7/7/2013,
 * Last modify content: change to APB interface
 */

module correlator(
	input	PCLK,
	input	PRESETn, 
	input	[31:0] PADDR, 
	input	PSELx, 
	input	PENABLE,
	input	PWRITE, 
	input	[31:0] PWDATA, 
	output	reg PREADY, 
	output	reg	[31:0] PRDATA,
	output	PSLVERR,

	input	adc_clk, 
	output	[7:0] test_point_cor, 
	// interrupt to processor
	output	reg sfd_int, 
	output	reg length_int, 
	// from FM decoder
	input	bit_dec,
	// from radio controller
	input	ack_en, 
	input	correlator_en,
	input	fifo_rst,
	// to fifo controller
	output	reg [7:0] DSN,
	output	reg DSN_sel,
	output	[7:0] fifo_data,
	output	reg fifo_WE, 
	output	reg packet_done, 
	output	reg isGlossy, 
	output	isAckGlossy, 
	// Inputs from AGC
	input	[7:0] RSSI_IN
);

assign PSLVERR = 0;


// Inputs from AGC
reg		[7:0]	rssi, next_rssi;

reg		[2:0]	fsm;


// ACK IOs
reg				next_DSN_sel;

// frame format
reg		[15:0]	FCF_reg, next_FCF_reg;
reg		[7:0]	next_DSN_reg;
reg		[159:0]	address_field, next_address_field;


// frame control
wire	[15:0]	dest_pan, src_pan;
wire	[63:0]	dest_addr, src_addr;
wire	[2:0]	frame_type = FCF_reg[2:0];
wire			security_enable = FCF_reg[3];
wire			frame_pending = FCF_reg[4];
wire			ack_req	= FCF_reg[5];
wire			broadcast2byte, broadcast8byte, glossy2byte;
reg				next_length_int;
assign broadcast2byte = (dest_addr[15:0]==16'hffff)? 1 : 0;
assign broadcast8byte = (dest_addr[63:16]==48'hffffffffffff)? 1 : 0;
assign glossy2byte = (dest_addr[15:0]==16'hfffe)? 1 : 0;
wire	isBroadCast = (FCF_reg[11:10]==2'b10)? broadcast2byte :
					  (FCF_reg[11:10]==2'b11)? (broadcast2byte&broadcast8byte) : 0;

//glossy
reg				next_packet_done;
reg				next_isGlossy;
reg				glossy_crc_start, next_glossy_crc_start;
wire	[15:0]	glossy_crc_result;
wire	Glossy = 	(FCF_reg[11:10]==2'b10)? glossy2byte :
					(FCF_reg[11:10]==2'b11)? (glossy2byte&broadcast8byte) : 0;

localparam TYPE_BEACON =	3'b000;
localparam TYPE_DATA =	3'b001;
localparam TYPE_ACK =	3'b010;
localparam TYPE_MACCMD =	3'b011;

// fifo IOs
reg				fifo_RE, next_fifo_WE;
reg		[7:0]	fifo_st_data, next_fifo_st_data;
wire	[7:0]	fifo_out;
wire			fifo_empty, fifo_full;

// store
reg		[6:0]	dat_length, next_dat_length, cur_length, next_cur_length;
reg		[3:0]	st_state, next_st_state;


// crc
wire			crc_ready;
reg				crc_start, next_crc_start, crc_clear, next_crc_clear, crc_correct, next_crc_correct;
reg		[1:0]	crc_cnt, next_crc_cnt;
wire	[15:0]	crc_result;
wire			isCrcCorrect = (crc_result==0)? 1 : 0;

localparam ST_RESET =			4'b0000;
localparam ST_SFD =				4'b0001;
localparam ST_LENGTH =			4'b0010;
localparam ST_DATA =				4'b0011;
localparam ST_DATA_PROC =		4'b0100;
localparam ST_DATA_PROC2 =		4'b0101;
localparam ST_CRC_STATE =		4'b0110;
localparam ST_CRC_STORE =		4'b0111;
localparam ST_WAIT_ZERO_STATE =	4'b1000;

// main controller
reg				next_sfd_interrupt;

wire			zero_locked, chip_locked;
reg				cmp_rst, next_cmp_rst;
reg				ack_set, next_ack_set;
wire	[7:0]	chip_decode_byte;
wire			sfd_locked;
wire			byte_update;

// testpoint
assign	test_point_cor = {sfd_int, length_int, packet_done, crc_correct, fifo_WE, fifo_RE, fifo_full, fifo_empty};
//assign	test_point_cor = chip_decode_byte;

assign	isAckGlossy = (isGlossy | DSN_sel);

wire	[7:0]	glossy_data = (cur_length==3)? (chip_decode_byte + 1) : chip_decode_byte;	
assign fifo_data = (cur_length==(dat_length-1))? glossy_crc_result[7:0] : (cur_length==dat_length)? glossy_crc_result[15:8] : glossy_data;

always @ (posedge PCLK or negedge PRESETn)
begin
    if (~PRESETn)
    begin
		fsm <= 0;
		PREADY <= 0;
		fifo_RE	<= 1'b0;
		PRDATA <= 0;
    end
    else
    begin
		case (fsm)
		    0:
		    begin
				if (PSELx & PENABLE)
					fsm <= 1;
				fifo_RE		<= 1'b0;
		    end
		    
		    1:
		    begin
				if ((~PWRITE) && (PADDR[4:2]==0))
				begin
					fsm	<= 2;
					fifo_RE <= 1;
				end
				else
					fsm <= 4;
		    end

			2:
			begin
				fifo_RE <= 0;
				fsm <= 3;
			end

			3:
			begin
				fsm <= 4;
			end

			4:
			begin
				PREADY <= 1;
				fsm <= 5;
				case (PADDR[4:2])
					// data: 0x40050000
					0: begin PRDATA <= {24'b0, fifo_out}; end

					// pan ID src/dest: 0x40050004	
					1: begin PRDATA <= {src_pan, dest_pan}; end

					// crc / address mode: 0x4005000c
					3: begin PRDATA <= {13'b0, fifo_empty, ack_set, crc_correct, FCF_reg}; end

					// source address bit 31:0 0x40050010
					4: begin PRDATA <= src_addr[31:0]; end

					// source address bit 63:32 0x40050014
					5: begin PRDATA <= src_addr[63:32]; end

					// dest address bit 31:0 0x40050018
					6: begin PRDATA <= dest_addr[31:0]; end

					// dest address bit 63:32 0x4005001c
					7: begin PRDATA <= dest_addr[63:32]; end
				endcase
			end

			5:
			begin
				if (~PSELx)
				begin
					PREADY <= 0;
					fsm <= 0;
				end
			end

		endcase
    end
end

wire corr_self_rst = (PRESETn & correlator_en);

always @ (posedge adc_clk or negedge corr_self_rst)
begin
    if (~corr_self_rst)
    begin
		fifo_WE				<= 0;
		st_state			<= ST_RESET;
		fifo_st_data		<= 0;
		dat_length			<= 0;
		sfd_int				<= 0;
		cur_length			<= 0;
		crc_start			<= 0;
		crc_clear			<= 0;
		crc_cnt				<= 0;
		crc_correct			<= 0;
		DSN					<= 0;
		DSN_sel				<= 0;
		FCF_reg				<= 0;
		address_field		<= 0;
		cmp_rst				<= 1;
		length_int			<= 0;
		packet_done			<= 0;
		isGlossy			<= 0;
		glossy_crc_start	<= 0;
		ack_set				<= 0;
		rssi				<= 0;
    end
    else
    begin
		fifo_WE				<= next_fifo_WE;
		st_state			<= next_st_state;
		fifo_st_data		<= next_fifo_st_data;
		dat_length			<= next_dat_length;
		sfd_int				<= next_sfd_interrupt;
		cur_length			<= next_cur_length;
		crc_start			<= next_crc_start;
		crc_clear			<= next_crc_clear;
		crc_cnt				<= next_crc_cnt;
		crc_correct			<= next_crc_correct;
		DSN					<= next_DSN_reg;
		DSN_sel				<= next_DSN_sel;
		FCF_reg				<= next_FCF_reg;
		address_field		<= next_address_field;
		cmp_rst				<= next_cmp_rst;
		length_int			<= next_length_int;
		packet_done			<= next_packet_done;
		isGlossy			<= next_isGlossy;
		glossy_crc_start	<= next_glossy_crc_start;
		ack_set				<= next_ack_set;
		rssi				<= next_rssi;
    end
end

always @ (*)
begin
	
	next_fifo_st_data	= chip_decode_byte;
	next_st_state		= st_state;
	next_dat_length		= dat_length;
	next_cur_length		= cur_length;
	next_sfd_interrupt	= sfd_int;
	// FIFO control
	next_fifo_WE		= 0;
	next_crc_start		= 0;
	next_crc_clear		= crc_clear;
	next_crc_cnt		= crc_cnt;
	next_crc_correct	= crc_correct;
	next_DSN_reg		= DSN;
	next_DSN_sel		= 0;
	next_FCF_reg		= FCF_reg;
	next_address_field	= address_field;
	next_cmp_rst		= 1;
	next_length_int		= length_int;
	next_packet_done	= packet_done;
	next_isGlossy		= isGlossy;
	next_glossy_crc_start = 0;
	next_ack_set		= ack_set;
	next_rssi			= rssi;

	
	case (st_state)
		ST_RESET:
		begin
			next_packet_done = 0;
			next_fifo_st_data = 0;
			next_length_int = 0;
			next_DSN_reg = 0;
			next_DSN_sel = 0;
			next_isGlossy = 0;
			next_st_state = ST_SFD;
			next_sfd_interrupt = 0;
		end

		ST_SFD:
		begin
			if (sfd_locked)
			begin
				next_st_state = ST_LENGTH;
				next_sfd_interrupt = 1;
				next_rssi = RSSI_IN;
			end
		end

		ST_LENGTH:
		begin
			next_cur_length = 0;
			next_crc_clear = 0;
			next_crc_clear = 1;
			if (byte_update)
			begin
				if ((chip_decode_byte[7]) || (chip_decode_byte<5))
				begin
					next_st_state = ST_RESET;
					next_cmp_rst = 0;
				end
				else
				begin
					next_dat_length = chip_decode_byte;
					next_fifo_WE = 1;
					next_length_int = 1;
					next_st_state = ST_DATA;
				end
			end
		end

		ST_DATA:
		begin
			next_crc_clear = 0;
			if (byte_update)
				next_st_state = ST_DATA_PROC;
		end

		ST_DATA_PROC:
		begin
			next_st_state = ST_DATA_PROC2;
			next_cur_length = cur_length + 1;
			next_crc_start = 1;

			if (cur_length < (dat_length-2))
				next_glossy_crc_start = 1;

			if (cur_length < (dat_length-1))
				next_fifo_WE = 1;

			if (cur_length==(dat_length-2))
				next_fifo_st_data = rssi;

			case (cur_length)
				0: begin next_FCF_reg[7:0] = chip_decode_byte; end
				1: begin next_FCF_reg[15:8] = chip_decode_byte; end
				2: begin next_DSN_reg = chip_decode_byte; end
				3: begin next_address_field[159:152] = chip_decode_byte; end
				4: begin next_address_field[151:144] = chip_decode_byte; end
				5: begin next_address_field[143:136] = chip_decode_byte; end
				6: begin next_address_field[135:128] = chip_decode_byte; end
				7: begin next_address_field[127:120] = chip_decode_byte; end
				8: begin next_address_field[119:112] = chip_decode_byte; end
				9: begin next_address_field[111:104] = chip_decode_byte; end
				10: begin next_address_field[103:96] = chip_decode_byte; end
				11: begin next_address_field[95:88] = chip_decode_byte; end
				12: begin next_address_field[87:80] = chip_decode_byte; end
				13: begin next_address_field[79:72] = chip_decode_byte; end
				14: begin next_address_field[71:64] = chip_decode_byte; end
				15: begin next_address_field[63:56] = chip_decode_byte; end
				16: begin next_address_field[55:48] = chip_decode_byte; end
				17: begin next_address_field[47:40] = chip_decode_byte; end
				18: begin next_address_field[39:32] = chip_decode_byte; end
				19: begin next_address_field[31:24] = chip_decode_byte; end
				20: begin next_address_field[23:16] = chip_decode_byte; end
				21: begin next_address_field[15:8] = chip_decode_byte; end
				22: begin next_address_field[7:0] = chip_decode_byte; end
			endcase
		end

		ST_DATA_PROC2:
		begin
			next_crc_cnt = 0;
			if (cur_length == dat_length)
				next_st_state = ST_CRC_STATE;
			else 
			begin
				next_st_state = ST_DATA;
			end
		end

		ST_CRC_STATE:
		begin
			if (~(crc_cnt[1] & crc_cnt[0]))
				next_crc_cnt = crc_cnt + 1;
			else if (crc_ready)
			begin
				next_fifo_WE = 1;
				next_st_state = ST_CRC_STORE;
				next_fifo_st_data = isCrcCorrect;
				next_crc_correct = isCrcCorrect;
			end
		end

		ST_CRC_STORE:
		begin
			if (crc_correct)
			begin
				if ((DSN>0) && (Glossy))
					next_isGlossy = 1;
				else if (ack_req & ack_en & (~isBroadCast) & (~Glossy))
				begin
					next_DSN_sel = 1;
					next_ack_set = 1;
				end
			end
			next_st_state = ST_RESET;
			next_length_int = 0;
			next_packet_done = 1;
			next_cmp_rst = 0;
		end

	endcase
end

wire	[7:0]	f0counter;
wire	[30:0]	seq0, seq1, seq2, seq3, seq4, seq5, seq6, seq7;
wire	[3:0]	chip_decode_idx;
frame_ctl ctl_0(.FCF(FCF_reg), .address_field(address_field), .src_pan(src_pan), .dest_pan(dest_pan), 
														.dest_addr(dest_addr), .src_addr(src_addr));

crc rx_crc(	.clk(adc_clk), .resetn(PRESETn), .clear(crc_clear), 
			.data_in(chip_decode_byte), .crc_out(crc_result), .start(crc_start), .out_ready(crc_ready));

crc glossy_crc(	.clk(adc_clk), .resetn(PRESETn), .clear(crc_clear), 
			.data_in(glossy_data), .crc_out(glossy_crc_result), .start(glossy_crc_start), .out_ready());

zero_finding f0(.adc_clock(adc_clk), .resetn(PRESETn & cmp_rst), .input_seq(bit_dec), .zero_found(zero_locked), 
				.bit_counter(f0counter), .in_seq0(seq0), .in_seq1(seq1), .in_seq2(seq2), .in_seq3(seq3), 
				.in_seq4(seq4), .in_seq5(seq5), .in_seq6(seq6), .in_seq7(seq7));

sequence_compare cmp0(.adc_clock(adc_clk), .resetn(PRESETn & cmp_rst), .zero_found(zero_locked), 
					.output_idx(chip_decode_idx), .idx_found(chip_locked), .seq_counter(f0counter),
					.seq0_in(seq0), .seq1_in(seq1), .seq2_in(seq2), .seq3_in(seq3), 
					.seq4_in(seq4), .seq5_in(seq5), .seq6_in(seq6), .seq7_in(seq7));

sfd_sync sfd_sync0( .resetn(PRESETn), .clk(adc_clk), .clearn(cmp_rst), .input_hb(chip_decode_idx), 
					.zero_locked(zero_locked), .chip_locked(chip_locked), .idx_out(chip_decode_byte), 
					.SFD_LOCKED(sfd_locked), .update(byte_update));

em_fifo rx_fifo(
    .DATA(fifo_st_data),
    .Q(fifo_out),
    .WE(fifo_WE), 
    .RE(fifo_RE),
    .WCLOCK(adc_clk),
	.RCLOCK(PCLK),
    .FULL(fifo_full),
    .EMPTY(fifo_empty),
    .RESET(PRESETn & fifo_rst)
);

endmodule   
