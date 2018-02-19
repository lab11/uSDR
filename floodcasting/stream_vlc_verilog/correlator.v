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
 * Last modify content: move SFD detect
 */

module correlator(
	input	HRESETn, 

	input	adc_clk, 
	output	[7:0] test_point_cor, 
	// interrupt to processor
	output	reg sfd_int, 
	output	reg length_int, 
	output	reg glossy_int,
	// from FM decoder
	input	bit_dec,
	// from radio controller
	input	ack_en, 
	input	correlator_en,
	input	VLC_PDn,
	// to fifo controller
	output	reg [7:0] DSN,
	output	reg DSN_sel,
	output	[7:0] fifo_data,
	output	reg fifo_WE, 
	output	reg packet_done, 
	output	reg isGlossy, 
	output	isAckGlossy, 
	// Inputs from AGC
	input	[7:0] RSSI_IN,
	// Outputs to VLC DAC 
	output	VLC_DAC_CS,
	output	VLC_DAC_WR, 
	output	[7:0] VLC_DAC_DATA,
	output	VLC_DAC_GAIN, 
	output	VLC_DAC_LDAC, 
	output	VLC_DAC_CLR,
	output	VLC_DAC_PD, 
	output	VLC_DAC_A0, 
	output	VLC_DAC_A1
);

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

//parameter GLOSSY_HOP_THRESHOLD = 8'd127;
//wire			DSNLessThanThreshold = (DSN < GLOSSY_HOP_THRESHOLD)? 1 : 0; 

parameter TYPE_BEACON =	3'b000;
parameter TYPE_DATA =	3'b001;
parameter TYPE_ACK =	3'b010;
parameter TYPE_MACCMD =	3'b011;

// fifo IOs
reg				next_fifo_WE;
reg		[7:0]	fifo_st_data, next_fifo_st_data;
wire	[7:0]	fifo_out1, fifo_out2, fifo_out;
wire			fifo_empty1, fifo_empty2;

// store
reg		[6:0]	dat_length, next_dat_length, cur_length, next_cur_length;
reg		[3:0]	st_state, next_st_state;


// crc
wire			crc_ready;
reg				crc_start, next_crc_start, crc_clear, next_crc_clear, crc_correct, next_crc_correct;
reg		[1:0]	crc_cnt, next_crc_cnt;
wire	[15:0]	crc_result;
wire			isCrcCorrect = (crc_result==0)? 1 : 0;

parameter ST_RESET =			4'b0000;
parameter ST_SFD =				4'b0001;
parameter ST_LENGTH =			4'b0010;
parameter ST_DATA =				4'b0011;
parameter ST_DATA_PROC =		4'b0100;
parameter ST_DATA_PROC2 =		4'b0101;
parameter ST_CRC_STATE =		4'b0110;
parameter ST_CRC_STORE =		4'b0111;
parameter ST_WAIT_ZERO_STATE =	4'b1000;

// main controller
reg				next_sfd_interrupt;

wire			zero_locked, chip_locked;
reg				cmp_rst, next_cmp_rst;
wire	[7:0]	chip_decode_byte;
wire			sfd_locked;
wire			byte_update;

// testpoint
//assign	test_point_cor[3:0] = {sfd_int, length_int, packet_done, crc_correct};
//assign	test_point_cor = chip_decode_byte;


assign	isAckGlossy = (isGlossy | DSN_sel);

wire	[7:0]	glossy_data = (cur_length==3)? (chip_decode_byte - 1'b1) : chip_decode_byte;	
assign fifo_data = (cur_length==(dat_length-1))? glossy_crc_result[7:0] : (cur_length==dat_length)? glossy_crc_result[15:8] : glossy_data;

parameter FIFO1 = 1'b0;
parameter FIFO2 = 1'b1;
parameter GLOSSY_FIRE_GUARD = 9'd511;
// VLC DAC
reg		vlc_dac_start;
wire	vlc_dac_ready;
reg		fifo_rst1, next_fifo_rst1, fifo_rst2, next_fifo_rst2;
reg		[10:0] vlc_play_cnt;
reg		fifo_sel, next_fifo_sel, fifo_play_sel;
reg		d_fifo_we, next_d_fifo_we;
wire	d_fifo1_we = (fifo_sel==FIFO1)? d_fifo_we : 0;
wire	d_fifo2_we = (fifo_sel==FIFO2)? d_fifo_we : 0;
reg		[1:0] vlc_dac_state;
reg		fifo_immed_clr;
reg		fifo_start_immed, next_fifo_start_immed;
reg		delay_cnt_start, next_delay_cnt_start;
reg		[23:0]	delay_cnt, next_delay_cnt;
parameter FS_D8 = 11'd2000;

wire [7:0] dat_length_tt = (dat_length + 8'd12);
wire [14:0] mult_out;
wire [23:0] PACKET_DELAY = (mult_out<<9);

reg		fifo_RE;
assign	fifo_out = (fifo_play_sel==FIFO1)? fifo_out1 : fifo_out2;
wire	fifo_RE1 = (fifo_play_sel==FIFO1)? fifo_RE : 0;
wire	fifo_RE2 = (fifo_play_sel==FIFO2)? fifo_RE : 0;
wire	fifo_empty = (fifo_play_sel==FIFO1)? fifo_empty1 : fifo_empty2;
wire	fifo2_empty = (fifo_sel==FIFO1)? fifo_empty1 : fifo_empty2;

reg		next_glossy_int;
reg		glossy_mask, next_glossy_mask;
reg		[8:0]	mask_cnt, next_mask_cnt;
reg		[2:0]	bit_state;
wire			fifo_out_bit_extract = ((fifo_out & (1'b1<<bit_state))>0)? 1 : 0;
reg		[7:0]	vlc_pwr;
parameter VLC_PWR_HIGH = 8'd204;
parameter VLC_PWR_LOW = 8'd51;
assign test_point_cor = {fifo_sel, fifo_play_sel, glossy_int, fifo_start_immed, fifo_RE, fifo_empty, fifo2_empty, d_fifo_we};

always @ (posedge adc_clk)
begin
    if (~(HRESETn&correlator_en&VLC_PDn))
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
    end
end

always @ (posedge adc_clk)
begin
	if (~(HRESETn&VLC_PDn))
	begin
		fifo_rst1			<= 1;
		fifo_rst2			<= 1;
		d_fifo_we			<= 0;
		fifo_sel			<= FIFO2;
		fifo_start_immed	<= 0;
		delay_cnt_start		<= 0;
		delay_cnt			<= PACKET_DELAY;
		mask_cnt			<= GLOSSY_FIRE_GUARD;
		glossy_int			<= 0;
		glossy_mask			<= 0;
	end
	else
	begin
		fifo_rst1			<= next_fifo_rst1;
		fifo_rst2			<= next_fifo_rst2;
		d_fifo_we			<= next_d_fifo_we;
		fifo_sel			<= next_fifo_sel;
		fifo_start_immed	<= next_fifo_start_immed;
		delay_cnt_start		<= next_delay_cnt_start;
		delay_cnt			<= next_delay_cnt;
		mask_cnt			<= next_mask_cnt;
		glossy_int			<= next_glossy_int;
		glossy_mask			<= next_glossy_mask;
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
	next_d_fifo_we		= 0;
	next_fifo_sel		= fifo_sel;
	next_fifo_start_immed= fifo_start_immed;
	next_delay_cnt_start= delay_cnt_start;
	next_delay_cnt		= delay_cnt;
	next_fifo_rst1		= 1;
	next_fifo_rst2		= 1;
	next_glossy_int		= glossy_int;
	next_glossy_mask	= glossy_mask;
	next_mask_cnt		= mask_cnt;

	if (fifo_immed_clr & fifo_start_immed)
		next_fifo_start_immed = 0;

	if (delay_cnt_start)
	begin
		if (delay_cnt)
			next_delay_cnt = delay_cnt - 1'b1;
		else
		begin
			next_fifo_start_immed = 1;
			next_delay_cnt_start = 0;
			next_glossy_mask = 1;
			next_mask_cnt = GLOSSY_FIRE_GUARD;
		end
	end

	if (glossy_mask)
	begin
		if (mask_cnt)
			next_mask_cnt = mask_cnt - 1'b1;
		else
		begin
			next_glossy_mask = 0;
			next_glossy_int = 0;
		end
	end

	
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

			
			if (FCF_reg[7] & (~glossy_int))
			begin
				if (cur_length==8)
				begin
					if (fifo_sel==fifo_play_sel)
					begin
						next_fifo_sel = ~fifo_sel;
						if (fifo_sel==FIFO1)
							next_fifo_rst2 = 0;
						else
							next_fifo_rst1 = 0;
					end
					else
					begin
						if (fifo_sel==FIFO1)
							next_fifo_rst1 = 0;
						else
							next_fifo_rst2 = 0;
					end
				end
				else if ((cur_length<(dat_length-2))&&(cur_length>8))
				begin
					next_d_fifo_we = 1;
				end
			end

			/*
			if (cur_length==(dat_length-2))
				next_fifo_st_data = rssi;
			*/

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
				//next_fifo_st_data = isCrcCorrect;
				next_st_state = ST_CRC_STORE;
				// crc error, clear fifo
				if (FCF_reg[7])
				begin
					if (isCrcCorrect)
					begin
						if (~glossy_mask)
						begin
							next_delay_cnt_start = 1;
							next_glossy_int = 1;
							next_delay_cnt = PACKET_DELAY;
						end
					end
					// flush fifo
					else if (~glossy_int)
					begin
						if (fifo_sel==FIFO1)
							next_fifo_rst1 = 0;
						else
							next_fifo_rst2 = 0;
					end
				end

				next_crc_correct = isCrcCorrect;
			end
		end

		ST_CRC_STORE:
		begin
			if (crc_correct)
			begin
				if ((DSN>0) && Glossy )
					next_isGlossy = 1;
				else if (ack_req & ack_en & (~isBroadCast) & (~Glossy))
				begin
					next_DSN_sel = 1;
				end
			end
			next_st_state = ST_RESET;
			next_length_int = 0;
			next_packet_done = 1;
			next_cmp_rst = 0;
		end

	endcase
end

always @ (posedge adc_clk)
begin
	if (~(HRESETn&VLC_PDn))
	begin
		vlc_play_cnt <= FS_D8 - 1'b1;
		bit_state <= 7;
		fifo_play_sel <= FIFO1;
		fifo_immed_clr <= 0;
		vlc_dac_start <= 0;
		vlc_dac_state <= 0;
		vlc_pwr <= 0;
		fifo_RE <= 0;
	end
	else
	begin
		case (vlc_dac_state)
			0:
			begin
				if (vlc_dac_start & (~vlc_dac_ready))
					vlc_dac_start <= 0;

				if (fifo_start_immed & vlc_dac_ready)
				begin
					vlc_dac_state <= 1;
					fifo_immed_clr <= 1;
					fifo_play_sel <= fifo_sel;
				end
			end

			1:
			begin
				vlc_play_cnt	<= 1000;
				bit_state		<= 7;
				if (~fifo_empty)
				begin
					if (~fifo_start_immed)
					begin
						fifo_immed_clr <= 0;
						vlc_dac_state <= 2;
					end
				end
				else
					vlc_dac_state <= 0;
			end

			2:
			begin
				if (fifo_start_immed)
				begin
					vlc_dac_state <= 0;
					fifo_RE <= 0;
				end
				else
				begin
					if (fifo_RE)
						fifo_RE <= 0;

					if (vlc_dac_start & (~vlc_dac_ready))
						vlc_dac_start <= 0;

					if (vlc_play_cnt)
					begin
						vlc_play_cnt <= vlc_play_cnt - 1'b1;
						if ((vlc_play_cnt==1000)&&(bit_state==7))
						begin
							if (~fifo_empty)
								fifo_RE <= 1;
							else
								vlc_dac_state <= 0;
						end
					end
					else
					begin
						vlc_play_cnt <= FS_D8 - 1'b1;
						if (bit_state)
							bit_state <= bit_state - 1'b1;
						else
							bit_state <= 7;

						if (vlc_dac_ready)
						begin
							if (fifo_out_bit_extract)
								vlc_pwr <= VLC_PWR_HIGH;
							else
								vlc_pwr <= VLC_PWR_LOW;
							vlc_dac_start <= 1;
						end
					end
				end
			end
		endcase
	end
end

wire	[7:0]	f0counter;
wire	[30:0]	seq0, seq1, seq2, seq3, seq4, seq5, seq6, seq7;
wire	[3:0]	chip_decode_idx;
frame_ctl ctl_0(.FCF(FCF_reg), .address_field(address_field), .src_pan(src_pan), .dest_pan(dest_pan), 
														.dest_addr(dest_addr), .src_addr(src_addr));

crc rx_crc(	.clk(adc_clk), .nreset(HRESETn), .clear(crc_clear), 
			.data_in(chip_decode_byte), .crc_out(crc_result), .start(crc_start), .out_ready(crc_ready));

crc glossy_crc(	.clk(adc_clk), .nreset(HRESETn), .clear(crc_clear), 
			.data_in(glossy_data), .crc_out(glossy_crc_result), .start(glossy_crc_start), .out_ready());

zero_finding f0(.adc_clock(adc_clk), .resetn(HRESETn & cmp_rst), .input_seq(bit_dec), .zero_found(zero_locked), 
				.bit_counter(f0counter), .in_seq0(seq0), .in_seq1(seq1), .in_seq2(seq2), .in_seq3(seq3), 
				.in_seq4(seq4), .in_seq5(seq5), .in_seq6(seq6), .in_seq7(seq7));

sequence_compare cmp0(.adc_clock(adc_clk), .resetn(HRESETn & cmp_rst), .zero_found(zero_locked), 
					.output_idx(chip_decode_idx), .idx_found(chip_locked), .seq_counter(f0counter),
					.seq0_in(seq0), .seq1_in(seq1), .seq2_in(seq2), .seq3_in(seq3), 
					.seq4_in(seq4), .seq5_in(seq5), .seq6_in(seq6), .seq7_in(seq7));

sfd_sync sfd_sync0( .resetn(HRESETn), .clk(adc_clk), .clear(cmp_rst), .input_hb(chip_decode_idx), 
					.zero_locked(zero_locked), .chip_locked(chip_locked), .idx_out(chip_decode_byte), 
					.SFD_LOCKED(sfd_locked), .update(byte_update));

dac_write dac0(	.clk(adc_clk), .resetn(HRESETn), .ch0(vlc_pwr), .ch1(8'd0), .ch2(8'd0), .ch3(8'd0), 
				.start(vlc_dac_start), .pwr(VLC_PDn), .ready(vlc_dac_ready), .clear(1'b0), .num_of_channels(2'd0),
				.CS(VLC_DAC_CS), .WR(VLC_DAC_WR), .DATA(VLC_DAC_DATA), .GAIN(VLC_DAC_GAIN), .LDAC(VLC_DAC_LDAC), .CLR(VLC_DAC_CLR), 
				.PD(VLC_DAC_PD), .A0(VLC_DAC_A0), .A1(VLC_DAC_A1));


/*
r_fifo em_fifo(
    .DATA(fifo_st_data),
    .Q(fifo_out1),
    .WE(d_fifo1_we), 
    .RE(fifo_RE1),
    .WCLOCK(adc_clk),
    .RCLOCK(adc_clk),
    .FULL(),
    .EMPTY(fifo_empty1),
    .RESET(HRESETn & fifo_rst1 & VLC_PDn)
);
*/

fifo128x8x8 em_fifo(
    .DATA(fifo_st_data),
    .Q(fifo_out1),
    .WE(d_fifo1_we), 
    .RE(fifo_RE1),
    .WCLOCK(adc_clk),
    .RCLOCK(adc_clk),
    .FULL(),
    .EMPTY(fifo_empty1),
	.AEMPTY(),
    .RESET(HRESETn & fifo_rst1 & VLC_PDn)
);

fifo128x8x8 vlc_fifo(
    .DATA(fifo_st_data),
    .Q(fifo_out2),
    .WE(d_fifo2_we),
    .RE(fifo_RE2),
	.WCLOCK(adc_clk),
    .RCLOCK(adc_clk),
    .FULL(),
    .EMPTY(fifo_empty2),
	.AEMPTY(),
    .RESET(HRESETn & fifo_rst2 & VLC_PDn)
);

unsigned_mult8x7 umult0(
	.DataA(dat_length_tt),
	.DataB(DSN[6:0]),
	.Mult(mult_out)
);

endmodule   
