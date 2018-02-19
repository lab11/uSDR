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

module correlator( 	HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA,
					adc_clk, test_point, sfd_int, length_int, bit_dec,
					ack_en, ack_ready, DSN, DSN_sel, mode, agc_latch,
					fifo_data, fifo_WE, packet_done, isGlossy );

input	    	adc_clk;
input			bit_dec;

// Inputs for radio control
input			mode, ack_en;

output			sfd_int, length_int, agc_latch;

// IOs for AHB bus
input 			HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input 	[31:0] 	HADDR, HWDATA;
input 	[2:0] 	HSIZE, HBURST;
input 	[3:0] 	HPROT;
input 	[1:0] 	HTRANS;

output 			HREADYOUT;
output 	[1:0] 	HRESP;
output 	[31:0] 	HRDATA;

reg				hwrite_reg, hready_reg;
reg		[1:0]	fsm;

assign HRESP		= 2'b00;
reg		[31:0]	hrdata_reg;
reg		[7:0]	haddr_reg;
assign HRDATA		= hrdata_reg;

// ACK IOs
input			ack_ready;
output	[7:0]	DSN;
output			DSN_sel;
reg				DSN_sel, next_DSN_sel;

// frame format
reg		[15:0]	FCF_reg, next_FCF_reg;
reg		[7:0]	DSN_reg, next_DSN_reg;
reg		[159:0]	address_field, next_address_field;
assign DSN = DSN_reg;

//wire			mode;
`define	MODE_TX		1
`define	MODE_RX		0

// frame control
wire	[15:0]	dest_pan, src_pan;
wire	[63:0]	dest_addr, src_addr;
wire	[2:0]	frame_type = FCF_reg[2:0];
wire			security_enable = FCF_reg[3];
wire			frame_pending = FCF_reg[4];
wire			ack_req	= FCF_reg[5];
wire			broadcast2byte, broadcast8byte, glossy2byte;
reg				length_int, next_length_int;
cmp_16bit_const  cmp16_const_0 (.DataA(dest_addr[15:0]), .AEB(broadcast2byte));
cmp_48bit_const	 cmp48_const_0 (.DataA(dest_addr[63:16]), .AEB(broadcast8byte));
cmp_glossy_const glossy_const_0 (.DataA(dest_addr[15:0]), .AEB(glossy2byte));
wire	isBroadCast = (FCF_reg[11:10]==2'b10)? broadcast2byte :
					  (FCF_reg[11:10]==2'b11)? (broadcast2byte&broadcast8byte) : 0;

//glossy
output	[7:0]	fifo_data;
output			fifo_WE;
output			packet_done;
output			isGlossy;
reg				packet_done, next_packet_done;
reg				isGlossy, next_isGlossy;
reg				glossy_crc_start, next_glossy_crc_start;
wire	[15:0]	glossy_crc_result;
wire	Glossy = 	(FCF_reg[11:10]==2'b10)? glossy2byte :
					(FCF_reg[11:10]==2'b11)? (glossy2byte&broadcast8byte) : 0;

`define GLOSSY_HOP_THRESHOLD 8'd127

`define	TYPE_BEACON 3'b000
`define TYPE_DATA	3'b001
`define TYPE_ACK	3'b010
`define TYPE_MACCMD	3'b011

// fifo IOs
reg				fifo_RE, fifo_WE, next_fifo_WE;
wire	[7:0]	fifo_out;

// store
reg		[7:0]	st_data, next_st_data;
reg		[7:0]	dat_length, next_dat_length, cur_length, next_cur_length;
reg		[3:0]	st_state, next_st_state;
reg				sfd_hb, len_hb, dat_hb, next_sfd_hb, next_len_hb, next_dat_hb;

wire	[7:0]	glossy_data = (cur_length==3)? (st_data + 1) : st_data;	
wire	[7:0]	fifo_data = (cur_length==(dat_length-1))? glossy_crc_result[7:0] : (cur_length==dat_length)? glossy_crc_result[15:8] : glossy_data;

// crc
wire			crc_ready;
reg				crc_start, next_crc_start, crc_clear, next_crc_clear, crc_correct, next_crc_correct;
reg		[1:0]	crc_cnt, next_crc_cnt;
wire	[15:0]	crc_result;

`define	ST_SYN_STATE		4'b0000
`define ST_SYNC_INTER_STATE 4'b1000
`define ST_SFD_STATE0		4'b1001
`define ST_SFD_STATE1		4'b1010
`define ST_SFD_INTER_STATE	4'b1011
`define ST_LEN_STATE		4'b1100
`define ST_LEN_INTER_STATE	4'b1101
`define ST_DAT_STATE		4'b1110
`define ST_DAT_INTER_STATE	4'b1111
`define ST_CRC_STATE		4'b0001
`define ST_WAIT_ZERO_STATE	4'b0010
`define ST_RESET			4'b0011



//assign HREADYOUT = ~fifo_is_empty & hready_reg;
assign HREADYOUT = hready_reg;

// main controller
reg				sfd_interrupt, next_sfd_interrupt, agc_latch, next_agc_latch;


wire			zero_locked, chip_locked;
reg				cmp_rst, next_cmp_rst;
reg				ack_set, next_ack_set;
wire	[3:0]	chip_decode_idx;
wire	[7:0]	byte_received = {chip_decode_idx, st_data[7:4]};

// testpoint
output	[15:0]	test_point;
assign	test_point[7:4] = (chip_decode_idx & {4{st_state[3]}});
assign	test_point[3:0] = {sfd_int, length_int, packet_done, crc_correct};

//assign	test_point[3:0] = {sfd_int, length_int, addr_int, crc_int};
//assign	test_point[3:0] = {out_stream, bit_dec_test, zero_locked, chip_locked};
//assign	test_point[15:8] = I_smp;
assign	test_point[15:8] = 8'b0;
//assign	test_point[7:0] = Q_smp;

assign	sfd_int = sfd_interrupt;

always @ (posedge HCLK)
begin
    if (~HRESETn)
    begin
		fsm	    <= 2'b00;
		hwrite_reg  <= 1'b0;
		hready_reg	<= 1;
		fifo_RE		<= 1'b0;
		hrdata_reg	<= 0;
		haddr_reg	<= 0;
    end
    else
    begin
		case (fsm)
		    2'b00:
		    begin
				if (HSEL && HREADY && HTRANS[1] && HSIZE==3'b010 )
				begin
					hwrite_reg	<= HWRITE;
					fsm		<= 2'b01;
					hready_reg <= 0;
					haddr_reg	<= HADDR[7:0];
				end
				fifo_RE		<= 1'b0;
		    end
		    
		    2'b01:
		    begin
				fsm	<= 2'b10;
				if (~hwrite_reg)
				begin
					if (haddr_reg==0)	// read fifo 
						fifo_RE		<= 1'b1;
				end
		    end

			2'b10:
			begin
				fifo_RE <= 0;
				fsm <= 2'b11;
			end

			2'b11:
			begin
				case (haddr_reg[4:2])
					// data: 0x40050000
					0: begin hrdata_reg <= {24'b0, fifo_out}; end

					// pan ID src/dest: 0x40050004	
					1: begin hrdata_reg <= {src_pan, dest_pan}; end

					// crc / address mode: 0x4005000c
					3: begin hrdata_reg <= {14'b0, ack_set, crc_correct, FCF_reg}; end

					// source address bit 31:0: 0x40050010
					4: begin hrdata_reg <= src_addr[31:0]; end

					// source address bit 63:32: 0x40050014
					5: begin hrdata_reg <= src_addr[63:32]; end

					// dest address bit 15:0: 0x40050018
					6: begin hrdata_reg <= dest_addr[15:0]; end

					// dest address bit 31:16: 0x4005001c
					7: begin hrdata_reg <= dest_addr[31:16]; end
				endcase
				hready_reg <= 1;
				fsm		<= 2'b00;
			end

		endcase
    end
end

always @ (posedge adc_clk)
begin
    if (~HRESETn)
    begin
		fifo_WE				<= 0;
		st_state			<= `ST_RESET;
		st_data				<= 0;
		sfd_hb				<= 0;
		len_hb				<= 0;
		dat_hb				<= 0;
		dat_length			<= 0;
		sfd_interrupt		<= 0;
		cur_length			<= 0;
		crc_start			<= 0;
		crc_clear			<= 0;
		crc_cnt				<= 0;
		crc_correct			<= 0;
		DSN_reg				<= 0;
		DSN_sel				<= 0;
		FCF_reg				<= 0;
		address_field		<= 0;
		cmp_rst				<= 1;
		length_int			<= 0;
		agc_latch			<= 0;
		packet_done			<= 0;
		isGlossy			<= 0;
		glossy_crc_start	<= 0;
		ack_set				<= 0;
    end
    else
    begin
		fifo_WE				<= next_fifo_WE;
		st_state			<= next_st_state;
		st_data				<= next_st_data;
		sfd_hb				<= next_sfd_hb;
		len_hb				<= next_len_hb;
		dat_hb				<= next_dat_hb;
		dat_length			<= next_dat_length;
		sfd_interrupt		<= next_sfd_interrupt;
		cur_length			<= next_cur_length;
		crc_start			<= next_crc_start;
		crc_clear			<= next_crc_clear;
		crc_cnt				<= next_crc_cnt;
		crc_correct			<= next_crc_correct;
		DSN_reg				<= next_DSN_reg;
		DSN_sel				<= next_DSN_sel;
		FCF_reg				<= next_FCF_reg;
		address_field		<= next_address_field;
		cmp_rst				<= next_cmp_rst;
		length_int			<= next_length_int;
		agc_latch			<= next_agc_latch;
		packet_done			<= next_packet_done;
		isGlossy			<= next_isGlossy;
		glossy_crc_start	<= next_glossy_crc_start;
		ack_set				<= next_ack_set;
    end
end

always @ (*)
begin
	
	next_st_data		= st_data;
	next_st_state		= st_state;
	next_sfd_hb			= sfd_hb;
	next_len_hb			= len_hb;
	next_dat_hb			= dat_hb;
	next_dat_length		= dat_length;
	next_cur_length		= cur_length;
	next_sfd_interrupt	= sfd_interrupt;
	// FIFO control
	next_fifo_WE		= 0;
	next_crc_start		= crc_start;
	next_crc_clear		= crc_clear;
	next_crc_cnt		= crc_cnt;
	next_crc_correct	= crc_correct;
	next_DSN_reg		= DSN_reg;
	next_DSN_sel		= 0;
	next_FCF_reg		= FCF_reg;
	next_address_field	= address_field;
	next_cmp_rst		= 1;
	next_length_int		= length_int;
	next_agc_latch		= agc_latch;
	next_packet_done	= packet_done;
	next_isGlossy		= isGlossy;
	next_glossy_crc_start = glossy_crc_start;
	next_ack_set		= ack_set;

	
	case (st_state)
		`ST_RESET:
		begin
			next_packet_done = 0;
			next_st_data = 0;
			next_length_int = 0;
			next_DSN_reg = 0;
			next_DSN_sel = 0;
			next_agc_latch = 0;
			if (mode==`MODE_RX)
				next_st_state = `ST_WAIT_ZERO_STATE;
		end

		`ST_WAIT_ZERO_STATE:
		begin
			if (zero_locked)
				next_st_state = `ST_SYN_STATE;
		end

		// wait for 0x00
		`ST_SYN_STATE:
		begin
			if (chip_locked)
			begin
				next_st_data = byte_received;
				next_st_state = `ST_SYNC_INTER_STATE;
			end
		end

		`ST_SYNC_INTER_STATE:
		begin
			if (chip_locked==0)
			begin
				if (st_data==0)
					next_st_state = `ST_SFD_STATE0;
				else
					next_st_state = `ST_SYN_STATE;
			end
		end

		// wait for 0x70
		`ST_SFD_STATE0:
		begin
			if (chip_locked)
			begin
				next_st_data = byte_received;
				next_st_state = `ST_SFD_INTER_STATE;
				next_sfd_hb = 0;
			end
		end

		// wait for 0xa7
		`ST_SFD_STATE1:
		begin
			if (chip_locked)
			begin
				next_st_data = byte_received;
				next_st_state = `ST_SFD_INTER_STATE;
				next_sfd_hb = 1;
			end
		end

		`ST_SFD_INTER_STATE:
		begin
			if (chip_locked==0)
			begin
				case (sfd_hb)
					0:
					begin
						if (st_data==0)
							next_st_state = `ST_SFD_STATE0;
						else 
							next_st_state = `ST_SFD_STATE1;
					end
					1:
					begin
						if (st_data==8'ha7)
						begin
							next_st_state = `ST_LEN_STATE;
							next_sfd_interrupt = 1;
							next_agc_latch = 1;
						end
						else
						begin // invalid SFD
							next_st_state = `ST_RESET;
							next_cmp_rst = 0;
						end
					end
				endcase
			end
			next_len_hb = 0;
		end

		`ST_LEN_STATE:
		begin
			next_crc_clear = 1;
			next_sfd_interrupt = 0;
			next_ack_set = 0;
			next_FCF_reg = 0;
			next_address_field = 0;
			next_crc_correct = 0;
			next_isGlossy = 0;
			if (chip_locked)
			begin
				next_st_data = byte_received;
				next_st_state = `ST_LEN_INTER_STATE;
				next_dat_length = byte_received;
			end
		end

		`ST_LEN_INTER_STATE:
		begin
			next_cur_length = 0;
			next_dat_hb = 0;
			next_crc_clear = 0;
			if (chip_locked==0)
			begin
				next_len_hb = ~len_hb;
				if (~len_hb)
				begin
					next_st_state = `ST_LEN_STATE;
				end
				else
				begin
					if ((dat_length[7]) || (dat_length<5))	// invalid data length
					begin
						next_st_state = `ST_RESET;
						next_cmp_rst = 0;
					end
					else
					begin
						next_st_state = `ST_DAT_STATE;
						next_fifo_WE = 1;
						next_length_int = 1;
					end
				end
			end
		end

		`ST_DAT_STATE:
		begin
			if (chip_locked)
			begin
				next_st_data = byte_received;
				next_st_state = `ST_DAT_INTER_STATE;
				if (dat_hb==1)
				begin
					next_cur_length = cur_length + 1;
					next_fifo_WE = 1;
					next_crc_start = 1;

					if (cur_length < (dat_length-2))
						next_glossy_crc_start = 1;
					else
						next_glossy_crc_start = 0;

					case (cur_length)
						0: begin next_FCF_reg[7:0] = byte_received; end
						1: begin next_FCF_reg[15:8] = byte_received; end
						2: begin next_DSN_reg = byte_received; end
						3: begin next_address_field[159:152] = byte_received; end
						4: begin next_address_field[151:144] = byte_received; end
						5: begin next_address_field[143:136] = byte_received; end
						6: begin next_address_field[135:128] = byte_received; end
						7: begin next_address_field[127:120] = byte_received; end
						8: begin next_address_field[119:112] = byte_received; end
						9: begin next_address_field[111:104] = byte_received; end
						10: begin next_address_field[103:96] = byte_received; end
						11: begin next_address_field[95:88] = byte_received; end
						12: begin next_address_field[87:80] = byte_received; end
						13: begin next_address_field[79:72] = byte_received; end
						14: begin next_address_field[71:64] = byte_received; end
						15: begin next_address_field[63:56] = byte_received; end
						16: begin next_address_field[55:48] = byte_received; end
						17: begin next_address_field[47:40] = byte_received; end
						18: begin next_address_field[39:32] = byte_received; end
						19: begin next_address_field[31:24] = byte_received; end
						20: begin next_address_field[23:16] = byte_received; end
						21: begin next_address_field[15:8] = byte_received; end
						22: begin next_address_field[7:0] = byte_received; end
					endcase
				end
			end
		end

		`ST_DAT_INTER_STATE:
		begin
			next_crc_start = 0;
			next_glossy_crc_start = 0;
			next_crc_cnt = 0;
			if (cur_length == dat_length)
				next_st_state = `ST_CRC_STATE;
			else 
			begin
				if (chip_locked==0)
				begin
					next_dat_hb = ~dat_hb;
					next_st_state = `ST_DAT_STATE;
				end
			end
		end

		`ST_CRC_STATE:
		begin
			if (~(crc_cnt[1] & crc_cnt[0]))
				next_crc_cnt = crc_cnt + 1;
			else if (crc_ready)
			begin
				if (crc_result==0)
				begin
					next_crc_correct = 1;
					if ((DSN_reg < `GLOSSY_HOP_THRESHOLD) && (Glossy==1))
					begin
						next_isGlossy = 1;
					end
					else if (ack_ready & ack_req & ack_en & ~isBroadCast & ~Glossy)
					begin
						next_DSN_sel = 1;
						next_ack_set = 1;
					end
				end
				next_packet_done = 1;
				next_st_state = `ST_RESET;
				next_cmp_rst = 0;
			end
		end

	endcase
end

wire	[7:0]	f0counter;
wire	[30:0]	seq0, seq1, seq2, seq3, seq4, seq5, seq6, seq7;
frame_ctl ctl_0(.FCF(FCF_reg), .address_field(address_field), .src_pan(src_pan), .dest_pan(dest_pan), 
														.dest_addr(dest_addr), .src_addr(src_addr));

crc rx_crc(	.clk(adc_clk), .nreset(HRESETn), .clear(crc_clear), 
			.data_in(st_data), .crc_out(crc_result), .start(crc_start), .out_ready(crc_ready));

crc glossy_crc(	.clk(adc_clk), .nreset(HRESETn), .clear(crc_clear), 
			.data_in(glossy_data), .crc_out(glossy_crc_result), .start(glossy_crc_start), .out_ready());

zero_finding f0(.adc_clock(adc_clk), .resetn(HRESETn & cmp_rst), .input_seq(bit_dec), .zero_found(zero_locked), 
				.bit_counter(f0counter), .in_seq0(seq0), .in_seq1(seq1), .in_seq2(seq2), .in_seq3(seq3), 
				.in_seq4(seq4), .in_seq5(seq5), .in_seq6(seq6), .in_seq7(seq7));

sequence_compare cmp0(.adc_clock(adc_clk), .resetn(HRESETn & cmp_rst), .zero_found(zero_locked), 
					.output_idx(chip_decode_idx), .idx_found(chip_locked), .seq_counter(f0counter),
					.seq0_in(seq0), .seq1_in(seq1), .seq2_in(seq2), .seq3_in(seq3), 
					.seq4_in(seq4), .seq5_in(seq5), .seq6_in(seq6), .seq7_in(seq7));

r_fifo em_fifo(
    .DATA(st_data),
    .Q(fifo_out),
    .WE(fifo_WE), 
    .RE(fifo_RE),
    .WCLOCK(adc_clk),
    .RCLOCK(HCLK),
    .FULL(),
    .EMPTY(),
    .RESET(HRESETn)
);

endmodule   
