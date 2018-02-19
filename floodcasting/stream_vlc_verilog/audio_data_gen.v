
`include "sdr_def.v"

module audio_data_gen(
	input clk,
	input resetn,

	// from radio ctrl
	input audio_mode,
	input [6:0] audio_pkt_length,
	input ready,
	// serial ADC
	input AUDIO_ADC_SDATA,

	// serial ADC
	output AUDIO_ADC_SCLK,
	output AUDIO_ADC_CS,

	// to idx control
	output reg [7:0] data_out,
	output reg data_wr,

	// start radio transmission
	output reg tx_fire,
	output [7:0] testpt
);

wire [7:0] PKT_LEN = audio_pkt_length + 7'd11;
parameter PKT_FCF1 = 8'hc1;	// FCF[7] = 1, this bit is a reserved bit
parameter PKT_FCF2 = 8'h88;
parameter PKT_DSN = 8'd125;
parameter PKT_DEST_PAN1 = 8'h22;
parameter PKT_DEST_PAN2 = 8'h00;
parameter PKT_DEST_ADDR1 = 8'hfe;
parameter PKT_DEST_ADDR2 = 8'hff;
parameter PKT_SRC_ADDR1 = 8'h05;
parameter PKT_SRC_ADDR2 = 8'h00;

reg	[6:0]	pkt_cnt;
reg	[2:0]	fsm;
reg	[3:0]	state;

// adc interface
wire [15:0]	audio_adc_data_out;
wire		audio_adc_dat_valid, audio_adc_ready;
reg			audio_adc_start;

reg	[10:0]	fs_cnt;
reg			wr_en;

// fifo interface
reg			fifo_we, fifo_re;
wire		fifo_empty, fifo_full;
wire [7:0]	fifo_Q;
assign testpt = {ready, data_wr, audio_mode, fifo_empty, fifo_we, fifo_full, fifo_re, tx_fire};

wire audio_rst = (audio_mode==`AUDIO_RECORD)? 1 : 0;

always @ (posedge clk)
begin
	if (~(resetn&audio_rst))
	begin
		state <= 0;
		data_out <= 0;
		data_wr <= 0;
		pkt_cnt <= 0;
		fifo_re <= 0;
		tx_fire <= 0;
		fsm <= 0;
	end
	else
	begin
		case (fsm)
			0:
			begin
				fsm <= 1;
			end

			1:
			begin
				tx_fire <= 0;
				if (ready)
				begin
					fsm <= 2;
					data_wr <= 1;
					case (state)
						0: begin data_out <= PKT_LEN; state <= state + 1; end
						1: begin data_out <= PKT_FCF1; state <= state + 1; end
						2: begin data_out <= PKT_FCF2; state <= state + 1; end
						3: begin data_out <= PKT_DSN; state <= state + 1; end
						4: begin data_out <= PKT_DEST_PAN1; state <= state + 1; end
						5: begin data_out <= PKT_DEST_PAN2; state <= state + 1; end
						6: begin data_out <= PKT_DEST_ADDR1; state <= state + 1; end
						7: begin data_out <= PKT_DEST_ADDR2; state <= state + 1; end
						8: begin data_out <= PKT_SRC_ADDR1; state <= state + 1; end
						9: begin data_out <= PKT_SRC_ADDR2; state <= state + 1; end
						10: begin data_out <= fifo_Q; pkt_cnt <= pkt_cnt + 1; end
					endcase
				end
			end

			2:
			begin
				if (~ready)
				begin
					data_wr <= 0;
					fsm <= 3;
				end
			end

			3:
			begin
				if (state==10)
				begin
					if (pkt_cnt<audio_pkt_length)
					begin
						if (~fifo_empty)
						begin
							fifo_re <= 1;
							fsm <= 4;
						end
					end
					else
					begin
						// loading completed
						fsm <= 7;
						tx_fire <= 1;
					end
				end
				else
					fsm <= 1;
			end

			4:
			begin
				fifo_re <= 0;
				fsm <= 5;
			end

			5:
			begin
				fsm <= 6;
			end

			6:
			begin
				fsm <= 1;
			end

			7:
			begin
				fsm <= 1;
				state <= 0;
				pkt_cnt <= 0;
			end
		endcase
	end
end

always @ (posedge clk)
begin
	if (~(resetn&audio_rst))
	begin
		fifo_we <= 0;
		fs_cnt <= 2000;
		audio_adc_start <= 0;
		wr_en <= 1;
	end
	else
	begin
		if (fs_cnt)
			fs_cnt <= fs_cnt - 1'b1;
		else
		begin
			if (audio_adc_ready)
			begin
				fs_cnt <= 2000;
				audio_adc_start <= 1;
			end
		end

		if (audio_adc_start & (~audio_adc_ready))
			audio_adc_start <= 0;

		if (audio_adc_dat_valid & wr_en & (~fifo_full))
		begin
			fifo_we <= 1;
			wr_en <= 0;
		end

		if (~audio_adc_dat_valid)
			wr_en <= 1;

		if (fifo_we)
			fifo_we <= 0;
	end
end

audio_adc_read audio_adc0(
	.clk(clk),
	.resetn(resetn),
	// serial ADC
	.cs(AUDIO_ADC_CS),
	.sclk(AUDIO_ADC_SCLK),
	.SDATA(AUDIO_ADC_SDATA),
	//
	.data_out(audio_adc_data_out),
	.ready(audio_adc_ready),
	.dat_valid(audio_adc_dat_valid),
	.start_conv(audio_adc_start)
);

fifo128x8x8 audio_fifo(
    .DATA(audio_adc_data_out[15:8]),
    .Q(fifo_Q),
    .WE(fifo_we),
    .RE(fifo_re),
    .CLK(clk),
    .FULL(fifo_full),
    .EMPTY(fifo_empty),
    .RESET(resetn & audio_rst)
);

endmodule
