
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
 * Last modify: 2/21 '2013
 * Content: add define file
 * Note: 
 */


module sdrv2_top(
	input	[7:0]	in_phase,
	input	[7:0]	quad_phase,
	input			RSSI_DIN,
	input			ADC_CLK,

	input			tx_fire,

	output			RSSI_CLK,
	output			RSSI_CS,

	output			ADC_S1,

	output			DAC_DACEN,
	output			DAC_PD,
	output			DAC_CLK,
	output	[7:0]	DAC_DOUT,

	output	[7:0]	LED,

	output			RF_RXTX,
	output			RF_SHDN,
	output			RF_DOUT,
	output			RF_CS,
	output			RF_CLK,
	output	[6:0]	RF_GAIN,

	output	[15:0]	TEST_PTS,

	// output to processor
	output			length_int,
	output			packet_done,
	output			ready_to_transmit,
	output			sfd_int,
	output			tx_complete,
	output			glossy_int,
	output			VLC_FIFO_AEMPTY,
	output			VLC_FIFO_EMPTY,

	// output to Audio DAC
	output			VLC_DAC_CS,
	output			VLC_DAC_WR,
	output	[7:0]	VLC_DAC_DATA,
	output			VLC_DAC_GAIN,
	output			VLC_DAC_LDAC, 
	output			VLC_DAC_CLR,
	output			VLC_DAC_PD, 
	output			VLC_DAC_A0, 
	output			VLC_DAC_A1,
	
	// AHB_INTERFACE
	input			HRESETn, HCLK,
	input 			HSEL2, HSEL3,
	input			HWRITE2, HWRITE3,
	input			HMASTLOCK2, HMASTLOCK3,
	input			HREADY2, HREADY3,
	input 	[31:0] 	HADDR2, HADDR3,
	input	[31:0]	HWDATA2, HWDATA3,
	input 	[2:0] 	HSIZE2, HSIZE3,
	input	[2:0]	HBURST2, HBURST3,
	input 	[3:0] 	HPROT2, HPROT3,
	input 	[1:0] 	HTRANS2, HTRANS3,
	
	output 			HREADYOUT2, HREADYOUT3,
	output 	[1:0] 	HRESP2, HRESP3,
	output 	[31:0] 	HRDATA2, HRDATA3
);

// fm_demux -> correlator
wire			FMtoCorr_bit_decode;
wire	[7:0]	FMtoAgc_signal_strength;

// fifo_ctl -> radio_ctl 
wire			FIFOtoRadio_fifo_ready;
wire			FIFOtoRadio_sw_tx;
wire			FIFOtoRadio_tx_cpl;
wire	[2:0]	FIFOtoRadio_ack_gl_state;

// correlator -> radio_ctl
wire			CorrtoRadio_ack_glossy;

// to processor

// radio_ctl -> correlator
wire			RadiotoCorr_correlator_en;
wire			RadiotoCorr_ack_en;
wire			RadiotoCorrAudio_audio_mode;
// radio_ctl -> agc
wire			RadiotoAgc_agc_en;
// radio_ctl -> fifo_ctl
wire			RadiotoFIFO_ack_flush;
wire			RadiotoFIFO_tx_fire;

wire	IdxtoVlc_ready;
wire	[7:0] VlctoIdx_data_out;
wire	VlctoIdx_data_wr;
wire	VlctoRadio_tx_fire;
wire	[6:0] RadiotoVlc_audio_pkt_length;
wire	[7:0] RadiotoVlc_glossy_dsn;
wire	RadiotoVlc_fifo_we;
wire	[7:0] RadiotoVlc_fifo_dout;
wire	RadiotoCorr_PDn;

fx_demux fm0(	// Inputs
				// from fabric IOs
				.adc_clock(ADC_CLK), 
				.resetn(HRESETn),
				.in_phase(in_phase),
				.quad_phase(quad_phase),

				// Outputs
				// to correlator
				.bit_decode(FMtoCorr_bit_decode),
				// to agc
				.signal_strength(FMtoAgc_signal_strength)
				);


radio_ctl rc0(	.HSEL(HSEL2), .HADDR(HADDR2), .HWRITE(HWRITE2), .HSIZE(HSIZE2), .HBURST(HBURST2), 
				.HPROT(HPROT2), .HTRANS(HTRANS2), .HMASTLOCK(HMASTLOCK2), .HREADY(HREADY2), 
				.HWDATA(HWDATA2), .HRESETn(HRESETn), .HCLK(HCLK),
				.HREADYOUT(HREADYOUT2), .HRESP(HRESP2), .HRDATA(HRDATA2), 
				// Inputs
				// from fifo_ctl
				.fifo_ready(FIFOtoRadio_fifo_ready),
				.fifo_ctl_sw_tx(FIFOtoRadio_sw_tx),
				.fifo_tx_cpl(FIFOtoRadio_tx_cpl), 
				.ack_gl_state(FIFOtoRadio_ack_gl_state),
				// from correlator
				.agc_latch(sfd_int), 
				.length_int(length_int),
				.ack_glossy(CorrtoRadio_ack_glossy),
				// from processor
				.tx_fire(VlctoRadio_tx_fire),

				// Outputs
				// to fabric IOs
				.led_out(LED),
				.ADCIO_S1(ADC_S1),
				.DACIO_PD(DAC_PD),
				.DACIO_DACEN(DAC_DACEN), 
				.RFIO_SHDN(RF_SHDN),
				.RFIO_RXTX(RF_RXTX),
				.test_points(),
				// to correlator
				.correlator_en(RadiotoCorr_correlator_en),
				.ack_en(RadiotoCorr_ack_en), 
				.audio_mode(RadiotoCorrAudio_audio_mode),
				// to agc
				.agc_en(RadiotoAgc_agc_en),
				// to fifo_ctl
				.ack_flush(RadiotoFIFO_ack_flush),
				.fifo_ctl_tx_fire(RadiotoFIFO_tx_fire),
				// to processor
				.ready_to_transmit(ready_to_transmit),
				// to audio data generation
				.audio_pkt_length(RadiotoVlc_audio_pkt_length),
				.glossy_dsn(RadiotoVlc_glossy_dsn),
				.vlc_fifo_we(RadiotoVlc_fifo_we),
				.vlc_fifo_dout(RadiotoVlc_fifo_dout),
				.VLC_PDn(RadiotoCorr_PDn)
				);

wire	[7:0]	CorrtoFIFO_DSN;
wire			CorrtoFIFO_DSN_sel;
wire	[7:0]	CorrtoFIFO_rx_data;
wire			CorrtoFIFO_rx_fifo_we;
wire			CorrtoFIFO_isGlossy;

correlator cr0( .HRESETn(HRESETn),
				.adc_clk(ADC_CLK), 
				// Inputs
				// from radio_ctl
				.correlator_en(RadiotoCorr_correlator_en),
				.ack_en(RadiotoCorr_ack_en), 
				.VLC_PDn(RadiotoCorr_PDn),
				// from fm_demux
				.bit_dec(FMtoCorr_bit_decode),
				// from agc
				.RSSI_IN(),

				// Outputs
				// to fabric IOs
				.test_point_cor(TEST_PTS[7:0]),
				// to processor
				.sfd_int(sfd_int),
				.packet_done(packet_done),
				.length_int(length_int), // to radio_ctl, fifo_ctl as well
				.glossy_int(glossy_int),
				// to fifo_ctl
				.DSN(CorrtoFIFO_DSN),
				.DSN_sel(CorrtoFIFO_DSN_sel), 
				.fifo_data(CorrtoFIFO_rx_data),
				.fifo_WE(CorrtoFIFO_rx_fifo_we), 
				.isGlossy(CorrtoFIFO_isGlossy),
				// to radio_ctl
				.isAckGlossy(CorrtoRadio_ack_glossy),
				// Outputs to VLC DAC 
				.VLC_DAC_CS(VLC_DAC_CS),
				.VLC_DAC_WR(VLC_DAC_WR), 
				.VLC_DAC_DATA(VLC_DAC_DATA),
				.VLC_DAC_GAIN(VLC_DAC_GAIN), 
				.VLC_DAC_LDAC(VLC_DAC_LDAC), 
				.VLC_DAC_CLR(VLC_DAC_CLR),
				.VLC_DAC_PD(VLC_DAC_PD), 
				.VLC_DAC_A0(VLC_DAC_A0), 
				.VLC_DAC_A1(VLC_DAC_A1)
				);

wire			IdxtoFIFO_tfWE;
wire	[7:0]	IdxtoFIFO_tfdat;
wire			FIFOtoIdx_tffull;
wire			FIFOtoIdx_tfempty;
wire	[1:0]	FIFOtoIdx_owner;

fifo_ctl fc0(	// Inputs
				// from fabric IOs
				.clk(HCLK),
				.nreset(HRESETn),
				// from radio_ctl
				.ack_flush(RadiotoFIFO_ack_flush),
				.tx_fire(RadiotoFIFO_tx_fire),
				.audio_mode(RadiotoCorrAudio_audio_mode),
				// from idx_ctl
				.tfWE(IdxtoFIFO_tfWE),
				.tfdat(IdxtoFIFO_tfdat),
				// from correlator
				.DSN(CorrtoFIFO_DSN),
				.DSN_sel(CorrtoFIFO_DSN_sel),
				.rx_data(CorrtoFIFO_rx_data),
				.rx_WE(CorrtoFIFO_rx_fifo_we),
				.isGlossy(CorrtoFIFO_isGlossy),
				.length_int(length_int),

				// Outputs
				// to fabric IOs
				.data_out(DAC_DOUT),
				.half_clk_out(DAC_CLK), 
				.fifo_ctl_tp(),
				// to processor
				.tx_complete(tx_complete), 
				// to idx_ctl
				.tffull(FIFOtoIdx_tffull),
				.tfempty(FIFOtoIdx_tfempty),
				.owner(FIFOtoIdx_owner),
				// to radio_ctl
				.fifo_ready(FIFOtoRadio_fifo_ready),
				.sw_tx(FIFOtoRadio_sw_tx),
				.fifo_tx_cpl(FIFOtoRadio_tx_cpl),
				.ack_gl_state(FIFOtoRadio_ack_gl_state)
				);

idx_control ic0(.HRESETn(HRESETn), .HCLK(HCLK),
				.data_in(VlctoIdx_data_out),
				.data_wr(VlctoIdx_data_wr),
				.ready(IdxtoVlc_ready),
				// Inputs
				// from fifo_ctl
				.tx_fifo_full(FIFOtoIdx_tffull),
				.tx_fifo_empty(FIFOtoIdx_tfempty),
				.current_owner(FIFOtoIdx_owner), 
				// from radio ctrl
				.audio_mode(RadiotoCorrAudio_audio_mode),

				// Outputs
				// to fifo_ctl
				.tx_fifo_dout(IdxtoFIFO_tfdat),
				.tx_fifo_we(IdxtoFIFO_tfWE)
				);


agc agc0(		// Inputs
				// from fabric IOs
				.adc_clock(ADC_CLK),
				.HRESETn(HRESETn),
				.sdata(RSSI_DIN),
				// from max2831_spi
				// from radio_ctl
				.rx_agc_en(RadiotoAgc_agc_en),
				// from fm_demux
				.input_pwr(FMtoAgc_signal_strength),

				// Outputs
				// to fabric IOs
				.sclk(RSSI_CLK),
				.cs(RSSI_CS),
				.B(RF_GAIN),
				.test_points(),
				// to correlator
				.RSSI_OUT()
				);

max2831_spi ms0(.HSEL(HSEL3), .HADDR(HADDR3), .HWRITE(HWRITE3), .HSIZE(HSIZE3), .HBURST(HBURST3), 
				.HPROT(HPROT3), .HTRANS(HTRANS3), .HMASTLOCK(HMASTLOCK3), .HREADY(HREADY3), 
				.HWDATA(HWDATA3), .HRESETn(HRESETn), .HCLK(HCLK),
				.HREADYOUT(HREADYOUT3), .HRESP(HRESP3), .HRDATA(HRDATA3),
				
				// Inputs
				// from agc
				.rx_agc_sel(1'b0),
				.rx_gain(7'b0),

				// Outputs
				// to fabric IOs
				.DATOUT(RF_DOUT),
				.CLKOUT(RF_CLK),
				.ENOUT(RF_CS),
				// to agc
				.rx_gain_rdy(),
				.rx_agc_data_got()
				);

vlc_dat_gen vdg0(
				.clk(ADC_CLK),
				.resetn(HRESETn),
				.HCLK(HCLK),

				// from radio ctrl
				.audio_mode(RadiotoCorrAudio_audio_mode),
				.audio_pkt_length(RadiotoVlc_audio_pkt_length),
				.glossy_dsn(RadiotoVlc_glossy_dsn),
				.vlc_fifo_we(RadiotoVlc_fifo_we),
				.vlc_fifo_din(RadiotoVlc_fifo_dout),


				// to idx control
				.ready(IdxtoVlc_ready),
				.data_out(VlctoIdx_data_out),
				.data_wr(VlctoIdx_data_wr),

				// to processor
				.fifo_aempty(VLC_FIFO_AEMPTY),
				.fifo_empty(VLC_FIFO_EMPTY),

				// start radio transmission
				.tx_fire(VlctoRadio_tx_fire),
				.testpt(TEST_PTS[15:8])
);

endmodule
