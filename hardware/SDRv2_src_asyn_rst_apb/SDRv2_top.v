
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
	output			ADC_CLK_BUF,

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
	
	// APB_INTERFACE
	input			PCLK,
	input			PRESETn, 
	input			[31:0] PADDR, 
	input			PSEL0, PSEL1, PSEL2, PSEL3, 
	input			PENABLE,
	input			PWRITE, 
	input			[31:0] PWDATA, 
	output			PREADY0, PREADY1, PREADY2, PREADY3,
	output			[31:0] PRDATA0, PRDATA1, PRDATA2, PRDATA3,
	output			PSLVERR

);
wire	DAC_CLK_BUF;
assign PSLVERR = 1'b0;
assign ADC_CLK_BUF = (ADC_CLK & ADC_S1);
assign DAC_CLK = ((~DAC_PD)&DAC_DACEN)? DAC_CLK_BUF : 1'b0;

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
// radio_ctl -> agc
wire			RadiotoAgc_agc_en;
// radio_ctl -> fifo_ctl
wire			RadiotoFIFO_ack_flush;
wire			RadiotoFIFO_tx_fire;
wire			RadiotoFIFO_fifo_rst;


fx_demux fm0(	// Inputs
				// from fabric IOs
				.adc_clock(ADC_CLK), 
				.resetn(PRESETn),
				.in_phase(in_phase),
				.quad_phase(quad_phase),

				// Outputs
				// to correlator
				.bit_decode(FMtoCorr_bit_decode),
				// to agc
				.signal_strength(FMtoAgc_signal_strength)
				);


radio_ctl rc0(	.PCLK(PCLK), .PRESETn(PRESETn), .PADDR(PADDR), .PSELx(PSEL2), 
				.PENABLE(PENABLE), .PWRITE(PWRITE), .PWDATA(PWDATA), .PREADY(PREADY2), 
				.PRDATA(PRDATA2), .PSLVERR(),

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
				.tx_fire(tx_fire),

				// Outputs
				// to fabric IOs
				.led_out(LED),
				.ADCIO_S1(ADC_S1),
				.DACIO_PD(DAC_PD),
				.DACIO_DACEN(DAC_DACEN), 
				.RFIO_SHDN(RF_SHDN),
				.RFIO_RXTX(RF_RXTX),
				.test_points(TEST_PTS[15:8]),
				// to correlator
				.correlator_en(RadiotoCorr_correlator_en),
				.ack_en(RadiotoCorr_ack_en), 
				// to agc
				.agc_en(RadiotoAgc_agc_en),
				// to fifo_ctl
				.ack_flush(RadiotoFIFO_ack_flush),
				.fifo_ctl_tx_fire(RadiotoFIFO_tx_fire),
				.fifo_rst(RadiotoFIFO_fifo_rst),
				// to processor
				.ready_to_transmit(ready_to_transmit)
				);

wire	[7:0]	CorrtoFIFO_DSN;
wire			CorrtoFIFO_DSN_sel;
wire	[7:0]	CorrtoFIFO_rx_data;
wire			CorrtoFIFO_rx_fifo_we;
wire			CorrtoFIFO_isGlossy;
wire	[7:0]	AgctoCorr_rssi;

correlator cr0( .PCLK(PCLK), .PRESETn(PRESETn), .PADDR(PADDR), .PSELx(PSEL0), 
				.PENABLE(PENABLE), .PWRITE(PWRITE), .PWDATA(PWDATA), .PREADY(PREADY0), 
				.PRDATA(PRDATA0), .PSLVERR(),
				.adc_clk(ADC_CLK), 
				// Inputs
				// from radio_ctl
				.correlator_en(RadiotoCorr_correlator_en),
				.ack_en(RadiotoCorr_ack_en), 
				.fifo_rst(RadiotoFIFO_fifo_rst),
				// from fm_demux
				.bit_dec(FMtoCorr_bit_decode),
				// from agc
				.RSSI_IN(AgctoCorr_rssi),

				// Outputs
				// to fabric IOs
				.test_point_cor(TEST_PTS[7:0]),
				// to processor
				.sfd_int(sfd_int),
				.packet_done(packet_done),
				.length_int(length_int), // to radio_ctl, fifo_ctl as well
				// to fifo_ctl
				.DSN(CorrtoFIFO_DSN),
				.DSN_sel(CorrtoFIFO_DSN_sel), 
				.fifo_data(CorrtoFIFO_rx_data),
				.fifo_WE(CorrtoFIFO_rx_fifo_we), 
				.isGlossy(CorrtoFIFO_isGlossy),
				// to radio_ctl
				.isAckGlossy(CorrtoRadio_ack_glossy)
				);

wire			IdxtoFIFO_tfWE;
wire	[7:0]	IdxtoFIFO_tfdat;
wire			FIFOtoIdx_tffull;
wire			FIFOtoIdx_tfempty;
wire	[1:0]	FIFOtoIdx_owner;

fifo_ctl fc0(	// Inputs
				// from fabric IOs
				.clk(PCLK),
				.resetn(PRESETn),
				// from radio_ctl
				.ack_flush(RadiotoFIFO_ack_flush),
				.tx_fire(RadiotoFIFO_tx_fire),
				.fifo_rst(RadiotoFIFO_fifo_rst),
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
				.half_clk_out(DAC_CLK_BUF), 
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

idx_control ic0(.PCLK(PCLK), .PRESETn(PRESETn), .PADDR(PADDR), .PSELx(PSEL1), 
				.PENABLE(PENABLE), .PWRITE(PWRITE), .PWDATA(PWDATA), .PREADY(PREADY1), 
				.PRDATA(PRDATA1), .PSLVERR(),

				// Inputs
				// from fifo_ctl
				.tx_fifo_full(FIFOtoIdx_tffull),
				.tx_fifo_empty(FIFOtoIdx_tfempty),
				.current_owner(FIFOtoIdx_owner), 

				// Outputs
				// to fifo_ctl
				.tx_fifo_dout(IdxtoFIFO_tfdat),
				.tx_fifo_we(IdxtoFIFO_tfWE)
				);


agc agc0(		// Inputs
				// from fabric IOs
				.adc_clock(ADC_CLK),
				.RESETn(PRESETn),
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
				.RSSI_OUT(AgctoCorr_rssi)
				);

max2831_spi ms0(.PCLK(PCLK), .PRESETn(PRESETn), .PADDR(PADDR), .PSELx(PSEL3), 
				.PENABLE(PENABLE), .PWRITE(PWRITE), .PWDATA(PWDATA), .PREADY(PREADY3), 
				.PRDATA(PRDATA3), .PSLVERR(),
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

endmodule
