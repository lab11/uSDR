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

// todo: 

module radio_ctl( 	HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA, ack_en, led_out, agc_en, correlator_en,
					ack_flush, fifo_ctl_tx_fire,		// to fifo_ctl
					ack_ready, glossy_ready, fifo_ctl_sw_tx,			// from fifo_ctl
					fifo_ctl_glossy_ack_complete, fifo_ctl_tx_complete,	// from fifo_ctl
					agc_latch, length_int, packet_done,	// from correlator
					ready_to_transmit, 					// interrupt to processor
					tx_fire,							// from processor
					ADCIO_S1, DACIO_PD, DACIO_DACEN, RFIO_SHDN, RFIO_RXTX);

// IOs for descrete chips
output			ADCIO_S1;
output			DACIO_PD;
output			DACIO_DACEN;
output			RFIO_SHDN;
output			RFIO_RXTX;

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
reg		[7:0]	haddr_reg;
reg		[1:0]	fsm;
reg		[7:0]	HRDATA_reg;

assign HRDATA		= {24'b0, HRDATA_reg};
assign HRESP		= 2'b00;
assign HREADYOUT = hready_reg;

// IOs from processor
input			tx_fire;

// IOs from other blocks
input			agc_latch, length_int, packet_done;
input			ack_ready, glossy_ready, fifo_ctl_sw_tx; 
input			fifo_ctl_glossy_ack_complete, fifo_ctl_tx_complete;
output			ready_to_transmit;
output			correlator_en;
output			ack_en;
output	[7:0]	led_out;
output			agc_en;
output			ack_flush, fifo_ctl_tx_fire;

reg				ack_flush, fifo_ctl_tx_fire, next_fifo_ctl_tx_fire;
reg				correlator_en, next_correlator_en;
reg				ack_en, radio_off;
reg		[7:0]	led_out;
reg		[1:0]	agc_mode;

wire agc_en = (agc_mode==2'b11)? 1 : (agc_mode==0)? 0 : ~agc_latch;

// chip IO configuration

// mode define
`define RF_SHUTDOWN		2'b00
`define RF_STANDBY		2'b01
`define RF_RX			2'b10
`define RF_TX			2'b11

`define ADC_STANDBY		0
`define ADC_ON			1

`define DAC_STANDBY		2'b00
`define DAC_ON			2'b01
`define DAC_SHUTDOWN	2'b10

`define HCLK_FREQ		48		// 48MHz
`define RXTX_TURN_TIME	2*(`HCLK_FREQ)		// HCLK is 48MHz, MAX2831 requires 2us to turnaround, 2/(1/48) = 96 ticks
`define RF_WARMUP_TIME	60*(`HCLK_FREQ)		// 60us for warmup
// end of define

reg		[1:0]	RF_MODE, DAC_MODE, next_RF_MODE, next_DAC_MODE;
reg				ADC_MODE, next_ADC_MODE;
reg		[3:0]	radio_mode, next_radio_mode;
reg		[11:0]	turn_cnt, next_turn_cnt;

reg		RFIO_SHDN, RFIO_RXTX, DACIO_PD, DACIO_DACEN;

wire ADCIO_S1 = (ADC_MODE==`ADC_ON)? 1 : 0;

always @ *
begin
	case (RF_MODE)
		`RF_SHUTDOWN:
		begin
			RFIO_SHDN = 0;
			RFIO_RXTX = 0;
		end

		`RF_STANDBY:
		begin
			RFIO_SHDN = 0;
			RFIO_RXTX = 1;
		end

		`RF_RX:
		begin
			RFIO_SHDN = 1;
			RFIO_RXTX = 0;
		end

		`RF_TX:
		begin
			RFIO_SHDN = 1;
			RFIO_RXTX = 1;
		end
	endcase
end

always @ *
begin
	case (DAC_MODE)
		`DAC_STANDBY:
		begin
			DACIO_PD = 0;
			DACIO_DACEN = 0;
		end

		`DAC_ON:
		begin
			DACIO_PD = 0;
			DACIO_DACEN = 1;
		end

		`DAC_SHUTDOWN:
		begin
			DACIO_PD = 1;
			DACIO_DACEN = 0;
		end

		default:
		begin
			DACIO_PD = 1;
			DACIO_DACEN = 0;
		end

	endcase
end

`define	WAIT_FOR_TX_COMPLETE			4'b1000
`define	RX_TX_TURNAROUND				4'b0100
`define	TX_RX_TURNAROUND				4'b0011
`define	RX_IDLE							4'b0000
`define	RX_LISTEN						4'b0001
`define	RX_WAIT_FOR_ACK_GLOSSY			4'b0010
`define	RX_WAIT_FOR_ACK_GLOSSY_COMPLETE	4'b1001
`define	RADIO_OFF						4'b0101
`define	RADIO_WARMUP					4'b0110

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		RF_MODE <= `RF_RX;
		ADC_MODE <= `ADC_ON;
		DAC_MODE <= `DAC_ON;
		radio_mode <= `RX_IDLE;
		turn_cnt <= 0;
		fifo_ctl_tx_fire <= 0;
		correlator_en <= 1;
	end
	else
	begin
		RF_MODE <= next_RF_MODE;
		ADC_MODE <= next_ADC_MODE;
		DAC_MODE <= next_DAC_MODE;
		radio_mode <= next_radio_mode;
		turn_cnt <= next_turn_cnt;
		fifo_ctl_tx_fire <= next_fifo_ctl_tx_fire;
		correlator_en <= next_correlator_en;
	end
end

wire ready_to_transmit = (radio_mode==`RX_IDLE)? 1 : 0;

always @ *
begin
	next_RF_MODE = RF_MODE;
	next_ADC_MODE = ADC_MODE;
	next_DAC_MODE = DAC_MODE;
	next_radio_mode = radio_mode;
	next_turn_cnt = turn_cnt;
	next_fifo_ctl_tx_fire = 0;
	next_correlator_en = correlator_en;

	case (radio_mode)
		`WAIT_FOR_TX_COMPLETE:
		begin
			if (fifo_ctl_tx_complete)
			begin
				next_radio_mode = `TX_RX_TURNAROUND;
			end
			next_turn_cnt = `RXTX_TURN_TIME;
		end

		`RX_TX_TURNAROUND:
		begin
			next_turn_cnt = turn_cnt - 1;
			next_correlator_en = 0;
			if (turn_cnt==0)
			begin
				next_fifo_ctl_tx_fire = 1;
				next_radio_mode = `WAIT_FOR_TX_COMPLETE;
			end
		end

		`TX_RX_TURNAROUND:
		begin
			next_turn_cnt = turn_cnt - 1;
			next_RF_MODE = `RF_RX;
			next_ADC_MODE = `ADC_ON;
			next_correlator_en = 1;
			if (turn_cnt==0)
			begin
				next_radio_mode = `RX_IDLE;
			end
		end

		`RX_IDLE:
		begin
			if (tx_fire)
			begin
				next_RF_MODE = `RF_TX;
				next_ADC_MODE = `ADC_STANDBY;
				next_radio_mode = `RX_TX_TURNAROUND;
				next_turn_cnt = `RXTX_TURN_TIME;
			end
			else if (length_int)	// start listen only if legnth field is correct
			begin
				next_radio_mode = `RX_LISTEN;
			end
			else if (radio_off)
			begin
				next_radio_mode = `RADIO_OFF;
				next_RF_MODE = `RF_SHUTDOWN;
				next_ADC_MODE = `ADC_STANDBY;
				next_DAC_MODE = `DAC_SHUTDOWN;
				next_correlator_en = 0;
			end
		end

		`RX_LISTEN:
		begin
			if (packet_done)
			begin
				next_radio_mode = `RX_WAIT_FOR_ACK_GLOSSY;
				next_correlator_en = 0;
			end
		end

		`RX_WAIT_FOR_ACK_GLOSSY:
		// this state should be able to transit immediately,
		begin
			if (ack_ready & glossy_ready)	// both ready means no ack/glossy need to be sent
			begin
				next_radio_mode = `RX_IDLE;
				next_correlator_en = 1;
			end
			else if (fifo_ctl_sw_tx)
			begin
				next_radio_mode = `RX_WAIT_FOR_ACK_GLOSSY_COMPLETE;
				next_RF_MODE = `RF_TX;
				next_ADC_MODE = `ADC_STANDBY;
			end
		end

		`RX_WAIT_FOR_ACK_GLOSSY_COMPLETE:
		begin
			if (fifo_ctl_glossy_ack_complete)
			begin
				next_radio_mode = `TX_RX_TURNAROUND;
			end
		end

		`RADIO_OFF:
		begin
			if (~radio_off)
			begin
				next_radio_mode = `RADIO_WARMUP;
				next_RF_MODE = `RF_RX;
				next_DAC_MODE = `DAC_ON;	// DAC requires 50us to turn on
				next_turn_cnt = `RF_WARMUP_TIME;
			end
		end

		`RADIO_WARMUP:
		begin
			next_turn_cnt = turn_cnt - 1;
			if (turn_cnt==`RXTX_TURN_TIME)
			begin
				next_radio_mode = `TX_RX_TURNAROUND;
			end
		end

	endcase
end


always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		fsm	    <= 2'b00;
		hwrite_reg  <= 1'b0;
		hready_reg	<= 1;
		HRDATA_reg	<= 0;
		haddr_reg	<= 0;
		ack_en		<= 1;
		led_out		<= 8'hff;
		agc_mode	<= 0;
		ack_flush	<= 0;
		radio_off 	<= 0;
	end
	else
	begin
		case (fsm)
			2'b00:
			begin
				if (HSEL && HREADY && HTRANS[1] && HSIZE==3'b010 )
				begin
					hwrite_reg	<= HWRITE;
					fsm			<= 2'b01;
					hready_reg	<= 0;
					haddr_reg	<= HADDR[7:0];
				end
			end

			2'b01:
			begin
				fsm	<= 2'b10;
				case (hwrite_reg)
					1:
					begin
						case (haddr_reg[7:4])
							4'h2:
							begin
								led_out <= (~HWDATA[7:0]);
							end

							4'h5:
							begin
								ack_flush <= HWDATA[0];
							end

							4'h0:
							begin
								radio_off <= HWDATA[0];
								agc_mode <= HWDATA[2:1];
								ack_en <= HWDATA[3];
							end
						endcase
					end
					
					0:	// read
					begin
						case (haddr_reg[7:4])
							4'h0:	// 0000
							begin
								HRDATA_reg <= {radio_mode, ack_en, agc_mode, radio_off};
							end

							4'h2:
							begin
								HRDATA_reg <=  ~led_out;
							end

						endcase
					end
				endcase
			end

			2'b10:
			begin
				ack_flush <= 0;
				fsm <= 2'b11;
			end

			2'b11:
			begin
				hready_reg <= 1;
				fsm		<= 2'b00;
			end
		endcase
	end
end

endmodule
