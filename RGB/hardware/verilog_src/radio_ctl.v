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
					fifo_ready, fifo_ctl_sw_tx,			// from fifo_ctl
					fifo_tx_cpl, ack_gl_state,
					agc_latch, length_int, ack_glossy, // from correlator
					ready_to_transmit, 					// interrupt to processor
					tx_fire,							// from processor
					ADCIO_S1, DACIO_PD, DACIO_DACEN, RFIO_SHDN, RFIO_RXTX,
					test_points,
					LED_DAC_CS, LED_DAC_WR, LED_DAC_GAIN, LED_DAC_LDAC, LED_DAC_CLR, LED_DAC_PD, 
					LED_DAC_A0, LED_DAC_A1, LED_DAC_DATA,
					LED_COUNTER_EN);

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

// IOs for XLamp DAC
output			LED_DAC_CS, LED_DAC_WR, LED_DAC_GAIN, LED_DAC_LDAC, LED_DAC_CLR, LED_DAC_PD, LED_DAC_A0, LED_DAC_A1;
output	[7:0]	LED_DAC_DATA;
input			LED_COUNTER_EN;

reg		[7:0]	Rout, Gout, Bout, Wout;
reg		[7:0]	led_counter, led_counter_buf;
reg				led_counter_start;

wire			dac_ready;
reg				dac_start;

output	[7:0]	test_points;

reg				hwrite_reg, hready_reg;
reg		[7:0]	haddr_reg;
reg		[1:0]	fsm;
reg		[11:0]	HRDATA_reg;

assign HRDATA		= {20'b0, HRDATA_reg};
assign HRESP		= 2'b00;
assign HREADYOUT = hready_reg;

// IOs from processor
input			tx_fire;

// IOs from other blocks
input			agc_latch, length_int;
input			fifo_ready, fifo_ctl_sw_tx; 
input			fifo_tx_cpl;
input			ack_glossy;
output			ready_to_transmit;
output			correlator_en;
output			ack_en;
output	[7:0]	led_out;
output			agc_en;
output			ack_flush, fifo_ctl_tx_fire;
input	[2:0]	ack_gl_state;
reg				length_int_reg, fifo_tx_cpl_reg;

reg				ack_flush_reg, next_ack_flush, ack_flush, fifo_ctl_tx_fire, next_fifo_ctl_tx_fire;
reg				correlator_en, next_correlator_en;
reg				ack_en, radio_off;
reg		[7:0]	led_out;
reg		[1:0]	agc_mode;

wire agc_en = (agc_mode==2'b11)? 1 : (agc_mode==0)? 0 : ~agc_latch;

// chip IO configuration

// mode define
parameter RF_SHUTDOWN = 2'b00;
parameter RF_STANDBY = 2'b01;
parameter RF_RX = 2'b10;
parameter RF_TX = 2'b11;

parameter ADC_STANDBY = 0;
parameter ADC_ON = 1;

parameter DAC_STANDBY = 2'b00;
parameter DAC_ON = 2'b01;
parameter DAC_SHUTDOWN = 2'b10;

parameter HCLK_FREQ = 48;		// 48MHz
parameter RXTX_TURN_TIME = 2*(HCLK_FREQ);		// HCLK is 48MHz, MAX2831 requires 2us to turnaround, 2/(1/48) = 96 ticks
parameter RF_WARMUP_TIME = 1000*(HCLK_FREQ);	// 1000 us for crystal stablize
parameter AUTO_TX_TIME = 190*(HCLK_FREQ);

parameter WAIT_FOR_TX_COMPLETE =			4'b1100;
parameter RX_TX_TURNAROUND =				4'b0100;
parameter TX_RX_TURNAROUND =				4'b0011;
parameter RX_IDLE =							4'b0000;
parameter RX_COLLECT =						4'b0001;
parameter RX_WAIT_FOR_ACK_GLOSSY =			4'b0101;
parameter RX_WAIT_FOR_ACK_GLOSSY_COMPLETE =	4'b1101;
parameter RADIO_OFF =						4'b1000;
parameter RADIO_WARMUP =					4'b1001;
// end of define

reg		[1:0]	RF_MODE, DAC_MODE, next_RF_MODE, next_DAC_MODE;
reg				ADC_MODE, next_ADC_MODE;
reg		[3:0]	radio_mode, next_radio_mode;
reg		[15:0]	turn_cnt, next_turn_cnt;

reg		RFIO_SHDN, RFIO_RXTX, DACIO_PD, DACIO_DACEN;

wire ADCIO_S1 = (ADC_MODE==ADC_ON)? 1 : 0;

assign test_points = {radio_mode, fifo_ctl_sw_tx, fifo_tx_cpl_reg, fifo_ready, ack_glossy};

reg		[13:0]	auto_tx_cnt, next_auto_tx_cnt;
reg				auto_tx_reg, next_auto_tx_reg;
reg		[1:0]	auto_tx_state, next_auto_tx_state;
reg				auto_tx_en;
reg     [7:0]   irqn;

always @ (posedge HCLK)
begin
	if (~HRESETn | ~auto_tx_en)
	begin
		auto_tx_cnt <= AUTO_TX_TIME;
		auto_tx_state <= 0;
		auto_tx_reg <= 0;
	end
	else
	begin
		auto_tx_cnt <= next_auto_tx_cnt;
		auto_tx_state <= next_auto_tx_state;
		auto_tx_reg <= next_auto_tx_reg;
	end
end

always @ *
begin
	next_auto_tx_cnt = auto_tx_cnt;
	next_auto_tx_state = auto_tx_state;
	next_auto_tx_reg = auto_tx_reg;
	case(auto_tx_state)
		2'b00:
		begin
			next_auto_tx_cnt = AUTO_TX_TIME;
			next_auto_tx_reg = 0;
			if ((radio_mode==RX_COLLECT)&&(length_int_reg==0))
				next_auto_tx_state = 2'b01;
		end

		2'b01:
		begin
			next_auto_tx_cnt = auto_tx_cnt - 1;
			if (auto_tx_cnt==0)
				next_auto_tx_state = 2'b10;
		end

		2'b10:
		begin
			if (radio_mode==RX_IDLE)
			begin
				next_auto_tx_reg = 1;
				next_auto_tx_state = 2'b11;
			end
			else
				next_auto_tx_state = 2'b00;
		end

		2'b11:
		begin
			if (radio_mode!=RX_IDLE)
				next_auto_tx_state = 2'b00;
		end
	endcase
end

always @ *
begin
	case (RF_MODE)
		RF_SHUTDOWN:
		begin
			RFIO_SHDN = 0;
			RFIO_RXTX = 0;
		end

		RF_STANDBY:
		begin
			RFIO_SHDN = 0;
			RFIO_RXTX = 1;
		end

		RF_RX:
		begin
			RFIO_SHDN = 1;
			RFIO_RXTX = 0;
		end

		RF_TX:
		begin
			RFIO_SHDN = 1;
			RFIO_RXTX = 1;
		end
	endcase
end

always @ *
begin
	case (DAC_MODE)
		DAC_STANDBY:
		begin
			DACIO_PD = 0;
			DACIO_DACEN = 0;
		end

		DAC_ON:
		begin
			DACIO_PD = 0;
			DACIO_DACEN = 1;
		end

		DAC_SHUTDOWN:
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


always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		RF_MODE <= RF_SHUTDOWN;
		ADC_MODE <= ADC_STANDBY;
		DAC_MODE <= DAC_SHUTDOWN;
		radio_mode <= RADIO_OFF;
		turn_cnt <= RXTX_TURN_TIME;
		fifo_ctl_tx_fire <= 0;
		correlator_en <= 0;
		length_int_reg <= 0;
		fifo_tx_cpl_reg <= 0;
		ack_flush <= 0;
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
		length_int_reg <= length_int;
		fifo_tx_cpl_reg <= fifo_tx_cpl;
		ack_flush <= next_ack_flush;
	end
end

wire ready_to_transmit = (radio_mode==RX_IDLE)? 1 : 0;

always @ *
begin
	next_RF_MODE = RF_MODE;
	next_ADC_MODE = ADC_MODE;
	next_DAC_MODE = DAC_MODE;
	next_radio_mode = radio_mode;
	next_turn_cnt = turn_cnt;
	next_fifo_ctl_tx_fire = 0;
	next_correlator_en = correlator_en;
	next_ack_flush = 0;

	case (radio_mode)
		WAIT_FOR_TX_COMPLETE:
		begin
			next_turn_cnt = RXTX_TURN_TIME;
			if (fifo_tx_cpl_reg)
				next_radio_mode = TX_RX_TURNAROUND;
		end

		RX_TX_TURNAROUND:
		begin
			next_turn_cnt = turn_cnt - 1;
			next_correlator_en = 0;
			next_RF_MODE = RF_TX;
			next_ADC_MODE = ADC_STANDBY;
			//next_DAC_MODE = DAC_ON;
			if (turn_cnt==0)
			begin
				if (fifo_ready)
				begin
					next_fifo_ctl_tx_fire = 1;
					next_radio_mode = WAIT_FOR_TX_COMPLETE;
				end
				else
					next_radio_mode = RX_IDLE;
			end
		end

		TX_RX_TURNAROUND:
		begin
			next_turn_cnt = turn_cnt - 1;
			next_RF_MODE = RF_RX;
			next_ADC_MODE = ADC_ON;
			//next_DAC_MODE = DAC_STANDBY;
			if (turn_cnt==0)
				next_radio_mode = RX_IDLE;
		end

		RX_IDLE:
		begin
			next_turn_cnt = RXTX_TURN_TIME;
			next_correlator_en = 1;
			next_RF_MODE = RF_RX;
			next_ADC_MODE = ADC_ON;
			//next_DAC_MODE = DAC_STANDBY;
			next_DAC_MODE = DAC_ON;
			if (tx_fire | auto_tx_reg)
				next_radio_mode = RX_TX_TURNAROUND;
			else if (length_int_reg)	// start listen only if legnth field is correct
				next_radio_mode = RX_COLLECT;
			else if (radio_off)
				next_radio_mode = RADIO_OFF;
		end

		RX_COLLECT:
		begin
			next_DAC_MODE = DAC_SHUTDOWN;
			if (length_int_reg==0)
			begin
				if (ack_glossy)
					next_radio_mode = RX_WAIT_FOR_ACK_GLOSSY;
				else
					next_radio_mode = RX_IDLE;
			end
		end

		RX_WAIT_FOR_ACK_GLOSSY:
		begin
			next_DAC_MODE = DAC_ON;
			next_correlator_en = 0;
			if (ack_flush_reg)
			begin
				next_radio_mode = RX_IDLE;
				next_ack_flush = 1;
			end
			else if (fifo_ctl_sw_tx)	// 2us turnaround is taken care in fifo ctl
				next_radio_mode = RX_WAIT_FOR_ACK_GLOSSY_COMPLETE;
		end

		RX_WAIT_FOR_ACK_GLOSSY_COMPLETE:
		begin
			next_turn_cnt = RXTX_TURN_TIME;
			next_RF_MODE = RF_TX;
			next_ADC_MODE = ADC_STANDBY;
			//next_DAC_MODE = DAC_ON;
			if (fifo_tx_cpl_reg)
				next_radio_mode = TX_RX_TURNAROUND;
		end

		RADIO_OFF:
		begin
			next_RF_MODE = RF_SHUTDOWN;
			next_ADC_MODE = ADC_STANDBY;
			next_DAC_MODE = DAC_SHUTDOWN;
			next_turn_cnt = RF_WARMUP_TIME;
			next_correlator_en = 0;
			if (~radio_off)
				next_radio_mode = RADIO_WARMUP;
		end

		RADIO_WARMUP:
		begin
			next_RF_MODE = RF_RX;
			//next_DAC_MODE = DAC_STANDBY;	// DAC requires 50us to turn on
			next_DAC_MODE = DAC_ON;
			next_ADC_MODE = ADC_ON;
			next_turn_cnt = turn_cnt - 1;
			if (turn_cnt==RXTX_TURN_TIME)
				next_radio_mode = TX_RX_TURNAROUND;
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
		ack_flush_reg	<= 0;
		radio_off 	<= 1;
		auto_tx_en<= 0;
        irqn        <= 0;
		dac_start	<= 0;
		Rout <= 0;
		Gout <= 0;
		Bout <= 0;
		Wout <= 0;
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
                            4'h1:
                            begin
                                irqn <= HWDATA[7:0];
                            end

							4'h2:
							begin
								led_out <= (~HWDATA[7:0]);
							end

							4'h5:
							begin
								ack_flush_reg <= HWDATA[0];
							end

							4'h6:
							begin
								Rout <= HWDATA[7:0];
								Gout <= HWDATA[15:8];
								Bout <= HWDATA[23:16];
								Wout <= HWDATA[31:24];
							end

							4'h0:
							begin
								radio_off <= HWDATA[0];
								agc_mode <= HWDATA[2:1];
								ack_en <= HWDATA[3];
								auto_tx_en <= HWDATA[8];
							end
						endcase
					end
					
					0:	// read
					begin
						case (haddr_reg[7:4])
							4'h0:	// 0000
							begin
								HRDATA_reg <= {ack_gl_state, auto_tx_en, radio_mode, ack_en, agc_mode, radio_off};
							end

                            4'h1:
                            begin
                                HRDATA_reg <= {4'b0, irqn};
                            end

							4'h2:
							begin
								HRDATA_reg <=  {4'b0, ~led_out};
							end

							4'h7:
							begin
								HRDATA_reg <= {4'b0, led_counter_buf};
							end

						endcase
					end
				endcase
			end

			2'b10:
			begin
				if ((haddr_reg[7:4]==4'h6)&&(hwrite_reg))
				begin
					if (dac_ready)
					begin
						dac_start <= 1;
						fsm <= 2'b11;
					end
				end
				else
				begin
					ack_flush_reg <= 0;
					fsm <= 2'b11;
				end
			end

			2'b11:
			begin
				dac_start <= 0;
				hready_reg <= 1;
				fsm		<= 2'b00;
			end
		endcase
	end
end

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		led_counter <= 0;
		led_counter_buf <= 0;
		led_counter_start <= 0;
	end
	else
	begin
		if (LED_COUNTER_EN)
		begin
			led_counter <= led_counter + 1;
			led_counter_start <= 1;
		end
		else
		begin
			if (led_counter_start)
				led_counter_buf <= led_counter;
			led_counter_start <= 0;
			led_counter <= 0;
		end
	end
end


dac_write dac0(	.clk(HCLK), .resetn(HRESETn), .ch0(Rout), .ch1(Gout), .ch2(Bout), .ch3(Wout), .start(dac_start), .pwr(1'b1), 
				.ready(dac_ready), .clear(1'b0), .num_of_channels(2'd3), .CS(LED_DAC_CS), .WR(LED_DAC_WR), .DATA(LED_DAC_DATA), 
				.GAIN(LED_DAC_GAIN), .LDAC(LED_DAC_LDAC), .CLR(LED_DAC_CLR), .PD(LED_DAC_PD), .A0(LED_DAC_A0), .A1(LED_DAC_A1));

endmodule
