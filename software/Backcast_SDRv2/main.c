
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

#include <stdio.h>
#include <stdlib.h>

/**************************************************************************/
/* Firmware Includes */
/**************************************************************************/
#include "CMSIS/a2fxxxm3.h"
#include "drivers/mss_pdma/mss_pdma.h"
#include "drivers/mss_gpio/mss_gpio.h"
#include "drivers/mss_uart/mss_uart.h"
#include "drivers/mss_watchdog/mss_watchdog.h"
#include "drivers/mss_timer/mss_timer.h"

#include "maxim2831.h"
#include "tx_packet.h"
#include "radio_config.h"

#define GDB

volatile uint8_t PDMA_int, tx_complete_int;
volatile uint8_t TIM1_int, TIM2_int;
volatile uint8_t SW1_int, SW2_int, SW3_int, SW4_int;
volatile uint8_t rx_pkt_done_int, SFD_int, length_int;


/**************************************************************************/
/* Function Prototype*/
/**************************************************************************/

void Fabric_IRQHandler();		// Fab int handler
void pdma_tx_handler();		// pdma int handler
inline void set_led(uint32_t);
inline void gotolisten(uint32_t);
#ifdef TX
inline void config_next_wakeup(uint32_t);
#else
inline void initialize_probe(struct tx_packet_str* tp, uint32_t size, uint32_t* dptr);
inline void goto1stprobe(struct tx_packet_str*);
inline void goto2ndprobe(struct tx_packet_str*);
inline void gotoprobe_CW(struct tx_packet_str* tp);
inline uint8_t probe_timer_adj(uint8_t pts);
#endif
/**************************************************************************/
// pointers
/**************************************************************************/

uint32_t* const LED_ptr		= (uint32_t*)0x40070020;

/**************************************************************************/
/*Define*/
/**************************************************************************/
#define CLK_FREQ 				48		// 48MHz
#define TX_LISTEN_DUR			2000	// 2000us
#define TX_LISTEN_INIT_DUR		100000 	// 100ms
#define RXTX_TURN_DUR			192
#define GUARD_TIMER				CLK_FREQ*1500	// 1000us
#define	RX_LISTEN_DUR			(RXTX_TURN_DUR + (32*6) + 750)	// 192 turnaround, 4 preamble, 1 sfd, 1 length, 2 guard
#define RX_LISTEN_TIMER			CLK_FREQ*RX_LISTEN_DUR
#define TX_LISTEN_TIMER 		CLK_FREQ*TX_LISTEN_DUR
#define TX_LISTEN_INIT_TIMER 	CLK_FREQ*TX_LISTEN_INIT_DUR
#define RXTX_TURN_TIMER			CLK_FREQ*RXTX_TURN_DUR
#define ONE_SEC					CLK_FREQ*1000000
#define TWO_SEC					CLK_FREQ*2000000
#define INIT_PROBE				ONE_SEC*0.3
#define LENG_INT_LANTENCY		CLK_FREQ*(209)		// start tx to length int goes high takes 209 us,
#define CW_LENGTH				CLK_FREQ*10000		// 10 ms
#define RF_ON_LATENCY			CLK_FREQ*1500		// 1.5 ms

/**************************************************************************/
/*Main Program*/
/**************************************************************************/


int main()
{
	// configure timer
	MSS_TIM2_init(MSS_TIMER_ONE_SHOT_MODE);
	MSS_TIM2_enable_irq();
	#ifdef TX
	MSS_TIM1_init(MSS_TIMER_ONE_SHOT_MODE);
	#else
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
	#endif
	MSS_TIM1_enable_irq();

	MSS_TIM2_load_immediate(TWO_SEC);
	MSS_TIM2_start();

	// turn on RF, wait for DAC to power-on
	RF_shdn(0);
	uint32_t i;
	for (i=0;i<(uint32_t)(0.003*ONE_SEC);i++)
		__asm("nop;");
	init_system();
	

	// interrupt variables
	rx_pkt_done_int = 0;
	PDMA_int = 0;
	SFD_int = 0;
	length_int = 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	TIM1_int = 0;
	tx_complete_int= 0;

	// RF variables
	uint8_t res;
	#ifdef TX
	uint8_t timer1_set;
	uint8_t pkt_cnt_sync;
	uint8_t delay_tx;
	uint8_t pending_dat_number = 1;
	#else
	uint8_t probe_timer_shift = 1;
	uint8_t probe_cw;
	#endif
	
	// set RF gain
	res = setTXBaseBandVGAGain(30);
	res = setRXBaseBandVGAGain(10);
	res = setLNAGain(2);			// medium rx lna gain
	uint32_t frequency = 2405;
	res = setFreqDivider(frequency);
	

	rx_agc_en(3);

	struct radio* radio0;
	radio0 = malloc(sizeof(struct radio));
	(*radio0).multiple_addr_en = 0;		// disable multiple address
	(*radio0).radio_state = startup;

	struct rx_packet_str* received_packet;
	received_packet = malloc(sizeof(struct rx_packet_str));

	#ifdef TX
	struct tx_packet_str* led_pkt;
	led_pkt = malloc(sizeof(struct tx_packet_str));
	(*led_pkt).rp = radio0;				// link to radio0

	uint32_t* led_data_ptr;
	uint32_t data_size = 4;
	led_data_ptr = malloc(sizeof(uint32_t)*data_size);
	(*led_pkt).data_length = data_size;
	(*led_pkt).data_ptr = led_data_ptr;

	// set led packet
	set_frame_type(led_pkt, 1);		// data
	set_security(led_pkt, 0);		// disable security
	set_ack(led_pkt, 0);				// enable ack
	set_pan_id_comp(led_pkt, 1);		// enable intra-pan ID
	set_dest_addr_mode(led_pkt, 2);	// 2 bytes of dest address
	set_DSN(led_pkt, 0);				// DSN = 0
	set_dest_pan(led_pkt, 0x22);		// pan addr = 0x0022
	set_dest_addr(led_pkt, 0, 0xffff);

	#else
	struct tx_packet_str* probe1_pkt;
	probe1_pkt = malloc(sizeof(struct tx_packet_str));
	(*probe1_pkt).rp = radio0;				// link to radio0

	struct tx_packet_str* probe2_pkt;
	probe2_pkt = malloc(sizeof(struct tx_packet_str));
	(*probe2_pkt).rp = radio0;				// link to radio0

	struct tx_packet_str* probe_CW_pkt;
	probe_CW_pkt = malloc(sizeof(struct tx_packet_str));
	(*probe_CW_pkt).rp = radio0;				// link to radio0

	uint32_t* probe1_data_ptr;
	uint32_t probe1_size = 6;
	probe1_data_ptr = malloc(sizeof(uint32_t)*probe1_size);

	uint32_t* probe2_data_ptr;
	uint32_t probe2_size = 2;
	probe2_data_ptr = malloc(sizeof(uint32_t)*probe2_size);

	uint32_t* probe_CW_data_ptr;
	uint32_t probe_CW_size = 6;
	probe_CW_data_ptr = malloc(sizeof(uint32_t)*probe2_size);
	
	initialize_probe(probe1_pkt, probe1_size, probe1_data_ptr);
	initialize_probe(probe2_pkt, probe2_size, probe2_data_ptr);
	initialize_probe(probe_CW_pkt, probe_CW_size, probe_CW_data_ptr);
	#endif


	// set radio
	set_src_pan(radio0, 0x22);
	set_src_addr_mode(radio0, 2);	// 2 bytes of src address
	#ifdef TX
	set_src_addr(radio0, 0, 0x02);
	#else
	set_src_addr(radio0, 0, 0x05);
	#endif

	// receiver data array
	uint32_t rdata[128];

	MSS_GPIO_enable_irq (MSS_GPIO_16);		// enable length interrupt
	srand((MSS_TIM2_get_current_value() ^ 0xffffffff));
	MSS_TIM2_stop();

	while (1){
	#ifdef TX
		switch ((*radio0).radio_state){
			case startup:
				(*radio0).radio_state = off;
				(*radio0).sync = 0;
				(*radio0).tx_counter = 1;
				(*radio0).pending_dest_addr = 0x5;
				(*radio0).received_timer = TWO_SEC;
				RF_shdn(1);				// turn off RF
				MSS_TIM1_load_immediate((*radio0).received_timer);
				MSS_TIM1_start();
				auto_tx_en(0);
				pending_dat_number = 1;
				#ifndef GDB
				__ASM volatile ("WFI");
				#endif
			break;

			case off:
				if (TIM1_int){			// periodic wake up
					TIM1_int = 0;
					RF_shdn(0);			// turn on RF
					MSS_TIM1_stop();
					(*radio0).pending_data_nu = pending_dat_number;
					pkt_cnt_sync = 0;
					timer1_set = 0;
					delay_tx = 0;
					if (!(*radio0).sync)			// TXRX not sync yet, set to longer listen period
						gotolisten(ONE_SEC);
					else
						gotolisten(TX_LISTEN_TIMER);
					(*radio0).radio_state = listen;
				}
			break;

			case listen:
				if (length_int){
					length_int = 0;
					MSS_TIM2_load_immediate(ONE_SEC);
					(*radio0).radio_state = receive;
					auto_tx_en(1);	// enable auto fire
				}

				if (TIM2_int){
					TIM2_int = 0;
					if (!timer1_set){
						MSS_TIM1_load_immediate((*radio0).received_timer);
						MSS_TIM1_start();
					}
					RF_shdn(1);
					(*radio0).sync = 0;
					(*radio0).radio_state = off;
					#ifndef GDB
					__ASM volatile ("WFI");
					#endif
				}
			break;

			case receive:
				if (rx_pkt_done_int){
					rx_pkt_done_int = 0;
					uint8_t empty;
					do{
						read_fifo(rdata);
						empty = isFifoEmpty();
					}
					while(!empty);

					res = rx_packet_create(received_packet, rdata);

					uint8_t crc_correct, ack_req;
					crc_correct = (*received_packet).crc;
					ack_req = (*received_packet).ack_req;

					uint8_t gosleep = 1;

					if (crc_correct){
						uint32_t* rx_data = rdata + (*received_packet).payload_idx;

						uint8_t address_match = dest_addr_filter(received_packet, radio0);

						// address doesn't match but requests ACK
						// flush ack
						if ( (!address_match) & ack_req){
							ack_flush();
						}

						// broadcast 
						if (address_match==3){
							// probe packet signature, 0x11, 0x22
							if ((rx_data[0]==0x11)&&(rx_data[1]==0x22)){
								// first probe
								if ((*received_packet).packet_length>15){
									(*radio0).received_timer = (rx_data[2]<<24) | (rx_data[3]<<16) | (rx_data[4]<<8) | rx_data[5];
									config_next_wakeup((*radio0).received_timer);
									timer1_set = 1;
								}
								// src addr filtering
								// need to working on here
								if (((*radio0).pending_dest_addr==(*received_packet).src_addr_LSB) && (*radio0).pending_data_nu){

									// new probe coming, old packet has been received by receiver
									if ((pkt_cnt_sync)&&((*received_packet).dsn == ((*radio0).tx_counter + 1)))
										(*radio0).pending_data_nu -= 1;

									if ((*radio0).pending_data_nu){
										(*radio0).radio_state = transmit;
										set_led((*received_packet).dsn);
										(*radio0).tx_counter = (*received_packet).dsn;
										
										// load the data into TX fifo
										set_dest_addr(led_pkt, 0, (*radio0).pending_dest_addr);
										led_data_ptr[0] = 0x3f;
										led_data_ptr[1] = 0x06;
										led_data_ptr[2] = ((*radio0).tx_counter & 0xff00)>>8;
										led_data_ptr[3] = ((*radio0).tx_counter & 0xff);
										set_DSN(led_pkt, (*received_packet).dsn);				// DSN = 0
										data_trans(led_pkt);
										gosleep = 0;
										(*radio0).sync = 1;
										pkt_cnt_sync = 1;
									}
								}
							}
							// contention window
							else if ((rx_data[0]==0x11)&&(rx_data[1]==0x33)){
								if ((*radio0).pending_dest_addr==(*received_packet).src_addr_LSB){
									auto_tx_en(0);
									uint32_t cw = (rx_data[2]<<24) | (rx_data[3]<<16) | (rx_data[4]<<8) | rx_data[5];
									uint32_t next_tx_timer = (rand() % cw);
									TIM2_int = 0;
									RF_shdn(1);				// turn off RF
									MSS_TIM2_stop();
									MSS_TIM2_load_immediate(next_tx_timer);
									MSS_TIM2_start();

									(*radio0).radio_state = transmit;
									(*radio0).tx_counter = (*received_packet).dsn;
									set_led((*received_packet).dsn);
									
									// load the data into TX fifo
									set_dest_addr(led_pkt, 0, (*radio0).pending_dest_addr);
									led_data_ptr[0] = 0x3f;
									led_data_ptr[1] = 0x06;
									led_data_ptr[2] = ((*radio0).tx_counter & 0xff00)>>8;
									led_data_ptr[3] = ((*radio0).tx_counter & 0xff);
									set_DSN(led_pkt, (*received_packet).dsn);				// DSN = 0
									pkt_cnt_sync = 1;
									data_trans(led_pkt);
									(*radio0).radio_state = transmit;
									gosleep = 0;
									delay_tx = 1;
								}
							}
						}
					}

					if (gosleep){
						auto_tx_en(0);	// disable auto fire
						RF_shdn(1);
						if (!timer1_set){
							config_next_wakeup((*radio0).received_timer);
							MSS_TIM1_start();
						}
						if ((SW1_int) && (pending_dat_number>1)){
							SW1_int = 0;
							pending_dat_number -= 1;
						}
						if ((SW2_int) && (pending_dat_number<8)){
							SW2_int = 0;
							pending_dat_number += 1;
						}
						(*radio0).radio_state = off;
						#ifndef GDB
						__ASM volatile ("WFI");
						#endif
					}
				}
			break;

			case transmit:
				if (delay_tx & TIM2_int){
					RF_shdn(0);				// turn on RF
					tx_fire();
					TIM2_int = 0;
					delay_tx = 0;
				}

				if (PDMA_int){
					PDMA_int = 0;
				}

				if (tx_complete_int){
					tx_complete_int = 0;
					length_int = 0;			// clear pending length interrupt flag
					gotolisten(TX_LISTEN_TIMER);
					(*radio0).radio_state = listen;
				}
			break;
		}
	#else
		switch ((*radio0).radio_state){
			case startup:
				RF_shdn(1);
				auto_tx_en(0);
				(*radio0).radio_state = off;
				(*radio0).probe_counter = 0;
				(*radio0).probe_timer = ((uint32_t)ONE_SEC>>(probe_timer_shift-1));		// 1 sec
				(*radio0).contention_window = CW_LENGTH;
				MSS_TIM1_load_background((*radio0).probe_timer);
				MSS_TIM1_start();
				#ifndef GDB
				__ASM volatile ("WFI");
				#endif
			break;

			case off:
				if (TIM1_int){
					TIM1_int = 0;
					probe_cw = 0;
					goto1stprobe(probe1_pkt);
					res = setFreqDivider(frequency);	// reset frequency everytime after wake up
					(*radio0).radio_state = probe1;
				}
			break;

			case probe1:			// first probe, triggered by a periodic timer (timer 1)
				if (PDMA_int){
					RF_shdn(0);		// turn on RF
					tx_fire();
					PDMA_int = 0;
					(*radio0).radio_state = transmit;
				}
			break;

			case probe2:			// later probes, triggered by a received data packet
				if (PDMA_int){
					PDMA_int = 0;
					(*radio0).radio_state = transmit;
				}
			break;

			case transmit:
				if (tx_complete_int){
					tx_complete_int = 0;
					switch (probe_cw){
						case 0:
							gotolisten(RX_LISTEN_TIMER);
						break;
						case 1:
							gotolisten((*radio0).contention_window + LENG_INT_LANTENCY + RF_ON_LATENCY);
							probe_cw = 2;
						break;
						default:
						break;

					}
					(*radio0).radio_state = listen;
				}
			break;

			case listen:
				if (length_int){
					auto_tx_en(1);	// enable auto fire
					length_int = 0;
					if (!probe_cw)
						MSS_TIM2_stop();
					(*radio0).radio_state = receive;
				}

				if (TIM2_int){
					TIM2_int = 0;
					RF_shdn(1);
					if (SW1_int | SW2_int){
						MSS_TIM1_stop();
						probe_timer_shift = probe_timer_adj(probe_timer_shift);
						(*radio0).radio_state = startup;
					}
					else{
						(*radio0).radio_state = off;
						#ifndef GDB
						__ASM volatile ("WFI");
						#endif
					}
				}
			break;

			case receive:
				if (rx_pkt_done_int){
					uint8_t empty;
					rx_pkt_done_int = 0;
					do{
						read_fifo(rdata);
						empty = isFifoEmpty();
					}
					while(!empty);

					res = rx_packet_create(received_packet, rdata);

					uint8_t crc_correct, ack_req;
					crc_correct = (*received_packet).crc;
					ack_req = (*received_packet).ack_req;


					if (crc_correct){
						uint32_t* rx_data = rdata + (*received_packet).payload_idx;

						uint8_t address_match = dest_addr_filter(received_packet, radio0);

						// address doesn't match but requests ACK
						// flush ack
						if ( (!address_match) & ack_req){
							ack_flush();
						}

						// Unicast
						if (address_match==1){
						// led packet signature, 0x3f, 0x06
							if ((rx_data[0]==0x3f)&&(rx_data[1]==0x06)){
								if ((*received_packet).dsn==((*radio0).probe_counter & 0xff))
									(*radio0).probe_counter++;
								set_led(rx_data[3]);
								(*radio0).radio_state = probe2;
								goto2ndprobe(probe2_pkt);
							}
						}
					}
					// received a packet but CRC is incorrect, probe again with CW
					else{
						if (!probe_cw){
							probe_cw = 1;
							gotoprobe_CW(probe_CW_pkt);
							(*radio0).radio_state = probe2;
						}
						
						else{
							auto_tx_en(0);
							(*radio0).radio_state = listen;
						}
					}
				}
			break;

		}
	#endif
	}

	return 0;
}

#ifndef TX
inline void initialize_probe(struct tx_packet_str* tp, uint32_t size, uint32_t* dptr){
	(*tp).data_length = size;
	(*tp).data_ptr = dptr;
	set_frame_type(tp, 1);		// data
	set_security(tp, 0);		// disable security
	set_ack(tp, 0);				// enable ack
	set_pan_id_comp(tp, 1);		// enable intra-pan ID
	set_dest_addr_mode(tp, 2);	// 2 bytes of dest address
	set_DSN(tp, 0);				// DSN = 0
	set_dest_pan(tp, 0x22);		// pan addr = 0x0022
	set_dest_addr(tp, 0, 0xffff);
}

inline void goto1stprobe(struct tx_packet_str* tp){
	struct radio* radio_ptr = (*tp).rp;
	uint32_t* data_ptr = (*tp).data_ptr;
	uint32_t pt = (*radio_ptr).probe_timer;
	uint32_t pc = ((*radio_ptr).probe_counter & 0xff);
	data_ptr[0] = 0x11;
	data_ptr[1] = 0x22;
	data_ptr[2] = (pt & (0xff<<24))>>24;
	data_ptr[3] = (pt & (0xff<<16))>>16;
	data_ptr[4] = (pt & (0xff<<8))>>8;
	data_ptr[5] = (pt & 0xff);
	set_DSN(tp, pc);
	data_trans(tp);
}

inline void goto2ndprobe(struct tx_packet_str* tp){
	struct radio* radio_ptr = (*tp).rp;
	uint32_t* data_ptr = (*tp).data_ptr;
	uint32_t pc = ((*radio_ptr).probe_counter & 0xff);
	data_ptr[0] = 0x11;
	data_ptr[1] = 0x22;
	set_DSN(tp, pc);
	data_trans(tp);
}

inline void gotoprobe_CW(struct tx_packet_str* tp){
	struct radio* radio_ptr = (*tp).rp;
	uint32_t* data_ptr = (*tp).data_ptr;
	uint32_t cw = (*radio_ptr).contention_window;
	uint32_t pc = ((*radio_ptr).probe_counter & 0xff);
	data_ptr[0] = 0x11;
	data_ptr[1] = 0x33;
	data_ptr[2] = (cw & (0xff<<24))>>24;
	data_ptr[3] = (cw & (0xff<<16))>>16;
	data_ptr[4] = (cw & (0xff<<8))>>8;
	data_ptr[5] = (cw & 0xff);
	set_DSN(tp, pc);
	data_trans(tp);
}

inline uint8_t probe_timer_adj(uint8_t pts){
	uint8_t new_shift_val = pts;
	if (SW1_int){
		SW1_int = 0;
		if (pts>1)
			new_shift_val = pts - 1;
	}
	else if (SW2_int){
		SW2_int = 0;
		if (pts<7)
			new_shift_val = pts + 1;
	}
	return new_shift_val;
}
#endif

inline void gotolisten(uint32_t duration){
	MSS_TIM2_load_immediate(duration);	
	MSS_TIM2_start();
	MSS_GPIO_clear_irq(MSS_GPIO_16);
	TIM2_int = 0;
	length_int = 0;
}

#ifdef TX
inline void config_next_wakeup(uint32_t received_timer){
	uint32_t tim2_val, new_rx_timer;
	tim2_val = MSS_TIM2_get_current_value();
	MSS_TIM2_stop();
	MSS_TIM1_clear_irq();
	new_rx_timer = received_timer - (ONE_SEC - tim2_val) - LENG_INT_LANTENCY - GUARD_TIMER;
	MSS_TIM1_load_immediate(new_rx_timer);
	MSS_TIM1_start();
}
#endif


inline void set_led(uint32_t data_in){
	*LED_ptr = data_in;
}


// interrupt handlers
//
// pdms_tx_handler		TX transmit handler
// Fabric_IRQHandler	TX complete
// GPIO16_IRQHandler	RX length get
// GPIO17_IRQHandler	received a complete packet
// GPIO18_IRQHandler	ACK complete
// GPIO19_IRQHandler	Generate an ACK, switch mode
// GPIO22_IRQHandler	
// GPIO24_IRQHandler	SW1
// GPIO25_IRQHandler	SW2
// GPIO30_IRQHandler	SW3
// GPIO31_IRQHandler	SW4
// Timer1_IRQHandler
//

void pdma_tx_handler(){
    PDMA_clear_irq( PDMA_CHANNEL_1 );
	PDMA_int = 1;
}

void Fabric_IRQHandler(){
    NVIC_ClearPendingIRQ(Fabric_IRQn);

}

void GPIO16_IRQHandler(){
	MSS_GPIO_clear_irq( MSS_GPIO_16 );
	length_int = 1;
}

void GPIO17_IRQHandler(){
	// received a complete packet, read fifo
	MSS_GPIO_clear_irq( MSS_GPIO_17 );
	rx_pkt_done_int = 1;
}

void GPIO18_IRQHandler(){
	// ack complete, back to rx
	MSS_GPIO_clear_irq( MSS_GPIO_18 );
}

void GPIO19_IRQHandler(){
	// prepare to transmit an ack, switch to tx
	MSS_GPIO_clear_irq( MSS_GPIO_19 );
}

void GPIO22_IRQHandler(){
	MSS_GPIO_clear_irq( MSS_GPIO_22 );
}

void GPIO23_IRQHandler(){
	MSS_GPIO_clear_irq( MSS_GPIO_23 );
	SFD_int = 1;
}

void GPIO24_IRQHandler(){
	// SW1
	MSS_GPIO_clear_irq( MSS_GPIO_24 );
	SW1_int = 1;
}

void GPIO25_IRQHandler(){
	// SW2
	MSS_GPIO_clear_irq( MSS_GPIO_25 );
	SW2_int = 1;
}

void GPIO26_IRQHandler(){
	// SW3
	MSS_GPIO_clear_irq( MSS_GPIO_26 );
	SW3_int = 1;
}

void GPIO27_IRQHandler(){
	// SW4
	MSS_GPIO_clear_irq( MSS_GPIO_27 );
	SW4_int = 1;
}

void GPIO31_IRQHandler(){
	// tx complete
	MSS_GPIO_clear_irq( MSS_GPIO_31 );
    tx_complete_int = 1;
}

void Timer1_IRQHandler(){
	MSS_TIM1_clear_irq();
	TIM1_int = 1;
	//MSS_TIM1_stop();
}

void Timer2_IRQHandler(){
	MSS_TIM2_clear_irq();
	TIM2_int = 1;
}
