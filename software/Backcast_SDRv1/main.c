
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


volatile uint8_t PDMA_int, tx_complete_int;
volatile uint8_t TIM1_int, TIM2_int;
volatile uint8_t SW1_int, SW2_int, SW3_int, SW4_int;
volatile uint8_t rx_pkt_done_int, SFD_int, length_int, auto_fire_int;
uint8_t mode;


/**************************************************************************/
/* Function Prototype*/
/**************************************************************************/

void Fabric_IRQHandler();		// Fab int handler
void pdma_tx_handler();		// pdma int handler
inline void set_led(uint32_t);
void read_fifo(uint32_t*);
inline void gotolisten(uint32_t);
inline void auto_fire(uint32_t);
#ifdef TX
inline void config_next_wakeup(uint32_t);
#else
inline void goto1stprobe(struct tx_packet_str*);
inline void goto2ndprobe(struct tx_packet_str*);
#endif
/**************************************************************************/
// pointers
/**************************************************************************/

uint32_t* const RXData_ptr	= (uint32_t*)0x40050000;
uint32_t* const TXRXMode_ptr= (uint32_t*)0x40070000;
uint32_t* const LED_ptr		= (uint32_t*)0x40070020;
uint32_t* const auto_fire_ptr	= (uint32_t*)0x40070030;

/**************************************************************************/
/*Define*/
/**************************************************************************/
#define CLK_FREQ 				48		// 48MHz
#define TX_LISTEN_DUR			1000	// 1000us
#define TX_LISTEN_INIT_DUR		100000 	// 100ms
#define RXTX_TURN_DUR			192
#define GUARD					250		// 250us
#define	RX_LISTEN_DUR			(RXTX_TURN_DUR + (32*6) + GUARD)		// 192 turnaround, 4 preamble, 1 sfd, 1 length, 2 guard
#define RX_LISTEN_TIMER			CLK_FREQ*RX_LISTEN_DUR
#define TX_LISTEN_TIMER 		CLK_FREQ*TX_LISTEN_DUR
#define TX_LISTEN_INIT_TIMER 	CLK_FREQ*TX_LISTEN_INIT_DUR
#define RXTX_TURN_TIMER			CLK_FREQ*RXTX_TURN_DUR
#define ONE_SEC					CLK_FREQ*1000000
#define TWO_SEC					CLK_FREQ*2000000
#define INIT_PROBE				ONE_SEC*0.3
#define LENG_INT_LANTENCY		CLK_FREQ*(209 + GUARD)	// start tx to length int goes high takes 209 us, 1000us guard

/**************************************************************************/
/*Main Program*/
/**************************************************************************/


int main()
{
	init_system();

	// interrupt variables
	rx_pkt_done_int = 0;
	PDMA_int = 0;
	SFD_int = 0;
	length_int = 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	auto_fire_int = 0;
	TIM1_int = 0;
	tx_complete_int= 0;

	// RF variables
	uint8_t res;
	
	// set RF gain
	res = setTXBaseBandVGAGain(10);
	res = setRXBaseBandVGAGain(10);
	res = setLNAGain(2);			// medium rx lna gain
	uint32_t frequency = 2405;
	res = setFreqDivider(frequency);
	
	mode = 0;						// 0 for rx, 1 for tx
	set_mode(mode);

	rx_agc_en(1);

	struct radio* radio0;
	radio0 = malloc(sizeof(struct radio));
	(*radio0).multiple_addr_en = 0;		// disable multiple address
	(*radio0).radio_state = startup;

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

	uint32_t* probe1_data_ptr;
	uint32_t probe1_size = 6;
	probe1_data_ptr = malloc(sizeof(uint32_t)*probe1_size);
	(*probe1_pkt).data_length = probe1_size;
	(*probe1_pkt).data_ptr = probe1_data_ptr;

	uint32_t* probe2_data_ptr;
	uint32_t probe2_size = 2;
	probe2_data_ptr = malloc(sizeof(uint32_t)*probe2_size);
	(*probe2_pkt).data_length = probe2_size;
	(*probe2_pkt).data_ptr = probe2_data_ptr;

	// set probe1 packet
	set_frame_type(probe1_pkt, 1);		// data
	set_security(probe1_pkt, 0);		// disable security
	set_ack(probe1_pkt, 0);				// enable ack
	set_pan_id_comp(probe1_pkt, 1);		// enable intra-pan ID
	set_dest_addr_mode(probe1_pkt, 2);	// 2 bytes of dest address
	set_DSN(probe1_pkt, 0);				// DSN = 0
	set_dest_pan(probe1_pkt, 0x22);		// pan addr = 0x0022
	set_dest_addr(probe1_pkt, 0, 0xffff);
	// set probe2 packet
	set_frame_type(probe2_pkt, 1);		// data
	set_security(probe2_pkt, 0);		// disable security
	set_ack(probe2_pkt, 0);				// enable ack
	set_pan_id_comp(probe2_pkt, 1);		// enable intra-pan ID
	set_dest_addr_mode(probe2_pkt, 2);	// 2 bytes of dest address
	set_DSN(probe2_pkt, 0);				// DSN = 0
	set_dest_pan(probe2_pkt, 0x22);		// pan addr = 0x0022
	set_dest_addr(probe2_pkt, 0, 0xffff);

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
	MSS_TIM2_init(MSS_TIMER_ONE_SHOT_MODE);
	MSS_TIM2_enable_irq();
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
	MSS_TIM1_enable_irq();

	while (1){
	#ifdef TX
		switch ((*radio0).radio_state){
			case startup:
				(*radio0).radio_state = sleep;
				(*radio0).sync = 0;
				(*radio0).tx_counter = 1;
				(*radio0).pending_data_nu = 1;	// 1 packet pending
				(*radio0).pending_dest_addr = 0x5;
				RF_shdn(1);				// turn off RF
				MSS_TIM1_load_background(TWO_SEC);
				MSS_TIM1_start();
				auto_fire(0);
			break;

			case sleep:
				if (TIM1_int){			// periodic wake up
					TIM1_int = 0;
					set_mode(0);
					RF_shdn(0);			// turn on RF
					(*radio0).pending_data_nu = 3;	// 1 packet pending
					(*radio0).received_timer = 0;
					MSS_TIM1_stop();
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
					auto_fire(1);	// enable auto fire
				}

				if (TIM2_int){
					TIM2_int = 0;
					MSS_TIM1_start();
					RF_shdn(1);
					(*radio0).sync = 0;
					(*radio0).radio_state = sleep;
				}
			break;

			case receive:
				if (rx_pkt_done_int){
					rx_pkt_done_int = 0;
					read_fifo(rdata);

					uint8_t frame_type, crc_correct, ack_req, address_match, addr_mode;
					uint32_t dest_addr[2]; 
					uint32_t src_addr[2];
					uint8_t gosleep = 1;
					frame_filter(	radio0, &frame_type, &crc_correct, &ack_req, &address_match, &addr_mode, 
									dest_addr, src_addr);
					// address doesn't match but requests ACK
					// flush ack
					if ( (!address_match) & ack_req ){
						ack_flush();
					}

					// action only if CRC correct
					if (crc_correct){
						// broadcast, check frame
						if (address_match==3){
							// probe packet signature, 0x11, 0x22
							if ((rdata[10]==0x11)&&(rdata[11]==0x22)){
								// first probe
								if (rdata[0]>15){
									(*radio0).received_timer = (rdata[12]<<24) | (rdata[13]<<16) | (rdata[14]<<8) | rdata[15];
									config_next_wakeup((*radio0).received_timer);
								}
								// src addr filtering
								if (((*radio0).pending_dest_addr==src_addr[0])&&((*radio0).pending_data_nu)){
									(*radio0).radio_state = transmit;
									set_led(rdata[3]);
									
									// load the data into TX fifo
									set_dest_addr(led_pkt, 0, (*radio0).pending_dest_addr);
									led_data_ptr[0] = 0x3f;
									led_data_ptr[1] = 0x06;
									led_data_ptr[2] = ((*radio0).tx_counter & 0xff00)>>8;
									led_data_ptr[3] = ((*radio0).tx_counter & 0xff);
									set_DSN(led_pkt, led_data_ptr[3]);				// DSN = 0
									data_trans(led_pkt);
									(*radio0).tx_counter++;
									gosleep = 0;
									(*radio0).sync = 1;
								}
							}
						}
					}

					if (gosleep){
						auto_fire(0);	// disable auto fire
						RF_shdn(1);
						(*radio0).radio_state = sleep;
					}
				}
			break;

			case transmit:
				if (auto_fire_int & PDMA_int){
					set_mode(1);	// switch to TX
					auto_fire_int = 0;
					PDMA_int = 0;
				}

				if (tx_complete_int){
					set_mode(0);		// switch to RX
					(*radio0).pending_data_nu -= 1;
					tx_complete_int = 0;
					if ((*radio0).pending_data_nu){
						gotolisten(TX_LISTEN_TIMER);
						(*radio0).radio_state = listen;
					}
					else{
						auto_fire(0);	// disable auto fire
						RF_shdn(1);
						(*radio0).radio_state = sleep;
					}
				}
			break;
		}
	#else
		switch ((*radio0).radio_state){
			case startup:
				RF_shdn(1);
				auto_fire(0);
				(*radio0).radio_state = sleep;
				(*radio0).probe_counter = 0;
				(*radio0).probe_timer = ONE_SEC;		// 1 sec
				MSS_TIM1_load_background((*radio0).probe_timer);
				MSS_TIM1_start();
			break;

			case sleep:
				if (TIM1_int){
					TIM1_int = 0;
					goto1stprobe(probe1_pkt);
					(*radio0).probe_counter++;
					(*radio0).radio_state = probe1;
				}
			break;

			case probe1:			// first probe, triggered by a periodic timer (timer 1)
				if (PDMA_int){
					set_mode(1);	// switch to TX
					RF_shdn(0);		// turn on RF
					tx_fire();
					PDMA_int = 0;
					(*radio0).radio_state = transmit;
				}
			break;

			case probe2:			// later probes, triggered by a received data packet
				if (PDMA_int & auto_fire_int){
					set_mode(1);	// switch to TX
					PDMA_int = 0;
					auto_fire_int = 0;
					(*radio0).radio_state = transmit;
				}
			break;

			case transmit:
				if (tx_complete_int){
					tx_complete_int = 0;
					set_mode(0);		// switch to RX
					gotolisten(RX_LISTEN_TIMER);
					(*radio0).radio_state = listen;
					auto_fire(1);	// enable auto fire
				}
			break;

			case listen:
				if (length_int){
					length_int = 0;
					MSS_TIM2_stop();
					(*radio0).radio_state = receive;
				}

				if (TIM2_int){
					TIM2_int = 0;
					RF_shdn(1);
					(*radio0).radio_state = sleep;
				}
			break;

			case receive:
				if (rx_pkt_done_int){
					rx_pkt_done_int = 0;
					read_fifo(rdata);

					uint8_t frame_type, crc_correct, ack_req, address_match, addr_mode;
					uint32_t dest_addr[2]; 
					uint32_t src_addr[2];
					uint8_t gosleep = 1;
					frame_filter(	radio0, &frame_type, &crc_correct, &ack_req, &address_match, &addr_mode, 
									dest_addr, src_addr);
					// address doesn't match but requests ACK
					// flush ack
					if ( (!address_match) & ack_req){
						ack_flush();
					}

					// action only if CRC correct
					if (crc_correct){
						// address exactly match, check frame
						if (address_match==1){
							// led packet signature, 0x3f, 0x06
							if ((rdata[10]==0x3f)&&(rdata[11]==0x06)){
								gosleep = 0;
								set_led(rdata[13]);
								goto2ndprobe(probe2_pkt);
								//(*radio0).probe_counter++;
								(*radio0).radio_state = probe2;
							}
						}
					}

					if (gosleep){
						auto_fire(0);	// disable auto fire
						RF_shdn(1);
						(*radio0).radio_state = sleep;
					}
				}
			break;

		}
	#endif
	}

	return 0;
}

#ifndef TX
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
#endif

inline void gotolisten(uint32_t duration){
	MSS_TIM2_load_immediate(duration);	
	MSS_TIM2_start();
	MSS_GPIO_clear_irq(MSS_GPIO_16);
	length_int = 0;		// clear pending length interrupt
	TIM2_int = 0;
}

#ifdef TX
inline void config_next_wakeup(uint32_t received_timer){
	uint32_t tim2_val, new_rx_timer;
	tim2_val = MSS_TIM2_get_current_value();
	MSS_TIM2_stop();
	MSS_TIM1_clear_irq();
	if (received_timer){
		new_rx_timer = received_timer - (ONE_SEC - tim2_val) - LENG_INT_LANTENCY;
		MSS_TIM1_load_background(new_rx_timer);
	}
	MSS_TIM1_start();
}
#endif

void read_fifo(uint32_t* rdata){
	uint32_t rx_pkt_length;
	rx_pkt_length = *RXData_ptr;
	if (rx_pkt_length==0)
		return;

    uint32_t status_pdma = 0;

	*rdata = rx_pkt_length;

    PDMA_start
    (
		PDMA_CHANNEL_0,
		(uint32_t)RXData_ptr,
		(uint32_t)(rdata + 1),
		(uint16_t)rx_pkt_length
    );


    do
    {
        status_pdma = PDMA_status (PDMA_CHANNEL_0);
    }while (status_pdma == 0);
}

inline void auto_fire(uint32_t en){
	uint32_t temp = (en & 0x1);
	if (temp)
		MSS_GPIO_enable_irq (MSS_GPIO_22);		// enable auto fire interrupt
	else
		MSS_GPIO_disable_irq (MSS_GPIO_22);		// disable auto fire interrupt

	auto_fire_int = 0;
	*auto_fire_ptr = temp;
}

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
	if (mode==0)
		set_mode(0);
}

void GPIO19_IRQHandler(){
	// prepare to transmit an ack, switch to tx
	MSS_GPIO_clear_irq( MSS_GPIO_19 );
	if (mode==0)
		set_mode(1);
}

void GPIO22_IRQHandler(){
	MSS_GPIO_clear_irq( MSS_GPIO_22 );
	auto_fire_int = 1;
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
