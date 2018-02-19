
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
#include "rx_packet.h"

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

volatile uint8_t PDMA_int, tx_complete_int;
volatile uint8_t TIM1_int;
volatile uint8_t SW1_int, SW2_int, SW3_int, SW4_int;
volatile uint32_t rx_cnt, tx_counter;
volatile uint8_t rx_pkt_done_int, length_int;


/**************************************************************************/
/* Function Prototype*/
/**************************************************************************/

void Fabric_IRQHandler();		// Fab int handler
void pdma_tx_handler();		// pdma int handler
void set_led(uint32_t);
void set_interval(uint8_t);
/**************************************************************************/
// pointers
/**************************************************************************/


uint32_t* const LED_ptr		= (uint32_t*)0x40070020;


/**************************************************************************/
/*Main Program*/
/**************************************************************************/

#define LED
int main()
{
	RF_shdn(0);
	init_system();

	// interrupt variables
	TIM1_int = 0;
	tx_complete_int= 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	rx_pkt_done_int = 0;
	PDMA_int = 0;
	length_int = 0;

	// initial global variable
	rx_cnt = 0;
	tx_counter = 3;

	// RF variables
	uint8_t res;
	
	// set RF gain
	res = setTXBaseBandVGAGain(10);
	uint32_t frequency = 2405;
	res = setFreqDivider(frequency);

	rx_agc_en(3);

	struct radio* radio0;
	radio0 = malloc(sizeof(struct radio));
	radio0->multiple_addr_en = 0;		// disable multiple address

	struct tx_packet_str* led_pkt;
	led_pkt = malloc(sizeof(struct tx_packet_str));
	led_pkt->rp = radio0;				// link to radio0

	struct rx_packet_str* received_packet;
	received_packet = malloc(sizeof(struct rx_packet_str));

	uint32_t* led_data_ptr;
	uint32_t data_size = 4;
	led_data_ptr = malloc(sizeof(uint32_t)*data_size);
	led_pkt->data_length = data_size;
	led_pkt->data_ptr = led_data_ptr;

	// set led packet
	set_frame_type(led_pkt, 1);			// data
	set_security(led_pkt, 0);			// disable security
	set_pan_id_comp(led_pkt, 1);		// enable intra-pan ID
	set_dest_addr_mode(led_pkt, 2);		// 2 bytes of dest address
	set_DSN(led_pkt, 0);				// DSN = 0
	set_dest_pan(led_pkt, 0x22);		// pan addr = 0x0022
	set_frame_pending(led_pkt, 0);		// no frame pending

	#ifdef LED
	set_dest_addr(led_pkt, 0, 0xffff);
	set_ack(led_pkt, 0);
	#endif

	#ifdef GLOSSY
	set_dest_addr(led_pkt, 0, 0xfffe);
	set_ack(led_pkt, 0);
	#endif

	#ifdef ACK
	set_dest_addr(led_pkt, 0, 0x05);
	set_ack(led_pkt, 1);
	#endif

	// set radio
	set_src_pan(radio0, 0x22);
	set_src_addr_mode(radio0, 2);	// 2 bytes of src address
	set_src_addr(radio0, 0, 0x05);

	// set radio packet data
	led_data_ptr[0] = 0x3f;
	led_data_ptr[1] = 0x06;
	led_data_ptr[2] = 0x00;
	led_data_ptr[3] = 0x00;

	data_trans(led_pkt);

	// fifo read
	uint32_t rdata[128];

	// timer
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
	MSS_TIM1_enable_irq();
	uint8_t interval = 4;	// 4Hz
	set_interval(interval);

	while (1){
		if (TIM1_int & PDMA_int){
			// read gpios here
			tx_fire();

			PDMA_int = 0;
			TIM1_int = 0;
		}
		if (tx_complete_int==1){
			tx_complete_int = 0;
			tx_counter++;
			led_data_ptr[2] = (tx_counter & 0xff00)>>8;
			led_data_ptr[3] = (tx_counter & 0xff);
			#ifdef GLOSSY
			set_DSN(led_pkt, 126);				// DSN = 126, only forward once 
			#else
			set_DSN(led_pkt, led_data_ptr[3]);
			#endif
			data_trans (led_pkt);
			MSS_TIM1_start();
		}
		// RX read packet
		if (rx_pkt_done_int==1){
			rx_pkt_done_int = 0;
			read_fifo(rdata);

			res = rx_packet_create(received_packet, rdata);

			uint8_t crc_correct, ack_req;
			crc_correct = received_packet->crc;
			ack_req = received_packet->ack_req;
			if (crc_correct){
				uint32_t* rx_data = rdata + received_packet->payload_idx;

				uint8_t address_match = dest_addr_filter(received_packet, radio0);

				// address doesn't match but requests ACK
				// flush ack
				if ( (!address_match) & ack_req){
					ack_flush();
				}

				// broadcast 
				if (address_match==3){
				// led packet signature, 0x3f, 0x06
					if ((rx_data[0]==0x3f)&&(rx_data[1]==0x06)){
						set_led(rx_data[3]);
					}
				}

			}

		}
		if (SW1_int==1){
			SW1_int = 0;
			if (interval>1)
				interval = (interval>>1);
			set_interval(interval);
		}
		if (SW2_int==1){
			SW2_int = 0;
			if (interval<16)
				interval = (interval<<1);
			set_interval(interval);
		}
		if (SW3_int==1){
			SW3_int = 0;
			if (frequency>2405){
				frequency -= 5;
				res = setFreqDivider(frequency);
			}
		}
		if (SW4_int==1){
			SW4_int = 0;
			if (frequency<2480){
				frequency += 5;
				res = setFreqDivider(frequency);
			}
		}

	}

	return 0;

}

inline void set_led(uint32_t data_in){
	*LED_ptr = data_in;
}


void set_interval(uint8_t input_interval){
	MSS_TIM1_stop();
	switch (input_interval){
		case 1:
			MSS_TIM1_load_background(ONE_SEC);
		break;
		case 2:
			MSS_TIM1_load_background(ONE_SEC>>1);
		break;
		case 4:
			MSS_TIM1_load_background(ONE_SEC>>2);
		break;
		case 8:
			MSS_TIM1_load_background(ONE_SEC>>3);
		break;
		case 16:
			MSS_TIM1_load_background(ONE_SEC>>4);
		break;
	}
	MSS_TIM1_start();
}


// interrupt handlers
//
// pdms_tx_handler		TX transmit handler
// Fabric_IRQHandler	TX complete
// GPIO16_IRQHandler	RX length get
// GPIO17_IRQHandler	received a complete packet
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


void GPIO22_IRQHandler(){
	MSS_GPIO_clear_irq( MSS_GPIO_22 );
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
	MSS_TIM1_stop();
}
