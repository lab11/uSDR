
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

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

volatile uint8_t PDMA_int, tx_complete_int;
volatile uint8_t TIM1_int;
volatile uint8_t SW1_int, SW2_int, SW3_int, SW4_int;
volatile uint32_t rx_cnt, tx_counter;
volatile uint8_t rx_pkt_done_int;
uint8_t mode;


/**************************************************************************/
/* Function Prototype*/
/**************************************************************************/

void Fabric_IRQHandler();		// Fab int handler
void pdma_tx_handler();		// pdma int handler
void set_led(uint32_t);
void read_fifo();
void set_interval(uint8_t);
/**************************************************************************/
// pointers
/**************************************************************************/

uint32_t* const RXData_ptr	= (uint32_t*)0x40050000;
uint32_t* const TXRXMode_ptr= (uint32_t*)0x40070000;
uint32_t* const LED_ptr		= (uint32_t*)0x40070020;


/**************************************************************************/
/*Main Program*/
/**************************************************************************/

int main()
{
	init_system();

	// interrupt variables
	TIM1_int = 0;
	tx_complete_int= 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	rx_pkt_done_int = 0;
	PDMA_int = 0;

	// initial global variable
	rx_cnt = 0;
	tx_counter = 0;

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
		if (mode==1){
			if (PDMA_int==1){
				PDMA_int = 0;
				tx_fire();
			}
			if (tx_complete_int==1){
				mode = 0;
				set_mode(mode);
				tx_complete_int = 0;
				tx_counter++;
				led_data_ptr[2] = (tx_counter & 0xff00)>>8;
				led_data_ptr[3] = (tx_counter & 0xff);
				set_DSN(led_pkt, led_data_ptr[3]);				// DSN = 0
				data_trans (led_pkt);
				MSS_TIM1_start();
			}
		}
		// RX read packet
		else if (mode==0){
			if (rx_pkt_done_int==1){
				rx_pkt_done_int = 0;
				read_fifo(rdata);

				uint8_t frame_type, crc_correct, ack_req, address_match, addr_mode;
				uint32_t dest_addr[2]; 
				uint32_t src_addr[2];
				frame_filter(	radio0, &frame_type, &crc_correct, &ack_req, &address_match, &addr_mode, 
								dest_addr, src_addr);

				// address doesn't match but requests ACK
				// flush ack
				if ( (!address_match) & ack_req){
					ack_flush();
				}

				// action only if CRC correct
				if (crc_correct){
					// broadcast 
					if (address_match==3){
					// led packet signature, 0x3f, 0x06
						if ((rdata[10]==0x3f)&&(rdata[11]==0x06)){
							set_led(rdata[13]);
						}
					}
				}
				

			}
			if (TIM1_int==1){
				TIM1_int = 0;
				mode = 1;
				set_mode(mode);
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
	}

	return 0;
}



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
