
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

#include "radio_config.h"
#include "maxim2831.h"
#include "drivers/mss_pdma/mss_pdma.h"
#include "drivers/mss_gpio/mss_gpio.h"
#include "drivers/mss_uart/mss_uart.h"
#include "drivers/mss_watchdog/mss_watchdog.h"
#include "drivers/mss_timer/mss_timer.h"

extern uint8_t mode;
extern uint32_t * TXRXMode_ptr;
extern void pdma_tx_handler();

uint32_t * const auto_ack_reg = (uint32_t *) (0x40070010);
uint32_t * const rx_agc_reg = (uint32_t *) (0x40070040);
uint32_t * const ack_flush_reg = (uint32_t *) (0x40070050);
uint32_t * const crc_fcf_reg = (uint32_t *) (0x4005000c);
uint32_t * const pan_id_reg = (uint32_t *) (0x40050004);
uint32_t * const src_addr_reg = (uint32_t *) (0x40050010);
uint32_t * const dest_addr_reg = (uint32_t *) (0x40050018);

// addr[0] -> LSB of 1st address, addr[1] -> MSB of 1st address,
// addr[2] -> LSB of 2nd address, addr[2] -> MSB of 2nd address...
// maximum 7 additional address
void set_multiple_address(struct radio* r0, uint8_t en, uint8_t nu, uint32_t* map){
	(*r0).multiple_addr_en = (en&0x1);
	(*r0).num_of_multiple_addr = (nu&0x7);
	(*r0).multiple_addr_ptr = map;
}

inline void set_src_pan(struct radio* r0, uint16_t sp){
	(*r0).src_pan_ID = sp;
}

// sa1 = higher 32 bytes, only used in 64 bytes source address mode
// sa0 = lower 32 bytes
inline void set_src_addr(struct radio* r0, uint32_t sa1, uint32_t sa0){
	(*r0).src_addr_MSB = sa1;
	(*r0).src_addr_LSB = sa0;
}

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline uint8_t set_src_addr_mode(struct radio* r0, uint8_t sam){
	uint8_t temp = (sam&0x3);
	if (sam==1)
		return 0;
	else
		(*r0).src_addr_mode = temp;
	return 1;
}

uint8_t address_checker(struct radio* r0, uint8_t address_mode, uint32_t MSB, uint32_t LSB){
	uint8_t i;
	for (i=0;i<(*r0).num_of_multiple_addr;i++){
		if (address_mode==2){
			if ((((*r0).multiple_addr_ptr[(i<<1)] ^ LSB) & 0x0000ffff)==0)
				return 1;
		}
		else{
			if ((((*r0).multiple_addr_ptr[(i<<1)] ^ LSB) | ((*r0).multiple_addr_ptr[(i<<1)+1] ^ MSB))==0)
				return 1;
		}
	}
	return 0;
}

inline void ack_flush(){
	*ack_flush_reg = 1;
}

// return 1 if frame is valid
uint8_t frame_filter(	struct radio* r0, uint8_t* frame_type, uint8_t* crc_correct, uint8_t* ack_req, 
						uint8_t* address_match, uint8_t* addr_mode, uint32_t* dest_addr, uint32_t* src_addr){
	uint32_t pan_ID = *pan_id_reg;
	uint16_t pkt_dest_pan_ID = (pan_ID & 0x0000ffff);
	uint32_t crc_fcf = *crc_fcf_reg;

	*frame_type = (crc_fcf & 0x3);				// frame type
	*crc_correct = (crc_fcf & 0x10000)>>16;		// crc_valid, 1 = valid, 0 = invalid
	*ack_req = (crc_fcf & 0x20000)>>17;			// request ACK
	*address_match = 0;							// address match, 1 = match, 3 = broadcast or glossy, 0 = doesn't match

	uint16_t radio_src_pan_ID = (*r0).src_pan_ID;
	uint32_t radio_src_addr_LSB = (*r0).src_addr_LSB;
	uint32_t radio_src_addr_MSB = (*r0).src_addr_MSB;

	if (pkt_dest_pan_ID==radio_src_pan_ID){
		uint32_t pkt_dest_addr_LSB = *dest_addr_reg;
		uint32_t pkt_src_addr_LSB = *src_addr_reg;
		uint8_t address_mode = ((crc_fcf & 0xc00)>>10);	// extract 11:10
		*addr_mode = address_mode;
		*dest_addr = pkt_dest_addr_LSB;
		*src_addr  = pkt_src_addr_LSB;
		// 2 bytes for address
		if (address_mode==2){
			pkt_dest_addr_LSB = (pkt_dest_addr_LSB & 0x0000ffff);
			// broadcast or glossy
			if ((pkt_dest_addr_LSB==0xffff)||(pkt_dest_addr_LSB==0xfffe))
				*address_match = 3;	// address match;
			else if (pkt_dest_addr_LSB == (radio_src_addr_LSB & 0x0000ffff))
				*address_match = 1;	// address match;
			else if ((*r0).multiple_addr_en)
				*address_match = address_checker(r0, 2, 0, pkt_dest_addr_LSB);	// address match;
		}
		// 8 bytes for address
		else if (address_mode==3){
			uint32_t pkt_dest_addr_MSB = *(dest_addr_reg + 4);
			uint32_t pkt_src_addr_MSB = *(src_addr_reg + 4);
			*(dest_addr+1) = pkt_dest_addr_MSB;
			*(src_addr+1) = pkt_src_addr_MSB;
			// broadcast or glossy
			if (((pkt_dest_addr_LSB==0xffffffff)||(pkt_dest_addr_LSB==0xfffffffe))&&(pkt_dest_addr_MSB==0xffffffff))
				*address_match = 3;	// address match;
			else if ((pkt_dest_addr_MSB==radio_src_addr_MSB) && (pkt_dest_addr_LSB==radio_src_addr_LSB))
				*address_match = 1;	// address match;
			else if ((*r0).multiple_addr_en)
				*address_match = address_checker(r0, 3, pkt_dest_addr_MSB, pkt_dest_addr_LSB);	// address match;
		}
		else
			return 0;
	}

	return 1;		// need to add more condition
}


inline void auto_ack_en(uint8_t ack_en){
	*auto_ack_reg = ack_en;
}
// 11 for continues AGC loop
// 10 or 01 for SFD lock
// 00 for off AGC
inline void rx_agc_en(uint8_t agc_mode){
	*rx_agc_reg = agc_mode;
}

void DAC_shdn(uint8_t shdn){
	if (shdn)
		MSS_GPIO_set_output( MSS_GPIO_1, 1 );	// DAC down
	else {
		MSS_GPIO_set_output( MSS_GPIO_1, 0 );	// DAC up
		MSS_GPIO_set_output( MSS_GPIO_0, 1 );	// DAC DACEN
	}
}

void ADC_shdn(uint8_t shdn){
	if (shdn)
		MSS_GPIO_set_output( MSS_GPIO_8, 0 );	// ADC down
	else
		MSS_GPIO_set_output( MSS_GPIO_8, 1 );	// ADC up
}

void RF_shdn(uint8_t shdn){
	if (shdn){
		MSS_GPIO_set_output( MSS_GPIO_4, 0 );	// RF down
		MSS_GPIO_set_output( MSS_GPIO_2, 1 );	// RX/TX set to 1, enter shutdown mode
	}
	else{
		MSS_GPIO_set_output( MSS_GPIO_2, mode );	// TX
		MSS_GPIO_set_output( MSS_GPIO_4, 1 );	// RF up
	}
	DAC_shdn(shdn);
	ADC_shdn(shdn);
}

inline void tx_fire(){
	MSS_GPIO_set_output( MSS_GPIO_7, 1 );
	MSS_GPIO_set_output( MSS_GPIO_7, 0 );
}

void set_mode(uint8_t mode_in){
	if (mode_in){
		MSS_GPIO_set_output( MSS_GPIO_2, 1 );	// TX
		mode = 1;
	}
	else{
		MSS_GPIO_set_output( MSS_GPIO_2, 0 );	// RX
		mode = 0;
	}
	*TXRXMode_ptr = mode_in;
}


uint8_t read_mode(){
	if (*TXRXMode_ptr & 0x00000001)
		return 1;
	return 0;
}

void init_system()
{
	MSS_WD_disable();
	// RF registers default
	initialize_chip();

	// Uart init
	MSS_UART_init
    (
    	&g_mss_uart0,
    	MSS_UART_57600_BAUD,
    	MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT
    );

	// PDMA init
	PDMA_init();
	PDMA_configure
	(
	    PDMA_CHANNEL_1,
	    PDMA_MEM_TO_MEM,
	    PDMA_LOW_PRIORITY | PDMA_WORD_TRANSFER | PDMA_INC_SRC_FOUR_BYTES,
	    PDMA_DEFAULT_WRITE_ADJ
	);
    PDMA_configure
    (
       PDMA_CHANNEL_0,
       PDMA_MEM_TO_MEM,
       PDMA_LOW_PRIORITY | PDMA_WORD_TRANSFER | PDMA_INC_DEST_FOUR_BYTES,
       PDMA_DEFAULT_WRITE_ADJ
    );
	PDMA_set_irq_handler( PDMA_CHANNEL_1, pdma_tx_handler );
	PDMA_enable_irq( PDMA_CHANNEL_1 );

	// timer init
	/*
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
	MSS_TIM1_load_background(0x00ffffff);
	MSS_TIM1_enable_irq();
	*/

	//GPIO
	MSS_GPIO_init();
	MSS_GPIO_config( MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_1, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_2, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_3, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_4, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_5, MSS_GPIO_INPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_6, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_7, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_8, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_16, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_17, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_18, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_19, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_22, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_23, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_24, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_25, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_26, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_27, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_31, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_set_output( MSS_GPIO_0, 1 );				// DAC DACEN
	MSS_GPIO_set_output( MSS_GPIO_1, 0 );				// DAC PD
	MSS_GPIO_set_output( MSS_GPIO_2, 0 );				// RF RX/TX
	MSS_GPIO_set_output( MSS_GPIO_3, 1 );				// ADC DFS, 1 for 2's compliment
	MSS_GPIO_set_output( MSS_GPIO_4, 1 );				// RF SHDN
	MSS_GPIO_set_output( MSS_GPIO_6, 1 );				// RXHP	set to 1 by default
	MSS_GPIO_set_output( MSS_GPIO_7, 0 );				// tx fire
	MSS_GPIO_set_output( MSS_GPIO_8, 1 );				// ADC s1
	MSS_GPIO_disable_irq( MSS_GPIO_16 );				// Length interrupt
	MSS_GPIO_enable_irq( MSS_GPIO_17 );					// packet received interrupt
	MSS_GPIO_enable_irq( MSS_GPIO_18 );					// ACK packet complete
	MSS_GPIO_enable_irq( MSS_GPIO_19 );					// switch RX/TX
	MSS_GPIO_disable_irq( MSS_GPIO_22 );				
	MSS_GPIO_disable_irq( MSS_GPIO_23 );				// SFD
	MSS_GPIO_enable_irq( MSS_GPIO_24 );					// SW1
	MSS_GPIO_enable_irq( MSS_GPIO_25 );					// SW2
	MSS_GPIO_enable_irq( MSS_GPIO_26 );					// SW3
	MSS_GPIO_enable_irq( MSS_GPIO_27 );					// SW4
	MSS_GPIO_enable_irq( MSS_GPIO_31 );					// TX complete

	// Interrupt
	NVIC_EnableIRQ( DMA_IRQn );
	NVIC_EnableIRQ( Fabric_IRQn );
}
