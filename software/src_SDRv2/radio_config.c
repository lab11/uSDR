
/*
 * "Copyright (c) 2010-2012 The Regents of the University of Michigan.
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
 * Last update: 9/25/2012
 * Last update content:
 * If agc is disabled, set a default gain 
 */

#include "radio_config.h"
#include "maxim2831.h"
#include "drivers/mss_pdma/mss_pdma.h"
#include "drivers/mss_gpio/mss_gpio.h"
#include "drivers/mss_uart/mss_uart.h"
#include "drivers/mss_watchdog/mss_watchdog.h"
#include "drivers/mss_timer/mss_timer.h"

extern void pdma_tx_handler();

uint32_t * const radio_ctl_reg = (uint32_t *) (0x40070000); // 4bits radio_mode, 1bit ack_en, 2bits agc_mode, 1bit radio_off
uint32_t * const ack_flush_reg = (uint32_t *) (0x40070050);
/*
uint32_t * const crc_fcf_reg = (uint32_t *) (0x4005000c);
uint32_t * const pan_id_reg = (uint32_t *) (0x40050004);
uint32_t * const src_addr_reg = (uint32_t *) (0x40050010);
uint32_t * const dest_addr_reg = (uint32_t *) (0x40050018);
*/

#define RADIO_OFF_BIT_POS 0
#define AGC_MODE_BIT_POS 1
#define ACK_EN_BIT_POS 3
#define RADIO_MODE_BIT_POS 4
#define AUTO_TX_BIT_POS 8

#define RADIO_OFF_MASK 	(0x1<<RADIO_OFF_BIT_POS)
#define AGC_MODE_MASK 	(0x3<<AGC_MODE_BIT_POS)
#define ACK_EN_MASK 	(0x1<<ACK_EN_BIT_POS)
#define RADIO_MODE_MASK (0xf<<RADIO_MODE_BIT_POS)
#define AUTO_TX_MASK	(0x1<<AUTO_TX_BIT_POS)

// addr[0] -> LSB of 1st address, addr[1] -> MSB of 1st address,
// addr[2] -> LSB of 2nd address, addr[2] -> MSB of 2nd address...
// maximum 7 additional address
void set_multiple_address(struct radio* r0, uint8_t en, uint8_t nu, uint32_t* map){
	r0->multiple_addr_en = (en&0x1);
	r0->num_of_multiple_addr = (nu&0x7);
	r0->multiple_addr_ptr = map;
}

inline void set_src_pan(struct radio* r0, uint16_t sp){
	r0->src_pan_ID = sp;
}

// sa1 = higher 32 bytes, only used in 64 bytes source address mode
// sa0 = lower 32 bytes
inline void set_src_addr(struct radio* r0, uint32_t sa1, uint32_t sa0){
	r0->src_addr_MSB = sa1;
	r0->src_addr_LSB = sa0;
}

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline uint8_t set_src_addr_mode(struct radio* r0, uint8_t sam){
	uint8_t temp = (sam&0x3);
	if (sam==1)
		return 0;
	else
		r0->src_addr_mode = temp;
	return 1;
}

uint8_t address_checker(struct radio* r0, uint8_t address_mode, uint32_t MSB, uint32_t LSB){
	uint8_t i;
	for (i=0;i<r0->num_of_multiple_addr;i++){
		if (address_mode==2){
			if (((r0->multiple_addr_ptr[(i<<1)] ^ LSB) & 0x0000ffff)==0)
				return 1;
		}
		else{
			if (((r0->multiple_addr_ptr[(i<<1)] ^ LSB) | (r0->multiple_addr_ptr[(i<<1)+1] ^ MSB))==0)
				return 1;
		}
	}
	return 0;
}

inline void ack_flush(){
	*ack_flush_reg = 1;
}

inline void auto_tx_en(uint8_t en){
	uint32_t radio_ctl = *radio_ctl_reg;
	*radio_ctl_reg = ((radio_ctl & (~AUTO_TX_MASK)) | ((en&0x1)<<AUTO_TX_BIT_POS));
}

// 0: address doesn't match
// 1: match
// 3: broadcast
uint8_t dest_addr_filter(struct rx_packet_str* rp, struct radio* r0){
	uint8_t addr_match = 0;
	uint16_t radio_src_pan_ID = r0->src_pan_ID;
	uint32_t radio_src_addr_LSB = r0->src_addr_LSB;
	uint32_t radio_src_addr_MSB = r0->src_addr_MSB;
	uint8_t pkt_dest_pan_ID = rp->dest_pan_ID;

	if (pkt_dest_pan_ID==radio_src_pan_ID){
		uint32_t pkt_dest_addr_LSB = rp->dest_addr_LSB;
		uint8_t address_mode = rp->dest_addr_mode;
		// 2 bytes for address
		if (address_mode==2){
			pkt_dest_addr_LSB = (pkt_dest_addr_LSB & 0x0000ffff);
			// broadcast or glossy
			if ((pkt_dest_addr_LSB==0xffff)||(pkt_dest_addr_LSB==0xfffe))
				addr_match = 3;	// address match;
			else if (pkt_dest_addr_LSB == (radio_src_addr_LSB & 0x0000ffff))
				addr_match = 1;	// address match;
			else if (r0->multiple_addr_en)
				addr_match = address_checker(r0, 2, 0, pkt_dest_addr_LSB);	// address match;
		}
		// 8 bytes for address
		else if (address_mode==3){
			uint32_t pkt_dest_addr_MSB = rp->dest_addr_MSB;
			// broadcast or glossy
			if (((pkt_dest_addr_LSB==0xffffffff)||(pkt_dest_addr_LSB==0xfffffffe))&&(pkt_dest_addr_MSB==0xffffffff))
				addr_match = 3;	// address match;
			else if ((pkt_dest_addr_MSB==radio_src_addr_MSB) && (pkt_dest_addr_LSB==radio_src_addr_LSB))
				addr_match = 1;	// address match;
			else if (r0->multiple_addr_en)
				addr_match = address_checker(r0, 3, pkt_dest_addr_MSB, pkt_dest_addr_LSB);	// address match;
		}
	}
	return addr_match;
}

inline void auto_ack_en(uint8_t ack_en){
	// read out register first
	uint32_t radio_ctl = *radio_ctl_reg;
	radio_ctl = (radio_ctl & (~ACK_EN_MASK)) | ((ack_en & 0x1)<<ACK_EN_BIT_POS);
	*radio_ctl_reg = radio_ctl;
}

// 11 for continues AGC loop
// 10 or 01 for SFD lock
// 00 for off AGC
inline void rx_agc_en(uint8_t agc_mode){
	// read out register first
	uint32_t radio_ctl = *radio_ctl_reg;
	radio_ctl = (radio_ctl & (~AGC_MODE_MASK)) | ((agc_mode & 0x3)<<AGC_MODE_BIT_POS);
	*radio_ctl_reg = radio_ctl;
	if (agc_mode)
		enRXParVGAGainCtl();
	else{
		setLNAGain(2);
		setRXBaseBandVGAGain(20);
	}
}

// shdn = 1, turn off the radio
// shdn = 0, turn on the radio
void RF_shdn(uint8_t shdn){
	// read out register first
	uint32_t radio_ctl = *radio_ctl_reg;
	radio_ctl = (radio_ctl & (~RADIO_OFF_MASK)) | ((shdn & 0x1)<<RADIO_OFF_BIT_POS);
	*radio_ctl_reg = radio_ctl;
}

inline void tx_fire(){
	uint8_t radio_mode;
	do {
		radio_mode = get_radio_mode();
	}
	while(radio_mode!=0);
	MSS_GPIO_set_output( MSS_GPIO_7, 1 );
	MSS_GPIO_set_output( MSS_GPIO_7, 0 );
}


uint8_t get_radio_mode(){
	// read out register first
	uint32_t radio_ctl = *radio_ctl_reg;
	radio_ctl = ((radio_ctl & RADIO_MODE_MASK)>>RADIO_MODE_BIT_POS);
	return radio_ctl;
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
	MSS_GPIO_config( MSS_GPIO_7, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_16, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_19, MSS_GPIO_INPUT_MODE);	// ready to send data
	MSS_GPIO_config( MSS_GPIO_17, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_22, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_23, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_24, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_25, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_26, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_27, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_31, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_disable_irq( MSS_GPIO_16 );				// Length interrupt
	MSS_GPIO_enable_irq( MSS_GPIO_17 );					// packet received interrupt
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
