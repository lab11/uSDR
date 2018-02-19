
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
#include "rx_packet.h"
#include "maxim2831.h"
#include "mss_sdr.h"
#include "amac.h"

static unsigned int * const radio_ctl_reg = (unsigned int *) (0x40070000); // 4bits radio_mode, 1bit ack_en, 2bits agc_mode, 1bit radio_off
static unsigned int * const ack_flush_reg = (unsigned int *) (0x40070050);

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
void set_multiple_address(struct radio* r0, unsigned char en, unsigned char nu, unsigned int* map){
	r0->multiple_addr_en = (en&0x1);
	r0->num_of_multiple_addr = (nu&0x7);
	r0->multiple_addr_ptr = map;
}

inline void set_src_pan(struct radio* r0, unsigned short sp){
	r0->src_pan_ID = sp;
}

// sa1 = higher 32 bytes, only used in 64 bytes source address mode
// sa0 = lower 32 bytes
inline void set_src_addr(struct radio* r0, unsigned int sa1, unsigned int sa0){
	r0->src_addr_MSB = sa1;
	r0->src_addr_LSB = sa0;
}

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline unsigned char set_src_addr_mode(struct radio* r0, unsigned char sam){
	unsigned char temp = (sam&0x3);
	if (sam==1)
		return 0;
	else
		r0->src_addr_mode = temp;
	return 1;
}

unsigned char address_checker(struct radio* r0, unsigned char address_mode, unsigned int MSB, unsigned int LSB){
	unsigned char i;
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

inline void auto_tx_en(unsigned char en){
	unsigned int radio_ctl = *radio_ctl_reg;
	*radio_ctl_reg = ((radio_ctl & (~AUTO_TX_MASK)) | ((en&0x1)<<AUTO_TX_BIT_POS));
}

// 0: address doesn't match
// 1: match
// 3: broadcast
unsigned char dest_addr_filter(struct rx_packet_str* rp, struct radio* r0){
	unsigned char addr_match = 0;
	unsigned short radio_src_pan_ID = r0->src_pan_ID;
	unsigned int radio_src_addr_LSB = r0->src_addr_LSB;
	unsigned int radio_src_addr_MSB = r0->src_addr_MSB;
	unsigned char pkt_dest_pan_ID = rp->dest_pan_ID;

	if (pkt_dest_pan_ID==radio_src_pan_ID){
		unsigned int pkt_dest_addr_LSB = rp->dest_addr_LSB;
		unsigned char address_mode = rp->dest_addr_mode;
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
			unsigned int pkt_dest_addr_MSB = rp->dest_addr_MSB;
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

inline void auto_ack_en(unsigned char ack_en){
	// read out register first
	unsigned int radio_ctl = *radio_ctl_reg;
	radio_ctl = (radio_ctl & (~ACK_EN_MASK)) | ((ack_en & 0x1)<<ACK_EN_BIT_POS);
	*radio_ctl_reg = radio_ctl;
}

// 11 for continues AGC loop
// 10 or 01 for SFD lock
// 00 for off AGC
inline void rx_agc_en(unsigned char agc_mode){
	// read out register first
	unsigned int radio_ctl = *radio_ctl_reg;
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
void RF_shdn(unsigned char shdn){
	// read out register first
	unsigned int radio_ctl = *radio_ctl_reg;
	radio_ctl = (radio_ctl & (~RADIO_OFF_MASK)) | ((shdn & 0x1)<<RADIO_OFF_BIT_POS);
	*radio_ctl_reg = radio_ctl;
}

inline void tx_fire(){
	/*
	unsigned char radio_mode;
	do {
		radio_mode = get_radio_mode();
	}
	while(radio_mode!=0);
	*/
	unsigned int gpio_in;
	do{
		gpio_in = MSS_GPIO_get_input(MSS_GPIO_19);
	}
	while(gpio_in==0);
	MSS_GPIO_set_output( MSS_GPIO_7, 1 );
	MSS_GPIO_set_output( MSS_GPIO_7, 0 );
}


unsigned char get_radio_mode(){
	// read out register first
	unsigned int radio_ctl = *radio_ctl_reg;
	radio_ctl = ((radio_ctl & RADIO_MODE_MASK)>>RADIO_MODE_BIT_POS);
	return radio_ctl;
}

void init_system()
{
	//MSS_WD_disable();

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

	//GPIO
	// gpio 8~15 = on board gpio
	// gpio 16 = length interrupt
	// gpio 17 = packet rx complete interrupt
	// gpio 19 = ready to send data, (radio_ctl mode = idle listening)
	// gpio 23 = SFD interrupt
	// gpio 24~27 = SW1~SW4 interrupt
	// gpio 31 = tx complete

	MSS_GPIO_config( MSS_GPIO_7, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_8, MSS_GPIO_OUTPUT_MODE );
	MSS_GPIO_config( MSS_GPIO_9, MSS_GPIO_OUTPUT_MODE );
	//MSS_GPIO_config( MSS_GPIO_16, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_19, MSS_GPIO_INPUT_MODE);	
	MSS_GPIO_config( MSS_GPIO_17, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	//MSS_GPIO_config( MSS_GPIO_23, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_24, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_25, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_26, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_27, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	//MSS_GPIO_config( MSS_GPIO_31, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );

	// RF registers default
	initialize_chip();
}
