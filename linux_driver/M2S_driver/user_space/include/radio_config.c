
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
 * Last update: 9/25/2013
 * Last update content:
 * Adopt to M2S
 */

#include "radio_config.h"
#include "rx_packet.h"
#include "maxim2831.h"
#include "../user_driver/mss_pdma.h"
#include "m2s_sdr.h"
#include "sdr_def.h"

static unsigned int * const radio_ctl_reg = (unsigned int *) (FPGA_FABRIC_BASE + 0x20000); // 4bits radio_mode, 1bit ack_en, 2bits agc_mode, 1bit radio_off
static unsigned int * const ack_flush_reg = (unsigned int *) (FPGA_FABRIC_BASE + 0x20050);

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
	unsigned int gpio_in;
	M2S_GPIO_Type ready = READY_TO_TRANSMIT;
	M2S_GPIO_Type fire = TX_FIRE;
	do{
		gpio_in = gpio_get_value(fd, ready);
	}
	while(gpio_in==0);
	gpio_set_value(fd, fire, 1);
	gpio_set_value(fd, fire, 0);
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

	M2S_GPIO_Type io_tx_fire = TX_FIRE;
	M2S_GPIO_Type io_ready_to_transmit = READY_TO_TRANSMIT;
	M2S_GPIO_Type io_sfd_interrupt = SFD_INTERRUPT;
	M2S_GPIO_Type io_length_interrupt = LENGTH_INTERRUPT;
	M2S_GPIO_Type io_tx_completed = TX_COMPLETED;
	M2S_GPIO_Type io_packet_done = PACKET_DONE;
	
	if ((gpio_set_direction(fd, io_tx_fire, DIR_OUTPUT))<0)
		perror("unable to configure TX_FIRE\r\n");
		
	if ((gpio_set_direction(fd, io_ready_to_transmit, DIR_INPUT))<0)
		perror("unable to configure READY_TO_TRANSMIT\r\n");

	if ((gpio_set_direction(fd, io_sfd_interrupt, DIR_INPUT))<0)
		perror("unable to configure SFD_INTERRUPT\r\n");

	if ((gpio_set_direction(fd, io_length_interrupt, DIR_INPUT))<0)
		perror("unable to configure LENGTH_INTERRUPT\r\n");

	if ((gpio_set_direction(fd, io_tx_completed, DIR_INPUT))<0)
		perror("unable to configure LENGTH_INTERRUPT\r\n");

	if ((gpio_set_direction(fd, io_packet_done, DIR_INPUT))<0)
		perror("unable to configure LENGTH_INTERRUPT\r\n");


	// RF registers default
	initialize_chip();
}
