
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
 * Last update: 8/22/2012
 */

#include "rx_packet.h"
#include "sdr_def.h"

static unsigned int* const RXData_ptr	= (unsigned int*)FPGA_FABRIC_BASE;
static unsigned int* const crc_fcf_reg = (unsigned int*)(FPGA_FABRIC_BASE + 0xc);

#define FIFO_EMPTY_MASK 0x40000
#define FIFO_EMPTY_BIT_POS 18


inline unsigned char isFifoEmpty(){
	unsigned int fifo_reg = *crc_fcf_reg;
	fifo_reg = ((fifo_reg & FIFO_EMPTY_MASK)>>(FIFO_EMPTY_BIT_POS));
	return (unsigned char)fifo_reg;
}

int read_fifo(unsigned int* rdata){
	unsigned int rx_pkt_length;
	rx_pkt_length = *RXData_ptr;
	if (rx_pkt_length==0)
		return -1;


	*rdata = rx_pkt_length;


    PDMA_start
    (
		PDMA_CHANNEL_0,
		(unsigned int)RXData_ptr,
		(unsigned int)(rdata + 1),
		(unsigned short)rx_pkt_length
    );


/*
    unsigned int status_pdma = 0;
    do
    {
        status_pdma = PDMA_status (PDMA_CHANNEL_0);
    }while (status_pdma == 0);
	*/

	return 0;
}

unsigned char rx_packet_create(struct rx_packet_str* rp, unsigned int* rx_data){
	rp->packet_length = rx_data[0];
	rp->frame_type = rx_data[1] & 0x7;
	rp->security_en = (rx_data[1] & 0x8)>>3;
	rp->frame_pending = (rx_data[1] & 0x10)>>4;
	rp->ack_req = (rx_data[1] & 0x20)>>5;
	rp->intra_pan = (rx_data[1] & 0x40)>>6;
	rp->dest_addr_mode = (rx_data[2] & 0xc)>>2;
	rp->src_addr_mode = (rx_data[2] & 0xc0)>>6;
	rp->dsn = rx_data[3];
	rp->crc = rx_data[(rx_data[0])];
	rp->rssi = rx_data[(rx_data[0]-1)];

	unsigned char src_pan_id_head; 
	unsigned char dest_pan_id_present = 0;
	unsigned char src_pan_id_present = 0;
	unsigned char src_addr_present = 0;
	unsigned char src_addr_head;
	unsigned char payload_head = 4;

	if (rx_data[0]>5){
		if (rp->intra_pan){
			dest_pan_id_present = 1;
			src_addr_present = 1;
			// both address mode cannot not be zero
			if ((rp->dest_addr_mode) && (rp->src_addr_mode)){
				if (rp->dest_addr_mode==2)
					src_addr_head = 8;
				else
					src_addr_head = 14;
			}
			else
				return 0;
			
		}
		// no intra pan
		else {
			if (rp->dest_addr_mode){
				dest_pan_id_present = 1;
				if (rp->src_addr_mode){
					src_pan_id_present = 1;
					src_addr_present = 1;
					if (rp->dest_addr_mode==2)
						src_pan_id_head = 8;
					else
						src_pan_id_head = 14;
					src_addr_head = src_pan_id_head + 2;
				}
			}
			else{
				if (rp->src_addr_mode){
					src_pan_id_present = 1;
					src_addr_present = 1;
					src_pan_id_head = 4;
					src_addr_head = 6;
				}
			}
		}
	
		if (dest_pan_id_present){
			rp->dest_pan_ID = ((rx_data[5]<<8) | rx_data[4]);
			payload_head += 2;
			// 2 bytes for destination address
			if (rp->dest_addr_mode==2){
				rp->dest_addr_LSB = ((rx_data[7]<<8) | rx_data[6]);
				payload_head += 2;
			}
			else{
				rp->dest_addr_LSB = ((rx_data[9]<<24) | (rx_data[8]<<16) | (rx_data[7]<<8) | rx_data[6]);
				rp->dest_addr_MSB = ((rx_data[13]<<24) | (rx_data[12]<<16) | (rx_data[11]<<8) | rx_data[10]);
				payload_head += 8;
			}
		}
	
		if (src_pan_id_present){
			rp->src_pan_ID = ((rx_data[(src_pan_id_head+1)]<<8) | rx_data[src_pan_id_head]);
			payload_head += 2;
		}
	
		if (src_addr_present){
			if (rp->src_addr_mode==2){
				rp->src_addr_LSB = ((rx_data[(src_addr_head+1)]<<8) | rx_data[src_addr_head]);
				payload_head += 2;
			}
			else{
				rp->dest_addr_LSB = ((rx_data[(src_addr_head+3)]<<24) | (rx_data[(src_addr_head+2)]<<16) | (rx_data[(src_addr_head+1)]<<8) | rx_data[src_addr_head]);
				rp->dest_addr_MSB = ((rx_data[(src_addr_head+7)]<<24) | (rx_data[(src_addr_head+6)]<<16) | (rx_data[(src_addr_head+5)]<<8) | rx_data[(src_addr_head+4)]);
				payload_head += 8;
			}
		}
		rp->payload_idx = payload_head;
	}
	return 1;
}
