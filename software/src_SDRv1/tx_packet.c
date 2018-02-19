
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

#include "tx_packet.h"
#include "radio_config.h"
#include "drivers/mss_pdma/mss_pdma.h"

const uint32_t * dac_ptr = (uint32_t *)(0x40060000);


extern uint8_t PDMA_int;
extern void PDMA_start(pdma_channel_id_t, uint32_t, uint32_t, uint16_t);

uint8_t data_trans (struct tx_packet_str* tp){
	// set the payload size
	// 2 bytes FCF, 1 byte DSN, 20 bytes addr (maximum)
	// n bytes payload, 2 bytes FCS

	uint32_t tx_pkt_ptr[24] = {0};

	uint8_t result = calculate_address(tp, tx_pkt_ptr);
	if (result==0)
		return 0;

	if ((tp->pkt_overhead + tp->data_length)>122)
		return 0;
	else
		tx_pkt_ptr[0] = tp->pkt_overhead + tp->data_length + 5;	// 2 bytes FCF, 1 byte DSN, 2bytes FCS

	calculate_FCF(tp, tx_pkt_ptr);

	PDMA_int = 0;
	PDMA_start
	(
	    PDMA_CHANNEL_1,
	    (uint32_t)tx_pkt_ptr,
	    (uint32_t)dac_ptr,
	    (uint16_t)(tp->pkt_overhead+4)
	);

    do
    {
    }while (PDMA_int==0);

	PDMA_int = 0;

	PDMA_start
	(
	    PDMA_CHANNEL_1,
	    (uint32_t)tp->data_ptr,
	    (uint32_t)dac_ptr,
	    (uint16_t)tp->data_length
	);
	
	return 1;

}

// 0 -> Beacon
// 1 -> Data
// 2 -> Acknowledgment
// 3 -> Mac Command
inline void set_frame_type(struct tx_packet_str* tp, uint8_t ft){
	tp->frame_type = (ft&0x3);
}

inline void set_security(struct tx_packet_str* tp, uint8_t se){
	tp->security_en = (se&0x1);
}

inline void set_frame_pending(struct tx_packet_str* tp, uint8_t fp){
	tp->frame_pending = (fp&0x1);
}

// 0 -> no ack
// 1 -> ack
inline void set_ack(struct tx_packet_str* tp, uint8_t ar){
	tp->ack_req = (ar&0x1);
}

inline void set_pan_id_comp(struct tx_packet_str* tp, uint8_t ip){
	tp->intra_pan = (ip&0x1);
}

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline uint8_t set_dest_addr_mode(struct tx_packet_str* tp, uint8_t dam){
	uint8_t temp = (dam&0x3);
	if (temp==1)
		return 0;
	else
		tp->dest_addr_mode = temp;
	return 1;
}

inline void set_DSN(struct tx_packet_str* tp, uint8_t DSN){
	tp->dsn = DSN;
}

inline void set_dest_pan(struct tx_packet_str* tp, uint16_t dp){
	tp->dest_pan_ID = dp;
}

// da1 = higher 32 bytes, only used in 64 bytes destination address mode
// da0 = lower 32 bytes
inline void set_dest_addr(struct tx_packet_str* tp, uint32_t da1, uint32_t da0){
	tp->dest_addr_MSB = da1;
	tp->dest_addr_LSB = da0;
}

void calculate_FCF(struct tx_packet_str* tp, uint32_t* tx_pkt_ptr){
	struct radio* r1 = tp->rp;
	/*
	tx_pkt_ptr[1] = tp->frame_type;
	tx_pkt_ptr[1] = (tx_pkt_ptr[1] & 0xf7) + (tp->security_en<<3);
	tx_pkt_ptr[1] = (tx_pkt_ptr[1] & 0xef) + (tp->prame_pending<<4);
	tx_pkt_ptr[1] = (tx_pkt_ptr[1] & 0xdf) + (tp->ack_req<<5);
	tx_pkt_ptr[1] = (tx_pkt_ptr[1] & 0xbf) + (tp->intra_pan<<6);
	tx_pkt_ptr[2] = tp->dest_addr_mode<<2;
	tx_pkt_ptr[2] = (tx_pkt_ptr[2] & 0x3f) + (r1->src_addr_mode<<6);
	*/
	tx_pkt_ptr[1] = 0;
	tx_pkt_ptr[2] = 0;
	tx_pkt_ptr[1] = (tp->frame_type) + (tp->security_en<<3) + (tp->frame_pending<<4) + (tp->ack_req<<5) + (tp->intra_pan<<6);
	tx_pkt_ptr[2] = (tp->dest_addr_mode<<2) + (r1->src_addr_mode<<6);
	tx_pkt_ptr[3] = tp->dsn;
}

uint8_t calculate_address(struct tx_packet_str* tp, uint32_t* tx_pkt_ptr){
	
	tp->pkt_overhead = 0;
	
	uint32_t * src_addr_ptr;
	struct radio* r1 = tp->rp;

	// no destination address present
	if (tp->dest_addr_mode==0){
		if (tp->intra_pan)
			return 0;
		else{
			if (r1->src_addr_mode==0)
				return 1;
			else{
				tx_pkt_ptr[4] = (r1->src_pan_ID & 0x00ff);
				tx_pkt_ptr[5] = ((r1->src_pan_ID & 0xff00)>>8);
				src_addr_ptr = &tx_pkt_ptr[6];
				tp->pkt_overhead += 2;
			}
		}
	}
	else{
		if ((r1->src_addr_mode==0) && (tp->intra_pan))
			return 0;
		tx_pkt_ptr[4] = (tp->dest_pan_ID & 0x00ff);
		tx_pkt_ptr[5] = (tp->dest_pan_ID & 0xff00)>>8;
		tx_pkt_ptr[6] = tp->dest_addr_LSB & 0x000000ff;
		tx_pkt_ptr[7] = (tp->dest_addr_LSB & 0x0000ff00)>>8;
		tp->pkt_overhead += 4;

		if (tp->dest_addr_mode==2){
			if (tp->intra_pan){
				src_addr_ptr = &tx_pkt_ptr[8];
			}
			else{
				if (r1->src_addr_mode){
					tx_pkt_ptr[8] = (r1->src_pan_ID & 0x00ff);
					tx_pkt_ptr[9] = (r1->src_pan_ID & 0xff00)>>8;
					src_addr_ptr = &tx_pkt_ptr[10];
					tp->pkt_overhead += 2;
				}
				else
					return 1;
			}
		}
		else{
			tx_pkt_ptr[8] = (tp->dest_addr_LSB & 0x00ff0000)>>16;
			tx_pkt_ptr[9] = (tp->dest_addr_LSB & 0xff000000)>>24;
			tx_pkt_ptr[10] = tp->dest_addr_MSB & 0x000000ff;
			tx_pkt_ptr[11] = (tp->dest_addr_MSB & 0x0000ff00)>>8;
			tx_pkt_ptr[12] = (tp->dest_addr_MSB & 0x00ff0000)>>16;
			tx_pkt_ptr[13] = (tp->dest_addr_MSB & 0xff000000)>>24;
			tp->pkt_overhead += 6;
			if (tp->intra_pan){
				src_addr_ptr = &tx_pkt_ptr[14];
			}
			else{
				if (r1->src_addr_mode){
					tx_pkt_ptr[14] = (r1->src_pan_ID & 0x00ff);
					tx_pkt_ptr[15] = (r1->src_pan_ID & 0xff00)>>8;
					src_addr_ptr = &tx_pkt_ptr[16];
					tp->pkt_overhead += 2;
				}
				else
					return 1;
			}
		}
	}
	src_addr_ptr[0] = (r1->src_addr_LSB & 0x000000ff);
	src_addr_ptr[1] = (r1->src_addr_LSB & 0x0000ff00)>>8;
	tp->pkt_overhead += 2;
	if (r1->src_addr_mode==3){
		src_addr_ptr[2] = (r1->src_addr_LSB & 0x00ff0000)>>16;
		src_addr_ptr[3] = (r1->src_addr_LSB & 0xff000000)>>24;
		src_addr_ptr[4] = (r1->src_addr_MSB & 0x000000ff);
		src_addr_ptr[5] = (r1->src_addr_MSB & 0x0000ff00)>>8;
		src_addr_ptr[6] = (r1->src_addr_MSB & 0x00ff0000)>>16;
		src_addr_ptr[7] = (r1->src_addr_MSB & 0xff000000)>>24;
		tp->pkt_overhead += 6;
	}
	return 1;
}
