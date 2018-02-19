
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

#ifndef TX_PACKET_H_
#define TX_PACKET_H_

#include <stdio.h>
#include <stdlib.h>
#include "CMSIS/a2fxxxm3.h"

struct tx_packet_str{
	uint8_t frame_type;
	uint8_t security_en;
	uint8_t frame_pending;
	uint8_t ack_req;
	uint8_t intra_pan;
	uint8_t dest_addr_mode;
	uint8_t dsn;
	uint16_t dest_pan_ID;
	uint32_t dest_addr_LSB, dest_addr_MSB;
	uint8_t pkt_overhead;

	uint8_t data_length;
	uint32_t* data_ptr;
	struct radio* rp;
};

uint8_t data_trans (struct tx_packet_str* tp);

// 0 -> Beacon
// 1 -> Data
// 2 -> Acknowledgment
// 3 -> Mac Command
inline void set_frame_type(struct tx_packet_str* tp, uint8_t ft);

inline void set_security(struct tx_packet_str* tp, uint8_t se);

// 1 -> pending
// 0 -> no pending
inline void set_frame_pending(struct tx_packet_str* tp, uint8_t fp);

// 0 -> no ack
// 1 -> ack
inline void set_ack(struct tx_packet_str* tp, uint8_t ar);

inline void set_pan_id_comp(struct tx_packet_str* tp, uint8_t ip);

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline uint8_t set_dest_addr_mode(struct tx_packet_str* tp, uint8_t dam);

inline void set_DSN(struct tx_packet_str* tp, uint8_t DSN);

inline void set_dest_pan(struct tx_packet_str* tp, uint16_t dp);

inline void set_dest_addr(struct tx_packet_str* tp, uint32_t da1, uint32_t da0);

void calculate_FCF(struct tx_packet_str* tp, uint32_t* tx_pkt_ptr);

uint8_t calculate_address(struct tx_packet_str* tp, uint32_t* tx_pkt_ptr);

#endif
