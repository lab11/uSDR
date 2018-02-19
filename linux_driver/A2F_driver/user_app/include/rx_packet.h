
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

#ifndef _RX_PACKET_H_
#define _RX_PACKET_H_

struct rx_packet_str{
	unsigned char frame_type;
	unsigned char security_en;
	unsigned char frame_pending;
	unsigned char ack_req;
	unsigned char intra_pan;
	unsigned char dest_addr_mode;
	unsigned char dsn;
	unsigned short dest_pan_ID;
	unsigned int dest_addr_LSB, dest_addr_MSB;

	unsigned char packet_length;
	unsigned int payload_idx;

	unsigned char src_addr_mode;
	unsigned short src_pan_ID;
	unsigned int src_addr_LSB, src_addr_MSB;
	unsigned char rssi;
	unsigned char crc;
};

// check pending data in received fifo
inline unsigned char isFifoEmpty(void);

// read fifo out using dma
int read_fifo(unsigned int* rdata);

// create a received packet, frame filtering
unsigned char rx_packet_create(struct rx_packet_str* rp, unsigned int* rx_data);

#endif
