
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

#ifndef _RADIO_CONFIG_H_ 
#define _RADIO_CONFIG_H_ 

#include "rx_packet.h"


struct radio{
	unsigned char multiple_addr_en;
	unsigned char num_of_multiple_addr;
	unsigned int* multiple_addr_ptr;
	unsigned short src_pan_ID;
	unsigned char src_addr_mode;
	unsigned int src_addr_LSB, src_addr_MSB;
	#ifdef AMAC
	// Only for AMAC src
	#ifdef TX
	enum state{
		startup = 0,
		off		= 1,
		listen	= 2,
		receive	= 3,
		transmit= 4
	} radio_state;
	unsigned char pending_data_nu;			// test variables
	unsigned int pending_dest_addr;
	unsigned char sync;
	unsigned int tx_counter;
	unsigned int received_timer;
	#else
	enum state{
		startup = 0,
		off		= 1,
		listen	= 2,
		receive	= 3,
		probe1	= 5,
		probe2	= 6,
		transmit= 7
	} radio_state;
	unsigned int probe_timer;
	unsigned short probe_counter;
	unsigned int contention_window;
	#endif
	#endif
};


void init_system();

inline void tx_fire();

void RF_shdn(unsigned char shdn);	// 1 shutdown RF+DAC, 0 up RF+DAC

inline void auto_ack_en(unsigned char ack_en);

// 11 for continues AGC loop
// 10 or 01 for SFD lock
// 00 for off AGC
inline void rx_agc_en(unsigned char agc_mode);

inline void ack_flush();

inline void auto_tx_en(unsigned char en);

unsigned char get_radio_mode();

unsigned char dest_addr_filter(struct rx_packet_str* rp, struct radio* r0);

void set_multiple_address(struct radio* r0, unsigned char en, unsigned char nu, unsigned int* map);

unsigned char address_checker(struct radio* r0, unsigned char address_mode, unsigned int MSB, unsigned int LSB);

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline unsigned char set_src_addr_mode(struct radio* r0, unsigned char sam);

inline void set_src_pan(struct radio* r0, unsigned short sp);

inline void set_src_addr(struct radio* r0, unsigned int sa1, unsigned int sa0);

#endif
