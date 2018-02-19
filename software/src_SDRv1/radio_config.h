
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

#ifndef RADIO_CONFIG_H_ 
#define RADIO_CONFIG_H_ 

#include <stdio.h>
#include <stdlib.h>
#include "CMSIS/a2fxxxm3.h"
#include "amac.h"

struct radio{
	uint8_t multiple_addr_en;
	uint8_t num_of_multiple_addr;
	uint32_t* multiple_addr_ptr;
	uint16_t src_pan_ID;
	uint8_t src_addr_mode;
	uint32_t src_addr_LSB, src_addr_MSB;
	#ifdef AMAC
	// Only for AMAC src
	#ifdef TX
	enum state{
		startup = 0,
		sleep	= 1,
		listen	= 2,
		receive	= 3,
		transmit= 4
	} radio_state;
	uint8_t pending_data_nu;			// test variables
	uint32_t pending_dest_addr;
	uint8_t sync;
	uint32_t tx_counter;
	uint32_t received_timer;
	#else
	enum state{
		startup = 0,
		sleep	= 1,
		listen	= 2,
		receive	= 3,
		probe1	= 5,
		probe2	= 6,
		transmit= 7
	} radio_state;
	uint32_t probe_timer;
	uint16_t probe_counter;
	#endif
	#endif
};

void init_system();

uint8_t read_mode();		// 1 for TX, 0 for RX

void set_mode(uint8_t mode_in);	// 1 for TX, 0 for RX

inline void tx_fire();

void RF_shdn(uint8_t shdn);	// 1 shutdown RF+DAC, 0 up RF+DAC

void DAC_shdn(uint8_t shdn);	// 1 shutdown DAC, 0 up DAC

void ADC_shdn(uint8_t shdn); 	// 1 shutdown ADC, 0 up ADC

inline void auto_ack_en(uint8_t ack_en);

// 11 for continues AGC loop
// 10 or 01 for SFD lock
// 00 for off AGC
inline void rx_agc_en(uint8_t agc_mode);

inline void ack_flush();

uint8_t frame_filter(	struct radio* r0, uint8_t* frame_type, uint8_t* crc_correct, uint8_t* ack_req, 
						uint8_t* address_match, uint8_t* addr_mode, uint32_t* dest_addr, uint32_t* src_addr);

void set_multiple_address(struct radio* r0, uint8_t en, uint8_t nu, uint32_t* map);

uint8_t address_checker(struct radio* r0, uint8_t address_mode, uint32_t MSB, uint32_t LSB);

// 0 -> neither pan nor addr present
// 2 -> 2 bytes addr
// 3 -> 8 bytes addr
inline uint8_t set_src_addr_mode(struct radio* r0, uint8_t sam);

inline void set_src_pan(struct radio* r0, uint16_t sp);

inline void set_src_addr(struct radio* r0, uint32_t sa1, uint32_t sa0);

#endif
