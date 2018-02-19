
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
// Last modified: Jul. 4, '12 by Ye-sheng

#ifndef MAXIM2831_H_
#define MAXIM2831_H_

#include "CMSIS/a2fxxxm3.h"

/*
 * The function initialize_chip() sets all the registers in the MAX2831 
 * chip to their recommended settings. This function needs to be executed 
 * at the start of the main file. This is functions calls the setToDefaultRX
 * function for each register. It is important to note that it must be 
 * called every time at the start of the main to synchronize between the 
 * status register and the actual registers in the MAX2831 chip.
 *
 * Example : 	void main(void){
 * 					initialize_chip();
 * 			 		//Other tasks
 * 				}
 */
void initialize_chip();
//---------------------------Common Functions for all Registers:

/*
 * The setToDefaultRX function returns a register to its recommended 
 * value as per the datasheet. It takes as argument , the register number 
 * and sends an 18-bit SPI transmission which sets the corresponding 
 * register to its default recommended value. 
 * Example: setToDefaultRX(3); Results in the transmission: 
 * 0x793 which is the recommended value for R3.
 */
void setToDefaultRX(int register_number);

/*
 * The setRegisterValueRX function takes as argument te register to be 
 * programmed, and the value to be included in the register.IT is 
 * important to note that the register value is 18 BITS and INCLUDES THE
 * 4-BIT ADDRESS OF THE REGISTER. All 18-bits need to be programmed.
 * This function has been provided to add flexibility to the code.
 * It is ADVISED to use individual functions to safely manipulate values 
 * in the registers.
 * Only the 18 LSB bits are considered. Rest are discarded.
 * Example: setRegisterValueRX(3,0x45553);Results in transmission of 0x05553. 
 * The 4 is discarded.
 */
void setRegisterValueRX(int register_number,uint32_t register_value);
uint32_t getRegisterValueRX(int reg_number);

/*
 High LNAGain  = 3;
 Medium LNAGain = 2;
 Low LNAGain = 1 or 0;
*/
uint8_t setTXBaseBandVGAGain(uint8_t VGAGain);
uint8_t setRXBaseBandVGAGain(uint8_t VGAGain);
void enRXParVGAGainCtl();
uint8_t setLNAGain(uint8_t LNAGain);

/*
The 40MHZ reference clock is divided by 1 or 2 to generate a compare frequency. 
And then LO Frequency Divider = F(RF)/ F(comp). and the Frequency divdier is 
sent to the related register. See Table 11 and Table 12 for details. 
*/
uint8_t setFreqDivider(int freq);
int getFreqDivider();

/*
Low pass filter setting: The 3dB cut-off frequency = LPFMode * LPFFineAdjust

	Receiver: 
		LPFmode 0, 7.5MHZ	(11b)
		LPFmode 1, 8.5MHZ	(11g)
		LPFmode 2, 15MHZ	(Turbo 1)
		LPFmode 3, 18MHZ	(Turbo 2)
		
		LPFFineAdjust 0,	(90%)
		LPFFineAdjust 1,	(95%)
		LPFFineAdjust 2,	(100%)
		LPFFineAdjust 3,	(105%)
		LPFFineAdjust 4,	(110%)
	  
	Transmitter:
	
		LPFmode 0, 8MHZ		(11b)
		LPFmode 1, 11MHZ	(11g)
		LPFmode 2, 16.5MHZ	(Turbo 1)
		LPFmode 3, 22.5MHZ	(Turbo 2)
		
		LPFFineAdjust 0,	(90%)
		LPFFineAdjust 1, 	(95%)
		LPFFineAdjust 2,	(100%)
		LPFFineAdjust 3,	(105%)
		LPFFineAdjust 4,	(110%)
		LPFFineAdjust 5,	(115%)
*/

uint8_t setBaseBandLowPassFilterMode (uint8_t LPFMode);
uint8_t setTXBaseBandLowPassFilterModeFineAdjust (uint8_t LPFMode, uint8_t LPFFineAdjust);
uint8_t setRXBaseBandLowPassFilterModeFineAdjust (uint8_t LPFMode, uint8_t LPFFineAdjust);


// HighPass Filter

/*
RXHP should be 0 to use HighPassFilterMode Func

Only Receiver: 
  100 Hz : HPFMode 0
  4K : HPFMode 1
  30K: HPFMode 2 
	 
*/
/*
bool setBaseBandHighPassFilterMode(int HPFMode);
int getBaseBandHighPassFilterMode();
*/



#endif
