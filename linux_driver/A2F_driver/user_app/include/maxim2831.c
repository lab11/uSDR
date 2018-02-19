
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

#include "maxim2831.h"
#include "a2f/a2fm3.h"


#define MAX2831_BASE (FPGA_FABRIC_BASE + 0x30000)

typedef struct
{
    unsigned int reg0;
    unsigned int reg1;
    unsigned int reg2;
    unsigned int reg3;
    unsigned int reg4;
    unsigned int reg5;
    unsigned int reg6;
    unsigned int reg7;
    unsigned int reg8;
    unsigned int reg9;
    unsigned int reg10;
    unsigned int reg11;
    unsigned int reg12;
    unsigned int reg13;
    unsigned int reg14;
    unsigned int reg15;
} max2831_t;

#define MAX2831 ((max2831_t *) MAX2831_BASE)


unsigned int reg_status[16] = {	0x0000740,0x000119A,0x0001003,0x0000079,
							0x0003666,0x00000A4,0x0000060,0x0001022,
							0x0002021,0x00003B5,0x0001DA4,0x000007F,
							0x0000140,0x0000E92,0x000033B,0x0000145}; 

//Initialize the MAX chip. This function MUST be called before performing other functions.
void initialize_chip()
{
	int i=0;
	for(i=0;i<16;i++)
	{
		setToDefaultRX(i);	
	}
}


//Sets to Recommended values as shown in Table 14 of DataSheet; 
//Argument is Register Number we wish to return to default.
void setToDefaultRX(int register_number) 
{
	switch (register_number){
	case 0:
		MAX2831->reg0 = 0x0000740;
		reg_status[0] = 0x0000740;
	break;
	case 1:
		MAX2831->reg1 = 0x000119A;
		reg_status[1] = 0x000119A;
	break;
	case 2:
		MAX2831->reg2 = 0x0001003;
		reg_status[2] = 0x0001003;
	break;
	case 3:
		MAX2831->reg3 = 0x0000079;
		reg_status[3] = 0x0000079;
	break;
	case 4:
		MAX2831->reg4 = 0x0003666;
		reg_status[4] = 0x0003666;
	break;
	case 5:
		MAX2831->reg5 = 0x00000A4;
		reg_status[5] = 0x00000A4;
	break;
	case 6:
		MAX2831->reg6 = 0x0000060;
		reg_status[6] = 0x0000060;
	break;
	case 7:
		MAX2831->reg7 = 0x0001022;
		reg_status[7] = 0x0001022;
	break;
	case 8:
		MAX2831->reg8 = 0x0002021;
		reg_status[8] = 0x0002021;
	break;
	case 9:
		MAX2831->reg9 = 0x00003B5;
		reg_status[9] = 0x00003B5;
	break;
	case 10:
		MAX2831->reg10 = 0x0001DA4;
		reg_status[10] = 0x0001DA4;
	break;
	case 11:
		MAX2831->reg11 = 0x000007F;		// minimum RX gain
		reg_status[11] = 0x000007F;
	break;
	case 12:
		MAX2831->reg12 = 0x0000140;
		reg_status[12] = 0x0000140;
	break;
	case 13:
		MAX2831->reg13 = 0x0000E92;
		reg_status[13] = 0x0000E92;
	break;
	case 14:
		MAX2831->reg14 = 0x000033B;
		reg_status[14] = 0x000033B;
	break;
	case 15:
		MAX2831->reg15 = 0x0000145;
		reg_status[15] = 0x0000145;
	break;
	default:
	break;
	}
}

void setRegisterValueRX(int register_number, unsigned int value)
{
	switch (register_number)
	{
		case 0:
			MAX2831->reg0 = value;
			reg_status[0] = value;
		break;
		case 1:
			MAX2831->reg1 = value;
			reg_status[1] = value;
		break;
		case 2:
			MAX2831->reg2 = value;
			reg_status[2] = value;
		break;
		case 3:
			MAX2831->reg3 = value;
			reg_status[3] = value;
		break;
		case 4:
			MAX2831->reg4 = value;
			reg_status[4] = value;
		break;
		case 5:
			MAX2831->reg5 = value;
			reg_status[5] = value;
		break;
		case 6:
			MAX2831->reg6 = value;
			reg_status[6] = value;
		break;
		case 7:
			MAX2831->reg7 = value;
			reg_status[7] = value;
		break;
		case 8:
			MAX2831->reg8 = value;
			reg_status[8] = value;
		break;
		case 9:
			MAX2831->reg9 = value;
			reg_status[9] = value;
		break;
		case 10:
			MAX2831->reg10 = value;
			reg_status[10] = value;
		break;
		case 11:
			MAX2831->reg11 = value;
			reg_status[11] = value;
		break;
		case 12:
			MAX2831->reg12 = value;
			reg_status[12] = value;
		break;
		case 13:
			MAX2831->reg13 = value;
			reg_status[13] = value;
		break;
		case 14:
			MAX2831->reg14 = value;
			reg_status[14] = value;
		break;
		case 15:
			MAX2831->reg15 = value;
			reg_status[15] = value;
		break;
		default:
		break;
	}
}

unsigned int getRegisterValueRX(int reg_number)
{

	if(reg_number >= 0 && reg_number <= 15)
	{
		switch (reg_number){
			case 0:
				return reg_status[0];
			break;
			case 1:
				return reg_status[1];
			break;
			case 2:
				return reg_status[2];
			break;
			case 3:
				return reg_status[3];
			break;
			case 4:
				return reg_status[4];
			break;
			case 5:
				return reg_status[5];
			break;
			case 6:
				return reg_status[6];
			break;
			case 7:
				return reg_status[7];
			break;
			case 8:
				return reg_status[8];
			break;
			case 9:
				return reg_status[9];
			break;
			case 10:
				return reg_status[10];
			break;
			case 11:
				return reg_status[11];
			break;
			case 12:
				return reg_status[12];
			break;
			case 13:
				return reg_status[13];
			break;
			case 14:
				return reg_status[14];
			break;
			case 15:
				return reg_status[15];
			break;
		}
	}
	return 0xffffffff;
}




//----------------------------------Set and Get Gain Functions --------------------------------------------//


unsigned char setTXBaseBandVGAGain(unsigned char VGAGain)
{
	if (VGAGain > 63 || VGAGain < 0)
		return 0;
	else 
	{
		// For serial programming enable 
		if ((reg_status[9]&(1<<10))==0){
			reg_status[9] = reg_status[9] | 1<<10;
			MAX2831->reg9 = reg_status[9];
		}
		reg_status[12] = (reg_status[12] & (~0x3f))| VGAGain;
		MAX2831->reg12 = reg_status[12]; // clear the last five bits
		return 1;
	}
}

unsigned char setRXBaseBandVGAGain(unsigned char VGAGain)
{
	if (VGAGain > 31 || VGAGain < 0)
		return 0;
	else 
	{
		// For serial programming enable 
		if ((reg_status[8]&(1<<12))==0){
			reg_status[8] = reg_status[8] | 1<<12;
			MAX2831->reg8 = reg_status[8];
		}
		reg_status[11] = (reg_status[11]& (~0x1f))| VGAGain;
		MAX2831->reg11 = reg_status[11];  // clear the last five bits
		return 1;
	}
}

void enRXParVGAGainCtl(){
	reg_status[8] &= 0x2ff;
	MAX2831->reg8 = reg_status[8];
}

unsigned char setLNAGain(unsigned char LNAGain)
{
	if(LNAGain > 3 || LNAGain < 0)
		return 0;
	else {
		// For serial programming enable 
		if ((reg_status[8]&(1<<12))==0){
			reg_status[8] = reg_status[8] | 1<<12;
			MAX2831->reg8 = reg_status[8];
		}
		reg_status[11] = (reg_status[11] & (~0x60))| (LNAGain <<5);
		MAX2831->reg11 = reg_status[11]; //0b110000 clear 6,5 bit
		return 1;
	}

}


unsigned char setFreqDivider(int freq)
{
	double freqDivider =  reg_status[5] & 0x4 ? (double)freq /20 : (double)freq/40;
	int integerDivider = (int)freqDivider;
	int fractionalDivider = (freqDivider - integerDivider)*16384;
	if(freqDivider < 64 || freqDivider > 255)
		return 0;
	reg_status[3] = (reg_status[3] & (~0xff)) | integerDivider;
	reg_status[4] = fractionalDivider;
	MAX2831->reg3 = reg_status[3];
	MAX2831->reg4 = reg_status[4];
	return 1;
}

int getFreqDivider()
{
	int integerDivider = reg_status[3] & 0xff;
	double fractionalDivider = reg_status[4]>>14;
	return (reg_status[5] & 0x4 ? (integerDivider + fractionalDivider)* 20 : (integerDivider + fractionalDivider)* 40);
}


unsigned char setBaseBandLowPassFilterMode (unsigned char LPFMode)
{
	if(LPFMode < 0 || LPFMode > 3)
		return 0;
	else{
		reg_status[8] = (reg_status[8] & (~0x3))| LPFMode;
		MAX2831->reg8 = reg_status[8];
		return 1;
	}
}


unsigned char setRXBaseBandLowPassFilterModeFineAdjust (unsigned char LPFMode, unsigned char LPFFineAdjust)
{
	unsigned char setLPFMode = setBaseBandLowPassFilterMode(LPFMode);
	if(!setLPFMode)
		return 0;
	if(LPFFineAdjust < 0 || LPFFineAdjust > 4)
		return 0;

	reg_status[7] = (reg_status[7] & (~0x7)) | LPFFineAdjust;
	MAX2831->reg7 = reg_status[7];
	return 1;
}

unsigned char setTXBaseBandLowPassFilterModeFineAdjust (unsigned char LPFMode, unsigned char LPFFineAdjust)
{
	unsigned char setLPFMode = setBaseBandLowPassFilterMode(LPFMode);
	if(!setLPFMode)
		return 0;
	if(LPFFineAdjust <0 || LPFFineAdjust >5)
		return 0;

	reg_status[7] = (reg_status[7] & (~0x38))| LPFMode<<3;
	MAX2831->reg7 = reg_status[7];
	return 1;
}

/*
// HighPass Filter
bool setBaseBandHighPassFilterMode(int HPFMode)
{
	int mode = getRxTxMode();
	// check RXHP is 1 or 0, if 1 return false. 
		
		
	if(mode == 0 || mode == 1 || mode == 3 || mode == 5 )
	{
		return -1;
	}
	else if (mode == 2 || mode == 4 ) // Rx or Rxcalibration 
	{
		reg_status[7] = (reg_status[7] & (~0x3000)) | (HPFMode<<12);
		MAX2831->reg7 = reg_status[7];
		//mySPI_transfer(MAX2831->reg8);
		return true;		
	}
	else
	{
		return -1;
	}
}


int getBaseBandHighPassFilterMode()
{
	int mode = getRxTxMode();
	// check RXHP is 1 or 0, if 1 return false. 
				
	if(mode == 0 || mode == 1 || mode == 3 || mode == 5 )
	{
		return -1;
	}
	else if (mode == 2 || mode == 4 ) // Rx or Rxcalibration 
	{
		if((reg_status[7] & 0x3000)>>12 == 1 || (reg_status[7] & 0x3000)>>12 == 3)
			return 1;
		else 
			return ((reg_status[7] & 0x3000)>>12);
	}
	else
	{
		return -1;
	}
	
}
*/
