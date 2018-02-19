/*
 * This is a user-space application that reads /dev/sample
 * and prints the read characters to stdout
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include "include/mss_sdr.h"

#define PROG_MAX_SIZE 1024 // 1024 bytes
#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

inline int sleep_sdr();

int fd = -1;

static int* const TX_FIFO = (int*) 0x40070010;
static int* const LED_ptr = (int*)0x40070020;
static int* const audio_mode = (int*) 0x40070030;
static int* const audio_pkt_length_dsn = (int*) 0x40070040;

volatile int freq_div, freq_div2;
//static int* freq_frac2 = (int*)0x4008000c;
static int* freq_frac2 = (int*)0x40080040;
static int* freq_frac = (int*)0x4008000c;

volatile unsigned char TIM2_int, fifo_aempty_int, fifo_empty_int;
volatile int rx_pkt_cnt, num_of_trial;

int main(int argc, char **argv)
{
	char * app_name = argv[0];
	char * dev_name = "/dev/sdr";
	int ret = -1;
	int c, x;
	int irq = 0;

	// open SDR device 
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	RF_shdn(0);
	init_system();

	// Enabling IRQ
	MSS_GPIO_Type fifo_aempty 			= MSS_GPIO_16;
	MSS_GPIO_Type glossy_correct_int 	= MSS_GPIO_17;
	MSS_GPIO_Type tx_complete 			= MSS_GPIO_19;
	MSS_GPIO_Type fifo_empty 			= MSS_GPIO_31;

	MSS_GPIO_config( MSS_GPIO_16, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_17, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_19, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_31, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );

	MSS_GPIO_Type gpio24 = MSS_GPIO_24;
	MSS_GPIO_Type gpio25 = MSS_GPIO_25;

	MSS_GPIO_config( MSS_GPIO_24, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config( MSS_GPIO_25, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );

	// switches
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio24)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio25)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}


	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &fifo_aempty)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &tx_complete)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &glossy_correct_int)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &fifo_empty)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}


	// timer
	MSS_TIMER2_MODE_Type tim2_mode = MSS_TIMER_ONE_SHOT_MODE;
	if (ioctl(fd, TIMER2_IOC_INIT, &tim2_mode)<0){
		fprintf(stderr, "%s: unable to set mode%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	if (ioctl(fd, TIMER2_IOC_ENABLE_IRQ)<0){
		fprintf(stderr, "%s: unable to enable IRQ%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	// reading file
	int * programming_data;
	programming_data = malloc(sizeof(int)*PROG_MAX_SIZE);
	//int programming_data[PROG_MAX_SIZE] = {0};
	int file_length = 0;
	int file_length_cal = 0;
	int remain;
	FILE* pFile;

	// interrupt variables
	rx_pkt_cnt = 0;
	TIM2_int = 0;
	fifo_aempty_int = 0;
	fifo_empty_int = 0;

	// RF variables
	unsigned char res;
	
	// set RF gain
	initialize_chip();
	res = setTXBaseBandVGAGain(10);
	unsigned int frequency = 2405;
	res = setFreqDivider(frequency);


	rx_agc_en(3);
	char state = 0;
	int input;
	int sleep_en;
	int pkt_len;
	int tx_sub_state = 0;
	int dsn;
	int repit = 0;
	freq_div2 = (int)((((float)frequency*16384)/20));
	*freq_frac2 = freq_div2;
	freq_div = 20;

	int i;
	while (1){
		switch (state){
			case 0:
				printf("1: TX Mode\r\n");
				printf("2: RX Mode\r\n");
				printf("9: End\r\n");
				printf("Input selection: ");
				scanf("%d", &input);
				printf("\r\n");
				rx_pkt_cnt = 0;
				sleep_en = 0;
				state = 1;
			break;

			case 1:
				switch (input){
					case 1:
						switch (tx_sub_state){
							case 0:
								// capture AGC
								/*
								rx_agc_en(0);
								setLNAGain(2);
								setRXBaseBandVGAGain(10);
								*/

								// read file
								pFile = fopen("hex_sample.txt", "r");
								if (pFile==NULL){
									printf("Error opening file\r\n");
									goto Done;
								}

								char buf[200];
								while (fgets(buf, sizeof(buf), pFile) != NULL) {
								    int i = 0;
								    while (buf[i] != '\n') {
								        int b; // must use int with sscanf()
									    sscanf(&buf[i], "%2x", &b);
								        programming_data[file_length+i/2] = b;
								        i += 2;
								    }
									file_length += (i/2);
								}
								printf("File length = %d Bytes.\r\n", file_length);
								remain = file_length;

								if (file_length>PROG_MAX_SIZE){
									printf("Error, file size overflow!\r\n");
									goto Done;
								}
								// End of reading file
								printf("Selected TX\r\n");
								printf("Enter packet length (12~110): ");
								scanf("%d", &pkt_len);
								printf("Enter initial DSN (0~20): ");
								scanf("%d", &dsn);
								*audio_pkt_length_dsn = (dsn<<8) | pkt_len;
								printf("\r\nPacket Length: %d\r\n", pkt_len);
								tx_sub_state = 1;
							break;

							case 1:
								rx_pkt_cnt = 0;

								int temp = file_length % pkt_len;
								if (temp)
									file_length_cal = file_length + (pkt_len - temp);		// pad extra data 
								else
									file_length_cal = file_length;
								remain = file_length_cal;

								fifo_empty_int = 0;
								MSS_TIMER2_load_immediate(ONE_SEC<<2);
								MSS_TIMER2_start();
								*audio_mode = ((1<<1) | 1);
								PDMA_start
								(
	    							PDMA_CHANNEL_1,
	    							//(unsigned int)&programming_data[0],
	    							(unsigned int)programming_data,
	    							(unsigned int)TX_FIFO,
	    							(unsigned short)100
								);
								remain -= 100;
								sleep_en = 1;
								state = 2;
							break;

							case 2:
								goto Done;
								if (pkt_len < 107){
									pkt_len += 10;
									*audio_pkt_length_dsn = (dsn<<8) | pkt_len;
									printf("Packet Length: %d\r\n", pkt_len);
									tx_sub_state = 1;
									sleep_en = 0;
								}
								else{
									printf("Done!\r\n");
									goto Done;
								}
							break;

							default:
							break;
						}
					break;

					case 2:
						rx_pkt_cnt = 0;
						printf("Selected RX\r\n");
						*audio_mode = ((1<<1) | 0);
						sleep_en = 1;
						state = 2;
					break;

					case 9:
						goto Done;
					break;

					default:
						printf("Invalid input\r\n");
						state = 0;
					break;
				}
			break;

			default:
			break;
		}

		if (sleep_en){
			irq = sleep_sdr();
			//printf("IRQn: %d\r\n", irq);
		}
		
		if (fifo_aempty_int){
			fifo_aempty_int = 0;
			if (remain>100){
				PDMA_start
				(
	    			PDMA_CHANNEL_1,
	    			(unsigned int)&programming_data[(file_length_cal-remain)],
	    			(unsigned int)TX_FIFO,
	    			(unsigned short)100
				);
				remain -= 100;
			}
			else if (remain){
				for (i=0; i<remain; i++){
					*TX_FIFO = programming_data[file_length_cal-remain+i];
				}
				if (repit < 1){
					repit++;
					remain = file_length_cal;
				}
				else
					remain = 0;
			}
		}

		if (TIM2_int){
			TIM2_int = 0;
			// tx mode
			if ((input==1)&&(remain>0)){
				printf("remains data not sent yet, Error\r\n");
				goto Done;
			}
			else{
				printf("Number of packet received: %d\t\n", rx_pkt_cnt);
				TIM2_int = 0;
				state = 1;
				tx_sub_state = 2;
			}
		}

	}
	

Done:
	*audio_mode = 0;
	RF_shdn(1);
	if (fd >= 0) {
		close(fd);
	}
	return ret;
}

inline void set_led(unsigned int data_in){
	*LED_ptr = data_in;
}

inline int sleep_sdr(){
	int irq;
	if (ioctl(fd, SDR_IOC_SLEEP_PROC, &irq)<0){
		fprintf(stderr, "unable to sleep process: %s\n", strerror(errno));
		return -1;
	}
	switch ((IRQn_Type)irq){
		case GPIO31_IRQn:
			fifo_empty_int = 1;
		break;

		case GPIO17_IRQn:
			rx_pkt_cnt++;
			set_led(rx_pkt_cnt%256);
			MSS_TIMER2_stop();
			MSS_TIMER2_load_immediate(ONE_SEC<<2);
			MSS_TIMER2_start();
		break;

		case GPIO16_IRQn:
			fifo_aempty_int = 1;
		break;

		case GPIO19_IRQn:
			if (fifo_empty_int)
				*audio_mode = ((1<<1)|0);
		break;


		case TIMER2_IRQn: 
			*audio_mode = 0;
			TIM2_int = 1;
		break;

		case GPIO24_IRQn:
			if (freq_div<63){
				freq_div += 1;
			}
			else {
				freq_div = 0;
				freq_div2++;
				*freq_frac2 = freq_div2;
				printf("freq2: %d\r\n", freq_div2);
			}
			*freq_frac = ((freq_div<<8) + 120);
			printf("freq: %d\r\n", freq_div);
			printf("increase freq\r\n");
		break;

		case GPIO25_IRQn:
			if (freq_div){
				freq_div--;
			}
			else{
				freq_div = 63;
				freq_div2--;
				*freq_frac2 = freq_div2;
				printf("freq2: %d\r\n", freq_div2);
			}
			*freq_frac = ((freq_div<<8) + 120);
			printf("freq: %d\r\n", freq_div);
			printf("decrease freq\r\n");
		break;
	}
	return irq;
}
