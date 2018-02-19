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
#include <string.h>
#include "include/mss_sdr.h"
#include "include/maxim2831.h"
#include "include/tx_packet.h"
#include "include/rx_packet.h"
#include "include/radio_config.h"

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000
#define MAX_PKT_COUNT 1000

inline int sleep_sdr();

int fd = -1;
volatile unsigned char tx_cpl_int, glossy_cor_int, TIM2_int;
volatile int tx_pkt_cnt, rx_pkt_cnt, num_of_trial;

static int* irq_fabric = (int*) 0x40070010;
static int* const LED_ptr = (int*)0x40070020;
static int* const audio_mode = (int*) 0x40070030;
static int* const audio_pkt_length = (int*) 0x40070040;

inline void set_led(unsigned int data_in);

//#define DEBUG
//#define SCHEDULE_MAX
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
	MSS_GPIO_Type tx_complete_int = MSS_GPIO_31;
	MSS_GPIO_Type glossy_correct_int = MSS_GPIO_17;

	MSS_GPIO_config( MSS_GPIO_17, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &tx_complete_int)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &glossy_correct_int)<0){
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

	// interrupt variables
	tx_cpl_int = 0; glossy_cor_int = 0;
	tx_pkt_cnt = 0; rx_pkt_cnt = 0;
	TIM2_int = 0;

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

	while (1){
		switch (state){
			case 0:
				res = setTXBaseBandVGAGain(10);
				res = setFreqDivider(frequency);
				printf("1: TX Mode\r\n");
				printf("2: RX Mode\r\n");
				printf("9: End\r\n");
				printf("Input selection: ");
				scanf("%d", &input);
				printf("\r\n");
				tx_pkt_cnt = 0;
				rx_pkt_cnt = 0;
				sleep_en = 0;
				state = 1;
			break;

			case 1:
				switch (input){
					case 1:
						switch (tx_sub_state){
							case 0:
								printf("Selected TX\r\n");
								printf("Enter packet length: (78~110)");
								scanf("%d", &pkt_len);
								*audio_pkt_length = pkt_len;
								printf("\r\nEnter number of rounds: (%d us/round)", pkt_len*125);
								scanf("%d", &num_of_trial);
								printf("\r\nPacket Length: %d\r\n", pkt_len);
								tx_sub_state = 1;
							break;

							case 1:
								tx_pkt_cnt = 0;
								rx_pkt_cnt = 0;
								MSS_TIMER2_load_immediate(ONE_SEC<<2);
								MSS_TIMER2_start();
								*audio_mode = 1;
								sleep_en = 1;
								state = 2;
							break;

							case 2:
								if (pkt_len < 107){
									pkt_len += 3;
									*audio_pkt_length = pkt_len;
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
						*audio_mode = 0;
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

		if (sleep_en)
			irq = sleep_sdr();

		if (TIM2_int){
			printf("Number of packet received: %d\t\n", rx_pkt_cnt);
			TIM2_int = 0;
			state = 1;
			tx_sub_state = 2;
		}
	}

Done:
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
			tx_pkt_cnt++;
			if (tx_pkt_cnt==num_of_trial)
				*audio_mode = 0;
		break;

		case GPIO17_IRQn:
			rx_pkt_cnt++;
			set_led(rx_pkt_cnt%256);
			MSS_TIMER2_stop();
			MSS_TIMER2_load_immediate(ONE_SEC<<2);
			MSS_TIMER2_start();
		break;

		case TIMER2_IRQn: 
			*audio_mode = 0;
			TIM2_int = 1;
		break;
	}
	return irq;
}
