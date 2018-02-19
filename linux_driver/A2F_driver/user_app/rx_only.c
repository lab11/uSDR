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
#include "include/mss_sdr.h"
#include "include/maxim2831.h"
#include "include/tx_packet.h"
#include "include/rx_packet.h"
#include "include/radio_config.h"

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

void set_led(unsigned int);
int sleep_sdr();

unsigned int* const LED_ptr	= (unsigned int*)0x40070020;
int fd = -1;
volatile unsigned char PDMA_int, tx_complete_int;
volatile unsigned char TIM2_int;
volatile unsigned char SW1_int, SW2_int, SW3_int, SW4_int;
volatile unsigned char rx_pkt_done_int, length_int;

#define DEBUG
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
	MSS_GPIO_config( MSS_GPIO_8, MSS_GPIO_OUTPUT_MODE );

	// Enabling IRQ
	MSS_GPIO_Type gpio16 = MSS_GPIO_16;
	MSS_GPIO_Type gpio17 = MSS_GPIO_17;
	MSS_GPIO_Type gpio24 = MSS_GPIO_24;
	MSS_GPIO_Type gpio25 = MSS_GPIO_25;
	MSS_GPIO_Type gpio26 = MSS_GPIO_26;
	MSS_GPIO_Type gpio27 = MSS_GPIO_27;
	MSS_GPIO_Type gpio31 = MSS_GPIO_31;

/*
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio16)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	*/

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio17)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio24)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio25)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio26)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio27)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio31)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}

	// interrupt variables
	TIM2_int = 0;
	tx_complete_int= 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	rx_pkt_done_int = 0;
	PDMA_int = 0;
	length_int = 0;

	// initial global variable

	// RF variables
	unsigned char res;
	
	// set RF gain
	res = setTXBaseBandVGAGain(10);
	unsigned int frequency = 2405;
	res = setFreqDivider(frequency);

	rx_agc_en(3);

	struct radio* radio0;
	radio0 = malloc(sizeof(struct radio));
	radio0->multiple_addr_en = 0;		// disable multiple address

	struct rx_packet_str* received_packet;
	received_packet = malloc(sizeof(struct rx_packet_str));

	// set radio
	set_src_pan(radio0, 0x22);
	set_src_addr_mode(radio0, 2);	// 2 bytes of src address
	set_src_addr(radio0, 0, 0x05);

	// fifo read
	unsigned int rdata[128];

	// disable auto ACK
	auto_ack_en(0);

	while (1){
		#ifdef DEBUG
		//printf("sleep process\r\n");
		#endif
		irq = sleep_sdr();
		if (irq < 0)
			goto Done;

		// RX read packet
		if (rx_pkt_done_int){
			#ifdef DEBUG
			printf("detect packet\r\n");
			#endif
			rx_pkt_done_int = 0;
			read_fifo(rdata);

			res = rx_packet_create(received_packet, rdata);

			if (received_packet->crc){
			#ifdef DEBUG
			printf("DSN: 0x%x\r\n", received_packet->dsn);
			#endif

				unsigned char address_match = dest_addr_filter(received_packet, radio0);

				// address doesn't match but requests ACK
				// flush ack
				if ( (!address_match) & (received_packet->ack_req)){
					ack_flush();
				}

				unsigned int* rx_data = rdata + received_packet->payload_idx;

				// broadcast 
				if (address_match==3){
				// led packet signature, 0x3f, 0x06
					if ((rx_data[0]==0x3f)&&(rx_data[1]==0x06)){
						set_led(rx_data[3]);
					}
				}

			}

		}

		if (length_int){
			length_int = 0;
		}

		if (SW1_int){
			SW1_int = 0;
			goto Done;
		}
		if (SW2_int){
			SW2_int = 0;
			goto Done;
		}
		if (SW3_int){
			SW3_int = 0;
			if (frequency>2405){
				frequency -= 5;
				res = setFreqDivider(frequency);
			}
		}
		if (SW4_int){
			SW4_int = 0;
			if (frequency<2480){
				frequency += 5;
				res = setFreqDivider(frequency);
			}
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

// interrupt handlers
//
// pdms_tx_handler		TX transmit handler
// GPIO16_IRQHandler	RX length get
// GPIO17_IRQHandler	received a complete packet
// GPIO22_IRQHandler	
// GPIO24_IRQHandler	SW1
// GPIO25_IRQHandler	SW2
// GPIO30_IRQHandler	SW3
// GPIO31_IRQHandler	SW4
// Timer1_IRQHandler
//


int sleep_sdr(){
	int irq;
	if (ioctl(fd, SDR_IOC_SLEEP_PROC, &irq)<0){
		fprintf(stderr, "unable to sleep process: %s\n", strerror(errno));
		return -1;
	}
	switch ((IRQn_Type)irq){
		case PDMA_IRQn:	
			PDMA_int = 1;
		break;

		case TIMER2_IRQn: 
			TIM2_int = 1;
			MSS_TIMER2_stop();
		break;

		case GPIO16_IRQn:
			MSS_GPIO_set_output( MSS_GPIO_8, 1 );
			length_int = 1;
			MSS_GPIO_set_output( MSS_GPIO_8, 0 );
		break;

		case GPIO17_IRQn:
			rx_pkt_done_int = 1;
		break;

		case GPIO24_IRQn:
			SW1_int = 1;
		break;

		case GPIO25_IRQn:
			SW2_int = 1;
		break;

		case GPIO26_IRQn:
			SW3_int = 1;
		break;

		case GPIO27_IRQn:
			SW4_int = 1;
		break;

		case GPIO31_IRQn:
    		tx_complete_int = 1;
		break;
	}
	return irq;
}
