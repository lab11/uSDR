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

inline int sleep_sdr();

unsigned int* const LED_ptr	= (unsigned int*)0x40070020;
int fd = -1;
volatile unsigned char PDMA_int, tx_complete_int;
volatile unsigned char TIM2_int;
volatile unsigned char SW1_int, SW2_int, SW3_int, SW4_int;
volatile unsigned char rx_pkt_done_int, length_int;

static int* irq_fabric = (int*) 0x40070010;

static unsigned int rdata[128];

//#define DEBUG
//#define SCHEDULE_MAX
int main(int argc, char **argv)
{

	char * app_name = argv[0];
	char * dev_name = "/dev/sdr";
	int ret = -1;
	int c, x;
	int irq = 0;

	if (argc<4){
		printf("Insufficient parameter.\r\n");
		goto Done;
	}

	// open SDR device 
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	RF_shdn(0);
	init_system();

	// Enabling IRQ
	MSS_GPIO_Type gpio17 = MSS_GPIO_17;
	MSS_GPIO_Type gpio24 = MSS_GPIO_24;
	MSS_GPIO_Type gpio25 = MSS_GPIO_25;
	MSS_GPIO_Type gpio26 = MSS_GPIO_26;
	MSS_GPIO_Type gpio27 = MSS_GPIO_27;
	MSS_GPIO_Type gpio31 = MSS_GPIO_31;

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

	struct tx_packet_str* led_pkt;
	led_pkt = malloc(sizeof(struct tx_packet_str));
	led_pkt->rp = radio0;				// link to radio0

	unsigned int* led_data_ptr;
	unsigned int data_size = 4;
	led_data_ptr = malloc(sizeof(unsigned int)*data_size);
	led_pkt->data_length = data_size;
	led_pkt->data_ptr = led_data_ptr;

	// set led packet
	set_frame_type(led_pkt, 1);			// data
	set_security(led_pkt, 0);			// disable security
	set_pan_id_comp(led_pkt, 1);		// enable intra-pan ID
	set_dest_addr_mode(led_pkt, 2);		// 2 bytes of dest address
	set_DSN(led_pkt, 0);				// DSN = 0
	set_dest_pan(led_pkt, 0x22);		// pan addr = 0x0022
	set_frame_pending(led_pkt, 0);		// no frame pending

	set_dest_addr(led_pkt, 0, 0xffff);
	set_ack(led_pkt, 0);

	// set radio
	set_src_pan(radio0, 0x22);
	set_src_addr_mode(radio0, 2);	// 2 bytes of src address
	set_src_addr(radio0, 0, 0x05);

	char* Rvalue = argv[1];
	char* Gvalue = argv[2];
	char* Bvalue = argv[3];
	int rv, gv, bv;
	rv = 0; gv =0; bv = 0;
	int i;
	for (i=0; i<strlen(Rvalue); i++)
		rv = rv*10 + (int)(Rvalue[i] - 48);
	if (rv>255){
		printf("Invalid Red value: %x\r\n", rv);
		goto Done;
	}
	printf("R: %x\r\n", rv);
	for (i=0; i<strlen(Gvalue); i++)
		gv = gv*10 + (int)(Gvalue[i] - 48);
	if (gv>255){
		printf("Invalid Red value: %x\r\n", rv);
		goto Done;
	}
	printf("G: %x\r\n", gv);
	for (i=0; i<strlen(Bvalue); i++)
		bv = bv*10 + (int)(Bvalue[i] - 48);
	if (bv>255){
		printf("Invalid Red value: %x\r\n", rv);
		goto Done;
	}
	printf("B: %x\r\n", bv);
	// set radio packet data
	led_data_ptr[0] = 0xaa;
	led_data_ptr[1] = rv;
	led_data_ptr[2] = gv;
	led_data_ptr[3] = bv;

	data_trans(led_pkt);
	tx_fire();

	while (1){
		irq = sleep_sdr();
		if (tx_complete_int==1){
			goto Done;
		}

	}

Done:
	RF_shdn(1);
	if (fd >= 0) {
		close(fd);
	}
	return ret;

}

inline int sleep_sdr(){
	int irq;
	unsigned int gpio_in;
	do{
		gpio_in = MSS_GPIO_get_input(MSS_GPIO_30);
	}
	while(gpio_in==0);

	irq = *irq_fabric;

	MSS_GPIO_BIT_BAND->gpio_out[6] = 0;	// clear loop back gpio

	switch ((IRQn_Type)irq){
		case PDMA_IRQn:	
			PDMA_int = 1;
		break;

		case TIMER2_IRQn: 
			TIM2_int = 1;
			MSS_TIMER2_stop();
		break;

		case GPIO16_IRQn:
			length_int = 1;
		break;

		case GPIO17_IRQn:
			read_fifo(rdata);
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
