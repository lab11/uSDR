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
#include "include/maxim2831.h"
#include "include/tx_packet.h"
#include "include/rx_packet.h"
#include "include/radio_config.h"

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

void set_led(unsigned int);
void set_interval(unsigned char);
inline int sleep_sdr();

unsigned int* const LED_ptr	= (unsigned int*)0x40070020;
int fd = -1;
volatile unsigned char PDMA_int, tx_complete_int;
volatile unsigned char TIM2_int;
volatile unsigned char SW1_int, SW2_int, SW3_int, SW4_int;
volatile unsigned int tx_counter;
volatile unsigned char rx_pkt_done_int, length_int;

static int* irq_fabric = (int*) 0x40070010;

static unsigned int rdata[128];

#define LED
//#define DEBUG
//#define SCHEDULE_MAX
int main(int argc, char **argv)
{
	#ifdef SCHEDULE_MAX
	struct sched_param p;
	p.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (p.sched_priority<0){
		fprintf(stderr, "unable to set priority: %s\n", strerror(errno));
		goto Done;
	}
	if (sched_setscheduler(getpid(), SCHED_FIFO, &p)<0){
		fprintf(stderr, "unable to set schedular: %s\n", strerror(errno));
		goto Done;
	}
	#endif

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
	MSS_GPIO_Type gpio17 = MSS_GPIO_17;
	MSS_GPIO_Type gpio24 = MSS_GPIO_24;
	MSS_GPIO_Type gpio25 = MSS_GPIO_25;
	MSS_GPIO_Type gpio26 = MSS_GPIO_26;
	MSS_GPIO_Type gpio27 = MSS_GPIO_27;
	//MSS_GPIO_Type gpio31 = MSS_GPIO_31;

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
	/*
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio31)<0){
		fprintf(stderr, "unable to set irq: %s\n", strerror(errno));
		goto Done;
	}
	*/

	// interrupt variables
	TIM2_int = 0;
	tx_complete_int= 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	rx_pkt_done_int = 0;
	PDMA_int = 0;
	length_int = 0;

	// initial global variable
	tx_counter = 0;

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

	struct rx_packet_str* received_packet;
	received_packet = malloc(sizeof(struct rx_packet_str));

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

	#ifdef LED
	set_dest_addr(led_pkt, 0, 0xffff);
	set_ack(led_pkt, 0);
	#endif

	#ifdef GLOSSY
	set_dest_addr(led_pkt, 0, 0xfffe);
	set_ack(led_pkt, 0);
	#endif

	#ifdef ACK
	set_dest_addr(led_pkt, 0, 0x05);
	set_ack(led_pkt, 1);
	#endif

	// set radio
	set_src_pan(radio0, 0x22);
	set_src_addr_mode(radio0, 2);	// 2 bytes of src address
	set_src_addr(radio0, 0, 0x05);

	// set radio packet data
	led_data_ptr[0] = 0x3f;
	led_data_ptr[1] = 0x06;
	led_data_ptr[2] = 0x00;
	led_data_ptr[3] = 0x00;


	data_trans(led_pkt);

	// fifo read

	// timer
	MSS_TIMER2_MODE_Type tim2_mode = MSS_TIMER_PERIODIC_MODE;
	if (ioctl(fd, TIMER2_IOC_INIT, &tim2_mode)<0){
		fprintf(stderr, "%s: unable to set mode%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	if (ioctl(fd, TIMER2_IOC_ENABLE_IRQ)<0){
		fprintf(stderr, "%s: unable to enable IRQ%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	int test_int = 10;

	unsigned char interval = 1;	// 4Hz
	set_interval(interval);

	while (1){
		#ifdef DEBUG
			printf("sleep process\r\n");
		#endif
		irq = sleep_sdr();

		if (rx_pkt_done_int==1){
			MSS_GPIO_BIT_BAND->gpio_out[8] = 0;	// measure latency
			#ifdef DEBUG
				printf("detect packet\r\n");
			#endif
			rx_pkt_done_int = 0;

    		unsigned int status_pdma = 0;
    		do
    		{
    		    status_pdma = PDMA_status (PDMA_CHANNEL_0);
    		}while (status_pdma == 0);

			res = rx_packet_create(received_packet, rdata);

			if (received_packet->crc){

				unsigned char address_match = dest_addr_filter(received_packet, radio0);
				// address doesn't match but requests ACK
				// flush ack
				if ( (!address_match) & (received_packet->ack_req)){
					ack_flush();
					MSS_GPIO_BIT_BAND->gpio_out[8] = 1;	// measure latency
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
			MSS_GPIO_BIT_BAND->gpio_out[8] = 0;	// measure latency

		}

		//if (TIM2_int & PDMA_int){
		if (TIM2_int){
			#ifdef DEBUG
				printf("periodic transmit\r\n");
			#endif
			tx_fire();

			PDMA_int = 0;
			TIM2_int = 0;
			unsigned int gpio_in;
			do{
				gpio_in = MSS_GPIO_get_input(MSS_GPIO_19);
			}
			while(gpio_in==0);

			#ifdef DEBUG
				printf("tx complete\r\n");
			#endif
			tx_complete_int = 0;
			tx_counter++;
			led_data_ptr[2] = (tx_counter & 0xff00)>>8;
			led_data_ptr[3] = (tx_counter & 0xff);
			#ifdef GLOSSY
				set_DSN(led_pkt, 126);				// DSN = 126, only forward once 
			#else
				set_DSN(led_pkt, led_data_ptr[3]);
			#endif
			//printf("tx complete\r\n");
			data_trans (led_pkt);
			//printf("DMA complete\r\n");
			MSS_TIMER2_start();

		}
		/*
		if (tx_complete_int==1){
			#ifdef DEBUG
			printf("tx complete\r\n");
			#endif
			tx_complete_int = 0;
			tx_counter++;
			led_data_ptr[2] = (tx_counter & 0xff00)>>8;
			led_data_ptr[3] = (tx_counter & 0xff);
			#ifdef GLOSSY
			set_DSN(led_pkt, 126);				// DSN = 126, only forward once 
			#else
			set_DSN(led_pkt, led_data_ptr[3]);
			#endif
			data_trans (led_pkt);
			MSS_TIMER2_start();
		}
		*/
		// RX read packet
		if (SW1_int==1){
			SW1_int = 0;
			if (interval>1)
				interval = (interval>>1);
			set_interval(interval);
		}
		if (SW2_int==1){
			SW2_int = 0;
			if (interval<16)
				interval = (interval<<1);
			set_interval(interval);
		}
		if (SW3_int==1){
			SW3_int = 0;
			/*
			if (frequency>2405){
				frequency -= 5;
				res = setFreqDivider(frequency);
			}
			*/
			goto Done;
		}
		if (SW4_int==1){
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


void set_interval(unsigned char input_interval){
	MSS_TIMER2_stop();
	switch (input_interval){
		case 1:
			MSS_TIMER2_load_background(ONE_SEC);
		break;
		case 2:
			MSS_TIMER2_load_background(ONE_SEC>>1);
		break;
		case 4:
			MSS_TIMER2_load_background(ONE_SEC>>2);
		break;
		case 8:
			MSS_TIMER2_load_background(ONE_SEC>>3);
		break;
		case 16:
			MSS_TIMER2_load_background(ONE_SEC>>4);
		break;
	}
	MSS_TIMER2_start();
}

inline int sleep_sdr(){
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
			length_int = 1;
		break;

		case GPIO17_IRQn:
			read_fifo(rdata);
			rx_pkt_done_int = 1;
			MSS_GPIO_BIT_BAND->gpio_out[8] = 1;	// measure latency
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
