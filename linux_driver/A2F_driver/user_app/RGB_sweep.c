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
void set_interval(unsigned int);
inline int sleep_sdr();

int fd = -1;
volatile unsigned char TIM2_int;
volatile unsigned char SW1_int, SW2_int, SW3_int, SW4_int;

static int* irq_fabric = (int*) 0x40070010;
static int* RGB_LED = (int*) 0x40070060;
static int* LED_counter = (int*) 0x40070070;

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

	//RF_shdn(0);
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

	unsigned int interval = 1;	// 4Hz
	unsigned char Rin, Gin, Bin;
	Rin = 0;
	Gin = 0;
	Bin = 0;
	set_interval(interval);
	unsigned int counter = 0;
	unsigned char color = 0;

	while (1){
		irq = sleep_sdr();

		if (TIM2_int){
			unsigned int led_counter;
			led_counter = *LED_counter;
			printf("Counter Value: %d\r\n", led_counter);
			TIM2_int = 0;
			counter++;
			//Rin = (counter % 256);
			//Gin = ((counter>>8)%256);
			//Bin = ((counter>>16)%256);
			unsigned int temp1 = (counter % 512);
			if (temp1 > 255)
				temp1 = 511 - temp1;
				
			Rin = 0;
			Gin = 0;
			Bin = 0;
			switch (color){
				case 0:
					Rin = temp1;
				break;

				case 1:
					Gin = temp1;
				break;

				case 2:
					Bin = temp1;
				break;
			}
			unsigned int temp = (Bin<<16) | (Gin<<8) | Rin;
			*RGB_LED = temp;
		}
		if (SW1_int==1){
			SW1_int = 0;
			if (interval>1)
				interval -= 1;
			printf("frequency divider: %d\r\n", interval);
			set_interval(interval);
		}
		if (SW2_int==1){
			SW2_int = 0;
			if (interval<14)
				interval += 1;
			printf("frequency divider: %d\r\n", interval);
			set_interval(interval);
		}
		if (SW3_int==1){
			SW3_int = 0;
			goto Done;
		}
		if (SW4_int==1){
			SW4_int = 0;
			color++;
			color = (color % 3);
		}

	}

Done:
	*RGB_LED = 0;
	RF_shdn(1);
	if (fd >= 0) {
		close(fd);
	}
	return ret;

}


void set_interval(unsigned int input_interval){
	MSS_TIMER2_stop();
	MSS_TIMER2_load_background(ONE_SEC>>input_interval);
	MSS_TIMER2_start();
}

inline int sleep_sdr(){
	int irq;
	#ifdef SLEEP_EN
		if (ioctl(fd, SDR_IOC_SLEEP_PROC, &irq)<0){
			fprintf(stderr, "unable to sleep process: %s\n", strerror(errno));
			return -1;
		}
	#else
		unsigned int gpio_in;
		do{
			gpio_in = MSS_GPIO_get_input(MSS_GPIO_30);
		}
		while(gpio_in==0);
		MSS_GPIO_BIT_BAND->gpio_out[6] = 0;	// clear loop back gpio

		irq = *irq_fabric;

	#endif

	switch ((IRQn_Type)irq){
		case PDMA_IRQn:	
		break;

		case TIMER2_IRQn: 
			TIM2_int = 1;
		break;

		case GPIO16_IRQn:
		break;

		case GPIO17_IRQn:
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
		break;
	}
	return irq;
}
