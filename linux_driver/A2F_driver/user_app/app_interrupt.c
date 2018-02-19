/*
 * This is a user-space application that reads /dev/sample
 * and prints the read characters to stdout
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "include/mss_sdr.h"

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

int sleep_sdr(void);

volatile unsigned char PDMA_int, tx_complete_int;
volatile unsigned char TIM2_int;
volatile unsigned char SW1_int, SW2_int, SW3_int, SW4_int;
volatile unsigned int rx_cnt, tx_counter;
volatile unsigned char rx_pkt_done_int, length_int;
volatile int fd = -1;

int main(int argc, char **argv)
{
	char * app_name = argv[0];
	char * dev_name = "/dev/sdr";
	int ret = -1;
	int c, x;
	int irq = 0;

	TIM2_int = 0;
	tx_complete_int= 0;
	SW1_int = 0; SW2_int = 0; SW3_int = 0; SW4_int = 0;
	rx_pkt_done_int = 0;
	PDMA_int = 0;
	length_int = 0;

	// open timer2
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	MSS_TIMER2_MODE_Type tim2_mode = MSS_TIMER_PERIODIC_MODE;

	if (ioctl(fd, TIMER2_IOC_INIT, &tim2_mode)<0){
		fprintf(stderr, "%s: unable to set mode%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	MSS_TIMER2_load_background(ONE_SEC);

	if (ioctl(fd, TIMER2_IOC_ENABLE_IRQ)<0){
		fprintf(stderr, "%s: unable to enable IRQ%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	MSS_GPIO_Type gpio24 = MSS_GPIO_24;
	MSS_GPIO_Type gpio25 = MSS_GPIO_25;
	MSS_GPIO_Type gpio26 = MSS_GPIO_26;
	MSS_GPIO_Type gpio27 = MSS_GPIO_27;

	MSS_GPIO_config(MSS_GPIO_24, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config(MSS_GPIO_25, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config(MSS_GPIO_26, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_config(MSS_GPIO_27, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio24)<0){
		fprintf(stderr, "%s: unable issue command %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio25)<0){
		fprintf(stderr, "%s: unable issue command %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio26)<0){
		fprintf(stderr, "%s: unable issue command %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, &gpio27)<0){
		fprintf(stderr, "%s: unable issue command %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	//MSS_TIMER2_start();

	PDMA_init();

	PDMA_configure
	(
		PDMA_CHANNEL_1,
		PDMA_MEM_TO_MEM,
		PDMA_LOW_PRIORITY | PDMA_WORD_TRANSFER | PDMA_INC_SRC_FOUR_BYTES,
		PDMA_DEFAULT_WRITE_ADJ
	);

	PDMA_configure
	(
		PDMA_CHANNEL_0,
		PDMA_MEM_TO_MEM,
		PDMA_LOW_PRIORITY | PDMA_WORD_TRANSFER | PDMA_INC_SRC_FOUR_BYTES,
		PDMA_DEFAULT_WRITE_ADJ
	);

	PDMA_CHANNEL_ID_Type irq_ch, pdma_ch1 = PDMA_CHANNEL_1, pdma_ch0 = PDMA_CHANNEL_0;

/*
	if (ioctl(fd, PDMA_IOC_ENABLE_IRQ, &pdma_ch1)<0){
		fprintf(stderr, "%s: unable to enable IRQ%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	if (ioctl(fd, PDMA_IOC_ENABLE_IRQ, &pdma_ch0)<0){
		fprintf(stderr, "%s: unable to enable IRQ%s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}
	*/

	unsigned int status_pdma;	
	unsigned int data_ptr[256];
	unsigned int i;
	unsigned int* LED_ptr = (unsigned int *)0x40070020;

	for (i=0; i<256; i++)
		data_ptr[i] = i;
	
	PDMA_start
	(
		PDMA_CHANNEL_1,
		(unsigned int)data_ptr,
		(unsigned int)LED_ptr,
		(unsigned short)128
	);


    do
    {
        status_pdma = PDMA_status(PDMA_CHANNEL_1);
    }while (status_pdma == 0);

	
	//irq = sleep_sdr();

	if (irq < 0)
		goto Done;
	status_pdma = PDMA_status(PDMA_CHANNEL_1);
	printf("PDMA Status: %d\r\n", status_pdma);

	status_pdma = PDMA_status(PDMA_CHANNEL_0);
	printf("PDMA Status: %d\r\n", status_pdma);

	printf("irq asserted: %d\r\n", irq);
	
	ret = 0;

Done:
	if (fd >= 0) {
		close(fd);
	}
	return ret;
}

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
			length_int = 1;
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
	printf("IRQ%d called!\r\n", irq);
	return irq;
}
