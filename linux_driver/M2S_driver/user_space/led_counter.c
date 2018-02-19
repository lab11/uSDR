/*
 * This is a user-space application that reads /dev/sample
 * and prints the read characters to stdout
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include "include/sdr_def.h"
#include "include/maxim2831.h"
#include "include/tx_packet.h"
#include "include/rx_packet.h"
#include "include/radio_config.h"

#define CLK_FREQ	48	//48MHz
#define ONE_SEC		CLK_FREQ*1000000

inline void set_led(unsigned int);

unsigned int* const LED_ptr	= (unsigned int*)(FPGA_FABRIC_BASE + 0x40000);
int fd = -1;
volatile unsigned int tx_counter;


int pipe_read, pipe_write;

#define INTERVAL_SEC 2
#define INTERVAL_USEC 0

void alarm_handler(int signum)
{
	int dummy = 0;
	write(pipe_write, &dummy, sizeof(dummy));
}

#define LED
#define DEBUG
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
	char * dev_name = "/dev/sdr_m2s";
	int ret = -1;
	int irq = 0;
	ssize_t nread;
	fd_set readset;
	int nfds;

	// open SDR device 
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	int pipefds[2];
	if (pipe(pipefds) < 0) {
		perror("Could not create pipe");
		return -1;
	}
	pipe_read = pipefds[0];
	pipe_write = pipefds[1];

	struct sigaction action = {
		.sa_handler = alarm_handler,
		.sa_flags = SA_RESTART
	};
	sigaction(SIGALRM, &action, NULL);

	struct timeval expire_time = {
		.tv_sec = INTERVAL_SEC,
		.tv_usec = INTERVAL_USEC
	};

	struct itimerval timer = {
		.it_interval = expire_time,
		.it_value = expire_time
	};


	printf("hello~\r\n");
	RF_shdn(0);
	init_system();

	M2S_GPIO_Type io_tx_completed = TX_COMPLETED;
	M2S_GPIO_Type io_packet_done = PACKET_DONE;

	if ((gpio_enable_irq(fd, io_tx_completed))<0){
		perror("unable to enable TX_COMPLETED interrupt\r\n");
		goto Done;
	}

	if ((gpio_enable_irq(fd, io_packet_done))<0){
		perror("unable to enable PACKET_DONE interrupt\r\n");
		goto Done;
	}

	// initial global variable
	tx_counter = 0;

	// RF variables
	unsigned char res;
	
	// set RF gain
	res = setTXBaseBandVGAGain(10);
	unsigned int frequency = 2480;
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

	// received data
	unsigned int rdata[128];

	setitimer(ITIMER_REAL, &timer, NULL);
	nfds = ((fd > pipe_read) ? fd : pipe_read) + 1;

	while (1){
		#ifdef DEBUG
			printf("sleep process\r\n");
		#endif

		FD_ZERO(&readset);
		FD_SET(fd, &readset);
		FD_SET(pipe_read, &readset);

		ret = select(nfds, &readset, NULL, NULL, NULL);

		if (ret < 0) {
			if (errno == EINTR) {
				continue;
			} else {
				perror("could not select()");
				goto Done;
			}
		}

		if (FD_ISSET(fd, &readset)) {
			nread = read(fd, &irq, sizeof(irq));
			if (nread < 0) {
				perror("unable to get irq");
				goto Done;
			}
			
			switch ((IRQn_Type)irq){
				// TX completed
				case M2S_GPIO12_IRQn:
					#ifdef DEBUG
					printf("tx complete\r\n");
					#endif
					tx_counter++;
					led_data_ptr[2] = (tx_counter & 0xff00)>>8;
					led_data_ptr[3] = (tx_counter & 0xff);
					#ifdef GLOSSY
					set_DSN(led_pkt, 1);
					#else
					set_DSN(led_pkt, led_data_ptr[3]);
					#endif
					data_trans(led_pkt);
				break;

				
				// Packet done
				case M2S_GPIO10_IRQn:
					#ifdef DEBUG
					printf("detect packet\r\n");
					#endif
					read_fifo(rdata);
    				unsigned int status_pdma = 0;
    				do
    				{
    				    status_pdma = PDMA_status (PDMA_CHANNEL_0);
    				}while (status_pdma == 0);

					int i;
					for (i=0; i<rdata[0]; i++)
						printf("fifo%d: %x\r\n", i, rdata[i]);
					/*
					unsigned int * temp_ptr = 0x30000010;
					unsigned int control_reg = *temp_ptr;
					
					printf("control reg: %x\r\n", control_reg);
					*/

					res = rx_packet_create(received_packet, rdata);

					if (received_packet->crc){
						unsigned char address_match = dest_addr_filter(received_packet, radio0);
						// address doesn't match but requests ACK
						// flush ack
						if ( (!address_match) & (received_packet->ack_req))
							ack_flush();

						unsigned int* rx_data = rdata + received_packet->payload_idx;

						// broadcast 
						if (address_match==3){
						// led packet signature, 0x3f, 0x06
							if ((rx_data[0]==0x3f)&&(rx_data[1]==0x06)){
								set_led(rx_data[3]);
							}
						}
					}
				break;

				default:
					printf("IRQn: %d\r\n", irq);
				break;
			}
		}

		if (FD_ISSET(pipe_read, &readset)) {
			nread = read(pipe_read, &irq, sizeof(irq));
			if (nread < 0) {
				perror("unable to clear timer");
				goto Done;
			}
			
			#ifdef DEBUG
				printf("periodic transmit\r\n");
			#endif
			tx_fire();
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

