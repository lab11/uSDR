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
//#include "include/m2s_gpio.h"
#include "include/sdr_def.h"

int fd = -1;
int pipe_read, pipe_write;

#define INTERVAL_SEC 0
#define INTERVAL_USEC 300000

void alarm_handler(int signum)
{
	int dummy = 0;
	write(pipe_write, &dummy, sizeof(dummy));
}

int main(int argc, char **argv)
{
	char * app_name = argv[0];
	char * dev_name = "/dev/sdr_m2s";
	int ret = -1;
	int c, x;
	int irq = 0;
	ssize_t nread;
	fd_set readset;
	int nfds;

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

	setitimer(ITIMER_REAL, &timer, NULL);


	GPIO_DEV_t* gpio4;
	gpio4 = malloc(sizeof(GPIO_DEV_t));
	gpio4->m2s_gpio_t = M2S_GPIO_4;
	gpio4->m2s_gpio_dir_t = DIR_INPUT;

	GPIO_DEV_t* gpio6;
	gpio6 = malloc(sizeof(GPIO_DEV_t));
	gpio6->m2s_gpio_t = M2S_GPIO_6;
	gpio6->m2s_gpio_dir_t = DIR_INPUT;

	// open SDR device 
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_SET_DIRECTION, gpio4)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		goto Done;
	}

	int val = ioctl(fd, GPIO_IOC_GET_VAL, gpio4);
	
	if (val<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		goto Done;
	}

	printf("Value: %d\r\n", val);
	
	#if 0

	if (ioctl(fd, GPIO_IOC_SET_DIRECTION, gpio18)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_SET_DIRECTION, gpio6)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		goto Done;
	}

	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, gpio6)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		goto Done;
	}

	nfds = ((fd > pipe_read) ? fd : pipe_read) + 1;

	while (1) {

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
			
			printf("IRQn: %d\r\n", irq);
		}

		if (FD_ISSET(pipe_read, &readset)) {
			nread = read(pipe_read, &irq, sizeof(irq));
			if (nread < 0) {
				perror("unable to clear timer");
				goto Done;
			}
			
			printf("Timer\r\n");
		}
	}
	#endif

Done:
	if (fd >= 0) {
		close(fd);
	}
	if (pipe_read >= 0) {
		close(pipe_read);
	}
	if (pipe_write >= 0) {
		close(pipe_write);
	}
	return ret;


}


