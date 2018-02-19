
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "m2s_sdr.h"

int gpio_set_value(int fd, M2S_GPIO_Type gpio_type, unsigned val){
	GPIO_DEV_t* gpio = malloc(sizeof(GPIO_DEV_t));
	gpio->m2s_gpio_t = gpio_type;
	gpio->value = val;
	if (ioctl(fd, GPIO_IOC_SET_VAL, gpio)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		return -1;
	}
	free(gpio);
	return 0;
}

int gpio_set_direction(int fd, M2S_GPIO_Type gpio_type, M2S_GPIO_DIR_Type direction){
	GPIO_DEV_t* gpio = malloc(sizeof(GPIO_DEV_t));
	gpio->m2s_gpio_t = gpio_type;
	gpio->m2s_gpio_dir_t = direction;
	if (ioctl(fd, GPIO_IOC_SET_DIRECTION, gpio)<0){
		fprintf(stderr, "unable to set direction: %s\n", strerror(errno));
		return -1;
	}
	free(gpio);
	return 0;
}

int gpio_get_value(int fd, M2S_GPIO_Type gpio_type){
	int val;
	GPIO_DEV_t* gpio = malloc(sizeof(GPIO_DEV_t));
	gpio->m2s_gpio_t = gpio_type;
	val = ioctl(fd, GPIO_IOC_GET_VAL, gpio);
	if (val<0){
		fprintf(stderr, "unable to set direction: %s\n", strerror(errno));
		return -1;
	}
	free(gpio);
	return val;
}

int gpio_enable_irq(int fd, M2S_GPIO_Type gpio_type){
	GPIO_DEV_t* gpio = malloc(sizeof(GPIO_DEV_t));
	gpio->m2s_gpio_t = gpio_type;
	if (ioctl(fd, GPIO_IOC_ENABLE_IRQ, gpio)<0){
		fprintf(stderr, "unable to set interrupt: %s\n", strerror(errno));
		return -1;
	}
	free(gpio);
	return 0;
}

int gpio_disable_irq(int fd, M2S_GPIO_Type gpio_type){
	GPIO_DEV_t* gpio = malloc(sizeof(GPIO_DEV_t));
	gpio->m2s_gpio_t = gpio_type;
	if (ioctl(fd, GPIO_IOC_DISABLE_IRQ, gpio)<0){
		fprintf(stderr, "unable to set interrupt: %s\n", strerror(errno));
		return -1;
	}
	free(gpio);
	return 0;
}

int pdma_enable_irq(int fd, pdma_channel_id_t channel){
	if (ioctl(fd, PDMA_IOC_ENABLE_IRQ, &channel)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

int pdma_disable_irq(int fd, pdma_channel_id_t channel){
	if (ioctl(fd, PDMA_IOC_DISABLE_IRQ, &channel)<0){
		fprintf(stderr, "unable to set gpio: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

