
// only for kernel space
#ifndef _M2SXXX_H_
#define _M2SXXX_H_

#define M2S_GPIO_BASE 0x40013000
struct mss_gpio{
	unsigned int cfg[32];
	unsigned int irq;
	unsigned int in;
	unsigned int out;
};
#define M2S_GPIO_DEV ((struct mss_gpio*)(M2S_GPIO_BASE))

#endif
