
#ifndef _M2S_SDR_H_
#define _M2S_SDR_H_
#include "sdr_def.h"

int gpio_set_value(int fd, M2S_GPIO_Type gpio_type, unsigned val);

int gpio_set_direction(int fd, M2S_GPIO_Type gpio_type, M2S_GPIO_DIR_Type direction);

int gpio_get_value(int fd, M2S_GPIO_Type gpio_type);

int gpio_enable_irq(int fd, M2S_GPIO_Type gpio_type);

int gpio_disable_irq(int fd, M2S_GPIO_Type gpio_type);

int pdma_enable_irq(int fd, pdma_channel_id_t channel);

int pdma_disable_irq(int fd, pdma_channel_id_t channel);

#endif

