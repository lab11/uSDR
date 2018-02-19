
// This file need to be included for both kernel and user-space
#ifndef _SDR_DEF_H_
#define _SDR_DEF_H_

typedef enum {
	M2S_DMA_IRQn = 13,
	M2S_GPIO0_IRQn = 50,
	M2S_GPIO1_IRQn,
	M2S_GPIO2_IRQn,
	M2S_GPIO3_IRQn,
	M2S_GPIO4_IRQn,
	M2S_GPIO5_IRQn,
	M2S_GPIO6_IRQn,
	M2S_GPIO7_IRQn,
	M2S_GPIO8_IRQn,
	M2S_GPIO9_IRQn,
	M2S_GPIO10_IRQn,
	M2S_GPIO11_IRQn,
	M2S_GPIO12_IRQn,
	M2S_GPIO13_IRQn,
	M2S_GPIO14_IRQn,
	M2S_GPIO15_IRQn,
	M2S_GPIO16_IRQn,
	M2S_GPIO17_IRQn,
	M2S_GPIO18_IRQn,
	M2S_GPIO19_IRQn,
	M2S_GPIO20_IRQn,
	M2S_GPIO21_IRQn,
	M2S_GPIO22_IRQn,
	M2S_GPIO23_IRQn,
	M2S_GPIO24_IRQn,
	M2S_GPIO25_IRQn,
	M2S_GPIO26_IRQn,
	M2S_GPIO27_IRQn,
	M2S_GPIO28_IRQn,
	M2S_GPIO29_IRQn,
	M2S_GPIO30_IRQn,
	M2S_GPIO31_IRQn,
} IRQn_Type;

#define NB_OF_GPIO 32

typedef enum{
	M2S_GPIO_0 = 0,
	M2S_GPIO_1,
	M2S_GPIO_2,
	M2S_GPIO_3,
	M2S_GPIO_4,
	M2S_GPIO_5,
	M2S_GPIO_6,
	M2S_GPIO_7,
	M2S_GPIO_8,
	M2S_GPIO_9,
	M2S_GPIO_10,
	M2S_GPIO_11,
	M2S_GPIO_12,
	M2S_GPIO_13,
	M2S_GPIO_14,
	M2S_GPIO_15,
	M2S_GPIO_16,
	M2S_GPIO_17,
	M2S_GPIO_18,
	M2S_GPIO_19,
	M2S_GPIO_20,
	M2S_GPIO_21,
	M2S_GPIO_22,
	M2S_GPIO_23,
	M2S_GPIO_24,
	M2S_GPIO_25,
	M2S_GPIO_26,
	M2S_GPIO_27,
	M2S_GPIO_28,
	M2S_GPIO_29,
	M2S_GPIO_30,
	M2S_GPIO_31
} M2S_GPIO_Type;

typedef enum {
	DIR_INPUT,
	DIR_OUTPUT
} M2S_GPIO_DIR_Type;

typedef struct{
	M2S_GPIO_Type m2s_gpio_t;
	M2S_GPIO_DIR_Type m2s_gpio_dir_t;
	unsigned value;
} GPIO_DEV_t;

typedef enum{
	PDMA_CHANNEL_0 = 0,
	PDMA_CHANNEL_1, 
	PDMA_CHANNEL_2, 
	PDMA_CHANNEL_3, 
	PDMA_CHANNEL_4, 
	PDMA_CHANNEL_5, 
	PDMA_CHANNEL_6, 
	PDMA_CHANNEL_7 
} pdma_channel_id_t;

#define NB_OF_PDMA_CHANNELS     8

typedef struct{
	unsigned int CTRL;
	unsigned int STATUS;
	unsigned int BUFFER_A_SRC_ADDR;
	unsigned int BUFFER_A_DEST_ADDR;
	unsigned int BUFFER_A_TRANSFER_COUNT;
	unsigned int BUFFER_B_SRC_ADDR;
	unsigned int BUFFER_B_DEST_ADDR;
	unsigned int BUFFER_B_TRANSFER_COUNT;
} PDMA_Channel_TypeDef;

typedef struct{
	unsigned int RATIO_HIGH_LOW;
	unsigned int BUFFER_STATUS;
	unsigned int RESERVED[6];
	PDMA_Channel_TypeDef CHANNEL[NB_OF_PDMA_CHANNELS];
} PDMA_TypeDef;

#define PDMA_BASE 0x40003000
#define PDMA ((PDMA_TypeDef *)PDMA_BASE)

#define CLEAR_PORT_A_DONE_MASK      0x00000080u
#define CLEAR_PORT_B_DONE_MASK      0x00000100u

#define CHANNEL_0_STATUS_BITS_MASK     (unsigned short)0x0003
#define CHANNEL_1_STATUS_BITS_MASK     (unsigned short)0x000C
#define CHANNEL_2_STATUS_BITS_MASK     (unsigned short)0x0030
#define CHANNEL_3_STATUS_BITS_MASK     (unsigned short)0x00C0
#define CHANNEL_4_STATUS_BITS_MASK     (unsigned short)0x0300
#define CHANNEL_5_STATUS_BITS_MASK     (unsigned short)0x0C00
#define CHANNEL_6_STATUS_BITS_MASK     (unsigned short)0x3000
#define CHANNEL_7_STATUS_BITS_MASK     (unsigned short)0xC000


// IOCTL definition
#define SDR_IOC_MAGIC 's'
#define GPIO_IOC_SET_DIRECTION 	_IOW(SDR_IOC_MAGIC, 0, GPIO_DEV_t)
#define GPIO_IOC_SET_VAL 		_IOW(SDR_IOC_MAGIC, 1, GPIO_DEV_t)
#define GPIO_IOC_GET_VAL 		_IOW(SDR_IOC_MAGIC, 2, GPIO_DEV_t)
#define GPIO_IOC_ENABLE_IRQ     _IOW(SDR_IOC_MAGIC, 3, GPIO_DEV_t)
#define GPIO_IOC_DISABLE_IRQ    _IOW(SDR_IOC_MAGIC, 4, GPIO_DEV_t)
#define PDMA_IOC_ENABLE_IRQ     _IOW(SDR_IOC_MAGIC, 5, pdma_channel_id_t)
#define PDMA_IOC_DISABLE_IRQ    _IOW(SDR_IOC_MAGIC, 6, pdma_channel_id_t)
//#define SDR_IOC_SLEEP_PROC      _IOR(SDR_IOC_MAGIC, 7, int)

#define SDR_IOC_MAXNR 6

#define FPGA_FABRIC_BASE 0x30000000

#endif
