
#ifndef _A2FM3_H_
#define _A2FM3_H_
#define FPGA_FABRIC_BASE 0x40050000


/* IRQn */
typedef enum IRQn{
	PDMA_IRQn	= 13,
	TIMER2_IRQn = 15,
	FabricIrq0_IRQn = 34,
	FabricIrq0_IRQn = 35,
	FabricIrq0_IRQn = 36,
	FabricIrq0_IRQn = 37,
	FabricIrq0_IRQn = 38,
	FabricIrq0_IRQn = 39,
	FabricIrq0_IRQn = 40,
	FabricIrq0_IRQn = 41,
	FabricIrq0_IRQn = 42,
	FabricIrq0_IRQn = 43,
	FabricIrq0_IRQn = 44,
	FabricIrq0_IRQn = 45,
	FabricIrq0_IRQn = 46,
	FabricIrq0_IRQn = 47,
	FabricIrq0_IRQn = 48,
	FabricIrq0_IRQn = 49,
	GPIO0_IRQn 	= 50,	/*!< GPIO 0 interrupt  */
	GPIO1_IRQn 	= 51,	/*!< GPIO 1 interrupt  */
	GPIO2_IRQn 	= 52,	/*!< GPIO 2 interrupt  */
	GPIO3_IRQn 	= 53,	/*!< GPIO 3 interrupt  */
	GPIO4_IRQn 	= 54,	/*!< GPIO 4 interrupt  */
	GPIO5_IRQn 	= 55,	/*!< GPIO 5 interrupt  */
	GPIO6_IRQn 	= 56,	/*!< GPIO 6 interrupt  */
	GPIO7_IRQn 	= 57,	/*!< GPIO 7 interrupt  */
	GPIO8_IRQn 	= 58,	/*!< GPIO 8 interrupt  */
	GPIO9_IRQn 	= 59,	/*!< GPIO 9 interrupt  */
	GPIO10_IRQn	= 60,	/*!< GPIO 10 interrupt */
	GPIO11_IRQn	= 61,	/*!< GPIO 11 interrupt */
	GPIO12_IRQn	= 62,	/*!< GPIO 12 interrupt */
	GPIO13_IRQn	= 63,	/*!< GPIO 13 interrupt */
	GPIO14_IRQn	= 64,	/*!< GPIO 14 interrupt */
	GPIO15_IRQn	= 65,	/*!< GPIO 15 interrupt */
	GPIO16_IRQn	= 66,	/*!< GPIO 16 interrupt */
	GPIO17_IRQn	= 67,	/*!< GPIO 17 interrupt */
	GPIO18_IRQn	= 68,	/*!< GPIO 18 interrupt */
	GPIO19_IRQn	= 69,	/*!< GPIO 19 interrupt */
	GPIO20_IRQn	= 70,	/*!< GPIO 20 interrupt */
	GPIO21_IRQn	= 71,	/*!< GPIO 21 interrupt */
	GPIO22_IRQn	= 72,	/*!< GPIO 22 interrupt */
	GPIO23_IRQn	= 73,	/*!< GPIO 23 interrupt */
	GPIO24_IRQn	= 74,	/*!< GPIO 24 interrupt */
	GPIO25_IRQn	= 75,	/*!< GPIO 25 interrupt */
	GPIO26_IRQn	= 76,	/*!< GPIO 26 interrupt */
	GPIO27_IRQn	= 77,	/*!< GPIO 27 interrupt */
	GPIO28_IRQn	= 78,	/*!< GPIO 28 interrupt */
	GPIO29_IRQn	= 79,	/*!< GPIO 29 interrupt */
	GPIO30_IRQn	= 80, 	/*!< GPIO 30 interrupt */
	GPIO31_IRQn	= 81 	/*!< GPIO 31 interrupt */
} IRQn_Type;

/* GPIO Struct */
#define NB_OF_GPIO 32
#define MSS_IOMUX_BASE		0xE0042100
struct mss_iomux {
	unsigned int		cr[83];
};
#define MSS_IOMUX		((struct mss_iomux *)(MSS_IOMUX_BASE))

#define MSS_GPIO_BASE		0x40013000
struct mss_gpio {
	unsigned int		cfg[32];
	unsigned int 		irq;
	unsigned int 		in;
	unsigned int 		out;
	
};
#define MSS_GPIO		((struct mss_gpio *)(MSS_GPIO_BASE))

/* TIMER2 Struct */
#define MSS_TIMER2_BASE		0x40004018
struct mss_timer2 {
	unsigned int tim2_val;
	unsigned int tim2_loadval;
	unsigned int tim2_bgloadval;
	unsigned int tim2_ctrl;
	unsigned int tim2_ris;
	unsigned int tim2_mis;
};
#define MSS_TIMER2 ((struct mss_timer2 *)(MSS_TIMER2_BASE))

/* PDMA Struct */
typedef struct {
	unsigned int CRTL;
	unsigned int STATUS;
	unsigned int BUFFER_A_SRC_ADDR;
	unsigned int BUFFER_A_DEST_ADDR;
	unsigned int BUFFER_A_TRANSFER_COUNT;
	unsigned int BUFFER_B_SRC_ADDR;
	unsigned int BUFFER_B_DEST_ADDR;
	unsigned int BUFFER_B_TRANSFER_COUNT;
} PDMA_Channel_TypeDef;

#define NB_OF_PDMA_CHANNELS	8
#define MSS_PDMA_BASE		0x40004000	
struct mss_pdma {
	unsigned int RATIO_HIGH_LOW;
	unsigned int BUFFER_STATUS;
	unsigned int RESERVED[6];
	PDMA_Channel_TypeDef CHANNEL[8];
};
#define MSS_PDMA ((struct mss_pdma*)(MSS_PDMA_BASE))

/* Bit Band Address */
// #define BITBAND_ADDRESS(X)  ((X & 0xF0000000U) + 0x02000000U + ((X & 0xFFFFFU) << 5))
#define MSS_GPIO_BIT_BAND_BASE	0x42260000
struct mss_gpio_bit_band{
	unsigned int gpio_0_cgf[32];
	unsigned int gpio_1_cgf[32];
	unsigned int gpio_2_cgf[32];
	unsigned int gpio_3_cgf[32];
	unsigned int gpio_4_cgf[32];
	unsigned int gpio_5_cgf[32];
	unsigned int gpio_6_cgf[32];
	unsigned int gpio_7_cgf[32];
	unsigned int gpio_8_cgf[32];
	unsigned int gpio_9_cgf[32];
	unsigned int gpio_10_cgf[32];
	unsigned int gpio_11_cgf[32];
	unsigned int gpio_12_cgf[32];
	unsigned int gpio_13_cgf[32];
	unsigned int gpio_14_cgf[32];
	unsigned int gpio_15_cgf[32];
	unsigned int gpio_16_cgf[32];
	unsigned int gpio_17_cgf[32];
	unsigned int gpio_18_cgf[32];
	unsigned int gpio_19_cgf[32];
	unsigned int gpio_20_cgf[32];
	unsigned int gpio_21_cgf[32];
	unsigned int gpio_22_cgf[32];
	unsigned int gpio_23_cgf[32];
	unsigned int gpio_24_cgf[32];
	unsigned int gpio_25_cgf[32];
	unsigned int gpio_26_cgf[32];
	unsigned int gpio_27_cgf[32];
	unsigned int gpio_28_cgf[32];
	unsigned int gpio_29_cgf[32];
	unsigned int gpio_30_cgf[32];
	unsigned int gpio_31_cgf[32];
	unsigned int gpio_irq[32];
	unsigned int gpio_in[32];
	unsigned int gpio_out[32];
};
#define MSS_GPIO_BIT_BAND ((struct mss_gpio_bit_band*)(MSS_GPIO_BIT_BAND_BASE))

#define MSS_TIMER2_BIT_BAND_BASE 0x420a0300
struct mss_timer2_bit_band{
	unsigned int tim2_val[32];
	unsigned int tim2_loadval[32];
	unsigned int tim2_bgloadval[32];
	unsigned int tim2enable;
	unsigned int tim2mode;
	unsigned int tim2inten;
	unsigned int tim2_ctrl[29];
	unsigned int tim2_ris[32];
	unsigned int tim2_mis[32];
};
#define MSS_TIMER2_BIT_BAND ((struct mss_timer2_bit_band *)(MSS_TIMER2_BIT_BAND_BASE))

#endif
