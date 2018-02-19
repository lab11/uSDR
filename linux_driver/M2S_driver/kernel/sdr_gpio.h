
#ifndef _SDR_GPIO_H_
#define _SDR_GPIO_H_

#include <linux/ioctl.h>
#include "include/sdr_def.h"
#include "include/m2sxxx.h"

struct sdr_dev{
	struct cdev cdev;
};

#define GPIO_INT_ENABLE_MASK        0x00000008
#define GPIO_IRQ_EDGE_POSITIVE		0x00000040

#define GPIO_IRQ_BASE 50

static const IRQn_Type m2s_gpio_irq_lut[NB_OF_GPIO] = {
	M2S_GPIO0_IRQn,
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
	M2S_GPIO31_IRQn
};

static const M2S_GPIO_Type m2s_gpio_lut[NB_OF_GPIO]={
	M2S_GPIO_0,
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
	M2S_GPIO_31,
};

void M2S_GPIO_clear_irq(M2S_GPIO_Type port_id);
void M2S_GPIO_disable_irq(M2S_GPIO_Type port_id);
void M2S_GPIO_enable_irq(M2S_GPIO_Type port_id);
void M2S_GPIO_init(void);
/*

static inline void MSS_TIMER2_clear_irq(void);
static inline void MSS_TIMER2_disable_irq(void);
static inline void MSS_TIMER2_enable_irq(void);
void MSS_TIMER2_init(MSS_TIMER2_MODE_Type mode);


#define PDMA_MASTER_ENABLE  0x04
#define PDMA_SOFT_RESET     0x20

#define CHANNEL_0_STATUS_BITS_MASK     (unsigned short)0x0003
#define CHANNEL_1_STATUS_BITS_MASK     (unsigned short)0x000C
#define CHANNEL_2_STATUS_BITS_MASK     (unsigned short)0x0030
#define CHANNEL_3_STATUS_BITS_MASK     (unsigned short)0x00C0
#define CHANNEL_4_STATUS_BITS_MASK     (unsigned short)0x0300
#define CHANNEL_5_STATUS_BITS_MASK     (unsigned short)0x0C00
#define CHANNEL_6_STATUS_BITS_MASK     (unsigned short)0x3000
#define CHANNEL_7_STATUS_BITS_MASK     (unsigned short)0xC000

#define PDMA_IRQ_ENABLE_MASK    	0x00000040
static inline void MSS_PDMA_clear_irq(PDMA_CHANNEL_ID_Type channel_id);
static inline void MSS_PDMA_disable_irq(PDMA_CHANNEL_ID_Type channel_id);
static inline void MSS_PDMA_enable_irq(PDMA_CHANNEL_ID_Type channel_id);
void MSS_PDMA_init(void);
PDMA_CHANNEL_ID_Type get_channel_id_from_status(unsigned short status);
*/
//static const unsigned short g_pdma_status_mask[NB_OF_PDMA_CHANNELS] = {
//	(unsigned short)0x0003, /* PDMA_CHANNEL_0 */
//	(unsigned short)0x000C, /* PDMA_CHANNEL_1 */
//	(unsigned short)0x0030, /* PDMA_CHANNEL_2 */
//	(unsigned short)0x00C0, /* PDMA_CHANNEL_3 */
//	(unsigned short)0x0300, /* PDMA_CHANNEL_4 */
//	(unsigned short)0x0C00, /* PDMA_CHANNEL_5 */
//	(unsigned short)0x3000, /* PDMA_CHANNEL_6 */
//	(unsigned short)0xC000, /* PDMA_CHANNEL_7 */
//};

irqreturn_t sdr_irq_handler(int irq, void* dev_id, struct pt_regs* regs);
int sdr_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static int sdr_release(struct inode* inode, struct file* filp);
static int sdr_open(struct inode* inode, struct file* filp);
static void sdr_cleanup_module(void);
static void sdr_setup_cdev(struct sdr_dev *dev);
static int sdr_init_module(void);


#endif
