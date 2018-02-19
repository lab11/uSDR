
#ifndef _SDR_H_
#define _SDR_H_

#include <linux/ioctl.h>
#include "include/sdr_def.h"
#include "include/a2f/a2fm3.h"

struct sdr_dev{
	IRQn_Type gpio_irq[NB_OF_GPIO];
	IRQn_Type timer2_irq;
	IRQn_Type pdma_irq;
	struct cdev cdev;
};

static const IRQn_Type gpio_irq_lut[NB_OF_GPIO] = {
	GPIO0_IRQn,
	GPIO1_IRQn,
	GPIO2_IRQn,
	GPIO3_IRQn,
	GPIO4_IRQn,
	GPIO5_IRQn,
	GPIO6_IRQn,
	GPIO7_IRQn,
	GPIO8_IRQn,
	GPIO9_IRQn,
	GPIO10_IRQn,
	GPIO11_IRQn,
	GPIO12_IRQn,
	GPIO13_IRQn,
	GPIO14_IRQn,
	GPIO15_IRQn,
	GPIO16_IRQn,
	GPIO17_IRQn,
	GPIO18_IRQn,
	GPIO19_IRQn,
	GPIO20_IRQn,
	GPIO21_IRQn,
	GPIO22_IRQn,
	GPIO23_IRQn,
	GPIO24_IRQn,
	GPIO25_IRQn,
	GPIO26_IRQn,
	GPIO27_IRQn,
	GPIO28_IRQn,
	GPIO29_IRQn,
	GPIO30_IRQn,
	GPIO31_IRQn
};


static const MSS_GPIO_Type gpio_type_lut[NB_OF_GPIO] = {
	MSS_GPIO_0,
	MSS_GPIO_1,
	MSS_GPIO_2,
	MSS_GPIO_3,
	MSS_GPIO_4,
	MSS_GPIO_5,
	MSS_GPIO_6,
	MSS_GPIO_7,
	MSS_GPIO_8,
	MSS_GPIO_9,
	MSS_GPIO_10,
	MSS_GPIO_11,
	MSS_GPIO_12,
	MSS_GPIO_13,
	MSS_GPIO_14,
	MSS_GPIO_15,
	MSS_GPIO_16,
	MSS_GPIO_17,
	MSS_GPIO_18,
	MSS_GPIO_19,
	MSS_GPIO_20,
	MSS_GPIO_21,
	MSS_GPIO_22,
	MSS_GPIO_23,
	MSS_GPIO_24,
	MSS_GPIO_25,
	MSS_GPIO_26,
	MSS_GPIO_27,
	MSS_GPIO_28,
	MSS_GPIO_29,
	MSS_GPIO_30,
	MSS_GPIO_31
};

#define GPIO_INT_ENABLE_MASK        0x00000008UL

void MSS_GPIO_clear_irq(MSS_GPIO_Type port_id);
void MSS_GPIO_disable_irq(MSS_GPIO_Type port_id);
void MSS_GPIO_enable_irq(MSS_GPIO_Type port_id);
void MSS_GPIO_init(void);


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

static const unsigned short g_pdma_status_mask[NB_OF_PDMA_CHANNELS] = {
	(unsigned short)0x0003, /* PDMA_CHANNEL_0 */
	(unsigned short)0x000C, /* PDMA_CHANNEL_1 */
	(unsigned short)0x0030, /* PDMA_CHANNEL_2 */
	(unsigned short)0x00C0, /* PDMA_CHANNEL_3 */
	(unsigned short)0x0300, /* PDMA_CHANNEL_4 */
	(unsigned short)0x0C00, /* PDMA_CHANNEL_5 */
	(unsigned short)0x3000, /* PDMA_CHANNEL_6 */
	(unsigned short)0xC000, /* PDMA_CHANNEL_7 */
};

irqreturn_t sdr_irq_handler(int irq, void* dev_id, struct pt_regs* regs);
int sdr_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static int sdr_release(struct inode* inode, struct file* filp);
static int sdr_open(struct inode* inode, struct file* filp);
static void sdr_cleanup_module(void);
static void sdr_setup_cdev(struct sdr_dev *dev);
static int sdr_init_module(void);


#endif
