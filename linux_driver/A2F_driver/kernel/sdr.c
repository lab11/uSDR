
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include "sdr.h"
#include "include/a2f/nvic_ext.c"


static DECLARE_WAIT_QUEUE_HEAD(proc);
static int wake_flag;

unsigned int sdr_major = 168;
unsigned int sdr_minor = 0;
const char * sdr_name = "uSDR Driver";

struct sdr_dev* sdr_dev_ptr;

#define IRQn_FIFO_SIZE 64
spinlock_t irqn_spinlock = SPIN_LOCK_UNLOCKED;
static DEFINE_KFIFO(irqn_kfifo, IRQn_FIFO_SIZE);

irqreturn_t gpio_irq_handler(int irq, void* dev_id){
	MSS_GPIO_clear_irq((MSS_GPIO_Type)(irq%32));
	kfifo_in_locked(&irqn_kfifo, &irq, sizeof(int), &irqn_spinlock);
	wake_flag = 1;
	wake_up_interruptible(&proc);
	return IRQ_HANDLED;
}

void MSS_GPIO_clear_irq(MSS_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO){
		MSS_GPIO->irq = (unsigned int)(1<<gpio_idx);
		nvic_clr_pen_irq((unsigned int)gpio_irq_lut[gpio_idx]);
	}
}

void MSS_GPIO_disable_irq(MSS_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO){
		MSS_GPIO->cfg[gpio_idx] &= ~GPIO_INT_ENABLE_MASK;
		nvic_disable_irq(gpio_irq_lut[gpio_idx]);
	}
}

void MSS_GPIO_enable_irq(MSS_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO){
		MSS_GPIO->cfg[gpio_idx] |= GPIO_INT_ENABLE_MASK;
		nvic_enable_irq(gpio_irq_lut[gpio_idx]);
	}
}

void MSS_GPIO_init(void){
	int i;
	A2F_SYSREG->soft_rst_cr |= SYSREG_GPIO_SOFTRESET_MASK;
	for (i=0; i<NB_OF_GPIO; i++)
		nvic_clr_pen_irq(gpio_irq_lut[i]);
	A2F_SYSREG->soft_rst_cr &= ~SYSREG_GPIO_SOFTRESET_MASK;
}

irqreturn_t timer2_irq_handler(int irq, void* dev_id){
	wake_flag = 1;
	wake_up_interruptible(&proc);
	kfifo_in_locked(&irqn_kfifo, &irq, sizeof(int), &irqn_spinlock);
	MSS_TIMER2_clear_irq();
	return IRQ_HANDLED;
}

static inline void MSS_TIMER2_clear_irq(void){
	MSS_TIMER2->tim2_ris = 1;
	nvic_clr_pen_irq(TIMER2_IRQn);
}

static inline void MSS_TIMER2_disable_irq(void){
	MSS_TIMER2_BIT_BAND->tim2inten = 0;
	nvic_disable_irq(TIMER2_IRQn);
}

static inline void MSS_TIMER2_enable_irq(void){
	MSS_TIMER2_BIT_BAND->tim2inten = 1;
	nvic_enable_irq(TIMER2_IRQn);
}

void MSS_TIMER2_init(MSS_TIMER2_MODE_Type mode){
	nvic_disable_irq(TIMER2_IRQn);
	A2F_SYSREG->soft_rst_cr &= ~SYSREG_TIMER_SOFTRESET_MASK;
	MSS_TIMER2_BIT_BAND->tim2enable = 0;
	MSS_TIMER2_BIT_BAND->tim2inten = 0;
	MSS_TIMER2_BIT_BAND->tim2mode = (unsigned int)mode;
	MSS_TIMER2->tim2_ris = 1;
	nvic_clr_pen_irq(TIMER2_IRQn);
}

irqreturn_t pdma_irq_handler(int irq, void* dev_id){
	unsigned short status = (unsigned short)MSS_PDMA->BUFFER_STATUS;
	PDMA_CHANNEL_ID_Type channel_id;
	wake_flag = 1;
	wake_up_interruptible(&proc);
	kfifo_in_locked(&irqn_kfifo, &irq, sizeof(int), &irqn_spinlock);
	do {
		channel_id = get_channel_id_from_status( status );
		status &= (unsigned short)~g_pdma_status_mask[channel_id];
	}
	while ( 0U != status );
	MSS_PDMA_clear_irq(channel_id);
	return IRQ_HANDLED;
}

static inline void MSS_PDMA_clear_irq(PDMA_CHANNEL_ID_Type channel_id){
	/* Clear interrupt in PDMA controller. */
	MSS_PDMA->CHANNEL[channel_id].CRTL |= CLEAR_PORT_A_DONE_MASK;
	MSS_PDMA->CHANNEL[channel_id].CRTL |= CLEAR_PORT_B_DONE_MASK;

	/* Clear interrupt in Cortex-M3 NVIC. */
	nvic_clr_pen_irq(PDMA_IRQn);
}

static inline void MSS_PDMA_disable_irq(PDMA_CHANNEL_ID_Type channel_id){
	MSS_PDMA->CHANNEL[channel_id].CRTL &= ~PDMA_IRQ_ENABLE_MASK;
	nvic_disable_irq(PDMA_IRQn);
}

static inline void MSS_PDMA_enable_irq(PDMA_CHANNEL_ID_Type channel_id){
	MSS_PDMA->CHANNEL[channel_id].CRTL |= PDMA_IRQ_ENABLE_MASK;
	nvic_enable_irq(PDMA_IRQn);
}

void MSS_PDMA_init(void){
	A2F_SYSREG->ahb_matrix_cr |= PDMA_MASTER_ENABLE;
	A2F_SYSREG->soft_rst_cr |= PDMA_SOFT_RESET;
	nvic_clr_pen_irq(PDMA_IRQn);
	A2F_SYSREG->soft_rst_cr &= ~PDMA_SOFT_RESET;
}

PDMA_CHANNEL_ID_Type get_channel_id_from_status(unsigned short status){
	PDMA_CHANNEL_ID_Type channel_id = PDMA_CHANNEL_0;

	if ( status & CHANNEL_0_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_0;
	}
	else if ( status & CHANNEL_1_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_1;
	}
	else if ( status & CHANNEL_2_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_2;
	}
	else if ( status & CHANNEL_3_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_3;
	}
	else if ( status & CHANNEL_4_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_4;
	}
	else if ( status & CHANNEL_5_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_5;
	}
	else if ( status & CHANNEL_6_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_6;
	}
	else if ( status & CHANNEL_7_STATUS_BITS_MASK ){
		channel_id = PDMA_CHANNEL_7;
	}

	return channel_id;
}

void sleep_proc(void){
	wait_event_interruptible(proc, wake_flag != 0);
	wake_flag = 0;
}

int sdr_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg){
	int err = 0;
	int retval = 0;
	int irq_called;

	MSS_GPIO_Type port_id;
	MSS_TIMER2_MODE_Type mode;
	PDMA_CHANNEL_ID_Type channel_id;

	if (_IOC_TYPE(cmd) != SDR_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SDR_IOC_MAXNR) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd){
		case GPIO_IOC_ENABLE_IRQ:
			retval = __get_user(port_id, (MSS_GPIO_Type __user *) arg); 
			MSS_GPIO_enable_irq(port_id);
		break;

		case GPIO_IOC_DISABLE_IRQ:
			retval = __get_user(port_id, (MSS_GPIO_Type __user *) arg); 
			MSS_GPIO_disable_irq(port_id);
		break;

		case TIMER2_IOC_INIT:
			retval = __get_user(mode, (MSS_TIMER2_MODE_Type __user *) arg); 
			MSS_TIMER2_init(mode);
		break;

		case TIMER2_IOC_ENABLE_IRQ:
			MSS_TIMER2_enable_irq();
		break;

		case TIMER2_IOC_DISABLE_IRQ:
			MSS_TIMER2_disable_irq();
		break;

		case PDMA_IOC_ENABLE_IRQ:
			retval = __get_user(channel_id, (PDMA_CHANNEL_ID_Type __user *)arg);
			MSS_PDMA_enable_irq(channel_id);
		break;

		case PDMA_IOC_DISABLE_IRQ:
			retval = __get_user(channel_id, (PDMA_CHANNEL_ID_Type __user *)arg);
			MSS_PDMA_disable_irq(channel_id);
		break;

		case SDR_IOC_SLEEP_PROC:
			if (kfifo_is_empty(&irqn_kfifo))
				sleep_proc();

			/*
			if (kfifo_out_locked(&irqn_kfifo, &irq_called, sizeof(int), &irqn_spinlock))
				retval = __put_user(irq_called, (int __user *)arg);
			else
				printk(KERN_INFO "Abort, cannot get irq from fifo\r\n.");
			*/
			unsigned ret = kfifo_out_locked(&irqn_kfifo, &irq_called, sizeof(int), &irqn_spinlock);
			retval = __put_user(irq_called, (int __user *)arg);

		break;

	  	default:  
		return -ENOTTY;
	}
	return retval;
}

static int sdr_release(struct inode* inode, struct file* filp){
	int i;
	for (i = 0; i < NB_OF_GPIO; i++) {
		MSS_GPIO_disable_irq((MSS_GPIO_Type)i);
		free_irq((unsigned int)sdr_dev_ptr->gpio_irq[i], NULL);
	}

	MSS_TIMER2_disable_irq();
	free_irq((unsigned int)sdr_dev_ptr->timer2_irq, NULL);

	for (i=0; i<NB_OF_PDMA_CHANNELS; i++)
		MSS_PDMA_disable_irq((PDMA_CHANNEL_ID_Type)i);
	free_irq((unsigned int)sdr_dev_ptr->pdma_irq, NULL);
	return 0;
}

static int sdr_open(struct inode* inode, struct file* filp){
	struct sdr_dev *dev;
	int i;
	dev = container_of(inode->i_cdev, struct sdr_dev, cdev);
	filp->private_data = dev;
	/* Registering ISR */
	for (i=0; i<NB_OF_GPIO; i++){
		sdr_dev_ptr->gpio_irq[i] = gpio_irq_lut[i];
		if (request_irq((unsigned int)sdr_dev_ptr->gpio_irq[i], gpio_irq_handler, 0, "gpio", NULL)){
			printk(KERN_INFO "Abort, cannot get assigned irq%d\r\n.", (int)gpio_irq_lut[i]);
			return -1;
		}
	}
	MSS_GPIO_init();

	sdr_dev_ptr->timer2_irq = TIMER2_IRQn;
	if (request_irq((unsigned int)sdr_dev_ptr->timer2_irq, timer2_irq_handler, 0, "MSS Timer2", NULL)){
		printk(KERN_INFO "Abort, cannot get assigned irq%d\r\n.", (int)TIMER2_IRQn);
		return -1;
	}

	sdr_dev_ptr->pdma_irq = PDMA_IRQn;
	if (request_irq((unsigned int)sdr_dev_ptr->pdma_irq, pdma_irq_handler, 0, "MSS PDMA", NULL)){
		printk(KERN_INFO "Abort, cannot get assigned irq%d\r\n.", (int)PDMA_IRQn);
		return -1;
	}
	/* Init PDMA */
	MSS_PDMA_init();

	return 0;
}

static struct file_operations sdr_fops = {
	.owner = THIS_MODULE,
	.ioctl = sdr_ioctl,
	.open = sdr_open,
	.release = sdr_release
};

static void sdr_cleanup_module(void){
	dev_t dev;
	dev = MKDEV(sdr_major, sdr_minor);

	/* Get rid of our char dev entries */
	if (sdr_dev_ptr) {
		cdev_del(&sdr_dev_ptr->cdev);
		kfree(sdr_dev_ptr);
	}
	unregister_chrdev_region(dev, (NB_OF_GPIO + NB_OF_PDMA_CHANNELS + 1));
}

static void sdr_setup_cdev(struct sdr_dev *dev){
	int dev_no, err;

	dev_no = MKDEV(sdr_major, sdr_minor);
	cdev_init(&(dev->cdev), &sdr_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &sdr_fops;

	err = cdev_add(&(dev->cdev), dev_no, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding GPIO device\r\n", err);
}

static int sdr_init_module(void){
	int result = 0;
	dev_t dev;

	/* initiate device number */
	if (sdr_major){
		dev = MKDEV(sdr_major, sdr_minor);
		result = register_chrdev_region(dev, (NB_OF_GPIO + NB_OF_PDMA_CHANNELS + 1), sdr_name);
	}
	else {
		result = alloc_chrdev_region(&dev, sdr_minor, (NB_OF_GPIO + NB_OF_PDMA_CHANNELS + 1), sdr_name);
		sdr_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "GPIO: can't get major %d\n", sdr_major);
		return result;
	}

	/* allocate devices */
	sdr_dev_ptr = kmalloc(sizeof(struct sdr_dev), GFP_KERNEL);
	if (!sdr_dev_ptr) {
		result = -ENOMEM;
		goto fail;  /* Make this more graceful */
	}

	/* setup devices */
	sdr_setup_cdev(sdr_dev_ptr);
	wake_flag = 0;

	return 0;

	fail:
		sdr_cleanup_module();
		return result;
}
module_init(sdr_init_module);
module_exit(sdr_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ye-sheng Kuo");
MODULE_DESCRIPTION("Michigan uSDRv2 Driver");

