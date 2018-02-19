
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
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/poll.h>
#include "include/sdr_def.h"
#include "include/nvic_ext.h"
#include "include/m2sxxx.h"
#include "sdr_gpio.h"
#include "sdr_pdma.h"


static DECLARE_WAIT_QUEUE_HEAD(proc);
static int wake_flag;

unsigned int sdr_major = 168;
unsigned int sdr_minor = 0;
const char * sdr_name = "uSDR Driver";

struct sdr_dev* sdr_dev_ptr;

#define IRQn_FIFO_SIZE 64
spinlock_t irqn_spinlock = SPIN_LOCK_UNLOCKED;
static DEFINE_KFIFO(irqn_kfifo, IRQn_FIFO_SIZE);

const unsigned short g_pdma_status_mask[NB_OF_PDMA_CHANNELS] = {
	CHANNEL_0_STATUS_BITS_MASK,
	CHANNEL_1_STATUS_BITS_MASK,
	CHANNEL_2_STATUS_BITS_MASK,
	CHANNEL_3_STATUS_BITS_MASK,
	CHANNEL_4_STATUS_BITS_MASK,
	CHANNEL_5_STATUS_BITS_MASK,
	CHANNEL_6_STATUS_BITS_MASK,
	CHANNEL_7_STATUS_BITS_MASK
};

irqreturn_t gpio_irq_handler(int irq, void* dev_id){
	M2S_GPIO_clear_irq((M2S_GPIO_Type)(irq-GPIO_IRQ_BASE));	// 50 is the base
	kfifo_in_locked(&irqn_kfifo, &irq, sizeof(int), &irqn_spinlock);
	wake_flag = 1;
	wake_up_interruptible(&proc);
	return IRQ_HANDLED;
}

irqreturn_t pdma_irq_handler(int irq, void* dev_id){
	unsigned short status = (unsigned short)PDMA->BUFFER_STATUS;
	pdma_channel_id_t channel_id;
	wake_flag = 1;
	wake_up_interruptible(&proc);
	kfifo_in_locked(&irqn_kfifo, &irq, sizeof(int), &irqn_spinlock);
	do {
		channel_id = get_channel_id_from_status( status );
		status &= (unsigned short)~g_pdma_status_mask[channel_id];
	}
	while ( 0U != status );
	M2S_PDMA_clear_irq(channel_id);
	return IRQ_HANDLED;
}

void M2S_GPIO_clear_irq(M2S_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO){
		M2S_GPIO_DEV->irq = (unsigned int)(1<<gpio_idx);
		nvic_clr_pen_irq((unsigned int)m2s_gpio_irq_lut[gpio_idx]);
	}
}

void M2S_GPIO_disable_irq(M2S_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO){
		M2S_GPIO_DEV->cfg[gpio_idx] &= ~GPIO_INT_ENABLE_MASK;
		nvic_disable_irq(m2s_gpio_irq_lut[gpio_idx]);
	}
}

void M2S_GPIO_init(){
	int i;
	M2S_SYSREG->soft_reset_cr |= 	SYSREG_GPIO_SOFTRESET_MASK;
	M2S_SYSREG->soft_reset_cr |= (	SYSREG_GPIO_7_0_SOFTRESET_MASK | 
									SYSREG_GPIO_15_8_SOFTRESET_MASK |
									SYSREG_GPIO_23_16_SOFTRESET_MASK |
									SYSREG_GPIO_31_24_SOFTRESET_MASK );
	for (i=0; i<NB_OF_GPIO; i++){
		nvic_disable_irq(m2s_gpio_irq_lut[i]);
		nvic_clr_pen_irq(m2s_gpio_irq_lut[i]);
	}
	M2S_SYSREG->soft_reset_cr &= ~(	SYSREG_GPIO_7_0_SOFTRESET_MASK | 
									SYSREG_GPIO_15_8_SOFTRESET_MASK |
									SYSREG_GPIO_23_16_SOFTRESET_MASK |
									SYSREG_GPIO_31_24_SOFTRESET_MASK );
	M2S_SYSREG->soft_reset_cr &= 	~SYSREG_GPIO_SOFTRESET_MASK;
}

void M2S_GPIO_enable_irq(M2S_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO){
		M2S_GPIO_DEV->cfg[gpio_idx] |= (GPIO_IRQ_EDGE_POSITIVE | GPIO_INT_ENABLE_MASK);	// set positive edge trigger
		nvic_enable_irq(m2s_gpio_irq_lut[gpio_idx]);
	}
}

/* PDMA */
void M2S_PDMA_init(){
    /* Reset PDMA block. */
    M2S_SYSREG->soft_reset_cr |= PDMA_SOFT_RESET;
    /* Clear any previously pended MSS PDMA interrupt */
    nvic_clr_pen_irq(M2S_DMA_IRQn);
    /* Take PDMA controller out of reset*/
    M2S_SYSREG->soft_reset_cr &= ~PDMA_SOFT_RESET;
	// Rest of initialization goes in user space driver
}

inline void M2S_PDMA_enable_irq(pdma_channel_id_t channel_id){
    PDMA->CHANNEL[channel_id].CTRL |= PDMA_IRQ_ENABLE_MASK;
    nvic_enable_irq(M2S_DMA_IRQn);
}

inline void M2S_PDMA_disable_irq(pdma_channel_id_t channel_id){
    PDMA->CHANNEL[channel_id].CTRL &= ~PDMA_IRQ_ENABLE_MASK;
}

inline void M2S_PDMA_clear_irq(pdma_channel_id_t channel_id){
	PDMA->CHANNEL[channel_id].CTRL |= CLEAR_PORT_A_DONE_MASK;
	PDMA->CHANNEL[channel_id].CTRL |= CLEAR_PORT_B_DONE_MASK;
}

pdma_channel_id_t get_channel_id_from_status(unsigned short status){
	pdma_channel_id_t channel_id = PDMA_CHANNEL_0;

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

int sleep_proc(void){
	int rc = wait_event_interruptible(proc, wake_flag != 0);

	if (rc >= 0)
		wake_flag = 0;

	return rc;
}

int sdr_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg){
	int err = 0;
	int retval = 0;
	//int irq_called;
	pdma_channel_id_t channel_id;
	GPIO_DEV_t* gpio_dev = kmalloc(sizeof(GPIO_DEV_t), GFP_KERNEL);

	if (_IOC_TYPE(cmd) != SDR_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SDR_IOC_MAXNR) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd){
		case GPIO_IOC_SET_VAL:
			retval = copy_from_user(gpio_dev, (GPIO_DEV_t __user *) arg, sizeof(GPIO_DEV_t));
			gpio_set_value(gpio_dev->m2s_gpio_t, gpio_dev->value);
		break;

		case GPIO_IOC_SET_DIRECTION:
			retval = copy_from_user(gpio_dev, (GPIO_DEV_t __user *) arg, sizeof(GPIO_DEV_t));
			if (gpio_dev->m2s_gpio_dir_t==DIR_INPUT)
				gpio_direction_input(gpio_dev->m2s_gpio_t);
			else
				gpio_direction_output(gpio_dev->m2s_gpio_t, gpio_dev->value);
		break;

		case GPIO_IOC_GET_VAL:
			retval = copy_from_user(gpio_dev, (GPIO_DEV_t __user *) arg, sizeof(GPIO_DEV_t));
			return gpio_get_value(gpio_dev->m2s_gpio_t);
		break;

		case GPIO_IOC_ENABLE_IRQ:
			retval = copy_from_user(gpio_dev, (GPIO_DEV_t __user *) arg, sizeof(GPIO_DEV_t));
			M2S_GPIO_enable_irq(gpio_dev->m2s_gpio_t);
		break;

		case GPIO_IOC_DISABLE_IRQ:
			retval = copy_from_user(gpio_dev, (GPIO_DEV_t __user *) arg, sizeof(GPIO_DEV_t));
			M2S_GPIO_disable_irq(gpio_dev->m2s_gpio_t);
		break;

		case PDMA_IOC_ENABLE_IRQ:
			retval = __get_user(channel_id, (pdma_channel_id_t __user *)arg);
			M2S_PDMA_enable_irq(channel_id);
		break;

		case PDMA_IOC_DISABLE_IRQ:
			retval = __get_user(channel_id, (pdma_channel_id_t __user *)arg);
			M2S_PDMA_disable_irq(channel_id);
		break;

		/*
		case SDR_IOC_SLEEP_PROC:
			if (kfifo_is_empty(&irqn_kfifo))
				sleep_proc();

			if (kfifo_out_locked(&irqn_kfifo, &irq_called, sizeof(int), &irqn_spinlock))
				retval = __put_user(irq_called, (int __user *)arg);
			else {
				printk(KERN_INFO "abort!\r\n");
				return -1;
			}
		break;
		*/

	  	default:  
		return -ENOTTY;
	}
	kfree(gpio_dev);
	return retval;
}

static int sdr_release(struct inode* inode, struct file* filp){
	int i;
	for (i=0; i<NB_OF_GPIO; i++){
		gpio_free(m2s_gpio_lut[i]);
		M2S_GPIO_disable_irq((M2S_GPIO_Type)i);
		free_irq((unsigned int)m2s_gpio_irq_lut[i], NULL);
	}
	free_irq((unsigned int)M2S_DMA_IRQn, NULL);
	return 0;
}

static int sdr_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int rc = 0;
	int irq_called;

	if (count != sizeof(int) || *f_pos != 0)
		return -EINVAL;
	
	if (kfifo_is_empty(&irqn_kfifo))
	{
		rc = sleep_proc();
		if (rc < 0)
			return rc;
	}

	rc = kfifo_out_locked(&irqn_kfifo, &irq_called, sizeof(int), &irqn_spinlock);

	if (rc < 0)
		return rc;

	return __put_user(irq_called, (int __user *)buf);
}

static unsigned int sdr_poll(struct file *filp, poll_table *wait)
{
	poll_wait(filp, &proc, wait);

	if (!kfifo_is_empty(&irqn_kfifo))
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static int sdr_open(struct inode* inode, struct file* filp){
	struct sdr_dev* dev;
	int i, irq_num;
	dev = container_of(inode->i_cdev, struct sdr_dev, cdev);
	filp->private_data = dev;
	
	for (i=0; i<NB_OF_GPIO; i++){
		if (gpio_is_valid(m2s_gpio_lut[i])<0){
			printk(KERN_INFO "GPIO%d is invalid", i);
			return -1;
		}
		if (gpio_request(m2s_gpio_lut[i], "GPIO")<0){
			printk(KERN_INFO "GPIO%d is not available", i);
			return -1;
		}
		/* Registering ISR */
		irq_num = gpio_to_irq(m2s_gpio_lut[i]);
		if (request_irq((unsigned int)irq_num, gpio_irq_handler, 0, "gpio", NULL)){
			printk(KERN_INFO "Abort, cannot request irq%d\r\n.", irq_num);
			return -1;
		}
	}

	// Register PDMA IRQ handler
	if (request_irq(M2S_DMA_IRQn, pdma_irq_handler, 0, "pdma", NULL)){
		printk(KERN_INFO "Abort, cannot request irq%d\r\n.", irq_num);
		return -1;
	}
	return 0;
}

static struct file_operations sdr_fops = {
	.owner = THIS_MODULE,
	.ioctl = sdr_ioctl,
	.read = sdr_read,
	.poll = sdr_poll,
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
	unregister_chrdev_region(dev, NB_OF_GPIO);
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
		result = register_chrdev_region(dev, NB_OF_GPIO, sdr_name);
	}
	else {
		result = alloc_chrdev_region(&dev, sdr_minor, NB_OF_GPIO, sdr_name);
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

	// Initialize GPIO
	M2S_GPIO_init();

	// Initial PDMA
	M2S_PDMA_init();

	return 0;

	fail:
		sdr_cleanup_module();
		return result;
}
module_init(sdr_init_module);
module_exit(sdr_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ye-sheng Kuo");
MODULE_DESCRIPTION("M2S GPIO Driver");

