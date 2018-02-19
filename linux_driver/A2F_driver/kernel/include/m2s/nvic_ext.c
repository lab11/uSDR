
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/smp.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <asm/hardware/nvic.h>
//#include <linux/arch/arm/common/nvic.c>

static DEFINE_SPINLOCK(irq_controller_lock);

static void nvic_clr_pen_irq(unsigned int irq)
{
	u32 mask = 1 << (irq % 32);
	spin_lock(&irq_controller_lock);
	iowrite32(mask, NVIC_CLEAR_PENDING + irq / 32 * 4);
	spin_unlock(&irq_controller_lock);
}

static void nvic_enable_irq(unsigned int irq)
{
	u32 mask = 1 << (irq % 32);
	spin_lock(&irq_controller_lock);
	iowrite32(mask, NVIC_SET_ENABLE + irq / 32 * 4);
	spin_unlock(&irq_controller_lock);
}

static void nvic_disable_irq(unsigned int irq)
{
	u32 mask = 1 << (irq % 32);
	spin_lock(&irq_controller_lock);
	iowrite32(mask, NVIC_CLEAR_ENABLE + irq / 32 * 4);
	spin_unlock(&irq_controller_lock);
}
