/*
 * mc3190-cpld.c -- Driver for CPLD glue logic in MC3190
 *
 * By Mark Kennard <markkennard4@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mc3190.h>

static void __iomem *cpld_base;
static DEFINE_SPINLOCK(cpld_lock);

static int cpld_irq;

static void mc3190_cpld_irq_noop(unsigned int irq) { }

static struct irq_chip mc3190_btuart_irq_chip = {
    .name   = "cpld-btuart",
    .ack    = mc3190_cpld_irq_noop,
    .mask   = mc3190_cpld_irq_noop,
    .unmask = mc3190_cpld_irq_noop,
};

static irqreturn_t mc3190_cpld_isr(int irq, void *dev_id)
{
    u32 status;

    do {
        status = readl(cpld_base + MC3190_CPLD_REG_ISR);
        
        if (!(status & MC3190_CPLD_ISR_MASK))
            break;

        writel(status & MC3190_CPLD_ISR_MASK, cpld_base + MC3190_CPLD_REG_ISR);
        
        if (status & MC3190_CPLD_ISR_BTUART_BIT) {
            generic_handle_irq(IRQ_MC3190_BTUART);
        }

        /* Add other aggregated CPLD sources here as they're implemented */

    } while (1);

    return IRQ_HANDLED;
}

u16 mc3190_cpld_read(unsigned int reg)
{
    unsigned long flags;
    u16 val;

    spin_lock_irqsave(&cpld_lock, flags);
    val = readw(cpld_base + reg);
    spin_unlock_irqrestore(&cpld_lock, flags);

#ifdef CONFIG_MFD_MC3190_CPLD_DEBUG
    pr_info("%s: read 0x%04x from register 0x%02x\n", __func__, val, reg);
#endif

    return val;
}
EXPORT_SYMBOL_GPL(mc3190_cpld_read);

void mc3190_cpld_regon(u16 val, unsigned int reg)
{
    unsigned long flags;

#ifdef CONFIG_MFD_MC3190_CPLD_DEBUG
    pr_info("%s: pre-read from CPLD\n", __func__);
    mc3190_cpld_read(reg);
    pr_info("%s: writing 0x%08x to register 0x%02x\n", __func__, val, reg);
#endif
    spin_lock_irqsave(&cpld_lock, flags);
    writel(val, cpld_base + reg);
    spin_unlock_irqrestore(&cpld_lock, flags);

}
EXPORT_SYMBOL_GPL(mc3190_cpld_regon);

void mc3190_cpld_regoff(u16 val, unsigned int reg)
{
    unsigned long flags;

    u32 outval = val << 16;

#ifdef CONFIG_MFD_MC3190_CPLD_DEBUG
    pr_info("%s: writing 0x%08x to register 0x%02x\n", __func__, outval, reg);
#endif
    spin_lock_irqsave(&cpld_lock, flags);
    writel(outval, cpld_base + reg);
    spin_unlock_irqrestore(&cpld_lock, flags);

}
EXPORT_SYMBOL_GPL(mc3190_cpld_regoff);

void mc3190_cpld_rmw(u16 ormask, u16 andmask, unsigned int reg)
{
    unsigned long flags;
    u16 val;

    val = (mc3190_cpld_read(reg) & andmask) | ormask;

#ifdef CONFIG_MFD_MC3190_CPLD_DEBUG
    pr_info("%s: writing 0x%04x to register 0x%02x\n", __func__, val, reg);
#endif

    spin_lock_irqsave(&cpld_lock, flags);
    writew(val, cpld_base + reg);
    spin_unlock_irqrestore(&cpld_lock, flags);

#ifdef CONFIG_MFD_MC3190_CPLD_DEBUG
    pr_info("%s: readback\n", __func__);
    mc3190_cpld_read(reg);
#endif
}
EXPORT_SYMBOL_GPL(mc3190_cpld_rmw);

int mc3190_cpld_wait_bit(unsigned int reg, u16 bit, bool set, unsigned int timeout_ms)
{
    unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);

    do {
        u16 val = mc3190_cpld_read(reg);

        if (!!(val & bit) == set)
            return 0;

        msleep(1);
    } while (time_before(jiffies, deadline));

#ifdef CONFIG_MFD_MC3190_CPLD_DEBUG
    pr_warn("%s: timed out waiting for bit 0x%04x on reg 0x%02x to %s\n",
        __func__, bit, reg, set ? "set" : "clear");
#endif
    return -ETIMEDOUT;
}
EXPORT_SYMBOL_GPL(mc3190_cpld_wait_bit);

int mc3190_cpld_regon_wait(u16 val, unsigned int reg, u16 wait_bit, unsigned int timeout_ms)
{
    mc3190_cpld_regon(val, reg);
    return mc3190_cpld_wait_bit(reg, wait_bit, true, timeout_ms);
}
EXPORT_SYMBOL_GPL(mc3190_cpld_regon_wait);

int mc3190_cpld_regoff_wait(u16 val, unsigned int reg, u16 wait_bit, unsigned int timeout_ms)
{
    mc3190_cpld_regoff(val, reg);
    return mc3190_cpld_wait_bit(reg, wait_bit, false, timeout_ms);
}
EXPORT_SYMBOL_GPL(mc3190_cpld_regoff_wait);

static int __devinit mc3190_cpld_probe(struct platform_device *pdev)
{
    struct resource *res, *irq_res;
    u16 ret;
    u16 majorVersion;
    u16 minorVersion;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "Failed to get CPLD memory resource\n");
        return -ENXIO;
    }

    if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
        dev_err(&pdev->dev, "Failed to request CPLD memory region\n");
        return -EBUSY;
    }

    cpld_base = ioremap(res->start, resource_size(res));
    if (!cpld_base) {
        dev_err(&pdev->dev, "Failed to ioremap CPLD region\n");
        release_mem_region(res->start, resource_size(res));
        return -ENOMEM;
    }

    irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

    if (!irq_res) {
        dev_warn(&pdev->dev, "No IRQ resource; CPLD interrupt unavailable\n");
    } else {
        int err;

        cpld_irq = irq_res->start;
        set_irq_chip_and_handler(IRQ_MC3190_BTUART, &mc3190_btuart_irq_chip, handle_simple_irq);
        set_irq_flags(IRQ_MC3190_BTUART, IRQF_VALID);

        writel(readl(cpld_base + MC3190_CPLD_REG_ISR) & MC3190_CPLD_ISR_MASK,
            cpld_base + MC3190_CPLD_REG_ISR);

        err = request_irq(cpld_irq, mc3190_cpld_isr, IRQF_TRIGGER_RISING, "cpld-isr", NULL);
        if (err)
        dev_err(&pdev->dev, "Failed to request CPLD IRQ %d: %d\n", cpld_irq, err);
    }

    ret = mc3190_cpld_read(MC3190_CPLD_REG_VERSION);
    majorVersion = (ret & 0xFF) >> 5;
    minorVersion = (ret & 0x1F);

    dev_info(&pdev->dev, "MC3190 CPLD driver ready at IRQ %d (CPLD version: %d.%d)\n", cpld_irq, majorVersion, minorVersion);
    return 0;
}

static int __devexit mc3190_cpld_remove(struct platform_device *pdev)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    iounmap(cpld_base);
    if (res)
        release_mem_region(res->start, resource_size(res));

    free_irq(cpld_irq, NULL);

    return 0;
}

static struct platform_driver mc3190_cpld_driver = {
    .driver = {
        .name   = "mc3190-cpld",
        .owner  = THIS_MODULE,
    },
    .probe      = mc3190_cpld_probe,
    .remove     = __devexit_p(mc3190_cpld_remove),
};

static int __init mc3190_cpld_init(void)
{
    return platform_driver_register(&mc3190_cpld_driver);
}
subsys_initcall(mc3190_cpld_init); // or core_initcall / arch_initcall

static void __exit mc3190_cpld_exit(void)
{
    platform_driver_unregister(&mc3190_cpld_driver);
}
module_exit(mc3190_cpld_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol MC3190 CPLD Core Driver");
MODULE_LICENSE("GPL");