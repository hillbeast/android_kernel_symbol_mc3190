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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mc3190.h>

static void __iomem *cpld_base;
static DEFINE_SPINLOCK(cpld_lock);

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

static int __devinit mc3190_cpld_probe(struct platform_device *pdev)
{
    struct resource *res;

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


    u16 ret;
    u16 majorVersion;
    u16 minorVersion;

    ret = mc3190_cpld_read(MC3190_CPLD_REG_VERSION);
    majorVersion = (ret & 0xFF) >> 5;
    minorVersion = (ret & 0x1F);

    dev_info(&pdev->dev, "MC3190 CPLD driver ready (CPLD version: %d.%d)\n", majorVersion, minorVersion);
    return 0;
}

static int __devexit mc3190_cpld_remove(struct platform_device *pdev)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    iounmap(cpld_base);
    if (res)
        release_mem_region(res->start, resource_size(res));

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