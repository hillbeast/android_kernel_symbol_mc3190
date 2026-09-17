/*
 * mc3190-powerkey.c -- Driver for MC3190 Power Key
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>

#define SPMU_PHYS_BASE     0x40F50000
#define SPMU_MAP_SIZE      0x20
#define SPMU_PWRKEY_REG    0x18

static void __iomem *spmu_base;
static struct input_dev *pwrkey_input_dev;
static int pwrkey_irq = IRQ_WAKEUP0;

static irqreturn_t mc3190_pwrkey_irq_handler(int irq, void *dev_id)
{
    u32 reg_val;
    int key_pressed;

    reg_val = readl(spmu_base + SPMU_PWRKEY_REG);
    key_pressed = (reg_val & 0x1) ? 1 : 0;

    input_report_key(pwrkey_input_dev, KEY_POWER, key_pressed);
    input_sync(pwrkey_input_dev);
    writel(reg_val, spmu_base + SPMU_PWRKEY_REG);

    return IRQ_HANDLED;
}

static int __init mc3190_pwrkey_init(void)
{
    int ret;

    spmu_base = ioremap(SPMU_PHYS_BASE, SPMU_MAP_SIZE);
    if (!spmu_base) {
        pr_err("mc3190_pwrkey: Failed to ioremap SPMU registers\n");
        return -ENOMEM;
    }

    pwrkey_input_dev = input_allocate_device();
    if (!pwrkey_input_dev) {
        iounmap(spmu_base);
        return -ENOMEM;
    }

    pwrkey_input_dev->name = "MC3190 Power Key";
    pwrkey_input_dev->evbit[0] = BIT_MASK(EV_KEY);
    set_bit(KEY_POWER, pwrkey_input_dev->keybit);

    ret = input_register_device(pwrkey_input_dev);
    if (ret) {
        input_free_device(pwrkey_input_dev);
        iounmap(spmu_base);
        return ret;
    }

    ret = request_threaded_irq(pwrkey_irq, NULL, mc3190_pwrkey_irq_handler,
                               IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                               
                               "mc3190-pwrkey", NULL);
    if (ret) {
        input_unregister_device(pwrkey_input_dev);
        iounmap(spmu_base);
        return ret;
    }

    return 0;
}

static void __exit mc3190_pwrkey_exit(void)
{
    free_irq(pwrkey_irq, NULL);
    input_unregister_device(pwrkey_input_dev);
    iounmap(spmu_base);
}

module_init(mc3190_pwrkey_init);
module_exit(mc3190_pwrkey_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol MC3190 Power Key Driver");
MODULE_LICENSE("GPL");
