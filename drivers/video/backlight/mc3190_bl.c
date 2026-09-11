/*
 * mc3190_bl.c -- Driver for Symbol/Motorola MC3190 backlight
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
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mc3190.h>

static void mc3190_bl_power_on(struct mc3190_bl_data *d)
{
	// The device is a blackbox so the following code is just imitating what Windows CE does
	// Write to CPLD that the backlight needs turned on
	mc3190_cpld_write(MC3190_CPLD_LCD_BL_BIT, MC3190_CPLD_REG_LCD_1);
	mdelay(15);

	// Set the backlight brightness. CPLD handles the brightness change
	pwm_config(d->pwm, d->pdata->pwm_period_ns, d->pdata->pwm_period_ns);
	pwm_enable(d->pwm);

	mdelay(1);

	d->powered = 1;
}

static void mc3190_bl_power_off(struct mc3190_bl_data *d)
{
	// Set the backlight brightness (off)
	pwm_config(d->pwm, 0, d->pdata->pwm_period_ns);
	pwm_disable(d->pwm);

	// Tell the CPLD that the backlight should now be off
	mc3190_cpld_write(MC3190_CPLD_LCD_BL_BIT, MC3190_CPLD_REG_LCD_2);
	mdelay(20);

	d->powered = 0;
}

static int mc3190_bl_update_status(struct backlight_device *bl)
{
	struct mc3190_bl_data *d = bl_get_data(bl);
	struct mc3190_bl_platform_data *pdata = d->pdata;
	int brightness = bl->props.brightness;
	unsigned long duty_ns;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    (bl->props.state & BL_CORE_SUSPENDED))
		brightness = 0;

	if (brightness == 0) {
		if (d->powered)
			mc3190_bl_power_off(d);
		return 0;
	}

	if (!d->powered)
		mc3190_bl_power_on(d);

	duty_ns = (unsigned long)brightness * pdata->pwm_period_ns /
		  pdata->max_brightness;

	pwm_config(d->pwm, duty_ns, pdata->pwm_period_ns);
	pwm_enable(d->pwm);

	return 0;
}

static int mc3190_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops mc3190_bl_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= mc3190_bl_update_status,
	.get_brightness	= mc3190_bl_get_brightness,
};

static int __devinit mc3190_bl_probe(struct platform_device *pdev)
{
	struct mc3190_bl_platform_data *pdata = pdev->dev.platform_data;
	struct mc3190_bl_data *d;
	struct backlight_device *bl;
	struct backlight_properties props;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data\n");
		return -EINVAL;
	}

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->pdata = pdata;

	d->pwm = pwm_request(pdata->pwm_id, "mc3190-backlight");
	if (IS_ERR(d->pwm)) {
		dev_err(&pdev->dev, "failed to request PWM%d\n",
			pdata->pwm_id);
		ret = PTR_ERR(d->pwm);
		goto err_free;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = pdata->max_brightness;

	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, d,
					&mc3190_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight device\n");
		ret = PTR_ERR(bl);
		goto err_pwm;
	}

	platform_set_drvdata(pdev, bl);

	bl->props.brightness = pdata->dft_brightness;
	backlight_update_status(bl);

	dev_info(&pdev->dev, "MC3190 backlight driver loaded (PWM%d)\n",
		 pdata->pwm_id);

	return 0;

err_pwm:
	pwm_free(d->pwm);
err_free:
	kfree(d);
	return ret;
}

static int __devexit mc3190_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct mc3190_bl_data *d = bl_get_data(bl);

	backlight_device_unregister(bl);

	mc3190_bl_power_off(d);

	pwm_free(d->pwm);
	kfree(d);

	return 0;
}

static struct platform_driver mc3190_bl_driver = {
	.driver		= {
		.name	= "mc3190-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= mc3190_bl_probe,
	.remove		= __devexit_p(mc3190_bl_remove),
};

static int __init mc3190_bl_init(void)
{
	return platform_driver_register(&mc3190_bl_driver);
}
module_init(mc3190_bl_init);

static void __exit mc3190_bl_exit(void)
{
	platform_driver_unregister(&mc3190_bl_driver);
}
module_exit(mc3190_bl_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol MC3190 backlight driver");
MODULE_LICENSE("GPL");