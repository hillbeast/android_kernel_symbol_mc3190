/*
 * mc3190-touch.c -- Touchscreen Driver for Symbol MC3190
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
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/mc3190.h>

#define DRV_NAME "mc3190-touch"

#define MC3190_TOUCH_X_MIN   688    // FIXME: TBC
#define MC3190_TOUCH_X_MAX   4080   // FIXME: TBC
#define MC3190_TOUCH_Y_MIN   42     // FIXME: TBC
#define MC3190_TOUCH_Y_MAX   991    // FIXME: TBC

struct mc3190_touch {
	struct device *dev;
	struct input_dev *input;
};

void mc3190_touch_report(struct input_dev *input, u16 x, u16 y, bool touchstate)
{
	if (!input)
		return;

    if (touchstate == PWRMICRO_TOUCH_DOWN) {
        input_report_abs(input, ABS_X, x);
        input_report_abs(input, ABS_Y, y);
        input_report_key(input, BTN_TOUCH, 1);
    } else {
        input_report_key(input, BTN_TOUCH, 0);
    }
	input_sync(input);
}
EXPORT_SYMBOL_GPL(mc3190_touch_report);

static int mc3190_touch_probe(struct platform_device *pdev)
{
	struct mc3190_touch *touch;
	struct mc3190_pwrmicro *core = dev_get_drvdata(pdev->dev.parent);
	int ret;

	touch = devm_kzalloc(&pdev->dev, sizeof(*touch), GFP_KERNEL);
	if (!touch)
		return -ENOMEM;
	touch->dev = &pdev->dev;

	touch->input = input_allocate_device();
	if (!touch->input)
		return -ENOMEM;

	touch->input->name = "MC3190 Touchscreen";
	touch->input->phys = "mc3190/input0";
	touch->input->id.bustype = BUS_HOST;
	touch->input->dev.parent = &pdev->dev;

	__set_bit(EV_ABS, touch->input->evbit);
	__set_bit(EV_KEY, touch->input->evbit);
	__set_bit(BTN_TOUCH, touch->input->keybit);

	input_set_abs_params(touch->input, ABS_X, MC3190_TOUCH_X_MIN, MC3190_TOUCH_X_MAX, 0, 0);
	input_set_abs_params(touch->input, ABS_Y, MC3190_TOUCH_Y_MIN, MC3190_TOUCH_Y_MAX, 0, 0);

	ret = input_register_device(touch->input);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input device: %d\n", ret);
		input_free_device(touch->input);
		return ret;
	}

	platform_set_drvdata(pdev, touch);
	mc3190_set_touch_input(core, touch->input);

	dev_info(&pdev->dev, "MC3190 touchscreen driver registered\n");
	return 0;
}

static int mc3190_touch_remove(struct platform_device *pdev)
{
	struct mc3190_touch *touch = platform_get_drvdata(pdev);
	input_unregister_device(touch->input);

	return 0;
}

static struct platform_driver mc3190_touch_driver = {
	.driver = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe  = mc3190_touch_probe,
	.remove = mc3190_touch_remove,
};

static int __init mc3190_touch_init(void)
{
	return platform_driver_register(&mc3190_touch_driver);
}
module_init(mc3190_touch_init);

static void __exit mc3190_touch_exit(void)
{
	platform_driver_unregister(&mc3190_touch_driver);
}
module_exit(mc3190_touch_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Touchscreen driver for MC3190 PwrMicro AVR");
MODULE_LICENSE("GPL");