/*
 * mc3190-battery.c -- Power Supply/Battery Driver for Symbol MC3190
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
#include <linux/power_supply.h>
#include <linux/slab.h>

#include <linux/mc3190.h>

#define DRV_NAME "mc3190-battery"

struct mc3190_battery {
	struct device *dev;
	struct mc3190_pwrmicro *core;
	struct power_supply psy;
};

static enum power_supply_property mc3190_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

void mc3190_set_battery_psy(struct mc3190_pwrmicro *priv, struct power_supply *psy)
{
	priv->battery_psy = psy;
}
EXPORT_SYMBOL_GPL(mc3190_set_battery_psy);

int mc3190_get_battery_voltage(struct mc3190_pwrmicro *priv)
{
	u8 *data = priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].data;
	u16 mv = (data[0] << 8) | data[1];
	return mv * 1000;
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_voltage);

static int mc3190_mv_to_percent(int mv)
{
	if (mv >= 4200)
		return 100;
	if (mv <= 3400)
		return 0;

	if (mv > 4000)
		return 80 + (mv - 4000) * 20 / (4200 - 4000);
	if (mv > 3800)
		return 50 + (mv - 3800) * 30 / (4000 - 3800);
	if (mv > 3650)
		return 20 + (mv - 3650) * 30 / (3800 - 3650);
	if (mv > 3500)
		return 5 + (mv - 3500) * 15 / (3650 - 3500);

	return (mv - 3400) * 5 / (3500 - 3400);
}

int mc3190_get_battery_capacity(struct mc3190_pwrmicro *priv)
{
	u8 *mv_data;
	u8 percent = priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].data[1];
	u16 mv;

	if (percent != 0xFF)
		return percent;

	mv_data = priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].data;
	mv = (mv_data[0] << 8) | mv_data[1];

	if (mv == 0)
		return -ENODATA;

	return mc3190_mv_to_percent(mv);
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_capacity);

int mc3190_get_battery_status(struct mc3190_pwrmicro *priv)
{
	u8 status_flag = priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].data[0];

	switch (status_flag) {
	case 0x05:
	case 0x06:
		return POWER_SUPPLY_STATUS_CHARGING;
	case 0x07:
		return POWER_SUPPLY_STATUS_FULL;
	case 0x00:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_status);

bool mc3190_is_battery_present(struct mc3190_pwrmicro *priv)
{
	u8 percent = priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].data[1];
	return (percent != 0xFF);
}
EXPORT_SYMBOL_GPL(mc3190_is_battery_present);

static int mc3190_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct mc3190_battery *bat = container_of(psy, struct mc3190_battery, psy);
	struct mc3190_pwrmicro *core = bat->core;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mc3190_get_battery_status(core);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = mc3190_is_battery_present(core) ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mc3190_get_battery_voltage(core);
		break;

	case POWER_SUPPLY_PROP_CAPACITY: {
		int cap = mc3190_get_battery_capacity(core);
		if (cap < 0)
			return cap;
		val->intval = cap;
		break;
	}

	default:
		return -EINVAL;
	}

	return 0;
}

static int mc3190_battery_probe(struct platform_device *pdev)
{
	struct mc3190_battery *bat;
	struct mc3190_pwrmicro *core = dev_get_platdata(&pdev->dev);
	int ret;

	if (!core) {
		dev_err(&pdev->dev, "no core MFD platform data found\n");
		return -ENODEV;
	}

	bat = devm_kzalloc(&pdev->dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = &pdev->dev;
	bat->core = core;

	bat->psy.name = "mc3190-battery";
	bat->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->psy.properties = mc3190_battery_props;
	bat->psy.num_properties = ARRAY_SIZE(mc3190_battery_props);
	bat->psy.get_property = mc3190_battery_get_property;

	/* Register with Linux 3.4 Power Supply core */
	ret = power_supply_register(&pdev->dev, &bat->psy);
	if (ret) {
		dev_err(&pdev->dev, "failed to register power supply: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, bat);
	mc3190_set_battery_psy(core, &bat->psy);

	dev_info(&pdev->dev, "MC3190 battery driver registered\n");
	return 0;
}

static int mc3190_battery_remove(struct platform_device *pdev)
{
	struct mc3190_battery *bat = platform_get_drvdata(pdev);

	mc3190_set_battery_psy(bat->core, NULL);
	power_supply_unregister(&bat->psy);

	return 0;
}

static struct platform_driver mc3190_battery_driver = {
	.driver = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe  = mc3190_battery_probe,
	.remove = mc3190_battery_remove,
};

static int __init mc3190_battery_init(void)
{
	return platform_driver_register(&mc3190_battery_driver);
}
module_init(mc3190_battery_init);

static void __exit mc3190_battery_exit(void)
{
	platform_driver_unregister(&mc3190_battery_driver);
}
module_exit(mc3190_battery_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Battery driver for MC3190 PwrMicro AVR");
MODULE_LICENSE("GPL");