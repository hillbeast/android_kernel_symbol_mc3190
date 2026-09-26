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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/mc3190.h>

#define DRV_NAME "mc3190-battery"

struct mc3190_battery {
	struct device *dev;
	struct mc3190_pwrmicro *core;
	struct power_supply psy;
	struct power_supply ac;
};

static enum power_supply_property mc3190_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mc3190_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static ssize_t backup_voltage_now_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct mc3190_pwrmicro *priv = dev_get_drvdata(psy->dev->parent);

	int microvolts = mc3190_get_backup_battery_voltage(priv);

	return sprintf(buf, "%d\n", microvolts);
}
static DEVICE_ATTR(backup_voltage_now, 0444, backup_voltage_now_show, NULL);

int mc3190_get_pwr_status(struct mc3190_pwrmicro *priv)
{
	u8 raw_status = priv->register_data[MC3190_TAG_PWR_EVENT].data[1];
	
	return (raw_status & 0x80) ? 1 : 0; 
}
EXPORT_SYMBOL_GPL(mc3190_get_pwr_status);

void mc3190_set_battery_psy(struct mc3190_pwrmicro *priv, struct power_supply *psy)
{
	priv->battery_psy = psy;
}
EXPORT_SYMBOL_GPL(mc3190_set_battery_psy);

int mc3190_get_battery_current(struct mc3190_pwrmicro *priv)
{
	u8 high = priv->cmd10_subdata[MC3190_TAG_PWR_BATTINSTCURRENT].data[0];
	u8 low = priv->cmd10_subdata[MC3190_TAG_PWR_BATTINSTCURRENT].data[1];
	
	s16 raw_current = (high << 8) | low;

	return (int)raw_current * 4000; // Readback value appears to be roughly I / 4
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_current);

int mc3190_get_battery_voltage(struct mc3190_pwrmicro *priv)
{
	u8 *data = priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].data;
	u16 mv = (data[0] << 8) | data[1];
	return mv * 1000;
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_voltage);

int mc3190_get_backup_battery_voltage(struct mc3190_pwrmicro *priv)
{
	u8 high = priv->cmd10_subdata[MC3190_TAG_PWR_BACKUPBATTVOLT].data[0];
	u8 low = priv->cmd10_subdata[MC3190_TAG_PWR_BACKUPBATTVOLT].data[1];
	
	u16 raw_voltage = (high << 8) | low;

	return (int)raw_voltage * 1000; 
}
EXPORT_SYMBOL_GPL(mc3190_get_backup_battery_voltage);

int mc3190_get_battery_rated_capacity(struct mc3190_pwrmicro *priv)
{
	u8 high = priv->cmd10_subdata[MC3190_TAG_PWR_BATTRATEDCAP].data[0];
	u8 low = priv->cmd10_subdata[MC3190_TAG_PWR_BATTRATEDCAP].data[1];

	u16 mah = (high << 8) | low;

	return (int)mah * 1000;
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_rated_capacity);

#define MC3190_BATT_INTERNAL_RESISTANCE_MOHM 150
#define BATT_MAX_MV 4200
#define BATT_MIN_MV 3400

static int mc3190_mv_to_percent(int voltage_mv, int current_ua)
{
	int current_ma = current_ua / 1000;
	int compensated_mv;
	int percent;

	int v_drop_mv = (current_ma * MC3190_BATT_INTERNAL_RESISTANCE_MOHM) / 1000;
	compensated_mv = voltage_mv - v_drop_mv;

	if (compensated_mv >= BATT_MAX_MV)
		return 100;
	if (compensated_mv <= BATT_MIN_MV)
		return 0;

	percent = ((compensated_mv - BATT_MIN_MV) * 100) / (BATT_MAX_MV - BATT_MIN_MV);

	return percent;
}

int mc3190_get_battery_capacity(struct mc3190_pwrmicro *priv)
{
	u8 percent = priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].data[1];
	int mv, ua, ret;

	if (percent != 0xFF)
		return percent;

	mv = mc3190_get_battery_voltage(priv) / 1000;
	ua = mc3190_get_battery_current(priv);
	if (mv == 0)
		return -ENODATA;

	return mc3190_mv_to_percent(mv, ua);
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

int mc3190_get_battery_temperature(struct mc3190_pwrmicro *priv)
{
	u8 raw_temp = priv->cmd10_subdata[MC3190_TAG_PWR_BATTTEMPERATURE].data[1];

	return (int)raw_temp * 10;
}
EXPORT_SYMBOL_GPL(mc3190_get_battery_temperature);

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
	case POWER_SUPPLY_PROP_CAPACITY: {
		int cap = mc3190_get_battery_capacity(core);
		if (cap < 0)
			return cap;
		val->intval = cap;
		break;
	}

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = mc3190_get_battery_rated_capacity(core);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = mc3190_get_battery_current(core);
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mc3190_get_pwr_status(core);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = mc3190_is_battery_present(core) ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mc3190_get_battery_status(core);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = mc3190_get_battery_temperature(core);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mc3190_get_battery_voltage(core);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mc3190_ac_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct mc3190_battery *bat = container_of(psy, struct mc3190_battery, ac);
	struct mc3190_pwrmicro *priv = bat->core;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mc3190_get_pwr_status(priv);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mc3190_battery_probe(struct platform_device *pdev)
{
	struct mc3190_battery *bat;
	struct mc3190_pwrmicro *core = dev_get_drvdata(pdev->dev.parent);
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

	ret = power_supply_register(&pdev->dev, &bat->psy);
	if (ret) {
		dev_err(&pdev->dev, "failed to register power supply: %d\n", ret);
		return ret;
	}

	bat->ac.name = "usb";
	bat->ac.type = POWER_SUPPLY_TYPE_USB;
	bat->ac.properties = mc3190_ac_props;
	bat->ac.num_properties = ARRAY_SIZE(mc3190_ac_props);
	bat->ac.get_property = mc3190_ac_get_property;

	ret = power_supply_register(&pdev->dev, &bat->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register AC power supply: %d\n", ret);
		power_supply_unregister(&bat->psy);
		return ret;
	}

	ret = device_create_file(bat->psy.dev, &dev_attr_backup_voltage_now);
	if (ret) {
		dev_warn(&pdev->dev, "failed to create backup_voltage_now sysfs entry: %d\n", ret);
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
	device_remove_file(bat->psy.dev, &dev_attr_backup_voltage_now);
	power_supply_unregister(&bat->psy);
	power_supply_unregister(&bat->ac);

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