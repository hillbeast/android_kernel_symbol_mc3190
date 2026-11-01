/*
 * mc3190_lcd.c -- Driver for Symbol MC3190 LCD panel (L5F31157T00) and
 *                 its backlight.
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
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/pwm.h>
#include <linux/mc3190.h>

/* MIPI DCS command opcodes used by this panel */
#define DCS_ENTER_SLEEP_MODE    0x10
#define DCS_EXIT_SLEEP_MODE     0x11
#define DCS_SET_DISPLAY_OFF     0x28
#define DCS_SET_DISPLAY_ON      0x29
#define DCS_SET_COLUMN_ADDRESS  0x2a
#define DCS_SET_PAGE_ADDRESS    0x2b
#define DCS_SET_ADDRESS_MODE    0x36
#define DCS_SET_PIXEL_FORMAT    0x3a

/*
 * Each SPI word is 9 bits: bit 8 is D/C (0 = command, 1 = data),
 * bits 7:0 are the byte value. SSP3 must be configured for 9-bit
 * words (bits_per_word = 9); each entry here occupies one 16-bit
 * slot in the tx buffer.
 */
#define SPI_CMD(x)   ((u16)(x))
#define SPI_DATA(x)  ((u16)(0x100 | (x)))

static const u16 mc3190_panel_init_cmds[] = {
	/* CASET: columns 0..319 */
	SPI_CMD(DCS_SET_COLUMN_ADDRESS),
	SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x01), SPI_DATA(0x3f),

	/* PASET: pages 0..319 */
	SPI_CMD(DCS_SET_PAGE_ADDRESS),
	SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x01), SPI_DATA(0x3f),

	/* MADCTL */
	SPI_CMD(DCS_SET_ADDRESS_MODE), SPI_DATA(0x00),

	/* COLMOD: 0x66 = 18bpp / RGB666 */
	SPI_CMD(DCS_SET_PIXEL_FORMAT), SPI_DATA(0x66),

	/* Vendor unlock */
	SPI_CMD(0xf0), SPI_DATA(0x5a), SPI_DATA(0x5a),

	/* 0xF2 */
	SPI_CMD(0xf2),
	SPI_DATA(0x27), SPI_DATA(0x7c), SPI_DATA(0x0f), SPI_DATA(0x0a), SPI_DATA(0x05),
	SPI_DATA(0x08), SPI_DATA(0x08), SPI_DATA(0x00), SPI_DATA(0x08), SPI_DATA(0x08),
	SPI_DATA(0x00), SPI_DATA(0x01), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x7c),
	SPI_DATA(0x0a), SPI_DATA(0x05), SPI_DATA(0x0a), SPI_DATA(0x05),

	/* 0xF4 */
	SPI_CMD(0xf4),
	SPI_DATA(0x07), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00),
	SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x36),
	SPI_DATA(0x05), SPI_DATA(0x00), SPI_DATA(0x36), SPI_DATA(0x05),

	/* 0xF5 */
	SPI_CMD(0xf5),
	SPI_DATA(0x00), SPI_DATA(0x37), SPI_DATA(0x4b), SPI_DATA(0x00), SPI_DATA(0x00),
	SPI_DATA(0x04), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00),
	SPI_DATA(0x37), SPI_DATA(0x4b),

	/* 0xF6 */
	SPI_CMD(0xf6),
	SPI_DATA(0x05), SPI_DATA(0x00), SPI_DATA(0x08), SPI_DATA(0x03), SPI_DATA(0x00),
	SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00), SPI_DATA(0x00),

	/* 0xF7 */
	SPI_CMD(0xf7),
	SPI_DATA(0x48), SPI_DATA(0x81), SPI_DATA(0x10), SPI_DATA(0x02), SPI_DATA(0x00),

	/* 0xF8 */
	SPI_CMD(0xf8),
	SPI_DATA(0x66), SPI_DATA(0x00),

	/* 0xF9 */
	SPI_CMD(0xf9),
	SPI_DATA(0x17),

	/* 0xFA */
	SPI_CMD(0xfa),
	SPI_DATA(0x0b), SPI_DATA(0x03), SPI_DATA(0x08), SPI_DATA(0x0f), SPI_DATA(0x13),
	SPI_DATA(0x22), SPI_DATA(0x30), SPI_DATA(0x29), SPI_DATA(0x1d), SPI_DATA(0x19),
	SPI_DATA(0x1e), SPI_DATA(0x20), SPI_DATA(0x18), SPI_DATA(0x00), SPI_DATA(0x00),
	SPI_DATA(0x00),

	/* 0xFB */
	SPI_CMD(0xfb),
	SPI_DATA(0x0b), SPI_DATA(0x03), SPI_DATA(0x18), SPI_DATA(0x20), SPI_DATA(0x1e),
	SPI_DATA(0x19), SPI_DATA(0x1d), SPI_DATA(0x29), SPI_DATA(0x30), SPI_DATA(0x22),
	SPI_DATA(0x13), SPI_DATA(0x0f), SPI_DATA(0x08), SPI_DATA(0x00), SPI_DATA(0x00),
	SPI_DATA(0x00),

	/* Vendor lock */
	SPI_CMD(0xf0), SPI_DATA(0xa5), SPI_DATA(0xa5),
};

/*
 * Driver state.  There is exactly one panel on this board, so this is a
 * singleton.  All fields are protected by lcd_lock.
 */
static DEFINE_MUTEX(lcd_lock);

static struct {
	struct spi_device				*spi;
	u16								*tx;
	u16								*init_cmds;
	struct backlight_device			*bl;
	struct pwm_device				*pwm;
	struct mc3190_bl_platform_data	*pdata;

	bool							requested;
	bool							panel_on;
	bool							bl_on;
} lcd;

/* ------------------------------------------------------------------ */
/* SPI helpers (lcd_lock held)                                         */
/* ------------------------------------------------------------------ */

static int mc3190_lcd_spi_send(const u16 *words, int count)
{
	struct spi_transfer t = {
		.tx_buf = words,
		.len = count * sizeof(u16),
		.bits_per_word = 9,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(lcd.spi, &m);
}

static int mc3190_lcd_cmd(u16 cmd)
{
	lcd.tx[0] = cmd;
	return mc3190_lcd_spi_send(lcd.tx, 1);
}

/* ------------------------------------------------------------------ */
/* Backlight hardware (lcd_lock held)                                  */
/* ------------------------------------------------------------------ */

static void mc3190_bl_hw_on(void)
{
	/*
	 * The CPLD is a black box, so this imitates what Windows CE does:
	 * enable the backlight in the CPLD, then run the PWM.  The caller
	 * sets the real duty cycle immediately afterwards.
	 */
	mc3190_cpld_regon(MC3190_CPLD_LCD_BL_BIT, MC3190_CPLD_REG_BL);
	msleep(15);

	pwm_config(lcd.pwm, lcd.pdata->pwm_period_ns,
		   lcd.pdata->pwm_period_ns);
	pwm_enable(lcd.pwm);

	udelay(2000);
	lcd.bl_on = true;
}

static void mc3190_bl_hw_off(void)
{
	pwm_config(lcd.pwm, 0, lcd.pdata->pwm_period_ns);
	pwm_disable(lcd.pwm);

	mc3190_cpld_regoff(MC3190_CPLD_LCD_BL_BIT, MC3190_CPLD_REG_BL);
	msleep(20);

	lcd.bl_on = false;
}

/*
 * Bring the physical backlight in line with what the backlight class
 * wants AND what the panel is doing.  The backlight is lit only when the
 * panel is awake.  lcd_lock must be held.
 *
 * bl->props is read without the backlight core's ops_lock when called
 * from the panel path; these are plain ints and a stale read is corrected
 * by the update_status() call that always follows a write.
 */
static int mc3190_bl_apply(struct backlight_device *bl)
{
	struct mc3190_bl_platform_data *pdata = lcd.pdata;
	int brightness = 0;
	unsigned long duty_ns;
	int ret;

	if (!bl || !lcd.pwm)
		return 0;

	if (lcd.panel_on &&
	    bl->props.power == FB_BLANK_UNBLANK &&
	    bl->props.fb_blank == FB_BLANK_UNBLANK &&
	    !(bl->props.state & BL_CORE_SUSPENDED))
		brightness = bl->props.brightness;

	if (brightness <= 0) {
		if (lcd.bl_on)
			mc3190_bl_hw_off();
		return 0;
	}

	if (!lcd.bl_on)
		mc3190_bl_hw_on();

	duty_ns = (unsigned long)brightness * pdata->pwm_period_ns /
		  pdata->max_brightness;

	ret = pwm_config(lcd.pwm, duty_ns, pdata->pwm_period_ns);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Panel hardware (lcd_lock held)                                      */
/* ------------------------------------------------------------------ */

static int mc3190_lcd_cpld_down(void)
{
	/* post_suspend(): deassert both LCD control bits */
	mc3190_cpld_regoff(MC3190_CPLD_LCD_LCD_BIT_0, MC3190_CPLD_REG_LCD);
	mc3190_cpld_regoff(MC3190_CPLD_LCD_LCD_BIT_1, MC3190_CPLD_REG_LCD);

	/* deassert LCD enable, wait for CPLD to confirm not-ready */
	return mc3190_cpld_regoff_wait(MC3190_CPLD_LCD_EN_BIT,
				       MC3190_CPLD_REG_BL,
				       MC3190_CPLD_LCD_READY_BIT, 100);
}

static int mc3190_lcd_hw_on(void)
{
	struct device *dev = &lcd.spi->dev;
	int ret;

	/* init_pins(): enable LCD logic rail, wait for CPLD to confirm ready */
	ret = mc3190_cpld_regon_wait(MC3190_CPLD_LCD_EN_BIT,
				     MC3190_CPLD_REG_BL,
				     MC3190_CPLD_LCD_READY_BIT, 100);
	if (ret) {
		dev_err(dev, "timed out waiting for LCD ready\n");
		goto err_rail;
	}

	udelay(2000);

	/* panel_SPI_Init(): assert LCD bit0, deassert bit1 */
	mc3190_cpld_regon(MC3190_CPLD_LCD_LCD_BIT_0, MC3190_CPLD_REG_LCD);
	mc3190_cpld_regoff(MC3190_CPLD_LCD_LCD_BIT_1, MC3190_CPLD_REG_LCD);

	msleep(20);

	ret = mc3190_lcd_spi_send(lcd.init_cmds,
				  ARRAY_SIZE(mc3190_panel_init_cmds));
	if (ret) {
		dev_err(dev, "panel init failed: %d\n", ret);
		goto err_rail;
	}

	ret = mc3190_lcd_cmd(SPI_CMD(DCS_EXIT_SLEEP_MODE));
	if (ret)
		goto err_rail;

	msleep(120); /* mandatory DCS post-sleep-out settle time */

	ret = mc3190_lcd_cmd(SPI_CMD(DCS_SET_DISPLAY_ON));
	if (ret)
		goto err_rail;

	return 0;

err_rail:
	/* leave the CPLD in the powered-down state so a retry starts clean */
	mc3190_lcd_cpld_down();
	return ret;
}

static void mc3190_lcd_hw_off(void)
{
	struct device *dev = &lcd.spi->dev;
	int ret;

	/* panel_pre_suspend() */
	ret = mc3190_lcd_cmd(SPI_CMD(DCS_SET_DISPLAY_OFF));
	if (ret)
		dev_warn(dev, "display off failed: %d\n", ret);

	msleep(50);

	ret = mc3190_lcd_cmd(SPI_CMD(DCS_ENTER_SLEEP_MODE));
	if (ret)
		dev_warn(dev, "sleep in failed: %d\n", ret);

	msleep(220);

	/* Always drop the rail, even if the SPI commands failed. */
	ret = mc3190_lcd_cpld_down();
	if (ret)
		dev_warn(dev, "CPLD did not report LCD not-ready: %d\n", ret);
}

/*
 * Make the hardware match lcd.requested.  Idempotent.  lcd_lock held and
 * lcd.spi non-NULL.
 */
static void mc3190_lcd_apply_locked(void)
{
	if (lcd.requested && !lcd.panel_on) {
		if (mc3190_lcd_hw_on() == 0) {
			lcd.panel_on = true;
			mc3190_bl_apply(lcd.bl);
		}
	} else if (!lcd.requested && lcd.panel_on) {
		/* backlight first, then the panel */
		lcd.panel_on = false;
		mc3190_bl_apply(lcd.bl);
		mc3190_lcd_hw_off();
	}
}

/*
 * lcd_power callback for pxafb.  May be called before the SPI device has
 * probed; the request is remembered and applied from probe().
 */
void mc3190_panel_set_power(int on)
{
	mutex_lock(&lcd_lock);
	lcd.requested = !!on;
	if (lcd.spi)
		mc3190_lcd_apply_locked();
	mutex_unlock(&lcd_lock);
}
EXPORT_SYMBOL_GPL(mc3190_panel_set_power);

/* ------------------------------------------------------------------ */
/* Backlight class                                                     */
/* ------------------------------------------------------------------ */

static int mc3190_bl_update_status(struct backlight_device *bl)
{
	int ret;

	mutex_lock(&lcd_lock);
	ret = mc3190_bl_apply(bl);
	mutex_unlock(&lcd_lock);

	return ret;
}

static int mc3190_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops mc3190_bl_ops = {
	.update_status	= mc3190_bl_update_status,
	.get_brightness	= mc3190_bl_get_brightness,
};

/* ------------------------------------------------------------------ */
/* SPI driver                                                          */
/* ------------------------------------------------------------------ */

static int __devinit mc3190_lcd_panel_probe(struct spi_device *spi)
{
	struct mc3190_bl_platform_data *pdata = spi->dev.platform_data;
	struct backlight_properties props;
	struct backlight_device *bl;
	int ret;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data\n");
		return -EINVAL;
	}

	spi->bits_per_word = 9;
	spi->mode = SPI_MODE_0;

	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		return ret;
	}

	/*
	 * Fill in everything except lcd.spi / lcd.bl first.  Until lcd.spi
	 * is set, mc3190_panel_set_power() only records the request.
	 */
	lcd.tx = kzalloc(sizeof(u16), GFP_KERNEL);
	lcd.init_cmds = kmemdup(mc3190_panel_init_cmds,
				sizeof(mc3190_panel_init_cmds), GFP_KERNEL);
	if (!lcd.tx || !lcd.init_cmds) {
		ret = -ENOMEM;
		goto err_free;
	}

	lcd.pdata = pdata;
	lcd.pwm = pwm_request(pdata->pwm_id, "mc3190-backlight");
	if (IS_ERR(lcd.pwm)) {
		dev_err(&spi->dev, "failed to request PWM%d\n", pdata->pwm_id);
		ret = PTR_ERR(lcd.pwm);
		lcd.pwm = NULL;
		goto err_free;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = pdata->max_brightness;
	props.brightness = pdata->dft_brightness;
	props.power = FB_BLANK_UNBLANK;

	/* Keep the old name so /sys/class/backlight/mc3190-backlight stays put */
	bl = backlight_device_register("mc3190-backlight", &spi->dev, NULL,
				       &mc3190_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&spi->dev, "failed to register backlight device\n");
		ret = PTR_ERR(bl);
		goto err_pwm;
	}

	mutex_lock(&lcd_lock);
	lcd.spi = spi;
	lcd.bl = bl;
	lcd.panel_on = false;
	lcd.bl_on = false;
	/* Apply whatever pxafb asked for before we existed. */
	mc3190_lcd_apply_locked();
	mutex_unlock(&lcd_lock);

	dev_info(&spi->dev, "MC3190 LCD panel and backlight ready (PWM%d)\n",
		 pdata->pwm_id);
	return 0;

err_pwm:
	pwm_free(lcd.pwm);
	lcd.pwm = NULL;
err_free:
	kfree(lcd.init_cmds);
	kfree(lcd.tx);
	lcd.init_cmds = NULL;
	lcd.tx = NULL;
	lcd.pdata = NULL;
	return ret;
}

static int __devexit mc3190_lcd_panel_remove(struct spi_device *spi)
{
	struct backlight_device *bl;

	mutex_lock(&lcd_lock);
	bl = lcd.bl;
	lcd.bl = NULL;
	if (lcd.panel_on) {
		lcd.panel_on = false;
		if (lcd.bl_on)
			mc3190_bl_hw_off();
		mc3190_lcd_hw_off();
	}
	lcd.spi = NULL;
	mutex_unlock(&lcd_lock);

	/* Not under lcd_lock: this takes the backlight core's ops_lock. */
	backlight_device_unregister(bl);

	pwm_free(lcd.pwm);
	lcd.pwm = NULL;
	kfree(lcd.init_cmds);
	kfree(lcd.tx);
	lcd.init_cmds = NULL;
	lcd.tx = NULL;
	lcd.pdata = NULL;

	return 0;
}

static void mc3190_lcd_panel_shutdown(struct spi_device *spi)
{
	mutex_lock(&lcd_lock);
	lcd.requested = false;
	if (lcd.spi)
		mc3190_lcd_apply_locked();
	mutex_unlock(&lcd_lock);
}

/*
 * No .suspend/.resume here on purpose.  pxafb's own PM hooks drive
 * lcd_power() in the right order relative to the LCD controller
 * (controller running before DISPON, stopped after DISPOFF).  An
 * independent SPI-side resume could run before pxafb's and send DISPON
 * with no pixel clock.
 */
static struct spi_driver mc3190_lcd_panel_driver = {
	.driver = {
		.name	= "mc3190-lcd-panel",
		.owner	= THIS_MODULE,
	},
	.probe		= mc3190_lcd_panel_probe,
	.remove		= __devexit_p(mc3190_lcd_panel_remove),
	.shutdown	= mc3190_lcd_panel_shutdown,
};

static int __init mc3190_lcd_panel_init(void)
{
	return spi_register_driver(&mc3190_lcd_panel_driver);
}
module_init(mc3190_lcd_panel_init);

static void __exit mc3190_lcd_panel_exit(void)
{
	spi_unregister_driver(&mc3190_lcd_panel_driver);
}
module_exit(mc3190_lcd_panel_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol MC3190 LCD panel (L5F31157T00) and backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:mc3190-lcd-panel");