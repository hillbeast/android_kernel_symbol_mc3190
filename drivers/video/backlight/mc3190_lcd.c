/*
 * mc3190_lcd.c -- Driver for Symbol MC3190 LCD Panel
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

/* Panel is 320x320 */
#define PANEL_WIDTH   320
#define PANEL_HEIGHT  320

/*
 * Each SPI word is 9 bits: bit 8 is D/C (0 = command, 1 = data),
 * bits 7:0 are the byte value. SSP3 must be configured for 9-bit
 * words (bits_per_word = 9); each entry here occupies one 16-bit
 * slot in the tx buffer.
 */
#define SPI_CMD(x)   ((u16)(x))
#define SPI_DATA(x)  ((u16)(0x100 | (x)))

static int mc3190_panel_spi_send(struct spi_device *spi, const u16 *words, int count)
{
	struct spi_transfer t = {
		.tx_buf = words,
		.len = count * sizeof(u16),
		.bits_per_word = 9,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

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

int mc3190_panel_power_on(struct spi_device *spi)
{
	int ret;
	u16 sleep_out = SPI_CMD(DCS_EXIT_SLEEP_MODE);
	u16 disp_on = SPI_CMD(DCS_SET_DISPLAY_ON);

	/* init_pins(): enable LCD logic rail, wait for CPLD to confirm ready */
	ret = mc3190_cpld_regon_wait(MC3190_CPLD_LCD_EN_BIT, MC3190_CPLD_REG_BL,
				      MC3190_CPLD_LCD_READY_BIT, 100);
	if (ret) {
		dev_err(&spi->dev, "timed out waiting for LCD ready\n");
		return ret;
	}

	mdelay(1);

	/* panel_SPI_Init(): assert LCD bit0, deassert bit1 on the LCD control register */
	mc3190_cpld_regon(MC3190_CPLD_LCD_LCD_BIT_0, MC3190_CPLD_REG_LCD);
	mc3190_cpld_regoff(MC3190_CPLD_LCD_LCD_BIT_1, MC3190_CPLD_REG_LCD);

	mdelay(20);

	ret = mc3190_panel_spi_send(spi, mc3190_panel_init_cmds,
				     ARRAY_SIZE(mc3190_panel_init_cmds));
	if (ret)
		return ret;

	ret = mc3190_panel_spi_send(spi, &sleep_out, 1);
	if (ret)
		return ret;

	mdelay(120); /* mandatory DCS post-sleep-out settle time */

	ret = mc3190_panel_spi_send(spi, &disp_on, 1);
	if (ret)
		return ret;

	mc3190_bl_set_power(1);

	return 0;
}
EXPORT_SYMBOL_GPL(mc3190_panel_power_on);

int mc3190_panel_power_off(struct spi_device *spi)
{
	int ret;
	u16 disp_off = SPI_CMD(DCS_SET_DISPLAY_OFF);
	u16 sleep_in = SPI_CMD(DCS_ENTER_SLEEP_MODE);

	mc3190_bl_set_power(0);

	/* panel_pre_suspend() */
	ret = mc3190_panel_spi_send(spi, &disp_off, 1);
	if (ret)
		return ret;

	mdelay(50);

	ret = mc3190_panel_spi_send(spi, &sleep_in, 1);
	if (ret)
		return ret;

	mdelay(220);

	/* post_suspend(): deassert both LCD control bits */
	mc3190_cpld_regoff(MC3190_CPLD_LCD_LCD_BIT_0, MC3190_CPLD_REG_LCD);
	mc3190_cpld_regoff(MC3190_CPLD_LCD_LCD_BIT_1, MC3190_CPLD_REG_LCD);

	/* init_pins mirror: deassert LCD enable, wait for CPLD to confirm not-ready */
	return mc3190_cpld_regoff_wait(MC3190_CPLD_LCD_EN_BIT, MC3190_CPLD_REG_BL,
					MC3190_CPLD_LCD_READY_BIT, 100);
}
EXPORT_SYMBOL_GPL(mc3190_panel_power_off);

static struct spi_device *mc3190_panel_spi;

void mc3190_panel_set_power(int on)
{
	if (!mc3190_panel_spi) {
		pr_err("mc3190-lcd-panel: power callback fired before probe\n");
		return;
	}

	if (on)
		mc3190_panel_power_on(mc3190_panel_spi);
	else
		mc3190_panel_power_off(mc3190_panel_spi);
}
EXPORT_SYMBOL_GPL(mc3190_panel_set_power);

static int __devinit mc3190_lcd_panel_probe(struct spi_device *spi)
{
	int ret;

	spi->bits_per_word = 9;
	spi->mode = SPI_MODE_0;

	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		return ret;
	}

	mc3190_panel_spi = spi;

	dev_info(&spi->dev, "MC3190 LCD panel driver ready\n");
	return 0;
}

static int __devexit mc3190_lcd_panel_remove(struct spi_device *spi)
{
	mc3190_panel_spi = NULL;
	return 0;
}

static struct spi_driver mc3190_lcd_panel_driver = {
	.driver = {
		.name	= "mc3190-lcd-panel",
		.owner	= THIS_MODULE,
	},
	.probe	= mc3190_lcd_panel_probe,
	.remove	= __devexit_p(mc3190_lcd_panel_remove),
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
MODULE_DESCRIPTION("Symbol MC3190 LCD panel (L5F31157T00) SPI/DCS driver core");
MODULE_LICENSE("GPL");