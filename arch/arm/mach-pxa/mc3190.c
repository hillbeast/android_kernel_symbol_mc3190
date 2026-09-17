/*
 * linux/arch/arm/mach-pxa/mc3190.c
 *
 * Support for Symbol MC3190
 *
 * Copyright (C) 2006 Marvell International Ltd.
 *
 * 		By Mark Kennard <markkennard4@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/audio.h>
#include <mach/mmc.h>
#include <mach/ohci.h>
#include <mach/pxa2xx_spi.h>
#include <mach/pxa320.h>
#include <mach/pxafb.h>
#include <mach/pxa27x_keypad.h>
#include <mach/udc.h>
#include <plat/pxa3xx_nand.h>

#include <linux/mc3190.h>

#include "devices.h"
#include "generic.h"


int lcd_id;
int lcd_orientation;

struct platform_device mc3190_wm9713_audio = {
	.name		= "wm9713-codec",
	.id		= -1,
};

static mfp_cfg_t mfp_cfg[] __initdata = {
	/* LCD */
	GPIO6_2_LCD_LDD_0,
	GPIO7_2_LCD_LDD_1,
	GPIO8_2_LCD_LDD_2,
	GPIO9_2_LCD_LDD_3,
	GPIO10_2_LCD_LDD_4,
	GPIO11_2_LCD_LDD_5,
	GPIO12_2_LCD_LDD_6,
	GPIO13_2_LCD_LDD_7,
	GPIO14_2_LCD_FCLK,
	GPIO15_2_LCD_LCLK,
	GPIO16_2_LCD_PCLK,
	GPIO17_2_LCD_BIAS,
	GPIO63_LCD_LDD_8,
	GPIO64_LCD_LDD_9,
	GPIO65_LCD_LDD_10,
	GPIO66_LCD_LDD_11,
	GPIO67_LCD_LDD_12,
	GPIO68_LCD_LDD_13,
	GPIO69_LCD_LDD_14,
	GPIO70_LCD_LDD_15,
	GPIO14_PWM3_OUT,	/* backlight */ // FIXME: This is a GPIO according to RAM dumps (?)

	/* UART 1 / FFUART */
	GPIO75_UART1_RXD | MFP_LPM_EDGE_FALL,
	GPIO76_UART1_TXD,
	GPIO77_UART1_CTS,
	GPIO78_UART1_DCD,
	GPIO79_UART1_DSR | MFP_LPM_EDGE_FALL,
	GPIO80_UART1_RI,
	GPIO81_UART1_DTR,
	GPIO82_UART1_RTS,

	/* UART 2 */
	GPIO109_UART2_RTS,
	GPIO110_UART2_RXD,
	GPIO111_UART2_TXD,
	GPIO112_UART2_CTS,

	/* UART3 */
	GPIO91_UART3_RXD,
	GPIO92_UART3_TXD,

	/* AC97 */
	GPIO34_AC97_SYSCLK,
	GPIO35_AC97_SDATA_IN_0,
	GPIO37_AC97_SDATA_OUT,
	GPIO38_AC97_SYNC,
	GPIO39_AC97_BITCLK,
	GPIO40_AC97_nACRESET,

	/* SSP3 */
	GPIO71_SSP3_TXD,
	GPIO72_SSP3_RXD,
	GPIO89_SSP3_SCLK,
	GPIO90_SSP3_FRM,

	/* SSP4 */
	GPIO93_SSP4_SCLK,
	GPIO94_SSP4_FRM,
	GPIO95_SSP4_RXD,
	GPIO96_SSP4_TXD,

	/* I2C */
	GPIO32_I2C_SCL,
	GPIO33_I2C_SDA,

	/* Keypad */
	GPIO105_KP_DKIN_0 | MFP_LPM_EDGE_BOTH,
	GPIO106_KP_DKIN_1 | MFP_LPM_EDGE_BOTH,
	GPIO113_KP_MKIN_0 | MFP_LPM_EDGE_BOTH,
	GPIO114_KP_MKIN_1 | MFP_LPM_EDGE_BOTH,
	GPIO115_KP_MKIN_2 | MFP_LPM_EDGE_BOTH,
	GPIO116_KP_MKIN_3 | MFP_LPM_EDGE_BOTH,
	GPIO117_KP_MKIN_4 | MFP_LPM_EDGE_BOTH,
	GPIO118_KP_MKIN_5 | MFP_LPM_EDGE_BOTH,
	GPIO119_KP_MKIN_6 | MFP_LPM_EDGE_BOTH,
	GPIO120_KP_MKIN_7 | MFP_LPM_EDGE_BOTH,
	GPIO121_KP_MKOUT_0,
	GPIO122_KP_MKOUT_1,
	GPIO123_KP_MKOUT_2,
	GPIO124_KP_MKOUT_3,
	GPIO125_KP_MKOUT_4,
	GPIO126_KP_MKOUT_5,
	GPIO127_KP_MKOUT_6,
	GPIO5_2_KP_MKOUT_7,

	/* MMC1 for SDIO WiFi */
	GPIO18_MMC1_DAT0,
	GPIO19_MMC1_DAT1 | MFP_LPM_EDGE_BOTH,
	GPIO20_MMC1_DAT2,
	GPIO21_MMC1_DAT3,
	GPIO22_MMC1_CLK,
	GPIO23_MMC1_CMD,

	/* MMC2 for SD Card */
	GPIO24_MMC2_DAT0,
	GPIO25_MMC2_DAT1 | MFP_LPM_EDGE_BOTH,
	GPIO26_MMC2_DAT2,
	GPIO27_MMC2_DAT3,
	GPIO28_MMC2_CLK,
	GPIO29_MMC2_CMD,

	/* USB Host */
	GPIO2_2_USBH_PEN,
	GPIO3_2_USBH_PWR,

	/* PWM */
	GPIO12_PWM1_OUT,
	GPIO13_PWM2_OUT,

	/* Bus Devices */
	GPIO0_DRQ,
	GPIO2_RDY,
	GPIO3_nCS2,
	GPIO4_nCS3,

	/* UTM */
#if 0
	GPIO10_UTM_CLK,
	GPIO41_U2D_PHYDATA_0,
	GPIO42_U2D_PHYDATA_1,
	GPIO43_U2D_PHYDATA_2,
	GPIO44_U2D_PHYDATA_3,
	GPIO45_U2D_PHYDATA_4,
	GPIO46_U2D_PHYDATA_5,
	GPIO47_U2D_PHYDATA_6,
	GPIO48_U2D_PHYDATA_7,
	GPIO73_UTM_TXREADY,
	GPIO74_U2D_RESET,
	GPIO83_U2D_TXVALID,
	GPIO85_UTM_RXVALID,
	GPIO86_UTM_RXACTIVE,
	GPIO87_U2D_RXERROR,
	GPIO88_U2D_OPMODE0,
	GPIO99_U2D_XCVR_SEL,
	GPIO100_U2D_TERM_SEL,
	GPIO101_U2D_SUSPEND,
	GPIO102_UTM_LINESTATE_0,
	GPIO103_UTM_LINESTATE_1,
	GPIO104_U2D_OPMODE1,
#endif


	/* GPIOs or unused pins */
	GPIO1_GPIO,
	GPIO5_GPIO,
	GPIO6_GPIO,
	GPIO7_GPIO,
	GPIO8_GPIO | MFP_LPM_EDGE_BOTH | MFP_PULL_HIGH,		// USB_OTG IRQ
	GPIO9_GPIO | MFP_LPM_EDGE_FALL,						// CPLD IRQ
	GPIO11_GPIO,
	GPIO15_GPIO,
	GPIO16_GPIO,										// PwrMicro Reset
	GPIO17_GPIO,										// PwrMicro Unknown Function, appears to be ACK of some form
	GPIO30_GPIO,										// Red Notification LED
	GPIO31_GPIO,										// Green Notification LED
	GPIO36_GPIO,										// Unused AC97 pin. Could be used as GPIO
	GPIO57_GPIO,
//	GPIO58_GPIO,						// Not set in Windows. Linux does not have definition for this GPIO
	GPIO84_GPIO,						// USB_OTG (TBC)
	GPIO97_GPIO,
	GPIO98_GPIO,
	GPIO0_2_GPIO,
	GPIO1_2_GPIO,

	/* Camera Interface (unused but defined) */
	GPIO49_CI_DD_0,
	GPIO50_CI_DD_1,
	GPIO51_CI_DD_2,
	GPIO52_CI_DD_3,
	GPIO53_CI_DD_4,
	GPIO54_CI_DD_5,
	GPIO55_CI_DD_6,
	GPIO56_CI_DD_7,
	GPIO59_CI_MCLK,
	GPIO60_CI_PCLK,
	GPIO61_CI_HSYNC,
	GPIO62_CI_VSYNC,
};

static struct resource mc3190_cpld_resources[] = {
    {
        .start  	= MC3190_CPLD_BASE,
        .end    	= MC3190_CPLD_BASE + MC3190_CPLD_SZ - 1,
        .flags  	= IORESOURCE_MEM,
    },
};

static struct platform_device mc3190_cpld_device = {
    .name          	= "mc3190-cpld",
    .id            	= -1,
    .num_resources 	= ARRAY_SIZE(mc3190_cpld_resources),
    .resource      	= mc3190_cpld_resources,
};

static struct mc3190_bl_platform_data mc3190_backlight_data = {
	.pwm_id			= 2,
	.pwm_period_ns	= 23077,
	.max_brightness	= 99,
	.dft_brightness	= 60,
};

static struct gpio_led mc3190_leds[] = {
	[0] = {
		.name				= "mc3190-red",
		.default_trigger	= "none",
		.gpio				= mfp_to_gpio(MFP_PIN_GPIO30),
	},
	[1] = {
		.name				= "mc3190-green",
		.default_trigger	= "none",
		.gpio				= mfp_to_gpio(MFP_PIN_GPIO31),
	},
};

static struct gpio_led_platform_data mc3190_leds_info = {
	.leds		= mc3190_leds,
	.num_leds	= ARRAY_SIZE(mc3190_leds),
};

static struct platform_device mc3190_device_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &mc3190_leds_info,
	}
};

static struct platform_device mc3190_backlight_device = {
	.name	= "mc3190-backlight",
	.id	= -1,
	.dev	= {
		.parent = &pxa27x_device_pwm0.dev,
		.platform_data = &mc3190_backlight_data,
	},
};

static struct pxa2xx_spi_chip mc3190_lcd_chip_info = {
	.tx_threshold	= 1,
	.rx_threshold	= 1,
	.dma_burst_size	= 1,
	.timeout	= 200,
	.gpio_cs	= -1,
};

static struct spi_board_info mc3190_lcd_spi_board_info[] __initdata = {
    {
        .modalias        = "mc3190_lcd",
        .max_speed_hz    = 1625000,
        .bus_num         = 3,
        .chip_select     = 0,
        .mode            = SPI_MODE_3,
		.controller_data = &mc3190_lcd_chip_info,
    },
};

static struct pxafb_mode_info mc3190_lcd_mode = {
	.pixclock	= 115384,
	.xres		= 320,
	.yres		= 320,
	.bpp		= 16,

	.left_margin	= 27,
	.right_margin	= 7,
	.upper_margin	= 7,
	.lower_margin	= 8,

	.hsync_len	= 6,
	.vsync_len	= 1,
};

static struct pxafb_mach_info mc3190_lcd_info = {
	.modes			= &mc3190_lcd_mode,
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static void __init mc3190_init_lcd(void)
{
	platform_device_register(&mc3190_backlight_device);

	set_pxa_fb_info(&mc3190_lcd_info);
}

static struct resource mc3190_ssp4_resources[] = {
	[0] = {
		.start = 0x41A00000,
		.end   = 0x41A0003F,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SSP4,
		.end   = IRQ_SSP4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mc3190_pwrmicro_device = {		// FIXME: Rename the driver to pwrmicro
	.name          = "mc3190-pwrmicro",
	.id            = -1,
	.resource      = mc3190_ssp4_resources,
	.num_resources = ARRAY_SIZE(mc3190_ssp4_resources),
};

#if defined(CONFIG_MMC)
static int mc3190_mci_init(struct device *dev, irq_handler_t detect_int, void *data)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);

	if (mmc) {
		/* Tell the MMC core layer that the card cannot be removed at runtime */
		mmc->caps |= MMC_CAP_NONREMOVABLE;
	}

	return 0;
}

static struct pxamci_platform_data mc3190_mci_platform_data = {
	.detect_delay_ms= 200,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.gpio_card_detect = EXT_GPIO(0),
	.gpio_card_ro	= EXT_GPIO(2),
	.gpio_power	= -1,
};

static struct pxamci_platform_data mc3190_mci2_platform_data = {	// SD Card
	.detect_delay_ms	= 200,
	.ocr_mask			= MMC_VDD_32_33|MMC_VDD_33_34,
	.gpio_card_detect 	= -1,
	.gpio_card_ro		= -1,
	.gpio_power			= -1,
	.init               = mc3190_mci_init,
};

static void __init mc3190_init_mmc(void)
{
	pxa_set_mci_info(&mc3190_mci_platform_data);					// Unused?
	pxa3xx_set_mci2_info(&mc3190_mci2_platform_data);				// SD Card
}
#else
static inline void mc3190_init_mmc(void) {}
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int mc3190_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(5, 3, KEY_HOME), 	/* Left green button */
 	KEY(0, 1, KEY_BACK),		/* Right red button */

	/* Num Keys */
	KEY(2, 0, KEY_7), 		  KEY(0, 2, KEY_8), KEY(1, 0, KEY_9),   KEY(0, 3, KEY_MENU), /* Orange Alt Button */
	KEY(0, 4, KEY_4), 		  KEY(0, 5, KEY_5), KEY(0, 6, KEY_6),   KEY(0, 7, KEY_LEFTSHIFT),
	KEY(0, 0, KEY_1), 		  KEY(5, 4, KEY_2), KEY(1, 1, KEY_3),   KEY(1, 2, KEY_ENTER), 
	KEY(1, 3, KEY_BACKSPACE), KEY(1, 4, KEY_0), KEY(1, 5, KEY_DOT),

	/* Letter Keys */
	KEY(1, 6, KEY_LEFTCTRL), KEY(1, 7, KEY_A), KEY(5, 5, KEY_B), KEY(2, 1, KEY_C), KEY(2, 2, KEY_FN), /* Func button */ 
	KEY(2, 3, KEY_D), 		 KEY(2, 4, KEY_E), KEY(2, 5, KEY_F), KEY(2, 6, KEY_G), KEY(2, 7, KEY_H),
	KEY(3, 0, KEY_I),		 KEY(3, 1, KEY_J), KEY(3, 2, KEY_K), KEY(3, 3, KEY_L), KEY(3, 4, KEY_M),
	KEY(3, 5, KEY_N),		 KEY(3, 6, KEY_O), KEY(3, 7, KEY_P), KEY(4, 0, KEY_Q), KEY(4, 1, KEY_R), 
	KEY(4, 2, KEY_S),		 KEY(4, 3, KEY_T), KEY(4, 4, KEY_U), KEY(4, 5, KEY_V), KEY(4, 6, KEY_W),
	KEY(4, 7, KEY_SPACE),	 KEY(5, 0, KEY_X), KEY(5, 1, KEY_Y), KEY(5, 2, KEY_Z)
	/* KEY(4, 7) is Backlight button*/
};

static unsigned int mc3190_matrix_key_map_fn[] = {
    /* Num Keys */
    /* KEY(2, 0, KEY_7), */           KEY(0, 2, KEY_UP), /*   KEY(1, 0, KEY_9), */  /* KEY(0, 3, KEY_MENU), */
    KEY(0, 4, KEY_LEFT),              KEY(0, 5, KEY_ENTER),   KEY(0, 6, KEY_RIGHT), /* KEY(0, 7, KEY_LEFTSHIFT), */
    /* KEY(0, 0, KEY_1), */           KEY(5, 4, KEY_DOWN), /* KEY(1, 1, KEY_3), */  /* KEY(1, 2, KEY_ENTER), */ 
    KEY(1, 3, KEY_SPACE),          /* KEY(1, 4, KEY_0), */ /* KEY(1, 5, KEY_DOT), */

    /* Letter Keys */
    KEY(1, 6, KEY_LEFTALT),           KEY(1, 7, KEY_F1),      KEY(5, 5, KEY_F2),       KEY(2, 1, KEY_F3),   /* KEY(2, 2, KEY_ALT), */ /* Func button */ 
    KEY(2, 3, KEY_F4),                KEY(2, 4, KEY_F5),      KEY(2, 5, KEY_F6),       KEY(2, 6, KEY_F7),      KEY(2, 7, KEY_F8),
    KEY(3, 0, KEY_F9),                KEY(3, 1, KEY_F10),     KEY(3, 2, KEY_F11),      KEY(3, 3, KEY_F12),     KEY(3, 4, KEY_F13),
    KEY(3, 5, KEY_BRIGHTNESSUP),   /* KEY(3, 6, KEY_O),       KEY(3, 7, KEY_P),        KEY(4, 0, KEY_Q), */    KEY(4, 1, KEY_VOLUMEUP), 
    KEY(4, 2, KEY_BRIGHTNESSDOWN), /* KEY(4, 3, KEY_T),       KEY(4, 4, KEY_U),        KEY(4, 5, KEY_V), */    KEY(4, 6, KEY_VOLUMEDOWN),
    /* KEY(4, 7, KEY_SPACE),          KEY(5, 0, KEY_X),       KEY(5, 1, KEY_Y),        KEY(5, 2, KEY_Z) */
    /* KEY(4, 7) is Backlight button*/
};

static struct pxa27x_keypad_platform_data mc3190_keypad_info = {
	.matrix_key_rows		= 8,
	.matrix_key_cols		= 8,
	.matrix_key_map			= mc3190_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(mc3190_matrix_key_map),
	.matrix_key_map_fn		= mc3190_matrix_key_map_fn,
	.matrix_key_map_fn_size	= ARRAY_SIZE(mc3190_matrix_key_map_fn),

	.debounce_interval	 = 30,
};

static void __init mc3190_init_keypad(void)
{
	pxa_set_keypad_info(&mc3190_keypad_info);
}
#else
static inline void mc3190_init_keypad(void) {}
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct mtd_partition mc3190_nand_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x060000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x060000,
		.size        = 0x200000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[2] = {
		.name        = "Filesystem",
		.offset      = 0x0260000,
		.size        = 0x3000000,     /* 48M - rootfs */
	},
	[3] = {
		.name        = "MassStorage",
		.offset      = 0x3260000,
		.size        = 0x3d40000,
	},
	[4] = {
		.name        = "BBT",
		.offset      = 0x6FA0000,
		.size        = 0x80000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTE: we reserve some blocks at the end of the NAND flash for
	 * bad block management, and the max number of relocation blocks
	 * differs on different platforms. Please take care with it when
	 * defining the partition table.
	 */
};

static struct pxa3xx_nand_platform_data mc3190_nand_info = {
	.enable_arbiter	= 1,
	.parts			= mc3190_nand_partitions,
	.nr_parts		= ARRAY_SIZE(mc3190_nand_partitions),
};

static void __init mc3190_init_nand(void)
{
	pxa3xx_set_nand_info(&mc3190_nand_info);
}
#else
static inline void mc3190_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data mc3190_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT1 | ENABLE_PORT2 |
			  POWER_CONTROL_LOW | POWER_SENSE_LOW,
};

static void __init mc3190_init_ohci(void)
{
	pxa_set_ohci_info(&mc3190_ohci_info);
}
#else
static inline void mc3190_init_ohci(void) {}
#endif /* CONFIG_USB_OHCI_HCD || CONFIG_USB_OHCI_HCD_MODULE */

static struct pxa2xx_spi_master pxa_ssp3_spi_master_info = {
	.clock_enable   = CKEN_SSP3,
	.num_chipselect = 2,
	.enable_dma     = 1
};

struct platform_device pxa_spi_ssp3 = {
	.name          = "pxa2xx-spi",
	.id            = 3,
	.dev           = {
		.platform_data = &pxa_ssp3_spi_master_info,
	}
};

static int mc3190_udc_is_connected(void)
{
    return !!(mc3190_cpld_read(MC3190_CPLD_REG_USB_STATUS) &
              MC3190_CPLD_USB_CONNECTED_BIT);
}

static void mc3190_udc_command(int command)
{
    switch (command) {
    case PXA2XX_UDC_CMD_CONNECT:
		pr_info("%s: PXA2XX_UDC_CMD_CONNECT\n", __func__);
		// FIXME: Add code
        break;
    case PXA2XX_UDC_CMD_DISCONNECT:
		pr_info("%s: PXA2XX_UDC_CMD_DISCONNECT\n", __func__);
		// FIXME: Add code
        break;
    }
}

static struct pxa2xx_udc_mach_info mc3190_udc_info __initdata = {
    .udc_is_connected = mc3190_udc_is_connected,
	.gpio_pullup = -1,
    .gpio_vbus = -1,
	.udc_command = mc3190_udc_command,
};

static char *mc3190_usb_functions[] = {
    "adb",
};

static struct android_usb_platform_data mc3190_android_usb_pdata = {
    .vendor_id          = 0x05E0,
    .product_id         = 0x2001,
    .version            = 0x0100,
    .product_name       = "MC3190",
    .manufacturer_name  = "Symbol Technologies",
    .serial_number      = "MC3190000001",
    .num_functions      = ARRAY_SIZE(mc3190_usb_functions),
    .functions          = mc3190_usb_functions,
};

static struct platform_device mc3190_android_usb_device = {
    .name   = "android_usb",
    .id     = -1,
    .dev    = {
        .platform_data = &mc3190_android_usb_pdata,
    },
};

static struct platform_device *mc3190_devices[] __initdata = {
	&pxa_spi_ssp3,
	&mc3190_pwrmicro_device,
	&mc3190_android_usb_device,
	&mc3190_wm9713_audio,
};

static int __init mc3190_cpld_device_init(void)
{
    return platform_device_register(&mc3190_cpld_device);
}
arch_initcall(mc3190_cpld_device_init);

static void __init mc3190_init(void)
{
	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);

	pxa3xx_mfp_config(ARRAY_AND_SIZE(mfp_cfg));

	pxa_set_ac97_info(NULL);
	mc3190_init_lcd();
	mc3190_init_mmc();
	mc3190_init_keypad();
//	mc3190_init_nand(); 			// FIXME: Disabled as this interferes with data/address lines to CPLD

	platform_device_register(&mc3190_device_leds);

	platform_add_devices(ARRAY_AND_SIZE(mc3190_devices));
	spi_register_board_info(ARRAY_AND_SIZE(mc3190_lcd_spi_board_info));

    pxa_set_udc_info(&mc3190_udc_info);

//	mc3190_init_ohci();
}

MACHINE_START(MC3190, "Symbol MC3190")
	.phys_io	= 0x40000000,
	.boot_params	= 0xA0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa3xx_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= mc3190_init,
MACHINE_END
