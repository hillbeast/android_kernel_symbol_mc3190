/*
 * jedi.c -- Driver for Symbol "Jedi" WiFi module in MC3190
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include "jedi.h"

#define JEDI_VENDOR_ID  0x0013
#define JEDI_DEVICE_ID  0xc684

bool jedi_debug;
module_param_named(debug, jedi_debug, bool, 0644);
MODULE_PARM_DESC(debug, "Enable verbose Jedi driver logging (default off)");

struct jedi_private {
	struct sdio_func *func;
	struct net_device *ndev;
	bool irq_claimed;
};

/*
 * SDIO function interrupt handler (R2.4, JediIsr). Called by the MMC core
 * with the host already claimed. Only claimed AFTER the firmware has
 * booted: before that the chip has nothing sensible to say on DAT1 and an
 * un-acked level interrupt would storm.
 *
 * WinCE: read status byte at 0x04; if bit 2 is set, read the 16-bit
 * HOST_INT and write the same value back to HOST_INT_ACK.
 */
static void jedi_sdio_irq(struct sdio_func *func)
{
	u32 hostint;
	u8 st;
	int ret;

	st = sdio_readb(func, JEDI_REG_STATUS, &ret);
	if (ret || !(st & JEDI_STATUS_HOST_INT))
		return;

	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &hostint);
	if (ret)
		return;

	jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 2, hostint);

	/* TODO: hand hostint to the event/RX/TX handling (JediDpcThread). */
	jedi_dbg(func, "IRQ status=0x%02x HOST_INT=0x%04x\n", st, hostint);
}

static const struct sdio_device_id jedi_ids[] = {
	{ SDIO_DEVICE(JEDI_VENDOR_ID, JEDI_DEVICE_ID) },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(sdio, jedi_ids);

/*
 * R2.1 discriminating test. With the module powered and idle, CMD52 reads
 * of 0x40 and 0x41 are separate bytes. A fixed-address CMD53 read of 2
 * bytes at 0x40 should return byte 0x40 twice. Idle values may all be 0,
 * so this is most informative once the chip has been reset (see the
 * "after chip reset" register dump in jedi_fw.c).
 * Must be called with the host claimed.
 */
static void jedi_regtest(struct sdio_func *func)
{
	u8 b40, b41, pair[2] = { 0, 0 };
	u32 devstat = 0, intena = 0;
	int e40, e41, e54, e44;

	b40 = sdio_readb(func, JEDI_REG_HOST_INT, &e40);
	b41 = sdio_readb(func, JEDI_REG_HOST_INT + 1, &e41);
	sdio_readsb(func, pair, JEDI_REG_HOST_INT, 2);
	e54 = jedi_reg_read(func, JEDI_REG_DEVSTAT, 4, &devstat);
	e44 = jedi_reg_read(func, JEDI_REG_INT_ENABLE, 2, &intena);

	jedi_dbg(func,
		 "regtest: CMD52 0x40=0x%02x (err %d) 0x41=0x%02x (err %d); "
		 "CMD53 fixed readsb(0x40,2)=%02x %02x\n",
		 b40, e40, b41, e41, pair[0], pair[1]);
	jedi_dbg(func,
		 "regtest: DEVSTAT=0x%08x (err %d) INT_ENABLE=0x%04x (err %d)\n",
		 devstat, e54, intena, e44);
}

static int jedi_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	struct net_device *ndev;
	struct jedi_private *priv;
	int ret;

	jedi_dbg(func, "probing SDIO device Vendor: 0x%04x, Device: 0x%04x "
		 "(Func %d)\n", func->vendor, func->device, func->num);

	/* Allocate network device */
	ndev = alloc_etherdev(sizeof(struct jedi_private));
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->func = func;
	priv->ndev = ndev;
	priv->irq_claimed = false;
	sdio_set_drvdata(func, priv);

	/* Claim host for configuration */
	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret) {
		dev_err(&func->dev,
			"symbol_jedi: failed to enable SDIO function: %d\n",
			ret);
		sdio_release_host(func);
		goto err_free_dev;
	}

	/* WinCE sets block size 0x200 */
	ret = sdio_set_block_size(func, 512);
	if (ret) {
		dev_warn(&func->dev,
			 "symbol_jedi: unable to set block size to 512 (%d), "
			 "trying 64\n", ret);
		sdio_set_block_size(func, 64);
	}

	if (jedi_debug)
		jedi_regtest(func);

	sdio_release_host(func);

	/*
	 * Reset the chip into its boot ROM, upload the image, and wait for
	 * FW Init. jedi_fw_download() claims the host itself.
	 */
	ret = jedi_fw_download(func);
	if (ret) {
		dev_err(&func->dev,
			"symbol_jedi: firmware download failed: %d\n", ret);
		goto err_disable_func;
	}

	/* Firmware is running: now it is safe to take its interrupt. */
	sdio_claim_host(func);
	ret = sdio_claim_irq(func, jedi_sdio_irq);
	sdio_release_host(func);
	if (ret) {
		dev_err(&func->dev,
			"symbol_jedi: failed to claim SDIO IRQ: %d\n", ret);
		goto err_disable_func;
	}
	priv->irq_claimed = true;

	/* TODO: Implement net_device_ops and call register_netdev(ndev) */

	return 0;

err_disable_func:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
err_free_dev:
	sdio_set_drvdata(func, NULL);
	free_netdev(ndev);
	return ret;
}

static void jedi_remove(struct sdio_func *func)
{
	struct jedi_private *priv = sdio_get_drvdata(func);

	if (priv) {
		struct net_device *ndev = priv->ndev;

		sdio_claim_host(func);
		if (priv->irq_claimed)
			sdio_release_irq(func);
		sdio_disable_func(func);
		sdio_release_host(func);

		sdio_set_drvdata(func, NULL);
		free_netdev(ndev);	/* frees priv too */
	}
	pr_info("symbol_jedi: Removed\n");
}

static struct sdio_driver jedi_driver = {
	.name     = "symbol_jedi",
	.id_table = jedi_ids,
	.probe    = jedi_probe,
	.remove   = jedi_remove,
};

static int __init jedi_init(void)
{
	return sdio_register_driver(&jedi_driver);
}

static void __exit jedi_exit(void)
{
	sdio_unregister_driver(&jedi_driver);
}

module_init(jedi_init);
module_exit(jedi_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol Jedi SDIO Wi-Fi Driver");
MODULE_LICENSE("GPL");