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
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include "jedi.h"

#define JEDI_VENDOR_ID  0x0013
#define JEDI_DEVICE_ID  0xc684

struct jedi_private {
    struct sdio_func *func;
    struct net_device *ndev;
};

/* SDIO Interrupt Handler */
static void jedi_sdio_irq(struct sdio_func *func)
{
    struct jedi_private *priv = sdio_get_drvdata(func);

    /* TODO: Read interrupt status registers via sdio_readb/sdio_memcpy_fromio */
    dev_dbg(&func->dev, "SDIO IRQ triggered\n");
}

/* Match any SDIO Vendor/Device ID initially to capture enumeration */
static const struct sdio_device_id jedi_ids[] = {
    { SDIO_DEVICE(JEDI_VENDOR_ID, JEDI_DEVICE_ID) },
    { /* Sentinel */ },
};
MODULE_DEVICE_TABLE(sdio, jedi_ids);

static int jedi_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
    struct net_device *ndev;
    struct jedi_private *priv;
    int ret;

    pr_info("symbol_jedi: Probing SDIO device Vendor: 0x%04x, Device: 0x%04x (Func %d)\n",
            func->vendor, func->device, func->num);

    /* Allocate network device */
    ndev = alloc_etherdev(sizeof(struct jedi_private));
    if (!ndev)
        return -ENOMEM;

    priv = netdev_priv(ndev);
    priv->func = func;
    priv->ndev = ndev;
    sdio_set_drvdata(func, priv);

    /* Claim host for configuration */
    sdio_claim_host(func);

    ret = sdio_enable_func(func);
    if (ret) {
        pr_err("symbol_jedi: Failed to enable SDIO function: %d\n", ret);
        goto err_free_dev;
    }

    ret = sdio_claim_irq(func, jedi_sdio_irq);
    if (ret) {
        pr_err("symbol_jedi: Failed to claim SDIO IRQ: %d\n", ret);
        goto err_disable_func;
    }

    /* Set block size for register/FIFO transactions (default to 64 or 512 bytes) */
    ret = sdio_set_block_size(func, 512);
    if (ret) {
        pr_warn("symbol_jedi: Unable to set block size to 512\n");
        sdio_set_block_size(func, 64);
    }

    sdio_release_host(func);

    jedi_fw_download(func);

    /* TODO: Implement net_device_ops and call register_netdev(ndev) */

    return 0;

err_disable_func:
    sdio_disable_func(func);
err_free_dev:
    sdio_release_host(func);
    free_netdev(ndev);
    return ret;
}

static void jedi_remove(struct sdio_func *func)
{
    struct jedi_private *priv = sdio_get_drvdata(func);

    if (priv) {
        sdio_claim_host(func);
        sdio_release_irq(func);
        sdio_disable_func(func);
        sdio_release_host(func);

        if (priv->ndev)
            free_netdev(priv->ndev);
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