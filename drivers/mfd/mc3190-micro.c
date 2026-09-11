/*
 * mc3190-micro.c -- MFD Driver for Symbol MC3190 PwrMicro
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
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/mfd/core.h>
#include <linux/power_supply.h>

#include <linux/mc3190.h>

#ifdef CONFIG_MFD_MC3190_PM_DEBUG
#define DEBUG
#endif // CONFIG_MFD_MC3190_PM_DEBUG

#define DRV_NAME "mc3190-pwrmicro"

#define SSCR0   0x00
#define SSCR1   0x04
#define SSSR    0x08
#define SSDR    0x10

#define SSCR0_DSS_32BIT   0xF
#define SSCR0_EDSS        (1 << 20)
#define SSCR0_SSE         (1 << 7)

#define SSCR1_RIE       (1 << 0)
#define SSCR1_RWOT      (1 << 23)
#define SSCR1_SFRMDIR   (1 << 24)
#define SSCR1_SCLKDIR   (1 << 25)
#define SSCR1_SCFR      (1 << 28)

#define SSSR_RNE   (1 << 3)
#define SSSR_ROR   (1 << 7)
#define SSSR_BCE   (1 << 23)
#define SSSR_TUR   (1 << 21)

#define PWRMICRO_ACK_OUT_GPIO 17
#define PWRMICRO_RESET_GPIO   16

/*
 * AVR Opcodes:
 * ============
 * 
 * 0x01	: (UNCONFIRMED BUT LIKELY) Touchscreen related
 * 0x02	:
 * 0x03	: Touchscreen X/Y data		(from AVR)
 * 0x04	: System Command Ack 		(from AVR)
 * 0x05	: (TO DECODE) Sets DAT_mem_037c
 * 0x06	: "SPI: Low battery Wakeup Level" (DAT_c0cd953c in SPI.dll)
 * 0x07	: Battery/power status (07 xx yy zz - xx = UNKNOWN, yy = Charging Status? 05 when charging, 04 when not charging/full, zz = Battery Level)
 * 0x08	: (TO DECODE) Sets DAT_mem_0087, calls FUN_code_0438()
 * 0x09	: (TO DECODE) Writes to DAT_mem_0359, 0x0235, 0x0236
 * 0x0A	:
 * 0x0B	: (TO DECODE) Calls FUN_code_17a4(), "SPI: Get SPI Driver ID"
 * 0x0C	: (TO DECODE) Calls FUN_code_1a32() and exits immediately
 * 0x0D	:
 * 0x0E	: (DOESN'T SEEM RIGHT) Backlight / LED Level Control (0x0360, 0x0361, 0x0362)
 * 0x0F	: 
 * 0x10	:
 * 	   -05 : Battery mV
 * 	   -07 : Battery percent and charge status flags? (10 07 xx yy - xx = 00 discharging, 07 when full, 05 when charging, 06 charging nearly full (charge light stopped blinking), yy = battery percentage (shows FF when AVR doesn't want to show percent))
 * 0x11	: (TO DECODE) Calls FUN_code_11ac() (Secondary status/query response), "SPI: PwrEvnt"
 * 0x12	:
 * 0x13	: "SPI: Write Smart Batt EEPROM"
 * 0x14	: "SPI: Read Smart Batt EEPROM"
 * 0x15	: (UNCONFIRMED) Power Off / Reset Control (calls FUN_code_0cf0() if params are zero)
 * 0x16	: "SPI: LED State"
 * 0x17	: (TO DECODE) Sets DAT_mem_0376 "SPI: LED2 State"
 * 0x18	: "SPI: Touch Panel Configuration2"
 * 0x19	: (UNCONFIRMED) Reads diagnostic/ADC registers (DAT_mem_00db, DAT_mem_00da, 0x0088) "SPI: IEEE1725 Limits"
 * 0x1A	: (TO DECODE) Sets register DAT_mem_0088
 * 0x1B	:
 * 0x1C	:
 * 0x1D	: (UNCONFIRMED) Battery Fuel Gauge / Charger Thresholds (Checks thresholds like 100, 244/0xf4)
 * 0x1E	: "SPI: USB Charge Current"
 * 0x1F	: "SPI: Gifted Battery Command Response Message"
 * 0x20	: PwrMicro Firmware Version (20 xx yy zz - xx = major, yy = minor, zz = UNKNOWN)
 * 0x21	: PwrMicro Firmware Version (21 xx xx zz - xxxx = user, zz = UNKNOWN)
 * 0x22	: "SPI: Get Date First Use"
 * 0x23	: "SPI: Get Health Byte"
 * 0x24	: (TO DECODE) Sets DAT_mem_0191
 * 0x25	: (TO DECODE) Sets DAT_mem_0089, DAT_mem_008a
 * 0x26	:
 * 0x27	:
 * 0x28	: "SPI: Debug port mode"
 * 0x29	: "SPI: Serial Data Byte Recieved"
 * 0x2A	: (TO DECODE) Dynamic response pointer setup (CONCAT11(param1, param2)) "SPI: Flash Byte"
 * 0x2B	:
 * 0x2C	: "SPI: Debug/Test Data"
 * 0x2D	:
 * 0x2E	:
 * 0x2F	:
 * ...
 * 0x3F : "SPI: Command Error"
 */


#define MC3190_RETRY_BIT   0x80
#define MC3190_TAG_MASK    0x7F

#define MC3190_READY_DELAY_MS   300

#define PWRMICRO_STATE_TOUCH	1
#define PWRMICRO_STATE_BATTERY	2

static void pwrmicro_init_tags(struct mc3190_pwrmicro *priv) {
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].is_valid_register = true;
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].needs_update = true;
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].always_needs_update = true;
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].needed_initiator = MC3190_TAG_PWR_EVENT;
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].alternate_initiator = MC3190_TAG_PWR_EVENT_TBC;

	priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].always_needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB06].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB06].needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].always_needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTTEMPERATURE].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTTEMPERATURE].needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTTEMPERATURE].always_needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB09].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB09].needs_update = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB14].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB14].needs_update = true;

	priv->register_data[MC3190_TAG_PWR_EVENT].is_valid_register = true;
	priv->register_data[MC3190_TAG_PWR_EVENT].needs_update = true;
	priv->register_data[MC3190_TAG_PWR_EVENT].always_needs_update = true;
	priv->register_data[MC3190_TAG_PWR_EVENT].needed_initiator = MC3190_TAG_BATT_CAP_EVENT;

	priv->register_data[MC3190_TAG_VER].is_valid_register = true;
	priv->register_data[MC3190_TAG_VER].needs_update = true;
	priv->register_data[MC3190_TAG_VER].needed_initiator = MC3190_TAG_PWR_EVENT;

	priv->register_data[MC3190_TAG_USERVER].is_valid_register = true;
	priv->register_data[MC3190_TAG_USERVER].needs_update = true;
	priv->register_data[MC3190_TAG_USERVER].needed_initiator = MC3190_TAG_VER;

	priv->register_data[MC3190_TAG_CMD_ERR].is_valid_register = false;
	priv->register_data[MC3190_TAG_CMD_ERR].needs_update = false;
}

static void pwrmicro_send_ack(bool awaiting_packet) {
	gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 0);	// Quick pulse of ACK_OUT as an acknowledge of the packet just received
	udelay(3);
	gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 1);
	if (awaiting_packet) {
		udelay(100);
		gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 0);	// Long pulse of ACK_OUT to tell AVR it can send us another packet
	}

	return;
}

static void mc3190_stage_and_ack(struct mc3190_pwrmicro *priv, u32 word)
{
	writel(word, priv->regs + SSDR);
	priv->last_sent_word = word;
	pwrmicro_send_ack(true);
}

static void mc3190_request_information(struct mc3190_pwrmicro *priv, u8 rx_tag)
{
	int i = 0;
	for (i = 0; i < MC3190_TAG_CMD_ERR; i++) {
		if (priv->register_data[i].is_valid_register && (priv->register_data[i].needs_update) && 
				(priv->register_data[i].needed_initiator == rx_tag || priv->register_data[i].alternate_initiator == rx_tag)) {
			if (i == MC3190_TAG_PWR_EVENT_TBC) {
				int j = 0;
				for (j = 0; j <= MC3190_TAG_PWR_EVENT_TBC_SUB14; j++) {
					if (priv->cmd10_subdata[j].is_valid_register && priv->cmd10_subdata[j].needs_update) {
						priv->have_outstanding_request = true;
						priv->outstanding_request_tag = i;
						priv->outstanding_request_word = (i << 24) | (j << 16);
						dev_dbg(priv->dev, "cmd10 tag 0x%02x needs update: outstanding_request_word=0x%08x\n",
											j, priv->outstanding_request_word);
						return;
					}
				}
				priv->register_data[i].needs_update = false;
				continue;

			}
			priv->have_outstanding_request = true;
			priv->outstanding_request_tag = i;
			priv->outstanding_request_word = i << 24;
			dev_dbg(priv->dev, "tag 0x%02x needs update: outstanding_request_word=0x%08x\n",
								i, priv->outstanding_request_word);
			return;
		}
	}
}

static void mc3190_store_register_data(struct mc3190_pwrmicro *priv, u8 reg, u32 payload) {
	if (reg != MC3190_TAG_PWR_EVENT_TBC) {
		priv->register_data[reg].data[0] = payload >> 16 & 0xFF;
		priv->register_data[reg].data[1] = payload >> 8 & 0xFF;
		priv->register_data[reg].data[2] = payload & 0xFF;
		priv->register_data[reg].needs_update = false;
	} else {
		u8 subcmd = payload >> 16;
		priv->cmd10_subdata[subcmd].data[0] = payload >> 8 & 0xFF;
		priv->cmd10_subdata[subcmd].data[1] = payload & 0xFF;
		priv->cmd10_subdata[subcmd].needs_update = false;

	}
	
}

static void pwrmicro_reset_always_needs_updates(struct mc3190_pwrmicro *priv) {
	int i = 0;
	for (i = 0; i < MC3190_TAG_CMD_ERR; i++) {
		if (priv->register_data[i].is_valid_register && priv->register_data[i].always_needs_update) {
			priv->register_data[i].needs_update = true;
		}
	}	

	for (i = 0; i < MC3190_TAG_PWR_EVENT_TBC_SUB14; i++) {
		if (priv->cmd10_subdata[i].is_valid_register && priv->cmd10_subdata[i].always_needs_update) {
			priv->cmd10_subdata[i].needs_update = true;
		}
	}	
}


static void mc3190_dispatch(struct mc3190_pwrmicro *priv, u32 rx_word)
{
	u8 raw_tag = (rx_word >> 24) & 0xFF;
	bool is_retry = raw_tag & MC3190_RETRY_BIT;
	u8 tag = raw_tag & MC3190_TAG_MASK;
	u32 payload = rx_word & 0xFFFFFF;
	bool is_same_as_last = priv->have_last_cmd &&
			        tag == priv->last_cmd_tag &&
			        payload == priv->last_cmd_payload;

	if (priv->have_outstanding_request && tag == priv->outstanding_request_tag) {
		dev_dbg(priv->dev, "outstanding request 0x%02x answered: raw_tag=0x%02x payload=0x%06x\n",
			 tag, raw_tag, payload);
		priv->have_outstanding_request = false;
	}

	if ((is_retry || tag == 0x00) && is_same_as_last) {
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else if (priv->have_reply)
			mc3190_stage_and_ack(priv, priv->last_reply);
		else
			pwrmicro_send_ack(false);
		return;
	}

	if (tag == 0x00 && !is_same_as_last) {
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else
			pwrmicro_send_ack(false);
		return;
	}

	priv->have_last_cmd = true;
	priv->last_cmd_tag = tag;
	priv->last_cmd_payload = payload;
	priv->have_reply = false;

	switch (tag) {
	case MC3190_TAG_TOUCH_DATA: {
		u16 y = rx_word & 0xFFF;
		u16 x = ((rx_word >> 12) & 0xFF0) | ((rx_word & 0xF000) >> 4);
		dev_dbg(priv->dev, "touch data: raw=0x%08x x=%u y=%u\n", rx_word, x, y);

#ifdef CONFIG_TOUCHSCREEN_MC3190
		if (priv->touch_input)
			mc3190_touch_report(priv->touch_input, x, y, PWRMICRO_TOUCH_DOWN);
#endif // CONFIG_TOUCHSCREEN_MC3190

		priv->in_state_machine = true;
		priv->state_machine = PWRMICRO_STATE_TOUCH;
		pwrmicro_send_ack(false);
		break;
	}

	case MC3190_TAG_BATT_CAP_EVENT:
		if (!priv->ready_for_gated_tags) {
			dev_dbg(priv->dev, "tag 0x07 received but not ready yet - ignoring\n");
			break;
		}

		pwrmicro_reset_always_needs_updates(priv);
		dev_dbg(priv->dev, "Battery Capacity Event (rx_word=0x%08x)\n", rx_word);
		priv->in_state_machine = true;
		priv->state_machine = PWRMICRO_STATE_BATTERY;

		mc3190_store_register_data(priv, tag, payload);
#ifdef CONFIG_MC3190_BATTERY
		if (priv->battery_psy)
			power_supply_changed(priv->battery_psy);
#endif // CONFIG_MC3190_BATTERY

		if (!priv->have_outstanding_request)
			mc3190_request_information(priv, tag);
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else
			pwrmicro_send_ack(false);
		break;


	case MC3190_TAG_PWR_EVENT_TBC:
	case MC3190_TAG_PWR_EVENT:
		dev_dbg(priv->dev, "Power Event (rx_word=0x%08x)\n", rx_word);

		mc3190_store_register_data(priv, tag, payload);
#ifdef CONFIG_MC3190_BATTERY
		if (priv->battery_psy)
			power_supply_changed(priv->battery_psy);
#endif // CONFIG_MC3190_BATTERY

		if (!priv->have_outstanding_request)
			mc3190_request_information(priv, tag);
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else
			pwrmicro_send_ack(false);
		break;

	case MC3190_TAG_VER:
		dev_dbg(priv->dev, "Version Event (rx_word=0x%08x)\n", rx_word);

		priv->versionMajor = (payload >> 16);
		priv->versionMinor = (payload >> 8);

		mc3190_store_register_data(priv, tag, payload);
		if (!priv->have_outstanding_request)
			mc3190_request_information(priv, tag);
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else
			pwrmicro_send_ack(false);
		break;

	case MC3190_TAG_USERVER:
		dev_dbg(priv->dev, "User Version Event (rx_word=0x%08x)\n", rx_word);

		priv->versionUser = (payload >> 8);

		if (priv->versionMajor != 0 && priv->versionMinor != 0) {
			dev_info(priv->dev, "PwrMicro Firmware Version: %d.%d.%d\n", priv->versionMajor, priv->versionMinor, priv->versionUser);
		}

		mc3190_store_register_data(priv, tag, payload);
		if (!priv->have_outstanding_request)
			mc3190_request_information(priv, tag);
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else
			pwrmicro_send_ack(false);
		break;

	case MC3190_TAG_SYS_ACK:
		dev_dbg(priv->dev, "ACK Tag (rx_word=0x%08x)\n", rx_word);
		if (priv->in_state_machine) {
			switch (priv->state_machine) {
			case PWRMICRO_STATE_TOUCH:
#ifdef CONFIG_TOUCHSCREEN_MC3190
				if (priv->touch_input)
					mc3190_touch_report(priv->touch_input, 0, 0, PWRMICRO_TOUCH_UP);
#endif // CONFIG_TOUCHSCREEN_MC3190
				break;
			default:
				break;
			}
			priv->in_state_machine = false;
			priv->state_machine = 0;
		}
		if (priv->have_outstanding_request)
			mc3190_stage_and_ack(priv, priv->outstanding_request_word);
		else
			pwrmicro_send_ack(false);
		break;

	case MC3190_TAG_CMD_ERR:
		dev_err(priv->dev, "PwrMicro returned error 0x%08x for last tx (0x%08x)\n", rx_word, priv->last_sent_word);
		break;		

	default:
		dev_warn(priv->dev, "Unhandled AVR Tag: 0x%02x (rx_word=0x%08x)\n", tag, rx_word);
		pwrmicro_send_ack(false);
		break;
	}
}

static void mc3190_rx_work_fn(struct work_struct *work)
{
	struct mc3190_pwrmicro *priv = container_of(work, struct mc3190_pwrmicro, rx_work);
	unsigned long flags;
	u32 rx_word;
	bool have;

	spin_lock_irqsave(&priv->lock, flags);
	have = priv->have_rx_word;
	rx_word = priv->last_rx_word;
	priv->have_rx_word = false;
	spin_unlock_irqrestore(&priv->lock, flags);

	if (have)
		mc3190_dispatch(priv, rx_word);
}

static void mc3190_ready_work_fn(struct work_struct *work)
{
	struct mc3190_pwrmicro *priv = container_of(to_delayed_work(work),
						     struct mc3190_pwrmicro, ready_work);
	priv->ready_for_gated_tags = true;
}

static irqreturn_t mc3190_ssp4_irq(int irq, void *dev_id)
{
	struct mc3190_pwrmicro *priv = dev_id;
	u32 status = readl(priv->regs + SSSR);

	gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 1);	// Bring ACK GPIO high after receiving any packets

	if (status & SSSR_RNE) {
		unsigned long flags;
		u32 rx_word = readl(priv->regs + SSDR);

		spin_lock_irqsave(&priv->lock, flags);
		priv->last_rx_word = rx_word;
		priv->have_rx_word = true;
		spin_unlock_irqrestore(&priv->lock, flags);

		schedule_work(&priv->rx_work);
	}

	if (status & SSSR_ROR)
		dev_warn(priv->dev, "RX FIFO overrun (not cleared, matching WinCE)\n");
	if (status & SSSR_BCE)
		dev_warn(priv->dev, "bit count error (not cleared, matching WinCE)\n");

	/* WinCE's ISR clears ONLY TUR here - nothing else */
	if (status & SSSR_TUR)
		writel(status & SSSR_TUR, priv->regs + SSSR);

	return IRQ_HANDLED;
}

void mc3190_set_touch_input(struct mc3190_pwrmicro *priv, struct input_dev *input)
{
	priv->touch_input = input;
}
EXPORT_SYMBOL_GPL(mc3190_set_touch_input);

static int mc3190_pwrmicro_probe(struct platform_device *pdev)
{
	struct mc3190_pwrmicro *priv;
	struct mfd_cell mc3190_cells[] = {
		{	.name = "mc3190-touch", },
		{	.name = "mc3190-battery", },
	};

	struct resource *res;
	u32 sscr0, sscr1;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->lock);
	INIT_WORK(&priv->rx_work, mc3190_rx_work_fn);
	INIT_DELAYED_WORK(&priv->ready_work, mc3190_ready_work_fn);

	mc3190_cells[0].platform_data = priv;
	mc3190_cells[1].platform_data = priv;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	priv->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!priv->regs)
		return -ENOMEM;

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	ret = gpio_request_one(PWRMICRO_ACK_OUT_GPIO, GPIOF_OUT_INIT_HIGH,
				"pwrmicro-ack-out");
	if (ret) {
		dev_err(&pdev->dev, "failed to request ACK_OUT GPIO%d: %d\n",
			PWRMICRO_ACK_OUT_GPIO, ret);
		return ret;
	}

	priv->in_state_machine = false;

	writel(0, priv->regs + SSCR0);

	sscr0 = SSCR0_EDSS | SSCR0_DSS_32BIT;
	writel(sscr0, priv->regs + SSCR0);

	sscr1 = SSCR1_SCFR | SSCR1_SCLKDIR | SSCR1_SFRMDIR |
		SSCR1_RWOT | SSCR1_RIE;
	writel(sscr1, priv->regs + SSCR1);

	ret = devm_request_irq(&pdev->dev, priv->irq, mc3190_ssp4_irq,
				0, DRV_NAME, priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ%d: %d\n", priv->irq, ret);
		return ret;
	}

	sscr0 |= SSCR0_SSE;
	writel(sscr0, priv->regs + SSCR0);

	writel(0x00000000, priv->regs + SSDR);

	schedule_delayed_work(&priv->ready_work, msecs_to_jiffies(MC3190_READY_DELAY_MS));

	pwrmicro_init_tags(priv);

	platform_set_drvdata(pdev, priv);
	dev_info(&pdev->dev, "MC3190 PwrMicro driver active (IRQ%d)\n", priv->irq); // FIXME: get PM version from device

	mfd_add_devices(&pdev->dev, -1, mc3190_cells, ARRAY_SIZE(mc3190_cells), NULL, 0);
	return 0;
}

static int mc3190_pwrmicro_remove(struct platform_device *pdev)
{
	struct mc3190_pwrmicro *priv = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&priv->ready_work);
	cancel_work_sync(&priv->rx_work);
	writel(0, priv->regs + SSCR0);

	return 0;
}

static struct platform_driver mc3190_pwrmicro_driver = {
	.driver = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe  = mc3190_pwrmicro_probe,
	.remove = mc3190_pwrmicro_remove,
};

static int __init mc3190_pwrmicro_init(void)
{
	return platform_driver_register(&mc3190_pwrmicro_driver);
}
module_init(mc3190_pwrmicro_init);

static void __exit mc3190_pwrmicro_exit(void)
{
	platform_driver_unregister(&mc3190_pwrmicro_driver);
}
module_exit(mc3190_pwrmicro_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("MFD Driver for MC3190 AVR PwrMicro");
MODULE_LICENSE("GPL");