/*
 * Driver for TI CC2420 802.15.4 Wireless-PAN Networking controller
 *
 * Copyright (C) 2018 Xue Liu <liuxuenetmail@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

#include <linux/device.h>

#include "cc2420.h"

#define printdev(X) (&X->spi->dev)

#define CC2420_WRITEREG(x) (x)
#define CC2420_READREG(x) (0x40 | x)
#define CC2420_RAMADDR(x) ((x & 0x7F) | 0x80)
#define CC2420_RAMBANK(x) ((x >> 1) & 0xc0)
#define CC2420_WRITERAM(x) (x)
#define CC2420_READRAM(x) (0x20 | x)

#define CC2420_FREQ_MASK 	0x3FF
#define CC2420_ADR_DECODE_MASK	0x0B00
#define CC2420_FIFOP_THR_MASK	0x003F
#define CC2420_CRC_MASK		0x80
#define CC2420_RSSI_MASK	0x7F
#define CC2420_FSMSTATE_MASK	0x2F

#define CC2420_MANFIDLOW 	0x233D
#define CC2420_MANFIDHIGH 	0x3000

#define SPI_COMMAND_BUFFER	3
#define	HIGH			1
#define	LOW			0

#define RSSI_OFFSET 45

#define STATE_PDOWN 0
#define STATE_IDLE  1
#define STATE_RX_CALIBRATE		2
#define STATE_RX_CALIBRATE2		40

#define STATE_RX_SFD_SEARCH_MIN 3
#define STATE_RX_SFD_SEARCH_MAX 6
#define STATE_RX_FRAME			16
#define STATE_RX_FRAME2			40
#define STATE_RX_WAIT			14
#define STATE_RX_OVERFLOW		17
#define STATE_TX_ACK_CALIBRATE	48
#define STATE_TX_ACK_PREAMBLE_MIN	49
#define STATE_TX_ACK_PREAMBLE_MAX	51
#define STATE_TX_ACK_MIN			52
#define STATE_TX_ACK_MAX			54
#define STATE_TX_CALIBRATE			32
#define STATE_TX_PREAMBLE_MIN		34
#define STATE_TX_PREAMBLE_MAX		36
#define STATE_TX_FRAME_MIN			37
#define STATE_TX_FRAME_MAX			39
#define STATE_TX_UNDERFLOW			56

struct cc2420_local {
	struct spi_device *spi;		/* SPI device structure */
//	struct cc2420_platform_data *pdata;
	struct ieee802154_hw *hw;	/* IEEE-802.15.4 device */
	u8 *buf;			/* SPI TX/Rx data buffer */
	struct mutex buffer_mutex;	/* SPI buffer mutex */
	bool is_tx;			/* Flag for sync b/w Tx and Rx */
	int fifo_pin;			/* FIFO GPIO pin number */
	struct work_struct fifop_irqwork;/* Workqueue for FIFOP */
	spinlock_t lock;		/* Lock for is_tx*/
	struct completion tx_complete;	/* Work completion for Tx */
	bool promiscuous;               /* Flag for promiscuous mode */
};

static int cc2420_get_status(struct cc2420_local *lp, u8 *status)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 0,
		.tx_buf = lp->buf,
		.rx_buf = lp->buf,
	};
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&lp->buffer_mutex);
	lp->buf[xfer.len++] = CC2420_WRITEREG(CC2420_SNOP);
//	dev_dbg(&lp->spi->dev, "get status command buf[0] = %02x\n", lp->buf[0]);

	ret = spi_sync(lp->spi, &msg);
	if (!ret)
		*status = lp->buf[0];
//	dev_dbg(&lp->spi->dev, "status = %02x\n", lp->buf[0]);
	mutex_unlock(&lp->buffer_mutex);

	return ret;

}
static int cc2420_cmd_strobe(struct cc2420_local *lp,
				 u8 cmd)
{
	int ret;
	u8 status = 0xf;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 0,
		.tx_buf = lp->buf,
		.rx_buf = lp->buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&lp->buffer_mutex);
	lp->buf[xfer.len++] = CC2420_WRITEREG(cmd);
	dev_dbg(&lp->spi->dev, "cmd strobe buf[0] = %02x\n", lp->buf[0]);

	ret = spi_sync(lp->spi, &msg);
	if (!ret)
		status = lp->buf[0];
	dev_dbg(&lp->spi->dev, "status = %02x\n", lp->buf[0]);
	mutex_unlock(&lp->buffer_mutex);

	return ret;
}

static int cc2420_read_16_bit_reg(struct cc2420_local *lp,
				  u8 addr, u16 *data)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = lp->buf,
		.rx_buf = lp->buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&lp->buffer_mutex);
	lp->buf[0] = CC2420_READREG(addr);
	dev_dbg(&lp->spi->dev, "readreg addr buf[0] = %02x\n", lp->buf[0]);

	ret = spi_sync(lp->spi, &msg);
	dev_dbg(&lp->spi->dev, "status = %d\n", ret);
	mutex_unlock(&lp->buffer_mutex);
	dev_dbg(&lp->spi->dev, "status = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "reg[1] = %02x\n", lp->buf[1]);
	dev_dbg(&lp->spi->dev, "reg[2] = %02x\n", lp->buf[2]);
	if (!ret)
		*data = ((u16) (lp->buf[1]) << 8) | lp->buf[2];
	return ret;
}

static int cc2420_write_16_bit_reg_partial(struct cc2420_local *lp,
					   u8 addr, u16 data, u16 mask)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = lp->buf,
		.rx_buf = lp->buf,
	};
	dev_dbg(&lp->spi->dev, "data = %x\n", data);
	dev_dbg(&lp->spi->dev, "mask = %x\n", mask);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	mutex_lock(&lp->buffer_mutex);
	lp->buf[0] = CC2420_READREG(addr);
	dev_dbg(&lp->spi->dev, "read addr buf[0] = %02x\n", lp->buf[0]);
	ret = spi_sync(lp->spi, &msg);
	if (ret)
		goto err_ret;
	dev_dbg(&lp->spi->dev, "read buf[0] = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "buf[1] = %02x\n", lp->buf[1]);
	dev_dbg(&lp->spi->dev, "buf[2] = %02x\n", lp->buf[2]);

	lp->buf[0] = CC2420_WRITEREG(addr);

	lp->buf[1] &= ~(mask >> 8);
	lp->buf[2] &= ~(mask & 0xFF);
	lp->buf[1] |= (mask >> 8) & (data >> 8);
	lp->buf[2] |= (mask & 0xFF) & (data & 0xFF);
	dev_dbg(&lp->spi->dev, "writereg addr buf[0] = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "buf[1] = %02x\n", lp->buf[1]);
	dev_dbg(&lp->spi->dev, "buf[2] = %02x\n", lp->buf[2]);
	ret = spi_sync(lp->spi, &msg);
	if (ret)
		goto err_ret;
	dev_dbg(&lp->spi->dev, "return status buf[0] = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "buf[1] = %02x\n", lp->buf[1]);
	dev_dbg(&lp->spi->dev, "buf[2] = %02x\n", lp->buf[2]);

err_ret:
	mutex_unlock(&lp->buffer_mutex);
	return ret;
}

static int cc2420_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct cc2420_local *lp = hw->priv;
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	BUG_ON(page != 0);
	BUG_ON(channel < CC2420_MIN_CHANNEL);
	BUG_ON(channel > CC2420_MAX_CHANNEL);

	ret = cc2420_write_16_bit_reg_partial(lp, CC2420_FSCTRL, 
					      357 + 5*(channel - 11), CC2420_FREQ_MASK);

	return ret;
}

static int cc2420_write_ram(struct cc2420_local *lp, u16 addr, u8 len, u8 *data)
{
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len		= 2,
		.tx_buf		= lp->buf,
		.rx_buf		= lp->buf,
	};
	struct spi_transfer xfer_buf = {
		.len		= len,
		.tx_buf		= data,
	};

	mutex_lock(&lp->buffer_mutex);
	lp->buf[0] = CC2420_RAMADDR(addr);
	lp->buf[1] = CC2420_WRITERAM(CC2420_RAMBANK(addr));
	dev_dbg(&lp->spi->dev, "write ram addr buf[0] = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "ram bank buf[1] = %02x\n", lp->buf[1]);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_dbg(&lp->spi->dev, "spi status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_dbg(&lp->spi->dev, "cc2420 status = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "buf[1] = %02x\n", lp->buf[1]);

	mutex_unlock(&lp->buffer_mutex);
	return status;
}

static int cc2420_write_txfifo(struct cc2420_local *lp, u8 *data, u8 len)
{
	int status;
	/* Length byte must include FCS even if calculated in hardware */
	int len_byte = len + 2;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len		= 1,
		.tx_buf		= lp->buf,
		.rx_buf		= lp->buf,
	};
	struct spi_transfer xfer_len = {
		.len		= 1,
		.tx_buf		= &len_byte,
	};
	struct spi_transfer xfer_buf = {
		.len		= len,
		.tx_buf		= data,
	};

	mutex_lock(&lp->buffer_mutex);
	lp->buf[0] = CC2420_WRITEREG(CC2420_TXFIFO);
	dev_dbg(&lp->spi->dev, "TX_FIFO addr buf[0] = %02x\n", lp->buf[0]);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_len, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_dbg(&lp->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_dbg(&lp->spi->dev, "status = %d\n", status);
	dev_dbg(&lp->spi->dev, "buf[0] = %02x\n", lp->buf[0]);

	mutex_unlock(&lp->buffer_mutex);
	return status;
}

static int
cc2420_read_rxfifo(struct cc2420_local *lp, u8 *data, u8 *len, u8 *lqi)
{
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len		= 2,
		.tx_buf		= lp->buf,
		.rx_buf		= lp->buf,
	};
	struct spi_transfer xfer_buf = {
		.len		= *len,
		.rx_buf		= data,
	};

	mutex_lock(&lp->buffer_mutex);
	lp->buf[0] = CC2420_READREG(CC2420_RXFIFO);
	lp->buf[1] = 0x00;
	dev_dbg(&lp->spi->dev, "read rxfifo buf[0] = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "buf[1] = %02x\n", lp->buf[1]);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(lp->spi, &msg);
	dev_dbg(&lp->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_dbg(&lp->spi->dev, "status = %d\n", status);
	dev_dbg(&lp->spi->dev, "return status buf[0] = %02x\n", lp->buf[0]);
	dev_dbg(&lp->spi->dev, "length buf[1] = %02x\n", lp->buf[1]);

	*lqi = data[lp->buf[1] - 1] & 0x7f;
	*len = lp->buf[1]; /* it should be less than 130 */

	mutex_unlock(&lp->buffer_mutex);

	return status;
}


static int cc2420_tx(struct ieee802154_hw *dev, struct sk_buff *skb)
{
	struct cc2420_local *lp = dev->priv;
	int rc;
	unsigned long flags;
	u8 status = 0;

	pr_debug("%s\n", __func__);

	might_sleep();

	rc = cc2420_cmd_strobe(lp, CC2420_SFLUSHTX);
	if (rc)
		goto err_rx;
	rc = cc2420_write_txfifo(lp, skb->data, skb->len);
	if (rc)
		goto err_rx;

	/* TODO: test CCA pin */

	rc = cc2420_get_status(lp, &status);
	if (rc)
		goto err_rx;

	if (status & CC2420_STATUS_TX_UNDERFLOW) {
		dev_err(&lp->spi->dev, "cc2420 tx underflow!\n");
		goto err_rx;
	}

	spin_lock_irqsave(&lp->lock, flags);
	BUG_ON(lp->is_tx);
	lp->is_tx = 1;
	init_completion(&lp->tx_complete);
	spin_unlock_irqrestore(&lp->lock, flags);

	rc = cc2420_cmd_strobe(lp, CC2420_STXONCCA);
	if (rc)
		goto err;

	rc = wait_for_completion_interruptible(&lp->tx_complete);
	if (rc < 0)
		goto err;

	cc2420_cmd_strobe(lp, CC2420_SFLUSHTX);
	cc2420_cmd_strobe(lp, CC2420_SRXON);

	return rc;

err:
	spin_lock_irqsave(&lp->lock, flags);
	lp->is_tx = 0;
	spin_unlock_irqrestore(&lp->lock, flags);
err_rx:
	cc2420_cmd_strobe(lp, CC2420_SFLUSHTX);
	cc2420_cmd_strobe(lp, CC2420_SRXON);
	return rc;
}

static int cc2420_rx(struct cc2420_local *lp)
{
	u8 len = 128;
	u8 lqi = 0; /* link quality */
	u8 fcs = 0;
	int rc;
	struct sk_buff *skb;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	rc = cc2420_read_rxfifo(lp, skb_put(skb, len), &len, &lqi);
	if (len < 2) {
		kfree_skb(skb);
		return -EINVAL;
	}

	/* Check FCS flag */
	fcs = skb->data[len-1];
	if (!(fcs >> 7)) {
		dev_dbg(&lp->spi->dev, "Received packet with wrong FCS; ingnore.\n");
		kfree_skb(skb);
		return -EINVAL;
	}

	/* Clip last two bytes. When using hardware FCS they get replaced with
	 * correlation value, FCS flag and RSSI value */
	skb_trim(skb, len-2);

	ieee802154_rx_irqsafe(lp->hw, skb, lqi);

	dev_dbg(&lp->spi->dev, "RXFIFO: %d %d %x\n", rc, len, lqi);

	return 0;
}

static int
cc2420_set_hw_addr_filt(struct ieee802154_hw *dev,
			struct ieee802154_hw_addr_filt *filt, 
			unsigned long changed)
{
	struct cc2420_local *lp = dev->priv;
	u16 reg;
	int ret = 0;

	might_sleep();

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		dev_dbg(&lp->spi->dev,
			 "%s called for IEEE addr\n", __func__);
		ret = cc2420_write_ram(lp, CC2420_RAM_IEEEADR,
				       sizeof(filt->ieee_addr),
				       (u8 *)&filt->ieee_addr);
	}

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		u16 short_addr = le16_to_cpu(filt->short_addr);

		dev_dbg(&lp->spi->dev, "%s called for saddr\n", __func__);
		ret = cc2420_write_ram(lp, CC2420_RAM_SHORTADR, 
				       sizeof(short_addr),
				       (u8 *)&short_addr);
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		u16 panid = le16_to_cpu(filt->pan_id);

		dev_dbg(&lp->spi->dev, "%s called for pan id\n", __func__);
		ret = cc2420_write_ram(lp, CC2420_RAM_PANID, sizeof(panid), 
				 (u8 *)&panid);
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		dev_dbg(&lp->spi->dev,
			 "%s called for panc change\n", __func__);

		cc2420_read_16_bit_reg(lp, CC2420_MDMCTRL0, &reg);
		if (filt->pan_coord)
			reg |= 1 << CC2420_MDMCTRL0_PANCRD;
		else
			reg &= ~(1 << CC2420_MDMCTRL0_PANCRD);
		ret = cc2420_write_16_bit_reg_partial(lp, CC2420_MDMCTRL0,
						reg, 1 << CC2420_MDMCTRL0_PANCRD);
	}

	return 0;
}

static int cc2420_ed(struct ieee802154_hw *dev, u8 *level)
{
	struct cc2420_local *lp = dev->priv;
	u16 rssi;
	int ret;
	dev_dbg(&lp->spi->dev, "ed called\n");

	ret = cc2420_read_16_bit_reg(lp, CC2420_RSSI, &rssi);
	if (ret)
		return ret;

	/* P = RSSI_VAL + RSSI_OFFSET[dBm] */
	*level = (rssi & CC2420_RSSI_MASK) + RSSI_OFFSET;
	return ret;
}

static int cc2420_start(struct ieee802154_hw *dev)
{
	struct cc2420_local *lp = dev->priv;

	dev_dbg(printdev(lp), "%s\n", __func__);
	return cc2420_cmd_strobe(dev->priv, CC2420_SRXON);
}

static void cc2420_stop(struct ieee802154_hw *dev)
{
	cc2420_cmd_strobe(dev->priv, CC2420_SRFOFF);
}

static struct ieee802154_ops cc2420_ops = {
	.owner = THIS_MODULE,
	.xmit_sync = cc2420_tx,
//	.xmit_async = cc2420_tx, // TODO
	.ed = cc2420_ed,
	.start = cc2420_start,
	.stop = cc2420_stop,
	.set_channel = cc2420_set_channel,
	.set_hw_addr_filt = cc2420_set_hw_addr_filt,
//	.set_txpower	= cc2420_set_txpower, // TODO
//	.set_promiscuous_mode	= cc2420_set_promiscuous_mode, // TODO
};

static int cc2420_register(struct cc2420_local *lp)
{
	int ret = -ENOMEM;

	lp->hw = ieee802154_alloc_hw(sizeof(*lp), &cc2420_ops);
	if (!lp->hw)
		goto err_ret;

	lp->hw->priv = lp;
	lp->hw->parent = &lp->spi->dev;
	lp->hw->extra_tx_headroom = 0;
	ieee802154_random_extended_addr(&lp->hw->phy->perm_extended_addr);

	/* We do support only 2.4 Ghz */
	lp->hw->phy->supported.channels[0] = 0x7FFF800;
	lp->hw->flags = IEEE802154_HW_OMIT_CKSUM | IEEE802154_HW_AFILT;
//	| IEEE802154_HW_PROMISCUOUS

	lp->hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	lp->hw->phy->current_page = 0;
	lp->hw->phy->current_channel = 11;

	dev_dbg(&lp->spi->dev, "registered cc2420\n");
	ret = ieee802154_register_hw(lp->hw);
	if (ret)
		goto err_free_hw;

	return 0;
err_free_hw:
	ieee802154_free_hw(lp->hw);
err_ret:
	return ret;
}

static irqreturn_t cc2420_fifop_isr(int irq, void *data)
{
	struct cc2420_local *lp = data;

	schedule_work(&lp->fifop_irqwork);

	return IRQ_HANDLED;
}

static void cc2420_fifop_irqwork(struct work_struct *work)
{
	struct cc2420_local *lp
		= container_of(work, struct cc2420_local, fifop_irqwork);

	dev_dbg(&lp->spi->dev, "fifop interrupt received\n");

	if (gpio_get_value(lp->fifo_pin))
		cc2420_rx(lp);
	else
		dev_err(&lp->spi->dev, "rxfifo overflow\n");

	cc2420_cmd_strobe(lp, CC2420_SFLUSHRX);
	cc2420_cmd_strobe(lp, CC2420_SFLUSHRX);
}

static irqreturn_t cc2420_sfd_isr(int irq, void *data)
{
	struct cc2420_local *lp = data;
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);
	if (lp->is_tx) {
		lp->is_tx = 0;
		spin_unlock_irqrestore(&lp->lock, flags);
		dev_dbg(&lp->spi->dev, "SFD for TX\n");
		complete(&lp->tx_complete);
	} else {
		spin_unlock_irqrestore(&lp->lock, flags);
		dev_dbg(&lp->spi->dev, "SFD for RX\n");
	}

	return IRQ_HANDLED;
}

static int cc2420_get_platform_data(struct spi_device *spi,
				    struct cc2420_platform_data *pdata)
{
	struct device_node *np = spi->dev.of_node;
	struct cc2420_local *lp = spi_get_drvdata(spi);

	dev_dbg(&spi->dev, "%s\n", __func__);

	if (!np) {
		struct cc2420_platform_data *spi_pdata = spi->dev.platform_data;

		if (!spi_pdata)
			return -ENOENT;
		*pdata = *spi_pdata;
		lp->fifo_pin = pdata->fifo;
		return 0;
	}

	pdata->fifo = of_get_named_gpio(np, "fifo-gpio", 0);
	lp->fifo_pin = pdata->fifo;
	dev_dbg(&spi->dev, "get fifo-gpio: %d\n", pdata->fifo);

	pdata->fifop = of_get_named_gpio(np, "fifop-gpio", 0);
	dev_dbg(&spi->dev, "get fifop-gpio: %d\n", pdata->fifop);

	pdata->sfd = of_get_named_gpio(np, "sfd-gpio", 0);
	dev_dbg(&spi->dev, "get sfd-gpio: %d\n", pdata->sfd);

	pdata->cca = of_get_named_gpio(np, "cca-gpio", 0);
	dev_dbg(&spi->dev, "get cca-gpio: %d\n", pdata->cca);

	pdata->vreg = of_get_named_gpio(np, "vreg-gpio", 0);
	dev_dbg(&spi->dev, "get vreg-gpio: %d\n", pdata->vreg);

	pdata->reset = of_get_named_gpio(np, "reset-gpio", 0);
	dev_dbg(&spi->dev, "get reset-gpio: %d\n", pdata->reset);

	return 0;
}

static int cc2420_hw_init(struct cc2420_local *lp)
{
	int ret;
	u16 state;
	u8 status = 0xff;
	int timeout = 500; /* 500us delay */
	ret = cc2420_read_16_bit_reg(lp, CC2420_FSMSTATE, &state);
	if (ret)
		goto error_ret;
	/* reset has occured prior to this, so there should be no other option */
	if (state != STATE_PDOWN) {
		ret = -EINVAL;
		goto error_ret;
	}
	ret = cc2420_cmd_strobe(lp, CC2420_SXOSCON);
	if (ret)
		goto error_ret;

	do {
		ret = cc2420_get_status(lp, &status);
		if (ret)
			goto error_ret;
		if (timeout-- <= 0) {
			dev_err(&lp->spi->dev, "oscillator start failed!\n");
			return ret;
		}
		udelay(1);
	} while (!(status & CC2420_STATUS_XOSC16M_STABLE));

	dev_info(&lp->spi->dev, "oscillator succesfully brought up\n");

	return 0;
error_ret:
	return ret;
}

static int cc2420_probe(struct spi_device *spi)
{
	struct cc2420_local *lp;
	struct cc2420_platform_data pdata;
	int ret;

	dev_info(&spi->dev, "%s\n", __func__);

	lp = devm_kzalloc(&spi->dev, sizeof(*lp), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	spi_set_drvdata(spi, lp);

	ret = cc2420_get_platform_data(spi, &pdata);
	if (ret < 0) {
		dev_err(&spi->dev, "no platform data\n");
		return -EINVAL;
	}

	lp->spi = spi;

	lp->buf = devm_kzalloc(&spi->dev,
				 SPI_COMMAND_BUFFER, GFP_KERNEL);
	if (!lp->buf)
		return -ENOMEM;

	mutex_init(&lp->buffer_mutex);
	INIT_WORK(&lp->fifop_irqwork, cc2420_fifop_irqwork);
	spin_lock_init(&lp->lock);
	init_completion(&lp->tx_complete);

	/* Request all the gpio's */
	if (!gpio_is_valid(pdata.fifo)) {
		dev_err(&spi->dev, "fifo gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	ret = devm_gpio_request_one(&spi->dev, pdata.fifo,
				    GPIOF_IN, "fifo");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.cca)) {
		dev_err(&spi->dev, "cca gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	ret = devm_gpio_request_one(&spi->dev, pdata.cca,
				    GPIOF_IN, "cca");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.fifop)) {
		dev_err(&spi->dev, "fifop gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	ret = devm_gpio_request_one(&spi->dev, pdata.fifop,
				    GPIOF_IN, "fifop");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.sfd)) {
		dev_err(&spi->dev, "sfd gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	ret = devm_gpio_request_one(&spi->dev, pdata.sfd,
				    GPIOF_IN, "sfd");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.reset)) {
		dev_err(&spi->dev, "reset gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	ret = devm_gpio_request_one(&spi->dev, pdata.reset,
				    GPIOF_OUT_INIT_LOW, "reset");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.vreg)) {
		dev_err(&spi->dev, "vreg gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	ret = devm_gpio_request_one(&spi->dev, pdata.vreg,
				    GPIOF_OUT_INIT_LOW, "vreg");
	if (ret)
		goto err_hw_init;

	gpio_set_value(pdata.vreg, HIGH);
	usleep_range(100, 150);

	gpio_set_value(pdata.reset, HIGH);
	usleep_range(200, 250);

	ret = cc2420_hw_init(lp);
	if (ret)
		goto err_hw_init;

	/* Set up fifop interrupt */
	ret = devm_request_irq(&spi->dev,
			       gpio_to_irq(pdata.fifop),
			       cc2420_fifop_isr,
			       IRQF_TRIGGER_RISING,
			       dev_name(&spi->dev),
			       lp);
	if (ret) {
		dev_err(&spi->dev, "could not get fifop irq\n");
		goto err_hw_init;
	}

	/* Set up sfd interrupt */
	ret = devm_request_irq(&spi->dev,
			       gpio_to_irq(pdata.sfd),
			       cc2420_sfd_isr,
			       IRQF_TRIGGER_FALLING,
			       dev_name(&spi->dev),
			       lp);
	if (ret) {
		dev_err(&spi->dev, "could not get sfd irq\n");
		goto err_hw_init;
	}

	ret = cc2420_register(lp);
	if (ret)
		goto err_hw_init;

	return 0;

err_hw_init:
	mutex_destroy(&lp->buffer_mutex);
	flush_work(&lp->fifop_irqwork);
	return ret;
}

static int cc2420_remove(struct spi_device *spi)
{
	struct cc2420_local *lp = spi_get_drvdata(spi);

	dev_info(printdev(lp), "%s\n", __func__);

	mutex_destroy(&lp->buffer_mutex);
	flush_work(&lp->fifop_irqwork);

	ieee802154_unregister_hw(lp->hw);
	ieee802154_free_hw(lp->hw);

	return 0;
}

static const struct spi_device_id cc2420_ids[] = {
	{"cc2420", },
	{},
};
MODULE_DEVICE_TABLE(spi, cc2420_ids);

static const struct of_device_id cc2420_of_ids[] = {
	{.compatible = "ti,cc2420", },
	{},
};
MODULE_DEVICE_TABLE(of, cc2420_of_ids);

/* SPI driver structure */
static struct spi_driver cc2420_driver = {
	.driver = {
		.name = "cc2420",
		.of_match_table = of_match_ptr(cc2420_of_ids),
	},
	.id_table = cc2420_ids,
	.probe = cc2420_probe,
	.remove = cc2420_remove,
};
module_spi_driver(cc2420_driver);

MODULE_AUTHOR("Xue Liu <liuxuenetmail@gmail.com>");
MODULE_DESCRIPTION("CC2420 Transceiver Driver");
MODULE_LICENSE("GPL v2");
