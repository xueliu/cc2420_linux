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
#include <linux/irq.h>
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

#define CC2420_WRITEREG(x)	(x)
#define CC2420_READREG(x)	(0x40 | x)
#define CC2420_RAMADDR(x)	((x & 0x7F) | 0x80)
#define CC2420_RAMBANK(x)	((x >> 1) & 0xc0)
#define CC2420_WRITERAM(x)	(x)
#define CC2420_READRAM(x)	(0x20 | x)

#define REG_READ_MSK	0x40
#define REG_WRITE_MSK	0x0

//#define CC2420_FREQ_MASK 	0x3FF
#define CC2420_ADR_DECODE_MASK	0x0B00
#define CC2420_FIFOP_THR_MASK	0x003F
#define CC2420_CRC_MASK		0x80
#define CC2420_RSSI_MASK	0x7F
#define CC2420_FSMSTATE_MASK	0x2F

#define CC2420_MANFID 	0x033D
#define CC2420_VERSION 	0x3

#define	CC2420_FIFO_SIZE	128

#define SPI_COMMAND_BUFFER	3
#define	HIGH			1
#define	LOW			0

#define RSSI_OFFSET 45

#define STATE_PDOWN		0
#define STATE_IDLE		1
#define STATE_RX_CALIBRATE	2
#define STATE_RX_CALIBRATE2	40
#define STATE_RX_SFD_SEARCH_MIN 3
#define STATE_RX_SFD_SEARCH_MAX 6
#define STATE_RX_FRAME		16
#define STATE_RX_FRAME2		40
#define STATE_RX_WAIT		14
#define STATE_RX_OVERFLOW	17
#define STATE_TX_ACK_CALIBRATE	48
#define STATE_TX_ACK_PREAMBLE_MIN	49
#define STATE_TX_ACK_PREAMBLE_MAX	51
#define STATE_TX_ACK_MIN	52
#define STATE_TX_ACK_MAX	54
#define STATE_TX_CALIBRATE	32
#define STATE_TX_PREAMBLE_MIN	34
#define STATE_TX_PREAMBLE_MAX	36
#define STATE_TX_FRAME_MIN	37
#define STATE_TX_FRAME_MAX	39
#define STATE_TX_UNDERFLOW	56

#define STATUS_RSSI_VALID	0x01
#define STATUS_LOCK		0x04
#define STATUS_TX_ACTIVE	0x08
#define STATUS_ENC_BUSY		0x10
#define STATUS_TX_UNDERFLOW	0x20
#define STATUS_XOSC16M_STABLE	0x80

struct cc2420_local {
	struct spi_device *spi;		/* SPI device structure */
//	struct cc2420_platform_data *pdata;
	struct regmap *regmap;

	struct ieee802154_hw *hw;	/* IEEE-802.15.4 device */
	u8 *buf;			/* SPI TX/Rx data buffer */
	struct mutex buffer_mutex;	/* SPI buffer mutex */
	bool is_tx;			/* Flag for sync b/w Tx and Rx */
	int fifo_pin;			/* FIFO GPIO pin number */
	spinlock_t lock;		/* Lock for is_tx*/
	bool promiscuous;               /* Flag for promiscuous mode */

	int fifop_irq;
	int sfd_irq;

	/* receive handling */
	struct spi_message rxfifo_msg;
	u8 rxfifo_addr[1];
	struct spi_transfer rxfifo_xfer_addr;
//	u8 rxfifo_len[1];
//	struct spi_transfer rxfifo_xfer_len;
	u8 rxfifo_buf[CC2420_FIFO_SIZE];
	struct spi_transfer rxfifo_xfer_buf;

	/* transmit handling */
	struct spi_message txfifo_msg;
	u8 txfifo_addr[1];
	struct spi_transfer txfifo_xfer_addr;
	u8 txfifo_len[1];
	struct spi_transfer txfifo_xfer_len;
	struct spi_transfer txfifo_xfer_buf;
	struct sk_buff *tx_skb;

	/* register handling */
	struct spi_message reg_msg;
	u8 reg_addr[1];
	struct spi_transfer reg_xfer_addr;
	u8 reg_val[1];
	struct spi_transfer reg_xfer_val;

	/* command strobe handling */
	struct spi_message cmd_msg;
	u8 cmd_val[1];
	struct spi_transfer cmd_xfer_val;
};

struct cc2420_state_change {
	struct cc2420_local *lp;
	int irq;

	struct hrtimer timer;
	struct spi_message msg;
	struct spi_transfer trx;
	u8 buf[CC2420_FIFO_SIZE];

	void (*complete)(void *context);
	u8 from_state;
	u8 to_state;

	bool free;
};

struct cc2420_trac {
	u64 success;
	u64 success_data_pending;
	u64 success_wait_for_ack;
	u64 channel_access_failure;
	u64 no_ack;
	u64 invalid;
};


static bool
cc2420_reg_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CC2420_SNOP:
	case CC2420_SXOSCON:
	case CC2420_STXCAL:
	case CC2420_SRXON:
	case CC2420_STXON:
	case CC2420_STXONCCA:
	case CC2420_SRFOFF:
	case CC2420_SXOSCOFF:
	case CC2420_SFLUSHRX:
	case CC2420_SFLUSHTX:
	case CC2420_SACK:
	case CC2420_SACKPEND:
	case CC2420_SRXDEC:
	case CC2420_STXENC:
	case CC2420_SAES:
	case CC2420_MAIN:
	case RG_MDMCTRL0:
	case RG_MDMCTRL1:
	case CC2420_RSSI:
	case CC2420_SYNCWORD:
	case RG_TXCTRL:
	case CC2420_RXCTRL0:
	case CC2420_RXCTRL1:
	case RG_FSCTRL:
	case CC2420_SECCTRL0:
	case CC2420_SECCTRL1:
	case CC2420_BATTMON:
	case RG_IOCFG0:
	case CC2420_IOCFG1:
	case RG_MANFIDL:
	case RG_MANFIDH:
	case CC2420_FSMTC:
	case CC2420_MANAND:
	case CC2420_MANOR:
	case CC2420_AGCCTRL:
	case CC2420_AGCTST0:
	case CC2420_AGCTST1:
	case CC2420_AGCTST2:
	case CC2420_FSTST0:
	case CC2420_FSTST1:
	case CC2420_FSTST2:
	case CC2420_FSTST3:
	case CC2420_RXBPFTST:
	case CC2420_ADCTST:
	case CC2420_DACTST:
	case CC2420_TOPTST:
	case CC2420_RESERVED:
	case CC2420_TXFIFO: /* write only */
		return true;
	default:
		return false;
	}
}

static bool
cc2420_reg_readable(struct device *dev, unsigned int reg)
{
	bool rc;

	/* all writeable are also readable */
	rc = cc2420_reg_writeable(dev, reg);
	if (rc)
		return rc;

	/* readonly regs */
	switch (reg) {
	case RG_FSMSTATE:
	case CC2420_RXFIFO:
		return true;
	default:
		return false;
	}
}

static bool
cc2420_reg_volatile(struct device *dev, unsigned int reg)
{
	/* can be changed during runtime */
	switch (reg) {
	case RG_FSMSTATE:
	/* use them in spi_async and regmap so it's volatile */
		return true;
	default:
		return false;
	}
}

static bool
cc2420_reg_precious(struct device *dev, unsigned int reg)
{
	/* don't clear irq line on read */
	switch (reg) {
	case RG_FSMSTATE:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config cc2420_reg_regmap = {
	.name			= "cc2420_reg",
	.reg_bits		= 8,
	.val_bits		= 16,
	.write_flag_mask	= REG_WRITE_MSK,
	.read_flag_mask		= REG_READ_MSK,
	.cache_type		= REGCACHE_RBTREE,
	.writeable_reg		= cc2420_reg_writeable,
	.readable_reg		= cc2420_reg_readable,
	.volatile_reg		= cc2420_reg_volatile,
	.precious_reg		= cc2420_reg_precious,
	.fast_io		= true,
	.can_multi_write	= true,
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
//	dev_dbg(&lp->spi->dev, "cmd strobe buf[0] = %02x\n", lp->buf[0]);

	ret = spi_sync(lp->spi, &msg);
	if (!ret)
		status = lp->buf[0];
//	dev_dbg(&lp->spi->dev, "status = %02x\n", lp->buf[0]);
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

static inline int
cc2420_read_subreg(struct cc2420_local *lp,
		      unsigned int addr, unsigned int mask,
		      unsigned int shift, unsigned int *data)
{
	int ret;

	ret = regmap_read(lp->regmap, addr, data);
	if (!ret)
		*data = (*data & mask) >> shift;

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

static inline int
cc2420_write_subreg(struct cc2420_local *lp,
		       unsigned int addr, unsigned int mask,
		       unsigned int shift, unsigned int data)
{
	int ret;

	ret = regmap_update_bits(lp->regmap, addr, mask, data << shift);

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
	dev_dbg(&lp->spi->dev, "write ram addr = %02x\n", lp->buf[0]);
	dev_vdbg(&lp->spi->dev, "ram bank = %02x\n", lp->buf[1]);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(lp->spi, &msg);
	if (msg.status)
		status = msg.status;

	mutex_unlock(&lp->buffer_mutex);
	return status;
}

//static void
//cc2420_async_state_change(struct cc2420_local *lp,
//			     struct cc2420_state_change *ctx,
//			     const u8 state, void (*complete)(void *context));
//
//static void
//cc2420_async_error_recover_complete(void *context)
//{
//	struct cc2420_state_change *ctx = context;
//	struct cc2420_local *lp = ctx->lp;
//
//	if (ctx->free)
//		kfree(ctx);
//
//	ieee802154_wake_queue(lp->hw);
//}
//
//static void
//cc2420_async_error_recover(void *context)
//{
//	struct cc2420_state_change *ctx = context;
//	struct cc2420_local *lp = ctx->lp;
//
//	lp->is_tx = 0;
//	cc2420_async_state_change(lp, ctx, STATE_IDLE,
//				     cc2420_async_error_recover_complete);
//}
//
//static inline void
//cc2420_async_error(struct cc2420_local *lp,
//		      struct cc2420_state_change *ctx, int rc)
//{
//	dev_err(&lp->spi->dev, "spi_async error %d\n", rc);
//
//	cc2420_async_state_change(lp, ctx, STATE_IDLE,
//				     cc2420_async_error_recover);
//}
//
//
///* Generic function to get some register value in async mode */
//static void
//cc2420_async_read_reg(struct cc2420_local *lp, u8 reg,
//			 struct cc2420_state_change *ctx,
//			 void (*complete)(void *context))
//{
//	int rc;
//
//	u8 *tx_buf = ctx->buf;
//
//	tx_buf[0] = CC2420_READREG(reg);
//	ctx->msg.complete = complete;
//	rc = spi_async(lp->spi, &ctx->msg);
//	if (rc)
//		cc2420_async_error(lp, ctx, rc);
//}
//
//static void
//cc2420_async_write_reg(struct cc2420_local *lp, u8 reg, u8 val,
//			  struct cc2420_state_change *ctx,
//			  void (*complete)(void *context))
//{
//	int rc;
//
//	ctx->buf[0] = CC2420_WRITEREG(reg);
//	ctx->buf[1] = val;
//	ctx->msg.complete = complete;
//	rc = spi_async(lp->spi, &ctx->msg);
//	if (rc)
//		cc2420_async_error(lp, ctx, rc);
//}
//
//static void
//cc2420_async_state_change(struct cc2420_local *lp,
//			     struct cc2420_state_change *ctx,
//			     const u8 state, void (*complete)(void *context))
//{
//	/* Initialization for the state change context */
//	ctx->to_state = state;
//	ctx->complete = complete;
//	cc2420_async_read_reg(lp, RG_FSMSTATE, ctx,
//				 cc2420_state_change_start);
//}


//static int cc2420_write_txfifo(struct cc2420_local *lp, u8 *data, u8 len)
//{
//	int status;
//	/* Length byte must include FCS even if calculated in hardware */
//	int len_byte = len + 2;
//	struct spi_message msg;
//	struct spi_transfer xfer_head = {
//		.len		= 1,
//		.tx_buf		= lp->buf,
//		.rx_buf		= lp->buf,
//	};
//	struct spi_transfer xfer_len = {
//		.len		= 1,
//		.tx_buf		= &len_byte,
//	};
//	struct spi_transfer xfer_buf = {
//		.len		= len,
//		.tx_buf		= data,
//	};
//
//	mutex_lock(&lp->buffer_mutex);
//	lp->buf[0] = CC2420_WRITEREG(CC2420_TXFIFO);
//	dev_dbg(&lp->spi->dev, "write txfifo addr = %02x\n", lp->buf[0]);
//
//	spi_message_init(&msg);
//	spi_message_add_tail(&xfer_head, &msg);
//	spi_message_add_tail(&xfer_len, &msg);
//	spi_message_add_tail(&xfer_buf, &msg);
//
//	status = spi_sync(lp->spi, &msg);
//
//	if (msg.status)
//		status = msg.status;
//
//	mutex_unlock(&lp->buffer_mutex);
//	return status;
//}

//static int cc2420_tx(struct ieee802154_hw *dev, struct sk_buff *skb)
//{
//	struct cc2420_local *lp = dev->priv;
//	int rc;
//	unsigned long flags;
////	u8 status = 0;
//
//	pr_debug("%s\n", __func__);
//
//	might_sleep();
//
////	rc = cc2420_cmd_strobe(lp, CC2420_SFLUSHTX);
////	if (rc)
////		goto err_rx;
//	rc = cc2420_write_txfifo(lp, skb->data, skb->len);
//	if (rc)
//		goto err_rx;
//
//	/* TODO: test CCA pin */
//
////	rc = cc2420_get_status(lp, &status);
////	if (rc)
////		goto err_rx;
////
////	if (status & CC2420_STATUS_TX_UNDERFLOW) {
////		dev_err(&lp->spi->dev, "cc2420 tx underflow!\n");
////		goto err_rx;
////	}
//
//	spin_lock_irqsave(&lp->lock, flags);
//	BUG_ON(lp->is_tx);
//	lp->is_tx = 1;
//	init_completion(&lp->tx_complete);
//	spin_unlock_irqrestore(&lp->lock, flags);
//
//	rc = cc2420_cmd_strobe(lp, CC2420_STXONCCA);
//	if (rc)
//		goto err;
//
//	rc = wait_for_completion_interruptible(&lp->tx_complete);
//	if (rc < 0)
//		goto err;
//
//	cc2420_cmd_strobe(lp, CC2420_SFLUSHTX);
//	cc2420_cmd_strobe(lp, CC2420_SRXON);
//
//	return rc;
//
//err:
//	spin_lock_irqsave(&lp->lock, flags);
//	lp->is_tx = 0;
//	spin_unlock_irqrestore(&lp->lock, flags);
//err_rx:
//	cc2420_cmd_strobe(lp, CC2420_SFLUSHTX);
//	cc2420_cmd_strobe(lp, CC2420_SRXON);
//	return rc;
//}

//static void
//cc2420_handle_flush_txfifo_complete(void *context)
//{
//	struct cc2420_local *lp = context;
//	int ret;
//
////	struct spi_message msg;
////	struct spi_transfer xfer = {
////		.len = 0,
////		.tx_buf = lp->buf,
////		.rx_buf = lp->buf,
////	};
//
//	enable_irq(lp->sfd_irq);
//
//	dev_dbg(printdev(lp), "return status:0x%x\n", lp->cmd_val[0]);
//
//	dev_dbg(printdev(lp), "%s\n", __func__);
//
////	spi_message_init(&msg);
////	spi_message_add_tail(&xfer, &msg);
//
////	mutex_lock(&lp->buffer_mutex);
////	lp->buf[xfer.len++] = CC2420_SRXON;
////
//	lp->cmd_val[0] = CC2420_WRITEREG(CC2420_SRXON);
//	lp->cmd_msg.complete = NULL;
//	ret = spi_async(lp->spi, &lp->cmd_msg);
//	if (ret)
//		dev_err(printdev(lp), "failed to send receive command\n");
//
////	ret = spi_async(lp->spi, &msg);
//
////	lp->is_tx = 0;
////	mutex_unlock(&lp->buffer_mutex);
//}

//static void
//cc2420_handle_tx_complete(void *context)
//{
//	struct cc2420_local *lp = context;
//	int ret;
//
////	struct spi_message msg;
////	struct spi_transfer xfer = {
////		.len = 0,
////		.tx_buf = lp->buf,
////		.rx_buf = lp->buf,
////	};
////
//	dev_dbg(printdev(lp), "return status:0x%x\n", lp->cmd_val[0]);
//
//	dev_dbg(printdev(lp), "%s\n", __func__);
////
////	spi_message_init(&msg);
////	spi_message_add_tail(&xfer, &msg);
//
////	mutex_lock(&lp->buffer_mutex);
////	lp->buf[xfer.len++] = CC2420_SFLUSHTX;
////	msg.complete = cc2420_handle_flush_txfifo_complete;
//
//	lp->cmd_val[0] = CC2420_WRITEREG(CC2420_SFLUSHTX);
//	lp->cmd_msg.complete = cc2420_handle_flush_txfifo_complete;
//	ret = spi_async(lp->spi, &lp->cmd_msg);
//
////	ret = spi_async(lp->spi, &msg);
////	mutex_unlock(&lp->buffer_mutex);
//	if (ret)
//		dev_err(printdev(lp), "failed to send flush txfifo command\n");
//}

static void
cc2420_handle_stxon_complete(void *context )
{
	struct cc2420_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);

	lp->is_tx = 1;
	enable_irq(lp->sfd_irq);
//	ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);
}

static void
cc2420_handle_write_txfifo_complete(void *context)
{
	struct cc2420_local *lp = context;
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

//	lp->is_tx = 1;
//	enable_irq(lp->sfd_irq);

	lp->cmd_val[0] = CC2420_WRITEREG(CC2420_STXON);
	lp->cmd_msg.complete = cc2420_handle_stxon_complete;
//	lp->cmd_msg.complete = NULL;
	ret = spi_async(lp->spi, &lp->cmd_msg);

	if (ret)
		dev_err(printdev(lp), "failed to send flush command\n");
}

static int
cc2420_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct cc2420_local *lp = hw->priv;
	unsigned int len = skb->len;

	dev_dbg(printdev(lp), "%s\n", __func__);

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "cc2420 txfifo: ", DUMP_PREFIX_OFFSET, 16, 1,
		       skb->data, skb->len, 0);
#endif

	lp->tx_skb = skb;

	lp->txfifo_addr[0] = CC2420_WRITEREG(CC2420_TXFIFO);
	/* Length byte must include FCS even if calculated in hardware */
	lp->txfifo_len[0] = len + 2;

	lp->txfifo_xfer_buf.tx_buf = skb->data;
	lp->txfifo_xfer_buf.len = len;

	lp->txfifo_msg.complete = cc2420_handle_write_txfifo_complete;

	return spi_async(lp->spi, &lp->txfifo_msg);
}

static int cc2420_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct cc2420_local *lp = hw->priv;
	unsigned int reg_value;

	BUG_ON(page != 0);
	BUG_ON(channel < CC2420_MIN_CHANNEL);
	BUG_ON(channel > CC2420_MAX_CHANNEL);

	reg_value= 357 + CC2520_CHANNEL_SPACING * (channel - 11);

	dev_dbg(printdev(lp), "%s\n", __func__);

	return cc2420_write_subreg(lp, SG_FREQ, reg_value);
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

		cc2420_read_16_bit_reg(lp, RG_MDMCTRL0, &reg);
		if (filt->pan_coord)
			reg |= 1 << CC2420_MDMCTRL0_PANCRD;
		else
			reg &= ~(1 << CC2420_MDMCTRL0_PANCRD);
		ret = cc2420_write_16_bit_reg_partial(lp, RG_MDMCTRL0,
						reg, 1 << CC2420_MDMCTRL0_PANCRD);
	}

	return 0;
}

#define CC2420_MAX_TX_POWERS 0x7
static const s32 cc2420_powers[CC2420_MAX_TX_POWERS + 1] = {
	0, -100, -300, -500, -700, -1000, -1500, -2500,
};

static int
cc2420_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct cc2420_local *lp = hw->priv;
	u32 i;

	dev_dbg(printdev(lp), "%s\n", __func__);

	for (i = 0; i < lp->hw->phy->supported.tx_powers_size; i++) {
		if (lp->hw->phy->supported.tx_powers[i] == mbm) {
			return cc2420_write_subreg(lp, SG_PA_LVL, 0x1f - 4 * i);
		}
	}

	return -EINVAL;
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

	/* enable irq */
	enable_irq(lp->fifop_irq);
//	enable_irq(lp->sfd_irq);

	return cc2420_cmd_strobe(dev->priv, CC2420_SRXON);
}

static void cc2420_stop(struct ieee802154_hw *dev)
{
	struct cc2420_local *lp = dev->priv;

	disable_irq(lp->fifop_irq);
	disable_irq(lp->sfd_irq);

	cc2420_cmd_strobe(lp, CC2420_SRFOFF);
}

static struct ieee802154_ops cc2420_ops = {
	.owner = THIS_MODULE,
	.xmit_async = cc2420_xmit,
	.ed = cc2420_ed,
	.start = cc2420_start,
	.stop = cc2420_stop,
	.set_channel = cc2420_set_channel,
	.set_hw_addr_filt = cc2420_set_hw_addr_filt,
	.set_txpower	= cc2420_set_txpower, // TODO
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

	lp->hw->phy->supported.tx_powers = cc2420_powers;
	lp->hw->phy->supported.tx_powers_size = ARRAY_SIZE(cc2420_powers);
	lp->hw->phy->transmit_power = lp->hw->phy->supported.tx_powers[0];

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

static void
cc2420_handle_read_rxfifo_complete(void *context)
{
	struct cc2420_local *lp = context;
	struct sk_buff *skb;
	int ret;
	u8 len = lp->rxfifo_xfer_buf.len;
	u8 lqi = lp->rxfifo_buf[len - 1] & 0x7f;

	dev_dbg(printdev(lp), "%s\n", __func__);

	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_dbg(&lp->spi->dev, "corrupted frame received\n");
		len = IEEE802154_MTU;
	}

	len = len - 2;  /* get rid of frame check field */

	skb = dev_alloc_skb(len);
	if (!skb)
		return;

	memcpy(skb_put(skb, len), lp->rxfifo_buf, len);
	ieee802154_rx_irqsafe(lp->hw, skb, lqi);

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "cc2420 rxfifo: ", DUMP_PREFIX_OFFSET, 16, 1,
		       lp->rxfifo_buf, len, 0);
	pr_info("cc2420 rx: lqi: %02hhx\n", lqi);
#endif

	lp->reg_addr[0] = CC2420_SFLUSHRX;
	lp->reg_val[0] = CC2420_SFLUSHRX; /* send twice */
	lp->reg_msg.complete = NULL; //cc2420_handle_flush_rxfifo_complete;
	ret = spi_async(lp->spi, &lp->reg_msg);
	if (ret)
		dev_err(printdev(lp), "failed to send flush command\n");
}

static void
cc2420_handle_read_len_complete(void *context)
{
	struct cc2420_local *lp = context;
	u8 len;
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	enable_irq(lp->fifop_irq);

	/* get the length of received frame */
	len = lp->reg_val[0] & 0x7f;
//	dev_dbg(printdev(lp), "frame len: %d\n", len);

	/* prepare to read the rx buf */
	lp->rxfifo_msg.complete = cc2420_handle_read_rxfifo_complete;
	lp->rxfifo_addr[0] = CC2420_READREG(CC2420_RXFIFO);
	lp->rxfifo_xfer_buf.len = len;

	ret = spi_async(lp->spi, &lp->rxfifo_msg);
	if (ret)
		dev_err(printdev(lp), "failed to read rxfifo\n");
}

static irqreturn_t cc2420_fifop_isr(int irq, void *data)
{
	struct cc2420_local *lp = data;
	int ret;

	disable_irq_nosync(irq);

	dev_dbg(printdev(lp), "%s\n", __func__);

//	BUG_ON(lp->is_tx);

	if (gpio_get_value(lp->fifo_pin)) {
		/* read length of received packet */
		lp->reg_addr[0] = CC2420_READREG(CC2420_RXFIFO);
		lp->reg_msg.complete = cc2420_handle_read_len_complete;
		/* read rxfifo */
		ret = spi_async(lp->spi, &lp->reg_msg);
		if (ret) {
			enable_irq(irq);
			return IRQ_NONE;
		}
	} else {
		dev_err(&lp->spi->dev, "rxfifo overflow\n");
		lp->reg_addr[0] = CC2420_SFLUSHRX;
		lp->reg_val[0] = CC2420_SFLUSHRX; /* send twice */
		lp->reg_msg.complete = NULL;//cc2420_handle_flush_rxfifo_complete;
		ret = spi_async(lp->spi, &lp->reg_msg);
		if (ret)
			dev_err(printdev(lp), "failed to send flush command\n");
	}

	return IRQ_HANDLED;
}

static irqreturn_t cc2420_sfd_isr(int irq, void *data)
{
	struct cc2420_local *lp = data;
//	unsigned long flags;
	int ret;

	disable_irq_nosync(irq);
	if (lp->is_tx) {
		dev_dbg(&lp->spi->dev, "SFD for TX done\n");
		lp->is_tx = 0;

		ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);
		/* start RX */
//		lp->cmd_val[0] = CC2420_WRITEREG(CC2420_SFLUSHTX);
//		lp->cmd_msg.complete = cc2420_handle_flush_txfifo_complete;
		lp->cmd_val[0] = CC2420_WRITEREG(CC2420_SRXON);
		lp->cmd_msg.complete = NULL;
		ret = spi_async(lp->spi, &lp->cmd_msg);
		if (ret) {
			enable_irq(irq);
			return IRQ_NONE;
		}
	} else {
		enable_irq(irq);
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

static void
cc2420_setup_rxfifo_spi_messages(struct cc2420_local *lp)
{
	spi_message_init(&lp->rxfifo_msg);
	lp->rxfifo_msg.context = lp;

	lp->rxfifo_xfer_addr.len = 1;
	lp->rxfifo_xfer_addr.tx_buf = lp->rxfifo_addr;
	lp->rxfifo_xfer_addr.rx_buf = lp->rxfifo_addr;

	lp->rxfifo_xfer_buf.rx_buf = lp->rxfifo_buf;

	spi_message_add_tail(&lp->rxfifo_xfer_addr, &lp->rxfifo_msg);
	spi_message_add_tail(&lp->rxfifo_xfer_buf, &lp->rxfifo_msg);
}

static void
cc2420_setup_txfifo_spi_messages(struct cc2420_local *lp)
{
	spi_message_init(&lp->txfifo_msg);
	lp->txfifo_msg.context = lp;

	lp->txfifo_xfer_addr.len = 1;
	lp->txfifo_xfer_addr.tx_buf = lp->txfifo_addr;
	lp->txfifo_xfer_addr.rx_buf = lp->txfifo_addr;

	lp->txfifo_xfer_len.len = 1;
	lp->txfifo_xfer_len.tx_buf = lp->txfifo_len;

	spi_message_add_tail(&lp->txfifo_xfer_addr, &lp->txfifo_msg);
	spi_message_add_tail(&lp->txfifo_xfer_len, &lp->txfifo_msg);
	spi_message_add_tail(&lp->txfifo_xfer_buf, &lp->txfifo_msg);
}

static void
cc2420_setup_reg_spi_messages(struct cc2420_local *lp)
{
	spi_message_init(&lp->reg_msg);
	lp->reg_msg.context = lp;

	lp->reg_xfer_addr.len = 1;
	lp->reg_xfer_addr.tx_buf = lp->reg_addr;
	lp->reg_xfer_addr.rx_buf = lp->reg_addr;

	lp->reg_xfer_val.len = 1;
	lp->reg_xfer_val.rx_buf = lp->reg_val;
	lp->reg_xfer_val.tx_buf = lp->reg_val;

	spi_message_add_tail(&lp->reg_xfer_addr, &lp->reg_msg);
	spi_message_add_tail(&lp->reg_xfer_val, &lp->reg_msg);
}

static void
cc2420_setup_cmd_spi_messages(struct cc2420_local *lp)
{
	spi_message_init(&lp->cmd_msg);
	lp->cmd_msg.context = lp;

	lp->cmd_xfer_val.len = 1;
	lp->cmd_xfer_val.tx_buf = lp->cmd_val;
	lp->cmd_xfer_val.rx_buf = lp->cmd_val;

	spi_message_add_tail(&lp->cmd_xfer_val, &lp->cmd_msg);
}

static int cc2420_hw_init(struct cc2420_local *lp)
{
	int ret;
	u16 state;
	u8 status = 0xff;
	int timeout = 500; /* 500us delay */
	ret = cc2420_read_16_bit_reg(lp, RG_FSMSTATE, &state);
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

	/* enable AUTOACK */
	dev_dbg(&lp->spi->dev, "enable hardware AUTO ACK\n");
	cc2420_write_subreg(lp, SG_AUTOACK, 1);

	dev_info(&lp->spi->dev, "set fifo threshold to 127\n");
	cc2420_write_subreg(lp, SG_FIFOP_THR, 127);

	return 0;
error_ret:
	return ret;
}

static int cc2420_probe(struct spi_device *spi)
{
	struct cc2420_local *lp;
	struct cc2420_platform_data pdata;
	unsigned int manid, version, partnum;
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
	spin_lock_init(&lp->lock);

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

	/* setup regmap */
	lp->regmap = devm_regmap_init_spi(spi, &cc2420_reg_regmap);
	if (IS_ERR(lp->regmap)) {
		ret = PTR_ERR(lp->regmap);
		dev_err(&spi->dev, "Failed to allocate dar map: %d\n",
			ret);
		goto err_hw_init;
	}

	cc2420_setup_rxfifo_spi_messages(lp);
	cc2420_setup_reg_spi_messages(lp);
	cc2420_setup_txfifo_spi_messages(lp);
	cc2420_setup_cmd_spi_messages(lp);

	/* Check this is actually a cc2420 */
	ret = cc2420_read_subreg(lp, SR_MANFID, &manid);
	if (ret)
		goto err_hw_init;
	ret = cc2420_read_subreg(lp, SR_PARTNUM, &partnum);
	if (ret)
		goto err_hw_init;

	ret = cc2420_read_subreg(lp, SR_VERSION, &version);
	if (ret)
		goto err_hw_init;

	if (manid != CC2420_MANFID || version != CC2420_VERSION) {
		dev_warn(&spi->dev, "Incorrect manufacturer id or version:"
				    "%x, %x\n", manid, version);
	}

	dev_info(&lp->spi->dev, "Found Chipcon CC2420\n");
	dev_info(&lp->spi->dev, "Manufacturer ID:%x Version:%x Partnum:%x\n",
		   manid, version, partnum);

	ret = cc2420_hw_init(lp);
	if (ret)
		goto err_hw_init;

	lp->fifop_irq = gpio_to_irq(pdata.fifop);
	lp->sfd_irq = gpio_to_irq(pdata.sfd);

	/* Set up fifop interrupt */
	ret = devm_request_irq(&spi->dev,
			       lp->fifop_irq,
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
			       lp->sfd_irq,
			       cc2420_sfd_isr,
			       IRQF_TRIGGER_FALLING,
			       dev_name(&spi->dev),
			       lp);
	if (ret) {
		dev_err(&spi->dev, "could not get sfd irq\n");
		goto err_hw_init;
	}

	/* disable irqs*/
	disable_irq(lp->fifop_irq);
	disable_irq(lp->sfd_irq);

	ret = cc2420_register(lp);
	if (ret)
		goto err_hw_init;

	return 0;

err_hw_init:
	mutex_destroy(&lp->buffer_mutex);
	return ret;
}

static int cc2420_remove(struct spi_device *spi)
{
	struct cc2420_local *lp = spi_get_drvdata(spi);

	dev_info(printdev(lp), "%s\n", __func__);

	mutex_destroy(&lp->buffer_mutex);

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
