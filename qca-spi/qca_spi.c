/*
 *   Copyright (c) 2011, 2012, Qualcomm Atheros Communications Inc.
 *   Copyright (c) 2014, I2SE GmbH
 *
 *   Permission to use, copy, modify, and/or distribute this software
 *   for any purpose with or without fee is hereby granted, provided
 *   that the above copyright notice and this permission notice appear
 *   in all copies.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 *   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 *   WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL
 *   THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 *   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 *   LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 *   NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 *   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*   This module implements the Qualcomm Atheros SPI protocol for
 *   kernel-based SPI device; it is essentially an Ethernet-to-SPI
 *   serial converter;
 */

#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/version.h>

#include "qca_7k.h"
#include "qca_debug.h"
#include "qca_framing.h"
#include "qca_spi.h"

#define QCASPI_VERSION "0.2.1-i"
#define QCASPI_MODNAME "qcaspi"
#define QCASPI_DEF_MAC_ADDRESS "\x00\xB0\x52\xFF\xFF\x02"

#define MAX_DMA_BURST_LEN 5000

/*   Modules parameters     */
#define QCASPI_CLK_SPEED_MIN 1000000
#define QCASPI_CLK_SPEED_MAX 16000000
#define QCASPI_CLK_SPEED 8000000
static int qcaspi_clkspeed = QCASPI_CLK_SPEED;
module_param(qcaspi_clkspeed, int, 0);
MODULE_PARM_DESC(qcaspi_clkspeed, "SPI bus clock speed (Hz)");

#define QCASPI_LEGACY_MODE_MIN 0
#define QCASPI_LEGACY_MODE_MAX 1
static int qcaspi_legacy_mode = QCASPI_LEGACY_MODE_MIN;
module_param(qcaspi_legacy_mode, int, 0);
MODULE_PARM_DESC(qcaspi_legacy_mode, "Turn on/off legacy mode.");

#define QCASPI_BURST_LEN_MIN 1
#define QCASPI_BURST_LEN_MAX MAX_DMA_BURST_LEN
static int qcaspi_burst_len = MAX_DMA_BURST_LEN;
module_param(qcaspi_burst_len, int, 0);
MODULE_PARM_DESC(qcaspi_burst_len, "Number of data bytes per burst. Use 1-5000.");

#define QCASPI_MTU QCAFRM_ETHMAXMTU
#define QCASPI_TX_TIMEOUT (1 * HZ)

u32
disable_spi_interrupts(struct qcaspi *qca)
{
	u32 old_value = qcaspi_read_register(qca, SPI_REG_INTR_ENABLE);
	qcaspi_write_register(qca, SPI_REG_INTR_ENABLE, 0);
	return old_value;
}

u32
enable_spi_interrupts(struct qcaspi *qca, u32 intr_enable)
{
	u32 old_value = qcaspi_read_register(qca, SPI_REG_INTR_ENABLE);
	qcaspi_write_register(qca, SPI_REG_INTR_ENABLE, intr_enable);
	return old_value;
}

u32
qcaspi_write_burst(struct qcaspi *qca, u8 *src, u32 len)
{
	u16 cmd;
	struct spi_message msg;
	struct spi_transfer transfer[2];

	memset(&transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	cmd = cpu_to_be16(QCA7K_SPI_WRITE | QCA7K_SPI_EXTERNAL);
	transfer[0].tx_buf = &cmd;
	transfer[0].len = QCASPI_CMD_LEN;
	transfer[1].tx_buf = src;
	transfer[1].len = len;

	spi_message_add_tail(&transfer[0], &msg);
	spi_message_add_tail(&transfer[1], &msg);
	spi_sync(qca->spi_dev, &msg);

	if (msg.actual_length != QCASPI_CMD_LEN + len)
		return 0;

	return len;
}

u32
qcaspi_write_legacy(struct qcaspi *qca, u8 *src, u32 len)
{
	struct spi_message msg;
	struct spi_transfer transfer;

	memset(&transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	transfer.tx_buf = src;
	transfer.len = len;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(qca->spi_dev, &msg);

	if (msg.actual_length != len)
		return 0;

	return len;
}

u32
qcaspi_read_burst(struct qcaspi *qca, u8 *dst, u32 len)
{
	struct spi_message msg;
	u16 cmd;
	struct spi_transfer transfer[2];

	memset(&transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	cmd = cpu_to_be16(QCA7K_SPI_READ | QCA7K_SPI_EXTERNAL);
	transfer[0].tx_buf = &cmd;
	transfer[0].len = QCASPI_CMD_LEN;
	transfer[1].rx_buf = dst;
	transfer[1].len = len;

	spi_message_add_tail(&transfer[0], &msg);
	spi_message_add_tail(&transfer[1], &msg);
	spi_sync(qca->spi_dev, &msg);

	if (msg.actual_length != QCASPI_CMD_LEN + len)
		return 0;

	return len;
}

u32
qcaspi_read_legacy(struct qcaspi *qca, u8 *dst, u32 len)
{
	struct spi_message msg;
	struct spi_transfer transfer;

	memset(&transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	transfer.rx_buf = dst;
	transfer.len = len;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(qca->spi_dev, &msg);

	if (msg.actual_length != len)
		return 0;

	return len;
}

int
qcaspi_tx_frame(struct qcaspi *qca, struct sk_buff *skb)
{
	u32 count;
	u32 bytes_written;
	u32 offset;
	u32 len;

	len = skb->len;

	qcaspi_write_register(qca, SPI_REG_BFR_SIZE, len);
	if (qca->legacy_mode)
		qcaspi_tx_cmd(qca, QCA7K_SPI_WRITE | QCA7K_SPI_EXTERNAL);

	offset = 0;
	while (len) {
		count = len;
		if (count > qca->burst_len)
			count = qca->burst_len;

		if (qca->legacy_mode) {
			bytes_written = qcaspi_write_legacy(qca,
					skb->data + offset, count);
		} else {
			bytes_written = qcaspi_write_burst(qca,
					skb->data + offset, count);
		}

		if (bytes_written != count)
			return -1;

		offset += count;
		len -= count;
	}

	return 0;
}

int
qcaspi_transmit(struct qcaspi *qca)
{
	struct net_device_stats *n_stats = &qca->net_dev->stats;
	u32 available;
	u32 pkt_len;

	available = qcaspi_read_register(qca, SPI_REG_WRBUF_SPC_AVA);

	while (qca->txq.skb[qca->txq.head]) {
		pkt_len = qca->txq.skb[qca->txq.head]->len + QCASPI_HW_PKT_LEN;

		if (available < pkt_len) {
			qca->stats.write_buf_miss++;
			break;
		}

		if (qcaspi_tx_frame(qca, qca->txq.skb[qca->txq.head]) == -1) {
			qca->stats.write_err++;
			return -1;
		}

		n_stats->tx_packets++;
		n_stats->tx_bytes += qca->txq.skb[qca->txq.head]->len;
		available -= pkt_len;

		/* remove the skb from the queue */
		/* XXX After inconsistent lock states netif_tx_lock()
		 * has been replaced by netif_tx_lock_bh() and so on.
		 */
		netif_tx_lock_bh(qca->net_dev);
		dev_kfree_skb(qca->txq.skb[qca->txq.head]);
		qca->txq.skb[qca->txq.head] = NULL;
		qca->txq.head++;
		if (qca->txq.head >= TX_QUEUE_LEN)
			qca->txq.head = 0;
		netif_wake_queue(qca->net_dev);
		netif_tx_unlock_bh(qca->net_dev);
	}

	return 0;
}

int
qcaspi_receive(struct qcaspi *qca)
{
	struct net_device_stats *n_stats = &qca->net_dev->stats;
	u16 available;
	u32 bytes_read;
	u32 count;
	u8 *cp;

	/* Allocate rx SKB if we don't have one available. */
	if (!qca->rx_skb) {
		qca->rx_skb = netdev_alloc_skb(qca->net_dev,
				qca->net_dev->mtu + VLAN_ETH_HLEN);
		if (!qca->rx_skb) {
			netdev_dbg(qca->net_dev, "out of RX resources\n");
			qca->stats.out_of_mem++;
			return -1;
		}
	}

	/* Read the packet size. */
	available = qcaspi_read_register(qca, SPI_REG_RDBUF_BYTE_AVA);
	netdev_dbg(qca->net_dev, "qcaspi_receive: SPI_REG_RDBUF_BYTE_AVA: Value: %08x\n",
			available);

	if (available == 0) {
		netdev_dbg(qca->net_dev, "qcaspi_receive called without any data being available!\n");
		return -1;
	}

	qcaspi_write_register(qca, SPI_REG_BFR_SIZE, available);

	if (qca->legacy_mode)
		qcaspi_tx_cmd(qca, QCA7K_SPI_READ | QCA7K_SPI_EXTERNAL);

	while (available) {
		count = available;
		if (count > qca->burst_len)
			count = qca->burst_len;

		if (qca->legacy_mode) {
			bytes_read = qcaspi_read_legacy(qca, qca->rx_buffer,
					count);
		} else {
			bytes_read = qcaspi_read_burst(qca, qca->rx_buffer,
					count);
		}

		cp = qca->rx_buffer;

		netdev_dbg(qca->net_dev, "available: %d, byte read: %d\n",
				available, bytes_read);

		if (bytes_read)
			available -= bytes_read;
		else
			qca->stats.read_err++;

		while ((bytes_read--) && (qca->rx_skb)) {
			s32 retcode;
			retcode = qcafrm_fsm_decode(&qca->frm_handle,
					qca->rx_skb->data,
					skb_tailroom(qca->rx_skb),
					*cp);
			cp++;
			switch (retcode) {
			case QCAFRM_GATHER:
			case QCAFRM_NOHEAD:
				break;
			case QCAFRM_NOTAIL:
				netdev_dbg(qca->net_dev, "no RX tail\n");
				n_stats->rx_errors++;
				n_stats->rx_dropped++;
				break;
			case QCAFRM_INVLEN:
				netdev_dbg(qca->net_dev, "invalid RX length\n");
				n_stats->rx_errors++;
				n_stats->rx_dropped++;
				break;
			default:
				qca->rx_skb->dev = qca->net_dev;
				n_stats->rx_packets++;
				n_stats->rx_bytes += retcode;
				skb_put(qca->rx_skb, retcode);
				qca->rx_skb->protocol = eth_type_trans(
					qca->rx_skb, qca->rx_skb->dev);
				qca->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
				netif_rx_ni(qca->rx_skb);
				qca->rx_skb = netdev_alloc_skb(qca->net_dev,
					qca->net_dev->mtu + VLAN_ETH_HLEN);
				if (!qca->rx_skb) {
					netdev_dbg(qca->net_dev, "out of RX resources\n");
					n_stats->rx_errors++;
					qca->stats.out_of_mem++;
					break;
				}
			}
		}
	}

	return 0;
}

/*   Flush the tx queue. This function is only safe to
 *   call from the qcaspi_spi_thread.
 */

void
qcaspi_flush_txq(struct qcaspi *qca)
{
	int i;

	/* XXX After inconsistent lock states netif_tx_lock()
	 * has been replaced by netif_tx_lock_bh() and so on.
	 */
	netif_tx_lock_bh(qca->net_dev);
	for (i = 0; i < TX_QUEUE_LEN; i++) {
		if (qca->txq.skb[i])
			dev_kfree_skb(qca->txq.skb[i]);
		qca->txq.skb[i] = NULL;
		qca->txq.tail = 0;
		qca->txq.head = 0;
	}
	netif_tx_unlock_bh(qca->net_dev);
}

void
qcaspi_qca7k_sync(struct qcaspi *qca, int event)
{
	u16 signature;
	u16 spi_config;
	u16 wrbuf_space;
	static u16 reset_count;

	if (event == QCASPI_SYNC_CPUON) {
		/* Read signature twice, if not valid
		 * go back to unknown state.
		 */
		signature = qcaspi_read_register(qca, SPI_REG_SIGNATURE);
		signature = qcaspi_read_register(qca, SPI_REG_SIGNATURE);
		if (signature != QCASPI_GOOD_SIGNATURE) {
			qca->sync = QCASPI_SYNC_UNKNOWN;
			netdev_dbg(qca->net_dev, "sync: got CPU on, but signature was invalid, restart\n");
		} else {
			/* ensure that the WRBUF is empty */
			wrbuf_space = qcaspi_read_register(qca,
					SPI_REG_WRBUF_SPC_AVA);
			if (wrbuf_space != QCASPI_HW_BUF_LEN) {
				netdev_dbg(qca->net_dev, "sync: got CPU on, but wrbuf not empty. reset!\n");
				qca->sync = QCASPI_SYNC_UNKNOWN;
			} else {
				netdev_dbg(qca->net_dev, "sync: got CPU on, now in sync\n");
				qca->sync = QCASPI_SYNC_READY;
				return;
			}
		}
	}

	switch (qca->sync) {
	case QCASPI_SYNC_READY:
		/* Don't check signature after sync in burst mode. */
		if (!qca->legacy_mode)
			return;

		signature = qcaspi_read_register(qca, SPI_REG_SIGNATURE);
		if (signature != QCASPI_GOOD_SIGNATURE) {
			qca->sync = QCASPI_SYNC_UNKNOWN;
			netdev_dbg(qca->net_dev, "sync: bad signature, restart\n");
			/* don't reset right away */
			return;
		}
		break;
	case QCASPI_SYNC_UNKNOWN:
		/* Read signature, if not valid stay in unknown state */
		signature = qcaspi_read_register(qca, SPI_REG_SIGNATURE);
		if (signature != QCASPI_GOOD_SIGNATURE) {
			netdev_dbg(qca->net_dev, "sync: could not read signature to reset device, retry.\n");
			return;
		}

		/* TODO: use GPIO to reset QCA7000 in legacy mode*/
		netdev_dbg(qca->net_dev, "sync: resetting device.\n");
		spi_config = qcaspi_read_register(qca, SPI_REG_SPI_CONFIG);
		spi_config |= QCASPI_SLAVE_RESET_BIT;
		qcaspi_write_register(qca, SPI_REG_SPI_CONFIG, spi_config);

		qca->sync = QCASPI_SYNC_RESET;
		qca->stats.trig_reset++;
		reset_count = 0;
		break;
	case QCASPI_SYNC_RESET:
		reset_count++;
		netdev_dbg(qca->net_dev, "sync: waiting for CPU on, count %u.\n",
				reset_count);
		if (reset_count >= QCASPI_RESET_TIMEOUT) {
			/* reset did not seem to take place, try again */
			qca->sync = QCASPI_SYNC_UNKNOWN;
			qca->stats.reset_timeout++;
			netdev_dbg(qca->net_dev, "sync: reset timeout, restarting process.\n");
		}
		break;
	}
}

static int
qcaspi_spi_thread(void *data)
{
	struct qcaspi *qca = (struct qcaspi *) data;
	u32 intr_cause;
	u32 intr_enable;

	netdev_info(qca->net_dev, "SPI thread created\n");
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		if ((qca->intr_req == qca->intr_svc) &&
		    (qca->txq.skb[qca->txq.head] == NULL) &&
		    (qca->sync == QCASPI_SYNC_READY))
			schedule();

		set_current_state(TASK_RUNNING);

		netdev_dbg(qca->net_dev, "have work to do. int: %d, tx_skb: %p\n",
				qca->intr_req - qca->intr_svc,
				qca->txq.skb[qca->txq.head]);

		qcaspi_qca7k_sync(qca, QCASPI_SYNC_UPDATE);

		if (qca->sync != QCASPI_SYNC_READY) {
			netdev_dbg(qca->net_dev, "sync: not ready %u, turn off carrier and flush\n",
					(unsigned int) qca->sync);
			netif_carrier_off(qca->net_dev);
			qcaspi_flush_txq(qca);
			netif_wake_queue(qca->net_dev);
			msleep(1000);
		}

		if (qca->intr_svc != qca->intr_req) {
			qca->intr_svc = qca->intr_req;
			intr_enable = disable_spi_interrupts(qca);
			intr_cause = qcaspi_read_register(qca,
					SPI_REG_INTR_CAUSE);
			netdev_dbg(qca->net_dev, "interrupts: 0x%08x\n",
					intr_cause);

			if (intr_cause & SPI_INT_CPU_ON) {
				qcaspi_qca7k_sync(qca, QCASPI_SYNC_CPUON);

				/* not synced. */
				if (qca->sync != QCASPI_SYNC_READY)
					continue;

				qca->stats.device_reset++;
				intr_enable = (SPI_INT_CPU_ON |
					SPI_INT_PKT_AVLBL |
					SPI_INT_RDBUF_ERR |
					SPI_INT_WRBUF_ERR);
				netif_carrier_on(qca->net_dev);
			}

			if (intr_cause & SPI_INT_RDBUF_ERR) {
				/* restart sync */
				netdev_dbg(qca->net_dev, "===> rdbuf error!\n");
				qca->stats.read_buf_err++;
				qca->sync = QCASPI_SYNC_UNKNOWN;
				continue;
			}

			if (intr_cause & SPI_INT_WRBUF_ERR) {
				/* restart sync */
				netdev_dbg(qca->net_dev, "===> wrbuf error!\n");
				qca->stats.write_buf_err++;
				qca->sync = QCASPI_SYNC_UNKNOWN;
				continue;
			}

			/* can only handle other interrupts
			 * if sync has occured
			 */
			if (qca->sync == QCASPI_SYNC_READY) {
				if (intr_cause & SPI_INT_PKT_AVLBL)
					qcaspi_receive(qca);
			}

			qcaspi_write_register(qca, SPI_REG_INTR_CAUSE,
					intr_cause);
			enable_spi_interrupts(qca, intr_enable);
			netdev_dbg(qca->net_dev, "acking int: 0x%08x\n",
					intr_cause);
		}

		if (qca->txq.skb[qca->txq.head])
			qcaspi_transmit(qca);
	}
	set_current_state(TASK_RUNNING);
	netdev_info(qca->net_dev, "SPI thread exit\n");

	return 0;
}

static irqreturn_t
qcaspi_intr_handler(int irq, void *data)
{
	struct qcaspi *qca = (struct qcaspi *) data;
	qca->intr_req++;
	if (qca->spi_thread &&
		qca->spi_thread->state != TASK_RUNNING)
		wake_up_process(qca->spi_thread);

	return IRQ_HANDLED;
}

int
qcaspi_netdev_open(struct net_device *dev)
{
	struct qcaspi *qca = netdev_priv(dev);
	int ret = 0;

	if (!qca)
		return -EINVAL;

	qca->irq = gpio_to_irq(qca->intr_gpio);

	if (qca->irq < 0) {
		netdev_err(dev, "%s: failed to get IRQ from gpio %d: %d!\n",
					QCASPI_MODNAME, qca->intr_gpio, qca->irq);
		return qca->irq;
	}

	memset(&qca->txq, 0, sizeof(qca->txq));
	qca->intr_req = 0;
	qca->intr_svc = 0;
	qca->sync = QCASPI_SYNC_UNKNOWN;
	qcafrm_fsm_init(&qca->frm_handle);

	qca->spi_thread = kthread_run((void *)qcaspi_spi_thread,
			qca, "%s", dev->name);

	if (IS_ERR(qca->spi_thread)) {
		netdev_err(dev, "%s: unable to start kernel thread.\n",
				QCASPI_MODNAME);
		return PTR_ERR(qca->spi_thread);
	}

	ret = request_irq(qca->irq, qcaspi_intr_handler,
				IRQF_TRIGGER_RISING, dev->name, qca);
	if (ret) {
		netdev_err(dev, "%s: unable to get IRQ %d (irqval=%d).\n",
				QCASPI_MODNAME, qca->irq, ret);
		kthread_stop(qca->spi_thread);
		return ret;
	}

	netif_start_queue(qca->net_dev);

	return 0;
}

int
qcaspi_netdev_close(struct net_device *dev)
{
	struct qcaspi *qca = netdev_priv(dev);

	netif_stop_queue(dev);

	qcaspi_write_register(qca, SPI_REG_INTR_ENABLE, 0);
	free_irq(qca->irq, qca);
	qca->irq = 0;

	kthread_stop(qca->spi_thread);
	qca->spi_thread = NULL;
	qcaspi_flush_txq(qca);

	return 0;
}

netdev_tx_t
qcaspi_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	u32 frame_len;
	u8 *ptmp;
	struct qcaspi *qca = netdev_priv(dev);
	u32 new_tail;
	struct sk_buff *tskb;
	u8 pad_len = 0;

	if (skb->len < QCAFRM_ETHMINLEN)
		pad_len = QCAFRM_ETHMINLEN - skb->len;

	if (qca->txq.skb[qca->txq.tail]) {
		netdev_warn(qca->net_dev, "queue was unexpectedly full!\n");
		netif_stop_queue(qca->net_dev);
		qca->stats.queue_full++;
		return NETDEV_TX_BUSY;
	}

	if ((skb_headroom(skb) < QCAFRM_HEADER_LEN) ||
	    (skb_tailroom(skb) < QCAFRM_FOOTER_LEN + pad_len)) {
		tskb = skb_copy_expand(skb, QCAFRM_HEADER_LEN,
				QCAFRM_FOOTER_LEN + pad_len, GFP_ATOMIC);
		if (!tskb) {
			netdev_dbg(qca->net_dev, "could not allocate tx_buff in qcaspi_netdev_xmit\n");
			qca->stats.out_of_mem++;
			return NETDEV_TX_BUSY;
		}
		dev_kfree_skb(skb);
		skb = tskb;
	}

	frame_len = skb->len + pad_len;

	ptmp = skb_push(skb, QCAFRM_HEADER_LEN);
	qcafrm_create_header(ptmp, frame_len);

	if (pad_len) {
		ptmp = skb_put(skb, pad_len);
		memset(ptmp, 0, pad_len);
	}

	ptmp = skb_put(skb, QCAFRM_FOOTER_LEN);
	qcafrm_create_footer(ptmp);

	netdev_dbg(qca->net_dev, "Tx-ing packet: Size: 0x%08x\n",
			skb->len);

	new_tail = qca->txq.tail + 1;
	if (new_tail >= TX_QUEUE_LEN)
		new_tail = 0;

	if (qca->txq.skb[new_tail])
		netif_stop_queue(qca->net_dev);

	qca->txq.skb[qca->txq.tail] = skb;
	qca->txq.tail = new_tail;

	dev->trans_start = jiffies;

	if (qca->spi_thread &&
		qca->spi_thread->state != TASK_RUNNING)
		wake_up_process(qca->spi_thread);

	return NETDEV_TX_OK;
}

void
qcaspi_netdev_tx_timeout(struct net_device *dev)
{
	struct qcaspi *qca = netdev_priv(dev);
	netdev_info(qca->net_dev, "Transmit timeout at %ld, latency %ld\n",
			jiffies, jiffies - dev->trans_start);
	qca->net_dev->stats.tx_errors++;
	/* wake the queue if there is room */
	if (qca->txq.skb[qca->txq.tail] == NULL)
		netif_wake_queue(dev);
}

static int
qcaspi_netdev_init(struct net_device *dev)
{
	struct qcaspi *qca = netdev_priv(dev);

	dev->mtu = QCASPI_MTU;
	dev->type = ARPHRD_ETHER;
	qca->irq = 0;
	qca->clkspeed = qcaspi_clkspeed;
	qca->legacy_mode = qcaspi_legacy_mode;
	qca->burst_len = qcaspi_burst_len;
	qca->spi_thread = NULL;
	qca->buffer_size = (dev->mtu + VLAN_ETH_HLEN + QCAFRM_HEADER_LEN +
		QCAFRM_FOOTER_LEN + 4) * 4;

	memset(&qca->stats, 0, sizeof(struct qcaspi_stats));

	qca->rx_buffer = kmalloc(qca->buffer_size, GFP_KERNEL);
	if (!qca->rx_buffer)
		return -ENOBUFS;

	qca->rx_skb = netdev_alloc_skb(dev, qca->net_dev->mtu + VLAN_ETH_HLEN);
	if (!qca->rx_skb) {
		kfree(qca->rx_buffer);
		netdev_info(qca->net_dev, "Failed to allocate RX sk_buff.\n");
		return -ENOBUFS;
	}

	return 0;
}

static void
qcaspi_netdev_uninit(struct net_device *dev)
{
	struct qcaspi *qca = netdev_priv(dev);
	kfree(qca->rx_buffer);
	qca->buffer_size = 0;
	if (qca->rx_skb)
		dev_kfree_skb(qca->rx_skb);
}

int
qcaspi_netdev_change_mtu(struct net_device *dev, int new_mtu)
{
	if ((new_mtu < QCAFRM_ETHMINMTU) || (new_mtu > QCAFRM_ETHMAXMTU))
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}

static void
qcaspi_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *p)
{
	struct qcaspi *qca = netdev_priv(dev);

	strlcpy(p->driver, QCASPI_MODNAME, sizeof(p->driver));
	strlcpy(p->version, QCASPI_VERSION, sizeof(p->version));
	strlcpy(p->fw_version, "QCA7000", sizeof(p->fw_version));
	strlcpy(p->bus_info, dev_name(&qca->spi_dev->dev),
		sizeof(p->bus_info));
}

static int
qcaspi_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	cmd->transceiver = XCVR_INTERNAL;
	cmd->supported = SUPPORTED_10baseT_Half;
	ethtool_cmd_speed_set(cmd,  SPEED_10);
	cmd->duplex = DUPLEX_HALF;
	cmd->port = PORT_OTHER;
	cmd->autoneg = AUTONEG_DISABLE;

	return 0;
}

static const struct ethtool_ops ops = {
	.get_drvinfo = qcaspi_get_drvinfo,
	.get_link = ethtool_op_get_link,
	.get_settings = qcaspi_get_settings,
};

static int
qcaspi_netdev_set_mac_address(struct net_device *dev, void *p)
{
	struct qcaspi *qca = netdev_priv(dev);
	struct sockaddr *addr = p;
	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	netdev_info(qca->net_dev, "Setting MAC address to %pM.\n",
			dev->dev_addr);

	return 0;
}

static const struct net_device_ops qcaspi_netdev_ops = {
	.ndo_init = qcaspi_netdev_init,
	.ndo_uninit = qcaspi_netdev_uninit,
	.ndo_open = qcaspi_netdev_open,
	.ndo_stop = qcaspi_netdev_close,
	.ndo_start_xmit = qcaspi_netdev_xmit,
	.ndo_change_mtu = qcaspi_netdev_change_mtu,
	.ndo_set_mac_address = qcaspi_netdev_set_mac_address,
	.ndo_tx_timeout = qcaspi_netdev_tx_timeout,
};

void
qcaspi_netdev_setup(struct net_device *dev)
{
	struct qcaspi *qca = NULL;

	ether_setup(dev);

	dev->netdev_ops = &qcaspi_netdev_ops;
	SET_ETHTOOL_OPS(dev, &ops);
	dev->watchdog_timeo = QCASPI_TX_TIMEOUT;
	dev->flags = IFF_MULTICAST;
	dev->tx_queue_len = 100;
	memcpy(dev->dev_addr, QCASPI_DEF_MAC_ADDRESS, dev->addr_len);

	qca = netdev_priv(dev);
	memset(qca, 0, sizeof(struct qcaspi));
}

static const struct of_device_id qca_spi_of_match[] = {
	{ .compatible = "qca,qca7000" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, qca_spi_of_match);

static int
qca_spi_probe(struct spi_device *spi_device)
{
	struct qcaspi *qca = NULL;
	struct net_device *qcaspi_devs = NULL;
	int intr_gpio = 0;
	bool fast_probe = false;
	u32 signature;
	u16 prop = 0;
	int ret;

	if (!spi_device->dev.of_node) {
		dev_err(&spi_device->dev, "Missing device tree\n");
		return -EINVAL;
	}

	dev_info(&spi_device->dev, "SPI device probe (version %s)\n",
		QCASPI_VERSION);

	/* TODO: Make module parameter higher prio as device tree */
	if (of_property_read_u16(spi_device->dev.of_node,
		"legacy-mode", &prop) == 0)
		qcaspi_legacy_mode = prop;

	if (of_property_read_u16(spi_device->dev.of_node,
		"burst-length", &prop) == 0)
		qcaspi_burst_len = prop;

	if (of_find_property(spi_device->dev.of_node,
			"fast-probe", NULL)) {
		fast_probe = true;
	}

	intr_gpio = of_get_named_gpio(spi_device->dev.of_node,
			"intr-gpios", 0);

	if (!gpio_is_valid(intr_gpio)) {
		dev_err(&spi_device->dev, "Missing interrupt gpio\n");
		return -EINVAL;
	}

	ret = gpio_request_one(intr_gpio, GPIOF_IN, "qca7k_intr0");

	if (ret < 0) {
		dev_err(&spi_device->dev,
		"Failed to request interrupt gpio %d: %d!\n",
		intr_gpio, ret);
	}

	if ((qcaspi_clkspeed < QCASPI_CLK_SPEED_MIN) ||
	    (qcaspi_clkspeed > QCASPI_CLK_SPEED_MAX) ||
	    (qcaspi_legacy_mode < QCASPI_LEGACY_MODE_MIN) ||
	    (qcaspi_legacy_mode > QCASPI_LEGACY_MODE_MAX) ||
	    (qcaspi_burst_len < QCASPI_BURST_LEN_MIN) ||
	    (qcaspi_burst_len > QCASPI_BURST_LEN_MAX)) {
		dev_info(&spi_device->dev, "Invalid parameters (clkspeed=%d, legacy_mode=%d, burst_len=%d)\n",
			qcaspi_clkspeed, qcaspi_legacy_mode, qcaspi_burst_len);
		return -EINVAL;
	}
	dev_info(&spi_device->dev, "Get parameters (clkspeed=%d, legacy_mode=%d, burst_len=%d)\n",
	       qcaspi_clkspeed, qcaspi_legacy_mode, qcaspi_burst_len);

	spi_device->mode = SPI_MODE_3;
	spi_device->max_speed_hz = qcaspi_clkspeed;
	if (spi_setup(spi_device) < 0) {
		dev_err(&spi_device->dev, "Unable to setup SPI device\n");
		return -EFAULT;
	}

	qcaspi_devs = alloc_netdev(sizeof(struct qcaspi), "qca%d",
			qcaspi_netdev_setup);
	if (!qcaspi_devs) {
		dev_err(&spi_device->dev, "Unable to allocate memory for spi network device\n");
		return -ENOMEM;
	}
	qca = netdev_priv(qcaspi_devs);
	if (!qca) {
		free_netdev(qcaspi_devs);
		dev_err(&spi_device->dev, "Fail to retrieve private structure from net device\n");
		return -ENOMEM;
	}
	qca->net_dev = qcaspi_devs;
	qca->spi_dev = spi_device;
	qca->intr_gpio = intr_gpio;

	netif_carrier_off(qca->net_dev);

	if (!fast_probe) {
		signature = qcaspi_read_register(qca, SPI_REG_SIGNATURE);
		signature = qcaspi_read_register(qca, SPI_REG_SIGNATURE);

		if (signature != QCASPI_GOOD_SIGNATURE) {
			dev_err(&spi_device->dev, "Invalid signature (0x%04X)\n",
				signature);
			free_netdev(qcaspi_devs);
			return -EFAULT;
		}
	}

	if (register_netdev(qcaspi_devs)) {
		dev_info(&spi_device->dev, "Unable to register network device %s\n",
			qcaspi_devs->name);
		free_netdev(qcaspi_devs);
		return -EFAULT;
	}

	spi_set_drvdata(spi_device, qcaspi_devs);

	qcaspi_init_device_debugfs(qca);

	return 0;
}

static int
qca_spi_remove(struct spi_device *spi_device)
{
	struct net_device *qcaspi_devs = spi_get_drvdata(spi_device);
	struct qcaspi *qca = netdev_priv(qcaspi_devs);

	qcaspi_remove_device_debugfs(qca);

	if (gpio_is_valid(qca->intr_gpio))
		gpio_free(qca->intr_gpio);

	unregister_netdev(qcaspi_devs);
	free_netdev(qcaspi_devs);

	return 0;
}

static const struct spi_device_id qca_spi_id[] = {
	{ "qca7000", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, qca_spi_id);

static struct spi_driver qca_spi_driver = {
	.driver	= {
		.name	= QCASPI_MODNAME,
		.owner	= THIS_MODULE,
		.of_match_table = qca_spi_of_match,
	},
	.id_table = qca_spi_id,
	.probe    = qca_spi_probe,
	.remove   = qca_spi_remove,
};
module_spi_driver(qca_spi_driver);

MODULE_DESCRIPTION("Qualcomm Atheros SPI Driver");
MODULE_AUTHOR("Qualcomm Atheros Communications");
MODULE_AUTHOR("Stefan Wahren <stefan.wahren@i2se.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(QCASPI_VERSION);

