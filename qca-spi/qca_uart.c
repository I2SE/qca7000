/*
 *   Copyright (c) 2011, 2012, Qualcomm Atheros Communications Inc.
 *   Copyright (c) 2016, I2SE GmbH
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

/*   This module implements the Qualcomm Atheros UART protocol for
 *   kernel-based UART device; it is essentially an Ethernet-to-UART
 *   serial converter;
 */

#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/tty.h>
#include <linux/types.h>

#include "qca_common.h"
#include "qca_uart.h"

#define QCAUART_DRV_VERSION "0.0.5"
#define QCAUART_DRV_NAME "qcauart"
#define QCAUART_MTU QCAFRM_ETHMAXMTU
#define QCAUART_TX_TIMEOUT (1 * HZ)

void
qca_tty_receive(struct tty_struct *tty, const unsigned char *cp, char *fp, int count)
{
	struct qcauart *qca = tty->disc_data;

	if (!qca->rx_skb) {
		qca->rx_skb = netdev_alloc_skb(qca->net_dev, qca->net_dev->mtu +
					       VLAN_ETH_HLEN);
		if (!qca->rx_skb) {
			qca->stats.rx_errors++;
			qca->stats.rx_dropped++;
			return;
		}
	}

	while (count-- && qca->rx_skb) {
		s32 retcode = qcafrm_fsm_decode(&qca->frm_handle,
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
			qca->stats.rx_errors++;
			qca->stats.rx_dropped++;
			break;
		case QCAFRM_INVLEN:
			netdev_dbg(qca->net_dev, "invalid RX length\n");
			qca->stats.rx_errors++;
			qca->stats.rx_dropped++;
			break;
		default:
			qca->rx_skb->dev = qca->net_dev;
			qca->stats.rx_packets++;
			qca->stats.rx_bytes += retcode;
			skb_put(qca->rx_skb, retcode);
			qca->rx_skb->protocol = eth_type_trans(
						qca->rx_skb, qca->rx_skb->dev);
			qca->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
			netif_rx_ni(qca->rx_skb);
			qca->rx_skb = netdev_alloc_skb(qca->net_dev,
						       qca->net_dev->mtu +
						       VLAN_ETH_HLEN);
			if (!qca->rx_skb) {
				qca->stats.rx_errors++;
				break;
			}
		}
	}
}

void
qca_tty_wakeup(struct tty_struct *tty)
{
	struct qcauart *qca = tty->disc_data;
	int written;

	if (qca->tx_skb->len == 0) {
		dev_kfree_skb(qca->tx_skb);
		qca->tx_skb = NULL;
		clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		netdev_dbg(qca->net_dev, "%s: exit early\n", __func__);
		qca->stats.tx_packets++;
		if (netif_queue_stopped(qca->net_dev))
			netif_wake_queue(qca->net_dev);
		return;
	}

	written = tty->ops->write(qca->tty, qca->tx_skb->data,
				  qca->tx_skb->len);
	qca->stats.tx_bytes += written;
	skb_pull(qca->tx_skb, written);
}

int
qca_tty_open(struct tty_struct *tty)
{
	struct qcauart *qca = tty->disc_data;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!tty->ops->write)
		return -EOPNOTSUPP;

	qca->tty = tty;
	netif_carrier_on(qca->net_dev);

	return 0;
}

void
qca_tty_close(struct tty_struct *tty)
{
	struct qcauart *qca = tty->disc_data;

	if (!qca)
		return;

	netif_carrier_off(qca->net_dev);
}

static struct tty_ldisc_ops qca_ldisc = {
	.owner  = THIS_MODULE,
	.magic	= TTY_LDISC_MAGIC,
	.name	= "qca",
	.open	= qca_tty_open,
	.close	= qca_tty_close,
	.ioctl	= n_tty_ioctl_helper,
	.receive_buf = qca_tty_receive,
	.write_wakeup = qca_tty_wakeup,
};

int
qcauart_netdev_open(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	qca->tx_skb = NULL;
	qcafrm_fsm_init(&qca->frm_handle);
	netif_start_queue(qca->net_dev);

	return 0;
}

int
qcauart_netdev_close(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	netif_stop_queue(dev);

	if (qca->tx_skb) {
		dev_kfree_skb(qca->tx_skb);
		qca->tx_skb = NULL;
	}

	return 0;
}

netdev_tx_t
qcauart_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	u32 frame_len;
	u8 *ptmp;
	struct qcauart *qca = netdev_priv(dev);
	struct sk_buff *tskb;
	u8 pad_len = 0;
	int written;

	if (skb->len < QCAFRM_ETHMINLEN)
		pad_len = QCAFRM_ETHMINLEN - skb->len;

	if (skb_headroom(skb) < QCAFRM_HEADER_LEN ||
	    skb_tailroom(skb) < QCAFRM_FOOTER_LEN + pad_len) {
		tskb = skb_copy_expand(skb, QCAFRM_HEADER_LEN,
				       QCAFRM_FOOTER_LEN + pad_len, GFP_ATOMIC);
		if (!tskb) {
			netdev_dbg(qca->net_dev, "%s: could not allocate tx_buff\n",
				   __func__);
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

	netif_stop_queue(qca->net_dev);

	set_bit(TTY_DO_WRITE_WAKEUP, &qca->tty->flags);
	written = qca->tty->ops->write(qca->tty, skb->data, skb->len);
	qca->stats.tx_bytes += written;
	skb_pull(skb, written);

	qca->tx_skb = skb;
	netif_trans_update(dev);

	return NETDEV_TX_OK;
}

void
qcauart_netdev_tx_timeout(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	netdev_info(qca->net_dev, "Transmit timeout at %ld, latency %ld\n",
		    jiffies, jiffies - dev_trans_start(dev));
	qca->stats.tx_errors++;
	qca->stats.tx_dropped++;

	clear_bit(TTY_DO_WRITE_WAKEUP, &qca->tty->flags);

	if (qca->tx_skb) {
		dev_kfree_skb(qca->tx_skb);
		qca->tx_skb = NULL;
	}
}

static int
qcauart_netdev_init(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	/* Finish setting up the device info. */
	dev->mtu = QCAUART_MTU;
	dev->type = ARPHRD_ETHER;

	qca->rx_skb = netdev_alloc_skb(qca->net_dev,
				       qca->net_dev->mtu + VLAN_ETH_HLEN);
	if (!qca->rx_skb)
		return -ENOMEM;

	return 0;
}

static void
qcauart_netdev_uninit(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	if (qca->rx_skb)
		dev_kfree_skb(qca->rx_skb);
}

static const struct net_device_ops qcauart_netdev_ops = {
	.ndo_init = qcauart_netdev_init,
	.ndo_uninit = qcauart_netdev_uninit,
	.ndo_open = qcauart_netdev_open,
	.ndo_stop = qcauart_netdev_close,
	.ndo_start_xmit = qcauart_netdev_xmit,
	.ndo_change_mtu = qcacmn_netdev_change_mtu,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_tx_timeout = qcauart_netdev_tx_timeout,
	.ndo_validate_addr = eth_validate_addr,
};

static void
qcauart_netdev_setup(struct net_device *dev)
{
	struct qcauart *qca = NULL;

	dev->netdev_ops = &qcauart_netdev_ops
	dev->watchdog_timeo = QCAUART_TX_TIMEOUT;
	dev->flags = IFF_MULTICAST;
	dev->tx_queue_len = 100;

	qca = netdev_priv(dev);
	memset(qca, 0, sizeof(struct qcauart));
}

static const struct of_device_id qca_uart_of_match[] = {
	{ .compatible = "qca,qca7000-uart" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, qca_uart_of_match);

static int
qca_uart_probe(struct platform_device *pdev)
{
	struct qcauart *qca = NULL;
	struct net_device *qcauart_devs = NULL;
	const char *mac;
	int ret;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Missing device tree\n");
		return -EINVAL;
	}

	ret = tty_register_ldisc(N_QCA, &qca_ldisc);
	if (ret) {
		dev_err(&pdev->dev, "Can't register line discipline (ret = %d)\n",
			ret);
		return ret;
	}

	dev_info(&pdev->dev, "ver=%s\n", QCAUART_DRV_VERSION);

	qcauart_devs = alloc_etherdev(sizeof(struct qcauart));
	if (!qcauart_devs)
		return -ENOMEM;

	qcauart_netdev_setup(qcauart_devs);

	qca = netdev_priv(qcauart_devs);
	if (!qca) {
		dev_err(&pdev->dev, "Fail to retrieve private structure\n");
		free_netdev(qcauart_devs);
		return -ENOMEM;
	}
	SET_NETDEV_DEV(qcauart_devs, &pdev->dev);
	qca->net_dev = qcauart_devs;

	mac = of_get_mac_address(pdev->dev.of_node);

	if (mac)
		ether_addr_copy(qca->net_dev->dev_addr, mac);

	if (!is_valid_ether_addr(qca->net_dev->dev_addr)) {
		eth_hw_addr_random(qca->net_dev);
		dev_info(&pdev->dev, "Using random MAC address: %pM\n",
			 qca->net_dev->dev_addr);
	}

	netif_carrier_off(qca->net_dev);

	if (register_netdev(qcauart_devs)) {
		dev_err(&pdev->dev, "Unable to register net device %s\n",
			qcauart_devs->name);
		free_netdev(qcauart_devs);
		return -EFAULT;
	}

	return 0;
}

static int
qca_uart_remove(struct platform_device *pdev)
{
	struct qcauart *qca = platform_get_drvdata(pdev);
	int ret;

	unregister_netdev(qca->net_dev);
	ret = tty_unregister_ldisc(N_QCA);
	if (ret)
		dev_err(&pdev->dev, "Can't unregister line discipline\n");

	free_netdev(qca->net_dev);

	return 0;
}

static struct platform_driver qca_uart_driver = {
	.driver	= {
		.name	= QCAUART_DRV_NAME,
		.of_match_table = qca_uart_of_match,
	},
	.probe = qca_uart_probe,
	.remove = qca_uart_remove,
};
module_platform_driver(qca_uart_driver);

MODULE_DESCRIPTION("Qualcomm Atheros UART Driver");
MODULE_AUTHOR("Qualcomm Atheros Communications");
MODULE_AUTHOR("Stefan Wahren <stefan.wahren@i2se.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(QCAUART_DRV_VERSION);
MODULE_ALIAS_LDISC(N_QCA);
