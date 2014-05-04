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

/*   This file contains debugging routines for use in the QCA7K driver.
 */

#include <linux/debugfs.h>
#include <linux/ethtool.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "qca_7k.h"
#include "qca_spi.h"

/* The order of these strings must match the order of the fields in
 * struct qcaspi_stats
 * See qca_spi.h
 */
static const char qcaspi_gstrings_stats[][ETH_GSTRING_LEN] = {
	"Triggered resets",
	"Device resets",
	"Reset timeouts",
	"Read errors",
	"Write errors",
	"Read buffer errors",
	"Write buffer errors",
	"Out of memory",
	"Write buffer misses",
	"Transmit queue full",
	"SPI errors",
};

#ifdef CONFIG_DEBUG_FS

/*   Dumps the contents of all SPI slave registers.        */
static int
qcaspi_regs_dump(struct seq_file *s, void *what)
{
	struct reg {
		char *name;
		u32 address;
	};

	static struct reg regs[] = {
		{ "SPI_REG_BFR_SIZE", SPI_REG_BFR_SIZE },
		{ "SPI_REG_WRBUF_SPC_AVA", SPI_REG_WRBUF_SPC_AVA },
		{ "SPI_REG_RDBUF_BYTE_AVA", SPI_REG_RDBUF_BYTE_AVA },
		{ "SPI_REG_SPI_CONFIG", SPI_REG_SPI_CONFIG },
		{ "SPI_REG_SPI_STATUS", SPI_REG_SPI_STATUS },
		{ "SPI_REG_INTR_CAUSE", SPI_REG_INTR_CAUSE },
		{ "SPI_REG_INTR_ENABLE", SPI_REG_INTR_ENABLE },
		{ "SPI_REG_RDBUF_WATERMARK", SPI_REG_RDBUF_WATERMARK },
		{ "SPI_REG_WRBUF_WATERMARK", SPI_REG_WRBUF_WATERMARK },
		{ "SPI_REG_SIGNATURE", SPI_REG_SIGNATURE },
		{ "SPI_REG_ACTION_CTRL", SPI_REG_ACTION_CTRL }
	};

	struct qcaspi *qca = s->private;
	int i;

	for (i = 0; i < (sizeof(regs) / sizeof(struct reg)); i++) {
		u16 value;

		qcaspi_read_register(qca, regs[i].address, &value);
		seq_printf(s, "%-25s(0x%04x): 0x%04x\n",
			regs[i].name, regs[i].address, value);
	}

	return 0;
}

static int
qcaspi_stats_show(struct seq_file *s, void *what)
{
	struct qcaspi *qca = s->private;

	seq_printf(s, "Triggered resets    : %llu\n", qca->stats.trig_reset);
	seq_printf(s, "Device resets       : %llu\n", qca->stats.device_reset);
	seq_printf(s, "Reset timeouts      : %llu\n", qca->stats.reset_timeout);
	seq_printf(s, "Read errors         : %llu\n", qca->stats.read_err);
	seq_printf(s, "Write errors        : %llu\n", qca->stats.write_err);
	seq_printf(s, "Read buffer errors  : %llu\n", qca->stats.read_buf_err);
	seq_printf(s, "Write buffer errors : %llu\n", qca->stats.write_buf_err);
	seq_printf(s, "Out of memory       : %llu\n", qca->stats.out_of_mem);
	seq_printf(s, "Write buffer misses : %llu\n", qca->stats.write_buf_miss);
	seq_printf(s, "Transmit queue full : %llu\n", qca->stats.queue_full);
	seq_printf(s, "SPI errors          : %llu\n", qca->stats.spi_err);

	return 0;
}

static int
qcaspi_info_show(struct seq_file *s, void *what)
{
	struct qcaspi *qca = s->private;

	seq_printf(s, "RX buffer size   : %lu\n",
		(unsigned long) qca->buffer_size);

	seq_puts(s, "TX queue state   : ");

	if (qca->txq.skb[qca->txq.head])
		seq_puts(s, "empty");
	else if (qca->txq.skb[qca->txq.tail])
		seq_puts(s, "full");
	else
		seq_puts(s, "in use");

	seq_puts(s, "\n");

	seq_printf(s, "Sync state       : %u (",
		(unsigned int) qca->sync);
	switch (qca->sync) {
	case QCASPI_SYNC_UNKNOWN:
		seq_puts(s, "QCASPI_SYNC_UNKNOWN");
		break;
	case QCASPI_SYNC_RESET:
		seq_puts(s, "QCASPI_SYNC_RESET");
		break;
	case QCASPI_SYNC_READY:
		seq_puts(s, "QCASPI_SYNC_READY");
		break;
	default:
		seq_puts(s, "INVALID");
		break;
	}
	seq_puts(s, ")\n");

	seq_printf(s, "IRQ              : %d\n",
		qca->irq);
	seq_printf(s, "INTR REQ         : %u\n",
		qca->intr_req);
	seq_printf(s, "INTR SVC         : %u\n",
		qca->intr_svc);
	seq_printf(s, "INTR GPIO        : %d\n",
		gpio_get_value(qca->intr_gpio));

	seq_printf(s, "SPI max speed    : %lu\n",
		(unsigned long) qca->spi_dev->max_speed_hz);
	seq_printf(s, "SPI mode         : %x\n",
		qca->spi_dev->mode);
	seq_printf(s, "SPI chip select  : %u\n",
		(unsigned int) qca->spi_dev->chip_select);
	seq_printf(s, "SPI legacy mode  : %u\n",
		(unsigned int) qca->legacy_mode);
	seq_printf(s, "SPI burst length : %u\n",
		(unsigned int) qca->burst_len);

	return 0;
}

static int
qcaspi_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, qcaspi_regs_dump, inode->i_private);
}

static int
qcaspi_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, qcaspi_stats_show, inode->i_private);
}

static int
qcaspi_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, qcaspi_info_show, inode->i_private);
}

static const struct file_operations qcaspi_regs_ops = {
	.open = qcaspi_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations qcaspi_stats_ops = {
	.open = qcaspi_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations qcaspi_info_ops = {
	.open = qcaspi_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void
qcaspi_init_device_debugfs(struct qcaspi *qca)
{
	struct dentry *device_root;

	device_root = debugfs_create_dir(dev_name(&qca->net_dev->dev), NULL);
	qca->device_root = device_root;

	if (IS_ERR(device_root) || !device_root) {
		pr_warn("failed to create debugfs directory for %s\n",
			dev_name(&qca->net_dev->dev));
		return;
	}
	debugfs_create_file("regs", S_IFREG | S_IRUGO, device_root, qca,
			&qcaspi_regs_ops);

	debugfs_create_file("stats", S_IFREG | S_IRUGO, device_root, qca,
			&qcaspi_stats_ops);

	debugfs_create_file("info", S_IFREG | S_IRUGO, device_root, qca,
			&qcaspi_info_ops);
}

void
qcaspi_remove_device_debugfs(struct qcaspi *qca)
{
	debugfs_remove_recursive(qca->device_root);
}

#else /* CONFIG_DEBUG_FS */

void
qcaspi_init_device_debugfs(struct qcaspi *qca)
{
}

void
qcaspi_remove_device_debugfs(struct qcaspi *qca)
{
}

#endif

static void
qcaspi_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *p)
{
	struct qcaspi *qca = netdev_priv(dev);

	strlcpy(p->driver, QCASPI_DRV_NAME, sizeof(p->driver));
	strlcpy(p->version, QCASPI_DRV_VERSION, sizeof(p->version));
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

static void
qcaspi_get_ethtool_stats(struct net_device *dev, struct ethtool_stats *estats, u64 *data)
{
	struct qcaspi *qca = netdev_priv(dev);
	struct qcaspi_stats *st = &qca->stats;

	memcpy(data, st, ARRAY_SIZE(qcaspi_gstrings_stats) * sizeof(u64));
}

static void
qcaspi_get_strings(struct net_device *dev, u32 stringset, u8 *buf)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buf, &qcaspi_gstrings_stats, sizeof(qcaspi_gstrings_stats));
		break;
	default:
		WARN_ON(1);
		break;
	}
}

static int
qcaspi_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(qcaspi_gstrings_stats);
	default:
		return -EINVAL;
	}
}

static const struct ethtool_ops qcaspi_ethtool_ops = {
	.get_drvinfo = qcaspi_get_drvinfo,
	.get_link = ethtool_op_get_link,
	.get_settings = qcaspi_get_settings,
	.get_ethtool_stats = qcaspi_get_ethtool_stats,
	.get_strings = qcaspi_get_strings,
	.get_sset_count = qcaspi_get_sset_count,
};

void qcaspi_set_ethtool_ops(struct net_device *dev)
{
	SET_ETHTOOL_OPS(dev, &qcaspi_ethtool_ops);
}
