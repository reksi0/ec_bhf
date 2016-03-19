 /*
 * drivers/net/ethernet/ec_bhf.c
 *
 * Copyright (C) 2014 Darek Marcinkiewicz <reksio@newterm.pl>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* This is a driver for EtherCAT master module present on CCAT FPGA.
 * Those can be found on Bechhoff CX50xx industrial PCs.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif
#include <linux/platform_device.h>
#include <linux/init.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/stat.h>

#define TIMER_INTERVAL_NSEC	2000000

#define INFO_BLOCK_SIZE		0x10
#define INFO_BLOCK_TYPE		0x0
#define INFO_BLOCK_REV		0x2
#define INFO_BLOCK_BLK_CNT	0x4
#define INFO_BLOCK_TX_CHAN	0x4
#define INFO_BLOCK_RX_CHAN	0x5
#define INFO_BLOCK_OFFSET	0x8

#define EC_MII_OFFSET		0x4
#define EC_FIFO_OFFSET		0x8
#define EC_MAC_OFFSET		0xc
#define EC_RX_WIN_OFFSET	0x10
#define EC_TX_MEM_OFSSET	0x14

#define MAC_FRAME_ERR_CNT	0x0
#define MAC_RX_ERR_CNT		0x1
#define MAC_CRC_ERR_CNT		0x2
#define MAC_LNK_LST_ERR_CNT	0x3
#define MAC_TX_FRAME_CNT	0x10
#define MAC_RX_FRAME_CNT	0x14
#define MAC_TX_FIFO_LVL		0x20
#define MAC_DROPPED_FRMS	0x28
#define MAC_CONNECTED_CCAT_FLAG	0x78

#define MII_MAC_ADDR		0x8
#define MII_MAC_FILT_FLAG	0xe
#define MII_LINK_STATUS		0xf

#define FIFO_TX_REG		0x0
#define FIFO_TX_RESET		0x8
#define FIFO_RX_REG		0x10
#define FIFO_RX_ADDR_VALID	(1u << 31)
#define FIFO_RX_RESET		0x18

#define DMA_CHAN_OFFSET		0x1000
#define DMA_CHAN_SIZE		0x8

#define DMA_WINDOW_SIZE_MASK	0xfffffffc

#define ECAT_MASTER_DMA		0x14
#define ECAT_MASTER_NODMA	0x03

#define TX_MEM_FLAGS		0x2
#define TX_MEM_PKT_OFFSET	0x10

#define RX_MEM_PKT_OFFSET	0x10

#ifdef CONFIG_PCI
static struct pci_device_id ids[] = {
	{ PCI_DEVICE(0x15ec, 0x5000), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);
#endif

struct rx_dma_header {
#define RXHDR_NEXT_ADDR_MASK	0xffffffu
#define RXHDR_NEXT_VALID	(1u << 31)
	__le32 next;
#define RXHDR_NEXT_RECV_FLAG	0x1
	__le32 recv;
#define RXHDR_LEN_MASK		0xfffu
	__le16 len;
	__le16 port;
	__le32 reserved;
	u8 timestamp[8];
} __packed;

#define PKT_PAYLOAD_SIZE	0x7e8
struct rx_dma_desc {
	struct rx_dma_header header;
	u8 data[PKT_PAYLOAD_SIZE];
} __packed;

struct tx_dma_header {
	__le16 len;
#define TX_HDR_PORT_0		0x1
#define TX_HDR_PORT_1		0x2
	u8 port;
	u8 ts_enable;
#define TX_HDR_SENT		0x1
	__le32 sent;
	u8 timestamp[8];
} __packed;

struct tx_dma_desc {
	struct tx_dma_header header;
	u8 data[PKT_PAYLOAD_SIZE];
} __packed;

#define FIFO_SIZE		64

static long polling_frequency = TIMER_INTERVAL_NSEC;

struct bhf_dma {
	u8 *buf;
	size_t len;
	dma_addr_t buf_phys;

	u8 *alloc;
	size_t alloc_len;
	dma_addr_t alloc_phys;
};

struct ec_bhf_priv {
	struct net_device *net_dev;
	struct device *dev;

	void __iomem *io;

	int type;

	struct hrtimer hrtimer;

	void __iomem *ec_io;
	void __iomem *fifo_io;
	void __iomem *mii_io;
	void __iomem *mac_io;

	u64 stat_rx_bytes;
	u64 stat_tx_bytes;

	void (*process_timer)(struct ec_bhf_priv *);
};

struct ec_bhf_priv_dma {
	struct ec_bhf_priv base;

	void __iomem *dma_io;

	int tx_dma_chan;
	int rx_dma_chan;

	struct bhf_dma rx_buf;
	struct rx_dma_desc *rx_descs;
	int rx_dnext;
	int rx_dcount;

	struct bhf_dma tx_buf;
	struct tx_dma_desc *tx_descs;
	int tx_dcount;
	int tx_dnext;
};

struct ec_bhf_priv_no_dma {
	struct ec_bhf_priv base;

	void __iomem *tx_io;
	void __iomem *rx_io;
	u16 tx_size;
	u32 tx_npos;
};

#define PRIV_TO_DEV(priv) (priv->base.dev)

static void ec_bhf_reset(struct ec_bhf_priv *priv)
{
	iowrite8(0, priv->mac_io + MAC_FRAME_ERR_CNT);
	iowrite8(0, priv->mac_io + MAC_RX_ERR_CNT);
	iowrite8(0, priv->mac_io + MAC_CRC_ERR_CNT);
	iowrite8(0, priv->mac_io + MAC_LNK_LST_ERR_CNT);
	iowrite32(0, priv->mac_io + MAC_TX_FRAME_CNT);
	iowrite32(0, priv->mac_io + MAC_RX_FRAME_CNT);
	iowrite8(0, priv->mac_io + MAC_DROPPED_FRMS);

	iowrite8(0, priv->fifo_io + FIFO_TX_RESET);
	iowrite8(0, priv->fifo_io + FIFO_RX_RESET);

	iowrite8(0, priv->mac_io + MAC_TX_FIFO_LVL);
}

#ifdef CONFIG_PCI
static void ec_bhf_send_packet(struct ec_bhf_priv_dma *priv, struct tx_dma_desc *desc)
{
	u32 len = le16_to_cpu(desc->header.len) + sizeof(desc->header);
	u32 addr = (u8 *)desc - priv->tx_buf.buf;

	iowrite32((ALIGN(len, 8) << 24) | addr, priv->base.fifo_io + FIFO_TX_REG);
}

static int ec_bhf_desc_sent(struct tx_dma_desc *desc)
{
	return le32_to_cpu(desc->header.sent) & TX_HDR_SENT;
}

static void ec_bhf_process_tx_dma(struct ec_bhf_priv_dma *priv)
{

	if (unlikely(netif_queue_stopped(priv->base.net_dev))) {
		/* Make sure that we perceive changes to tx_dnext. */
		smp_rmb();

		if (ec_bhf_desc_sent(&priv->tx_descs[priv->tx_dnext]))
			netif_wake_queue(priv->base.net_dev);
	}
}

static int ec_bhf_dma_pkt_received(struct rx_dma_desc *desc)
{
	return le32_to_cpu(desc->header.recv) & RXHDR_NEXT_RECV_FLAG;
}

static void ec_bhf_add_rx_desc(struct ec_bhf_priv_dma *priv,
			       struct rx_dma_desc *desc)
{
	iowrite32(FIFO_RX_ADDR_VALID | ((u8 *)(desc) - priv->rx_buf.buf),
		  priv->base.fifo_io + FIFO_RX_REG);
}
#endif

static void ec_bhf_process_rx_no_dma(struct ec_bhf_priv_no_dma *priv)
{
	u16 pkt_size;

	pkt_size = le16_to_cpu(ioread16(priv->rx_io));
	while (pkt_size) {
		struct sk_buff *skb;
		dev_dbg(PRIV_TO_DEV(priv), "Got packet of size: %d\n, timestamp:%d%d",
			pkt_size, ioread32(priv->rx_io + 8), ioread32(priv->rx_io + 12));
		skb = netdev_alloc_skb(priv->base.net_dev, pkt_size);
		if (skb) {
			memcpy_fromio(skb_put(skb, pkt_size),
				      priv->rx_io + RX_MEM_PKT_OFFSET,
				      pkt_size - RX_MEM_PKT_OFFSET);

			skb->protocol = eth_type_trans(skb, priv->base.net_dev);
			priv->base.stat_rx_bytes += pkt_size;

			netif_rx(skb);
		} else {
			dev_err_ratelimited(PRIV_TO_DEV(priv),
					    "Couldn't allocate a skb_buff for a packet of size %u\n",
					    pkt_size);
		}

		/* ACK - none of this actually work*/
		//iowrite16(pkt_size, priv->rx_io);
		//iowrite16(0, priv->rx_io);
		iowrite16(0, priv->rx_io);
			
#if 0
		pkt_size = le16_to_cpu(ioread16(priv->rx_io));
#else
		break;
#endif
	}

}

#ifdef CONFIG_PCI
static void ec_bhf_process_rx_dma(struct ec_bhf_priv_dma *priv)
{
	struct rx_dma_desc *desc = &priv->rx_descs[priv->rx_dnext];

	while (ec_bhf_dma_pkt_received(desc)) {
		int pkt_size = (le16_to_cpu(desc->header.len) &
			       RXHDR_LEN_MASK) - sizeof(struct rx_dma_header) - 4;
		u8 *data = desc->data;
		struct sk_buff *skb;

		skb = netdev_alloc_skb(priv->base.net_dev, pkt_size);
		if (skb) {
			memcpy(skb_put(skb, pkt_size), data, pkt_size);
			skb->protocol = eth_type_trans(skb, priv->base.net_dev);
			priv->base.stat_rx_bytes += pkt_size;

			netif_rx(skb);
		} else {
			dev_err_ratelimited(PRIV_TO_DEV(priv),
					    "Couldn't allocate a skb_buff for a packet of size %u\n",
					    pkt_size);
		}

		desc->header.recv = 0;

		ec_bhf_add_rx_desc(priv, desc);

		priv->rx_dnext = (priv->rx_dnext + 1) % priv->rx_dcount;
		desc = &priv->rx_descs[priv->rx_dnext];
	}
}

static void ec_bhf_process_timer_dma(struct ec_bhf_priv *_priv)
{
	struct ec_bhf_priv_dma *priv = (struct ec_bhf_priv_dma *) _priv;

	ec_bhf_process_rx_dma(priv);
	ec_bhf_process_tx_dma(priv);
}
#endif

static void ec_bhf_process_timer_no_dma(struct ec_bhf_priv *_priv)
{
	struct ec_bhf_priv_no_dma *priv = (struct ec_bhf_priv_no_dma *) _priv;
	int fifo_level;

	ec_bhf_process_rx_no_dma(priv);

	fifo_level = ioread8(priv->base.mac_io + MAC_TX_FIFO_LVL);
	if (fifo_level > 0)
		dev_info(PRIV_TO_DEV(priv), "TX FIFO level %d\n", fifo_level);
}

static enum hrtimer_restart ec_bhf_timer_fun(struct hrtimer *timer)
{
	struct ec_bhf_priv *priv = container_of(timer, struct ec_bhf_priv,
						hrtimer);
	priv->process_timer(priv);

	if (!netif_running(priv->net_dev))
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ktime_set(0, polling_frequency));
	return HRTIMER_RESTART;
}

static int
ec_bhf_probe_ec_type(struct device *dev, void __iomem *io,
		     int *ec_type, void __iomem **ec_info)
{
	int block_count = ioread8(io + INFO_BLOCK_BLK_CNT);
	int type, i, err = -ENODEV;

	for (i = 0; i < block_count; i++) {
		type = ioread16(io + i * INFO_BLOCK_SIZE + INFO_BLOCK_TYPE);
		if (type == ECAT_MASTER_DMA || type == ECAT_MASTER_NODMA) {
			*ec_type = type;
			*ec_info = io + i * INFO_BLOCK_SIZE;
			err = 0;
		}
		dev_err(dev, "Block type:%x", (int)type);
	}

	if (err)
		dev_err(dev, "EtherCAT master block not found\n");
	return err;
}

static void ec_bhf_setup_priv(struct ec_bhf_priv *priv, void __iomem *ec_info)
{
	pr_err("%pK %pK\n", priv->io, ec_info);
	priv->ec_io = priv->io + ioread32(ec_info + INFO_BLOCK_OFFSET);
	pr_err("ec_io %pK\n", priv->ec_io);
	priv->mii_io = priv->ec_io + ioread32(priv->ec_io + EC_MII_OFFSET);
	pr_err("mii_io %pK\n", priv->mii_io);
	priv->fifo_io = priv->ec_io + ioread32(priv->ec_io + EC_FIFO_OFFSET);
	pr_err("fifo_io %pK\n", priv->fifo_io);
	priv->mac_io = priv->ec_io + ioread32(priv->ec_io + EC_MAC_OFFSET);
	pr_err("mac_io %pK\n", priv->mac_io);
}

static netdev_tx_t ec_bhf_start_xmit_no_dma(struct sk_buff *skb,
					    struct net_device *net_dev)
{
	struct ec_bhf_priv_no_dma *priv = netdev_priv(net_dev);
	unsigned pkt_len;

	if (skb_linearize(skb)) {
		dev_err(PRIV_TO_DEV(priv),
			 "Failed to lineralize skb, dropping packet");
		goto out;
	}

	pkt_len = skb_headlen(skb) + TX_MEM_PKT_OFFSET;

	if (pkt_len + priv->tx_npos > priv->tx_size)
		priv->tx_npos = 0;
	
	memcpy_toio(priv->tx_io + priv->tx_npos + TX_MEM_PKT_OFFSET,
		    skb->data, skb_headlen(skb));

	iowrite16(0, priv->tx_io + priv->tx_npos + TX_MEM_FLAGS);
	iowrite16(cpu_to_le16(skb_headlen(skb)), priv->tx_io + priv->tx_npos);

	iowrite32(priv->tx_npos, priv->base.fifo_io + FIFO_TX_REG);

	priv->tx_npos += ALIGN(pkt_len, 16);
out:
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

#ifdef CONFIG_PCI
static netdev_tx_t ec_bhf_start_xmit_dma(struct sk_buff *skb,
					 struct net_device *net_dev)
{
	struct ec_bhf_priv_dma *priv = netdev_priv(net_dev);
	struct tx_dma_desc *desc;
	unsigned len;

	desc = &priv->tx_descs[priv->tx_dnext];

	skb_copy_and_csum_dev(skb, desc->data);
	len = skb->len;

	memset(&desc->header, 0, sizeof(desc->header));
	desc->header.len = cpu_to_le16(len);
	desc->header.port = TX_HDR_PORT_0;

	ec_bhf_send_packet(priv, desc);

	priv->tx_dnext = (priv->tx_dnext + 1) % priv->tx_dcount;

	if (!ec_bhf_desc_sent(&priv->tx_descs[priv->tx_dnext])) {
		/* Make sure that updates to tx_dnext are perceived
		 * by timer routine.
		 */
		smp_wmb();

		netif_stop_queue(net_dev);
	}

	priv->base.stat_tx_bytes += len;

	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static int ec_bhf_alloc_dma_mem(struct ec_bhf_priv_dma *priv,
				struct bhf_dma *buf,
				int channel,
				int size)
{
	int offset = channel * DMA_CHAN_SIZE + DMA_CHAN_OFFSET;
	struct device *dev = PRIV_TO_DEV(priv);
	u32 mask;

	iowrite32(0xffffffff, priv->dma_io + offset);

	mask = ioread32(priv->dma_io + offset);
	mask &= DMA_WINDOW_SIZE_MASK;

	/* We want to allocate a chunk of memory that is:
	 * - aligned to the mask we just read
	 * - is of size 2^mask bytes (at most)
	 * In order to ensure that we will allocate buffer of
	 * 2 * 2^mask bytes.
	 */
	buf->len = min_t(int, ~mask + 1, size);
	buf->alloc_len = 2 * buf->len;

	buf->alloc = dma_alloc_coherent(dev, buf->alloc_len, &buf->alloc_phys,
					GFP_KERNEL);
	if (buf->alloc == NULL) {
		dev_err(dev, "Failed to allocate buffer\n");
		return -ENOMEM;
	}

	buf->buf_phys = (buf->alloc_phys + buf->len) & mask;
	buf->buf = buf->alloc + (buf->buf_phys - buf->alloc_phys);

	iowrite32(0, priv->dma_io + offset + 4);
	iowrite32(buf->buf_phys, priv->dma_io + offset);

	return 0;
}

static void ec_bhf_setup_tx_descs(struct ec_bhf_priv_dma *priv)
{
	int i = 0;

	priv->tx_dcount = priv->tx_buf.len / sizeof(struct tx_dma_desc);
	priv->tx_descs = (struct tx_dma_desc *)priv->tx_buf.buf;
	priv->tx_dnext = 0;

	for (i = 0; i < priv->tx_dcount; i++)
		priv->tx_descs[i].header.sent = cpu_to_le32(TX_HDR_SENT);
}

static void ec_bhf_setup_rx_descs(struct ec_bhf_priv_dma *priv)
{
	int i;

	priv->rx_dcount = priv->rx_buf.len / sizeof(struct rx_dma_desc);
	priv->rx_descs = (struct rx_dma_desc *)priv->rx_buf.buf;
	priv->rx_dnext = 0;

	for (i = 0; i < priv->rx_dcount; i++) {
		struct rx_dma_desc *desc = &priv->rx_descs[i];
		u32 next;

		if (i != priv->rx_dcount - 1)
			next = (u8 *)(desc + 1) - priv->rx_buf.buf;
		else
			next = 0;
		next |= RXHDR_NEXT_VALID;
		desc->header.next = cpu_to_le32(next);
		desc->header.recv = 0;
		ec_bhf_add_rx_desc(priv, desc);
	}
}
#endif

static void ec_bhf_start(struct ec_bhf_priv *priv)
{
	iowrite8(0, priv->mii_io + MII_MAC_FILT_FLAG);

	netif_start_queue(priv->net_dev);

	hrtimer_init(&priv->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->hrtimer.function = ec_bhf_timer_fun;
	hrtimer_start(&priv->hrtimer, ktime_set(0, polling_frequency),
		      HRTIMER_MODE_REL);
}

static int ec_bhf_open_no_dma(struct net_device *net_dev)
{
	struct ec_bhf_priv *priv = netdev_priv(net_dev);

	ec_bhf_reset(priv);

	ec_bhf_start(priv);

	return 0;
}

#ifdef CONFIG_PCI
static int ec_bhf_open_dma(struct net_device *net_dev)
{
	struct ec_bhf_priv_dma *priv = netdev_priv(net_dev);
	struct device *dev = PRIV_TO_DEV(priv);
	int err = 0;

	ec_bhf_reset(&priv->base);

	err = ec_bhf_alloc_dma_mem(priv, &priv->rx_buf, priv->rx_dma_chan,
				   FIFO_SIZE * sizeof(struct rx_dma_desc));
	if (err) {
		dev_err(dev, "Failed to allocate rx buffer\n");
		goto out;
	}
	ec_bhf_setup_rx_descs(priv);

	err = ec_bhf_alloc_dma_mem(priv, &priv->tx_buf, priv->tx_dma_chan,
				   FIFO_SIZE * sizeof(struct tx_dma_desc));
	if (err) {
		dev_err(dev, "Failed to allocate tx buffer\n");
		goto error_rx_free;
	}
	ec_bhf_setup_tx_descs(priv);

	ec_bhf_start(&priv->base);

	return 0;

error_rx_free:
	dma_free_coherent(dev, priv->rx_buf.alloc_len, priv->rx_buf.alloc,
			  priv->rx_buf.alloc_len);
out:
	return err;
}
#endif

static int ec_bhf_stop(struct net_device *net_dev)
{
	struct ec_bhf_priv *priv = netdev_priv(net_dev);

	hrtimer_cancel(&priv->hrtimer);

	ec_bhf_reset(priv);

	netif_tx_disable(net_dev);

	return 0;
}

#ifdef CONFIG_PCI
static int ec_bhf_stop_dma(struct net_device *net_dev)
{
	struct ec_bhf_priv_dma *priv = netdev_priv(net_dev);
	struct device *dev = PRIV_TO_DEV(priv);

	ec_bhf_stop(net_dev);

	dma_free_coherent(dev, priv->tx_buf.alloc_len,
			  priv->tx_buf.alloc, priv->tx_buf.alloc_phys);
	dma_free_coherent(dev, priv->rx_buf.alloc_len,
			  priv->rx_buf.alloc, priv->rx_buf.alloc_phys);

	return 0;
}
#endif

static struct rtnl_link_stats64 *
ec_bhf_get_stats(struct net_device *net_dev,
		 struct rtnl_link_stats64 *stats)
{
	struct ec_bhf_priv *priv = netdev_priv(net_dev);

	stats->rx_errors = ioread8(priv->mac_io + MAC_RX_ERR_CNT) +
				ioread8(priv->mac_io + MAC_CRC_ERR_CNT) +
				ioread8(priv->mac_io + MAC_FRAME_ERR_CNT);
	stats->rx_packets = ioread32(priv->mac_io + MAC_RX_FRAME_CNT);
	stats->tx_packets = ioread32(priv->mac_io + MAC_TX_FRAME_CNT);
	stats->rx_dropped = ioread8(priv->mac_io + MAC_DROPPED_FRMS);

	stats->tx_bytes = priv->stat_tx_bytes;
	stats->rx_bytes = priv->stat_rx_bytes;

	return stats;
}

static const struct net_device_ops ec_bhf_netdev_no_dma_ops = {
	.ndo_start_xmit		= ec_bhf_start_xmit_no_dma,
	.ndo_open		= ec_bhf_open_no_dma,
	.ndo_stop		= ec_bhf_stop,
	.ndo_get_stats64	= ec_bhf_get_stats,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr
};

#ifdef CONFIG_PCI
static const struct net_device_ops ec_bhf_netdev_dma_ops = {
	.ndo_start_xmit		= ec_bhf_start_xmit_dma,
	.ndo_open		= ec_bhf_open_dma,
	.ndo_stop		= ec_bhf_stop_dma,
	.ndo_get_stats64	= ec_bhf_get_stats,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr
};

static int ec_bhf_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	void __iomem *dma_io = NULL, *io, *ec_info;
	struct net_device *net_dev;
	struct ec_bhf_priv *priv;
	int err = 0, ec_type;

	err = pci_enable_device(dev);
	if (err)
		return err;

	pci_set_master(dev);

	err = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&dev->dev,
			"Required dma mask not supported, failed to initialize device\n");
		err = -EIO;
		goto err_disable_dev;
	}

	err = pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&dev->dev,
			"Required dma mask not supported, failed to initialize device\n");
		goto err_disable_dev;
	}

	err = pci_request_regions(dev, "ec_bhf");
	if (err) {
		dev_err(&dev->dev, "Failed to request pci memory regions\n");
		goto err_disable_dev;
	}

	io = pci_iomap(dev, 0, 0);
	if (!io) {
		dev_err(&dev->dev, "Failed to map pci card memory bar 0");
		err = -EIO;
		goto err_release_regions;
	}

	err = ec_bhf_probe_ec_type(&dev->dev, io, &ec_type, &ec_info);
	if (err)
		goto err_unmap;

	net_dev = alloc_etherdev(ec_type == ECAT_MASTER_DMA ?
				 sizeof(struct ec_bhf_priv_dma) :
			         sizeof(struct ec_bhf_priv_no_dma));
	if (net_dev == NULL) {
		err = -ENOMEM;
		goto err_unmap;
	}

	pci_set_drvdata(dev, net_dev);
	SET_NETDEV_DEV(net_dev, &dev->dev);

	net_dev->flags |= IFF_NOARP;
	net_dev->features = 0;

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;
	priv->type = ec_type;
	priv->dev = dev;
	priv->io = io;

	ec_bhf_setup_priv(priv, ec_info);

	if (ec_type == ECAT_MASTER_DMA) {
		struct ec_bhf_priv_dma *priv = netdev_priv(net_dev);

		net_dev->netdev_ops = &ec_bhf_netdev_dma_ops;
		priv->base.process_timer = ec_bhf_process_timer_dma;

		dma_io = pci_iomap(dev, 2, 0);
		if (!dma_io) {
			dev_err(&dev->dev,
				"Failed to map pci card memory bar 2");
			err = -EIO;
			goto err_free_net_dev;
		}

		priv->dma_io = dma_io;
		priv->tx_dma_chan = ioread8(ec_info + INFO_BLOCK_TX_CHAN);
		priv->rx_dma_chan = ioread8(ec_info + INFO_BLOCK_RX_CHAN);

	} else {
		struct ec_bhf_priv_no_dma *priv = netdev_priv(net_dev);

		net_dev->netdev_ops = &ec_bhf_netdev_no_dma_ops;
		priv->base.process_timer = ec_bhf_process_timer_no_dma;

		priv->tx_io = priv->base.ec_io +
			      ioread32(priv->base.ec_io + EC_TX_MEM_OFSSET);
		priv->rx_io = priv->base.ec_io +
			      ioread32(priv->base.ec_io + EC_RX_WIN_OFFSET);
		priv->tx_size = ioread32(priv->base.ec_io + 0x04);
		priv->tx_npos = 0;

		dev_err(&dev->dev, "tx: %p\n", priv->tx_io);
		dev_err(&dev->dev, "rx: %p\n", priv->rx_io);
		dev_err(&dev->dev, "tx_size: %x\n", (u32)priv->tx_size);
	}

	memcpy_fromio(net_dev->dev_addr, priv->mii_io + MII_MAC_ADDR, 6);

	dev_err(&dev->dev, "%pM", net_dev->dev_addr);

	err = register_netdev(net_dev);
	if (err < 0)
		goto err_unmap_dma_io;

	return 0;

err_unmap_dma_io:
	if (dma_io)
		pci_iounmap(dev, dma_io);
err_free_net_dev:
	free_netdev(net_dev);
err_unmap:
	pci_iounmap(dev, io);
err_release_regions:
	pci_release_regions(dev);
err_disable_dev:
	pci_clear_master(dev);
	pci_disable_device(dev);

	return err;
}

static void ec_bhf_remove(struct pci_dev *dev)
{
	struct net_device *net_dev = pci_get_drvdata(dev);
	struct ec_bhf_priv *priv = netdev_priv(net_dev);

	unregister_netdev(net_dev);

	if (priv->type == ECAT_MASTER_DMA) {
		struct ec_bhf_priv_dma *dma_priv =
			(struct ec_bhf_priv_dma*) priv;
		pci_iounmap(dev, dma_priv->dma_io);
	}

	pci_iounmap(dev, priv->io);
	pci_release_regions(dev);
	pci_clear_master(dev);
	pci_disable_device(dev);

	free_netdev(net_dev);
}

static struct pci_driver pci_driver = {
	.name		= "ec_bhf",
	.id_table	= ids,
	.probe		= ec_bhf_probe,
	.remove		= ec_bhf_remove,
};

static int __init ec_bhf_init(void)
{
	return pci_register_driver(&pci_driver);
}

static void __exit ec_bhf_exit(void)
{
	pci_unregister_driver(&pci_driver);
}

module_init(ec_bhf_init);
module_exit(ec_bhf_exit);

#else
static const struct of_device_id ec_bhf_of_id[] = {
	{ .compatible = "bhf,emi-ccat", } ,
	{ }
};

MODULE_DEVICE_TABLE(of, ec_bhf_of_id);

static int ec_bhf_probe(struct platform_device *pdev)
{
	struct ec_bhf_priv_no_dma *priv;
	struct net_device *net_dev;
	void __iomem *io, *ec_info;
	int err = 0, ec_type;

	if (!request_mem_region(0xf0000000, 0x02000000, pdev->name)) {
		dev_err(&pdev->dev, "Failed to request memory region.\n");
		return -EBUSY;
	}

	io = ioremap(0xf0000000, 0x02000000);
	if (io == NULL) {
		dev_err(&pdev->dev, "Failed to remap memory.\n");
		err = -EBUSY;
		goto err_release_mem;
	}

	err = ec_bhf_probe_ec_type(&pdev->dev, io, &ec_type, &ec_info);
	if (err) {
		dev_err(&pdev->dev, "Failed to find EtherCAT master.\n");
		goto err_unmap;
	}

	if (ec_type != ECAT_MASTER_NODMA) {
		dev_err(&pdev->dev, "Only EtherCAT master with no dma supported.\n");
		err = -ENOTSUPP;
		goto err_unmap;
	}

	net_dev = alloc_etherdev(sizeof(struct ec_bhf_priv_no_dma));
	if (net_dev == NULL) {
		err = -ENOMEM;
		goto err_unmap;
	}

	priv = netdev_priv(net_dev);
	priv->base.net_dev = net_dev;
	priv->base.dev = &pdev->dev;
	priv->base.type = ec_type;
	priv->base.io = io;

	ec_bhf_setup_priv(&priv->base, ec_info);

	priv = netdev_priv(net_dev);

	priv->base.io = io;

	net_dev->netdev_ops = &ec_bhf_netdev_no_dma_ops;
	priv->base.process_timer = ec_bhf_process_timer_no_dma;

	priv->tx_io = priv->base.ec_io +
		      ioread32(priv->base.ec_io + EC_TX_MEM_OFSSET);
	priv->rx_io = priv->base.ec_io +
		      ioread32(priv->base.ec_io + EC_RX_WIN_OFFSET);
	priv->tx_size = ioread32(priv->base.ec_io + 0x04);
	priv->tx_npos = 0;

	dev_err(&pdev->dev, "tx: %p\n", priv->tx_io);
	dev_err(&pdev->dev, "rx: %p\n", priv->rx_io);
	dev_err(&pdev->dev, "tx_size: %x\n", (u32)priv->tx_size);

	memcpy_fromio(net_dev->dev_addr, priv->base.mii_io + MII_MAC_ADDR, 6);

	err = register_netdev(net_dev);
	if (err < 0)
		goto err_unmap;

	platform_set_drvdata(pdev, net_dev);

	return 0;

err_unmap:
	iounmap(io);
err_release_mem:
	release_mem_region(0xf0000000, 0x02000000);

	return err;
}

static int ec_bhf_remove(struct platform_device *pdev)
{
	struct net_device *net_dev = platform_get_drvdata(pdev);
	struct ec_bhf_priv_no_dma *priv = netdev_priv(net_dev);

	unregister_netdev(net_dev);

	iounmap(priv->base.io);
	release_mem_region(0xf0000000, 0x02000000);

	free_netdev(net_dev);

	return 0;
}

static struct platform_driver ec_bhf_driver = {
	.driver = {
		.name = "ec_bhf",
		.of_match_table = ec_bhf_of_id,
	},
	.probe = ec_bhf_probe,
	.remove = ec_bhf_remove,
};

module_platform_driver(ec_bhf_driver);

#endif

module_param(polling_frequency, long, S_IRUGO);
MODULE_PARM_DESC(polling_frequency, "Polling timer frequency in ns");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dariusz Marcinkiewicz <reksio@newterm.pl>");
