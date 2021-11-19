/*
 * ethipcore altera soc FPGA driver
 *
 * (C) Copyright 2021 Aleksandr Dubovkin
 *
 */

#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <fdt_support.h>
#include <log.h>
#include <memalign.h>
#include <miiphy.h>
#include <net.h>
#include <asm/cache.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>

void ethipcore_init(struct ethipcore_priv *priv)
{
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	unsigned int val;

	priv->eth_rx_buf = dma_alloc_coherent(size, &priv->dma_addr_rx);
	flush_dcache_range(priv->eth_rx_buf, priv->eth_rx_buf +
				   ALIGN(ARM_DDR_PACKETS_BUF_LN * sizeof(DdrEthPack), PKTALIGN));
	priv->eth_tx_buf = dma_alloc_coherent(size, &priv->dma_addr_tx);
	flush_dcache_range(priv->eth_tx_buf, priv->eth_tx_buf +
				   ALIGN(ARM_DDR_PACKETS_BUF_LN * sizeof(DdrEthPack), PKTALIGN));

	writel(priv->dma_addr_rx/INTERFACE_WIDTH_BYTES, &pEth->wr.ddr_rcv_pckts_addr);
	writel(priv->dma_addr_tx/INTERFACE_WIDTH_BYTES, &pEth->wr.ddr_trm_pckts_addr);

	val = readl(&pEth->rd.cntrl);
	// Включаем eth ip core
	writel(val | work_ena_inp | ena_irq | priv->hsr_ena | priv->duplicate_accept_mode, &pEth->wr.cntrl);
}

static int ethipcore_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{
	struct ethipcore_priv *priv = bus->priv;
	u16 value = 0;

	ret = phyread(priv, addr, reg, &value);
	debug("%s 0x%x, 0x%x, 0x%x, 0x%x\n", __func__, addr, reg, value, ret);

	return value & 0xffff;
}

static int ethipcore_mdio_write(struct mii_dev *bus, int addr, int devad, int reg,
			  u16 value)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);

	debug("%s 0x%x, 0x%x, 0x%x\n", __func__, addr, reg, value);
	return phywrite(priv, addr, reg, value);
}

static int ethipcore_mdio_init(const char *name, struct ethipcore_priv *priv)
{
	struct mii_dev *bus = mdio_alloc();

	if (!bus) {
		printf("Failed to allocate MDIO bus\n");
		return -ENOMEM;
	}

	bus->read = ethipcore_mdio_read;
	bus->write = ethipcore_mdio_write;
	snprintf(bus->name, sizeof(bus->name), "%s", name);

	bus->priv = (void *)priv;

	return mdio_register(bus);
}

static int ethipcore_phy_init(struct ethipcore_priv *priv, void *dev)
{
	struct phy_device *phydev;
	unsigned int mask = 0xffffffff;

	if (priv->phyaddr)
		mask = 1 << priv->phyaddr;

	phydev = phy_find_by_mask(priv->bus, mask, priv->interface);
	if (!phydev)
		return -ENODEV;

	phy_connect_dev(phydev, dev);

	phydev->supported &= PHY_GBIT_FEATURES;
	phydev->advertising = phydev->supported;

	priv->phydev = phydev;
	phy_config(phydev);

	return 0;
}

static int ethipcore_write_hwaddr(struct udevice *dev)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	struct eth_pdata *pdata = dev_get_platdata(dev);
	u8 *hwaddr = pdata->enetaddr;
	u32 mac_lo, mac_hi;

	mac_lo = (hwaddr[3] << 24) | (hwaddr[2] << 16) |
		(hwaddr[1] << 8) | hwaddr[0];
	mac_hi = (hwaddr[5] << 8) | hwaddr[4];
	debug("Set MAC address to 0x%04x%08x\n", mac_hi, mac_lo);

	writel(mac_lo, &pEth->wr.our_mac_low32);
	writel(mac_hi, &pEth->wr.our_mac_hi16);

	return 0;
}

void ethipcore_hsr_mode(struct udevice *dev, int flg_on)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	unsigned int val;

	val = readl(&pEth->rd.cntrl) & ~prp_tagging;
	if (flg_on) {
		writel(val | hsr_tagging, &pEth->wr.cntrl);
	} else {
		writel(val & ~hsr_tagging, &pEth->wr.cntrl);
	}
}

void ethipcore_enable_irq(struct udevice *dev)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	unsigned int val;
	
	val = readl(&pEth->rd.cntrl);
	writel(val | work_ena_inp | ena_irq, &pEth->wr.cntrl);
}

void ethipcore_disable_irq(struct udevice *dev)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	unsigned int val;
	
	val = readl(&pEth->rd.cntrl);
	writel(val & ~ena_irq, &pEth->wr.cntrl);
}

void ethipcore_clear_irq(struct udevice *dev)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	struct ethipcore_private *priv = netdev_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	
	writel(1, &pEth->wr.clear_irq);
}

static int ethipcore_reset(struct ethipcore_priv *priv)
{
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;

	if(pEth == NULL) {
		return;
	}
	writel(0, &pEth->wr.cntrl);
	writel(0, &pEth->wr.cur_pckt_trm_pos);
	priv->cur_pckt_trm_pos = 0;
	priv->rx_cur_rd_pos = 0;
}

static int ethipcore_send(struct udevice *dev, void *packet, int length)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pDdrEthPack pTrmPack;
	u32 val, next_pack;
	pDdrEthPack pEth_tx_buf = (pDdrEthPack)priv->eth_tx_buf;
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	u8 cntrl_bits;

	next_pack = (priv->cur_pckt_trm_pos + 1) & (ARM_DDR_PACKETS_BUF_LN - 1); // место куда мы пишем отправляемые пакеты
	val = readl(&pEth->rd.ddr_trm_pckts_rd); // место с которого eth_ip вычитывает отправляемые пакеты
	if (next_pack == val){
		return NETDEV_TX_BUSY;
	}

 	pTrmPack = &pEth_tx_buf[priv->cur_pckt_trm_pos];

	memcpy((void*)&pTrmPack->hdr, packet, length);
	flush_dcache_range((ulong)packet, (ulong)packet + length);

	if (length < MIN_ETH_FRAME_CNT) {
		length = MIN_ETH_FRAME_CNT;
	}

	pTrmPack->desc.pack_sz = length;
    pTrmPack->desc.cntrl_bits = CNTRL_BITS_SRC_DEST_PORT_AB;
	priv->cur_pckt_trm_pos = next_pack;
	writel(priv->cur_pckt_trm_pos, &pEth->wr.cur_pckt_trm_pos); // начать передачу
	
	debug("%s(): packet sent\n", __func__);
	return 0;
}

static int ethipcore_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);

	if (packet)
		free(packet);
	
	return 0;
}

static int get_rx_pos(struct ethipcore_private *priv)
{
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	
	return readl(&pEth->rd.ddr_rcv_pckts_wr);
}

static int ethipcore_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pDdrEthPack peth_rx_buf = (pDdrEthPack)priv->eth_rx_buf;
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;

	ethipcore_clear_irq(dev);
	while (get_rx_pos(priv) != priv->rx_cur_rd_pos) {
		if(count >= limit) goto contn;

		pPack = &peth_rx_buf[priv->rx_cur_rd_pos];
		*packetp = pPack;
		count++;
		debug("recv %d bytes\n", pPack->desc.pack_sz;);
contn:
			priv->rx_cur_rd_pos = (priv->rx_cur_rd_pos + 1) & (ARM_DDR_PACKETS_BUF_LN - 1);
	}
	
	return count;
}

static int ethipcore_start(struct udevice *dev)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	u32 val;
	int ret;
	
	ethipcore_reset(priv);
	ethipcore_init(priv);

	/* Start up the PHY */
	ret = phy_startup(priv->phydev);
	if (ret) {
		debug("Could not initialize PHY %s\n",
		      priv->phydev->dev->name);
		return ret;
	}

	return ret;
}

static void ethoc_stop(struct udevice *dev)
{
	struct ethipcore_priv *priv = dev_get_priv(dev);
	pETH_comp_struct pEth = (pETH_comp_struct)priv->eth_ip_core_addr;
	unsigned int val;

	val = readl(&pEth->rd.cntrl);
	writel(val & ~(ena_irq), &pEth->wr.cntrl);
	msleep(200);
	writel(val & ~(work_ena_inp | ena_irq), &pEth->wr.cntrl);
	msleep(1);
	writel(0, &pEth->wr.cur_pckt_trm_pos);
	priv->cur_pckt_trm_pos = 0;
	priv->rx_cur_rd_pos = 0;
}

int ethipcore_probe(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct ethipcore_priv *priv = dev_get_priv(dev);
	void *blob = (void *)gd->fdt_blob;
	int node = dev_of_offset(dev);

	priv->interface = pdata->phy_interface;
	ethipcore_mdio_init(dev->name, priv);
	priv->bus = miiphy_get_dev_by_name(dev->name);

	ret = ethipcore_phy_init(priv, dev);

	return ret;
}

static int ethipcore_ofdata_to_platdata(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct ethipcore_priv *priv = dev_get_priv(dev);
	const char *phy_mode;

	pdata->iobase = (phys_addr_t)dev_read_addr(dev);
	priv->eth_ip_core_base = pdata->iobase;
	priv->mdio_base = priv->iobase;

	pdata->phy_interface = -1;
	phy_mode = dev_read_prop(dev, "phy-mode", NULL);
	if (phy_mode)
		pdata->phy_interface = phy_get_interface_by_name(phy_mode);
	if (pdata->phy_interface == -1) {
		debug("%s: Invalid PHY interface '%s'\n", __func__, phy_mode);
		return -EINVAL;
	}

	return 0;
}

static const struct eth_ops ethipcore_ops = {
	.start		= ethipcore_start,
	.send		= ethipcore_send,
	.recv		= ethipcore_recv,
	.free_pkt	= ethipcore_free_pkt,
	.stop		= ethipcore_stop,
	.write_hwaddr	= ethipcore_write_hwaddr,
};

static const struct udevice_id ethipcore_ids[] = {
	{ .compatible = "parma,ethipcore" },
	{}
};

U_BOOT_DRIVER(ethipcore) = {
	.name	= "ethipcore",
	.id	= UCLASS_ETH,
	.of_match = ethipcore_ids,
	.ops	= &ethipcore_ops,
	.ofdata_to_platdata = ethipcore_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct eth_pdata),
	.priv_auto_alloc_size = sizeof(struct ethipcore_priv),
	.probe	= ethipcore_probe,
};