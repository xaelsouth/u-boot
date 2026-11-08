// SPDX-License-Identifier: GPL-2.0-only
/*
 * Microchip ENCX24J600 SPI Ethernet driver.
 *
 * Based on the Linux ENCX24J600 driver:
 * Copyright 2015 Gridpoint, Jon Ringle <jringle@gridpoint.com>
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <net.h>
#include <spi.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>

#define ENC_BANK_SEL(bank)	(0xc0 | ((bank) << 1))
#define ENC_RESET		0xca
#define ENC_PKTDEC		0xcc
#define ENC_TXREQ		0xd4
#define ENC_RX_ENABLE		0xe8
#define ENC_RX_DISABLE		0xea
#define ENC_RCR			0x00
#define ENC_WCR			0x40
#define ENC_BFS			0x80
#define ENC_BFC			0xa0
#define ENC_WGPWRPT		0x6c
#define ENC_WRXRDPT		0x64
#define ENC_WGPDATA		0x2a
#define ENC_RRXDATA		0x2c

#define ENC_ETXST		0x00
#define ENC_ETXLEN		0x02
#define ENC_ERXST		0x04
#define ENC_ERXTAIL		0x06
#define ENC_EUDAST		0x16
#define ENC_EUDAND		0x18
#define ENC_ESTAT		0x1a
#define ENC_EIR			0x1c
#define ENC_ECON1		0x1e
#define ENC_ERXFCON		0x34
#define ENC_MACON1		0x40
#define ENC_MACON2		0x42
#define ENC_MABBIPG		0x44
#define ENC_MAIPG		0x46
#define ENC_MACLCON		0x48
#define ENC_MAMXFL		0x4a
#define ENC_MICMD		0x52
#define ENC_MIREGADR		0x54
#define ENC_MAADR3		0x60
#define ENC_MAADR2		0x62
#define ENC_MAADR1		0x64
#define ENC_MIWR		0x66
#define ENC_MIRD		0x68
#define ENC_MISTAT		0x6a
#define ENC_ECON2		0x6e
#define ENC_EIE			0x72
#define ENC_EIDLED		0x74

#define ENC_CLKRDY		BIT(12)
#define ENC_PHYDPX		BIT(10)
#define ENC_PHYLNK		BIT(8)
#define ENC_TXIF		BIT(3)
#define ENC_TXABTIF		BIT(2)
#define ENC_LINKIF		BIT(11)
#define ENC_TXRTS		BIT(1)
#define ENC_TXRST		BIT(6)
#define ENC_RX_FILTER		(BIT(6) | BIT(3) | BIT(0))
#define ENC_MAC_CONFIG		(BIT(14) | BIT(7) | BIT(5) | BIT(4) | BIT(1))
#define ENC_FULDPX		BIT(0)
#define ENC_MIIRD		BIT(0)
#define ENC_MIBUSY		BIT(0)
#define ENC_PHY_ADDR		BIT(8)
#define ENC_PHCON1		0x00
#define ENC_PHSTAT1		0x01
#define ENC_PHANA		0x04
#define ENC_PHSTAT3		0x1f
#define ENC_ANEN		BIT(12)
#define ENC_RENEG		BIT(9)
#define ENC_SPD100		BIT(13)
#define ENC_PFULDPX		BIT(8)
#define ENC_ANDONE		BIT(5)
#define ENC_PHY3SPD100		BIT(3)
#define ENC_PHY3DPX		BIT(4)
#define ENC_RXOK		BIT(7)

#define ENC_TX_START		0x0000
#define ENC_RX_START		0x0600
#define ENC_SRAM_SIZE		0x6000
#define ENC_MAX_FRAME		1518
#define ENC_RSV_SIZE		8

struct encx24j600_priv {
	int bank;
	u16 next_packet;
	bool forced_link;
	bool rx_fault;
};

static int enc_spi(struct udevice *dev, const u8 *cmd, uint cmdlen,
		   const void *out, void *in, uint len)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	int ret;

	ret = dm_spi_claim_bus(dev);
	if (ret) {
		dev_err(dev, "SPI bus claim failed: %d\n", ret);
		return ret;
	}

	if (!len) {
		ret = dm_spi_xfer(dev, cmdlen * 8, cmd, NULL,
				  SPI_XFER_BEGIN | SPI_XFER_END);
	} else {
		ret = dm_spi_xfer(dev, cmdlen * 8, cmd, NULL, SPI_XFER_BEGIN);
		if (!ret)
			ret = dm_spi_xfer(dev, len * 8, out, in, SPI_XFER_END);
	}
	if (ret) {
		priv->bank = -1;
		dm_spi_xfer(dev, 0, NULL, NULL, SPI_XFER_END);
		dev_err(dev, "SPI transfer failed: %d\n", ret);
	}
	dm_spi_release_bus(dev);
	return ret;
}

static int enc_cmd(struct udevice *dev, u8 cmd)
{
	return enc_spi(dev, &cmd, 1, NULL, NULL, 0);
}

static int enc_bank(struct udevice *dev, u8 reg)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	int bank = (reg >> 5) & 3;
	int ret;

	if ((reg & 0x1f) >= 0x16 || priv->bank == bank)
		return 0;
	ret = enc_cmd(dev, ENC_BANK_SEL(bank));
	if (ret) {
		priv->bank = -1;
		return ret;
	}
	priv->bank = bank;
	return 0;
}

static int enc_read(struct udevice *dev, u8 reg, u16 *value)
{
	u8 cmd = ENC_RCR | (reg & 0x1f);
	u8 data[2];
	int ret;

	ret = enc_bank(dev, reg);
	if (ret)
		return ret;
	ret = enc_spi(dev, &cmd, 1, NULL, data, sizeof(data));
	if (!ret)
		*value = data[0] | (data[1] << 8);
	return ret;
}

static int enc_write(struct udevice *dev, u8 reg, u16 value)
{
	u8 data[] = { ENC_WCR | (reg & 0x1f), value, value >> 8 };
	int ret;

	ret = enc_bank(dev, reg);
	if (ret)
		return ret;
	return enc_spi(dev, data, sizeof(data), NULL, NULL, 0);
}

static int enc_bits(struct udevice *dev, u8 reg, u16 mask, bool set)
{
	u8 data[] = { (set ? ENC_BFS : ENC_BFC) | (reg & 0x1f), mask };
	int ret;

	ret = enc_bank(dev, reg);
	if (ret)
		return ret;
	if (mask & 0xff) {
		ret = enc_spi(dev, data, sizeof(data), NULL, NULL, 0);
		if (ret)
			return ret;
	}
	if (mask >> 8) {
		data[0]++;
		data[1] = mask >> 8;
		return enc_spi(dev, data, sizeof(data), NULL, NULL, 0);
	}
	return 0;
}

static int enc_pointer(struct udevice *dev, u8 cmd, u16 value)
{
	u8 data[] = { cmd, value, value >> 8 };

	return enc_spi(dev, data, sizeof(data), NULL, NULL, 0);
}

static int enc_wait(struct udevice *dev, u8 reg, u16 mask, bool set,
		    unsigned int timeout_ms)
{
	ulong start = get_timer(0);
	u16 value;
	int ret;

	do {
		ret = enc_read(dev, reg, &value);
		if (ret)
			return ret;
		if (!!(value & mask) == set)
			return 0;
		udelay(50);
	} while (get_timer(start) < timeout_ms);
	return -ETIMEDOUT;
}

static int enc_phy_read(struct udevice *dev, u8 reg, u16 *value)
{
	int ret, cleanup;

	ret = enc_write(dev, ENC_MIREGADR, ENC_PHY_ADDR | reg);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MICMD, ENC_MIIRD);
	if (ret)
		return ret;
	udelay(26);
	ret = enc_wait(dev, ENC_MISTAT, ENC_MIBUSY, false, 10);
	cleanup = enc_write(dev, ENC_MICMD, 0);
	if (ret) {
		if (cleanup)
			dev_err(dev, "failed to stop PHY read: %d\n", cleanup);
		return ret;
	}
	if (cleanup)
		return cleanup;
	return enc_read(dev, ENC_MIRD, value);
}

static int enc_phy_write(struct udevice *dev, u8 reg, u16 value)
{
	int ret;

	ret = enc_write(dev, ENC_MIREGADR, ENC_PHY_ADDR | reg);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MIWR, value);
	if (ret)
		return ret;
	udelay(26);
	return enc_wait(dev, ENC_MISTAT, ENC_MIBUSY, false, 10);
}

static int enc_reset(struct udevice *dev)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	u16 value;
	int i, ret;

	priv->bank = -1;
	priv->forced_link = false;
	priv->rx_fault = false;
	for (i = 0; i < 10; i++) {
		ret = enc_write(dev, ENC_EUDAST, 0x1234);
		if (ret)
			return ret;
		ret = enc_read(dev, ENC_EUDAST, &value);
		if (ret)
			return ret;
		if (value == 0x1234)
			break;
		udelay(100);
	}
	if (i == 10)
		return -ENODEV;
	ret = enc_wait(dev, ENC_ESTAT, ENC_CLKRDY, true, 10);
	if (ret)
		return ret;
	ret = enc_cmd(dev, ENC_RESET);
	if (ret)
		return ret;
	priv->bank = -1;
	udelay(100);
	ret = enc_read(dev, ENC_EUDAST, &value);
	if (ret)
		return ret;
	if (value)
		return -EIO;
	udelay(1000);
	return 0;
}

static int enc_read_rom_hwaddr(struct udevice *dev)
{
	struct eth_pdata *plat = dev_get_plat(dev);
	const u8 regs[] = { ENC_MAADR1, ENC_MAADR2, ENC_MAADR3 };
	u16 value;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = enc_read(dev, regs[i], &value);
		if (ret)
			return ret;
		plat->enetaddr[i * 2] = value;
		plat->enetaddr[i * 2 + 1] = value >> 8;
	}
	return 0;
}

static int enc_write_hwaddr(struct udevice *dev)
{
	struct eth_pdata *plat = dev_get_plat(dev);
	const u8 regs[] = { ENC_MAADR1, ENC_MAADR2, ENC_MAADR3 };
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = enc_write(dev, regs[i], plat->enetaddr[i * 2] |
				(plat->enetaddr[i * 2 + 1] << 8));
		if (ret)
			return ret;
	}
	return 0;
}

static int enc_probe(struct udevice *dev)
{
	u16 id;
	int ret;

	ret = enc_reset(dev);
	if (ret) {
		dev_err(dev, "reset failed: %d\n", ret);
		return ret;
	}
	ret = enc_read(dev, ENC_EIDLED, &id);
	if (ret)
		return ret;
	if (((id >> 5) & 7) != 1) {
		dev_err(dev, "unexpected device ID: 0x%04x\n", id);
		return -ENODEV;
	}
	dev_info(dev, "ENCX24J600 revision %u\n", id & 0x1f);
	return 0;
}

static int enc_link(struct udevice *dev)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	ulong start = get_timer(0);
	u16 status, phy, control;
	int ret;

	do {
		ret = enc_read(dev, ENC_ESTAT, &status);
		if (ret)
			return ret;
		if (status & ENC_PHYLNK)
			break;
		mdelay(10);
	} while (get_timer(start) < 4000);
	if (!(status & ENC_PHYLNK))
		return -ENETDOWN;

	start = get_timer(0);
	do {
		ret = enc_phy_read(dev, ENC_PHSTAT1, &phy);
		if (ret)
			return ret;
		if (phy & ENC_ANDONE)
			break;
		mdelay(10);
	} while (get_timer(start) < 2000);

	if (!(phy & ENC_ANDONE)) {
		ret = enc_phy_read(dev, ENC_PHSTAT3, &phy);
		if (ret)
			return ret;
		ret = enc_phy_read(dev, ENC_PHCON1, &control);
		if (ret)
			return ret;
		control &= ~(ENC_ANEN | ENC_SPD100 | ENC_PFULDPX);
		if (phy & ENC_PHY3SPD100)
			control |= ENC_SPD100;
		if (phy & ENC_PHY3DPX)
			control |= ENC_PFULDPX;
		ret = enc_phy_write(dev, ENC_PHCON1, control);
		if (ret)
			return ret;
		priv->forced_link = true;
		dev_warn(dev, "autonegotiation timed out, using parallel detection\n");
	}

	ret = enc_read(dev, ENC_ESTAT, &status);
	if (ret)
		return ret;
	if (!(status & ENC_PHYLNK))
		return -ENETDOWN;
	ret = enc_bits(dev, ENC_MACON2, ENC_FULDPX, !!(status & ENC_PHYDPX));
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MABBIPG, status & ENC_PHYDPX ? 0x15 : 0x12);
	if (ret)
		return ret;
	if (!(status & ENC_PHYDPX)) {
		ret = enc_write(dev, ENC_MACLCON, 0x370f);
		if (ret)
			return ret;
	}
	return enc_bits(dev, ENC_EIR, ENC_LINKIF, false);
}

static int enc_start(struct udevice *dev)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	u16 phy;
	int ret;

	ret = enc_reset(dev);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_EIE, 0);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MACON1, 0x0009);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MACON2, ENC_MAC_CONFIG);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MAIPG, 0x0c12);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_ERXFCON, ENC_RX_FILTER);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_MAMXFL, ENC_MAX_FRAME);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_ERXST, ENC_RX_START);
	if (ret)
		return ret;
	ret = enc_pointer(dev, ENC_WRXRDPT, ENC_RX_START);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_ERXTAIL, ENC_SRAM_SIZE - 2);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_EUDAST, ENC_SRAM_SIZE);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_EUDAND, ENC_SRAM_SIZE + 1);
	if (ret)
		return ret;
	ret = enc_write_hwaddr(dev);
	if (ret)
		return ret;
	ret = enc_phy_write(dev, ENC_PHANA, 0x05e1);
	if (ret)
		return ret;
	ret = enc_phy_read(dev, ENC_PHCON1, &phy);
	if (ret)
		return ret;
	ret = enc_phy_write(dev, ENC_PHCON1, phy | ENC_ANEN | ENC_RENEG);
	if (ret)
		return ret;
	ret = enc_link(dev);
	if (ret) {
		dev_err(dev, "link setup failed: %d\n", ret);
		return ret;
	}
	priv->next_packet = ENC_RX_START;
	return enc_cmd(dev, ENC_RX_ENABLE);
}

static void enc_stop(struct udevice *dev)
{
	int ret = enc_cmd(dev, ENC_RX_DISABLE);

	if (ret)
		dev_err(dev, "failed to stop receiver: %d\n", ret);
}

static int enc_tx_reset(struct udevice *dev)
{
	int ret;

	ret = enc_bits(dev, ENC_ECON2, ENC_TXRST, true);
	if (ret)
		return ret;
	return enc_bits(dev, ENC_ECON2, ENC_TXRST, false);
}

static int enc_send(struct udevice *dev, void *packet, int length)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	u8 cmd = ENC_WGPDATA;
	u16 flags, phy;
	int ret;

	if (length <= 0 || length > ENC_MAX_FRAME - 4)
		return -EINVAL;
	ret = enc_read(dev, ENC_EIR, &flags);
	if (ret)
		return ret;
	if (flags & ENC_LINKIF) {
		if (priv->forced_link) {
			ret = enc_phy_read(dev, ENC_PHCON1, &phy);
			if (ret)
				return ret;
			ret = enc_phy_write(dev, ENC_PHCON1,
					    phy | ENC_ANEN | ENC_RENEG);
			if (ret)
				return ret;
			priv->forced_link = false;
		}
		ret = enc_link(dev);
		if (ret) {
			dev_err(dev, "link setup failed: %d\n", ret);
			return ret;
		}
	}
	ret = enc_wait(dev, ENC_ECON1, ENC_TXRTS, false, 100);
	if (ret == -ETIMEDOUT) {
		int reset = enc_tx_reset(dev);

		dev_err(dev, "transmitter busy timeout\n");
		if (reset)
			dev_err(dev, "TX reset failed: %d\n", reset);
	}
	if (ret)
		return ret;
	ret = enc_bits(dev, ENC_EIR, ENC_TXIF | ENC_TXABTIF, false);
	if (ret)
		return ret;
	ret = enc_pointer(dev, ENC_WGPWRPT, ENC_TX_START);
	if (ret)
		return ret;
	ret = enc_spi(dev, &cmd, 1, packet, NULL, length);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_ETXST, ENC_TX_START);
	if (ret)
		return ret;
	ret = enc_write(dev, ENC_ETXLEN, length);
	if (ret)
		return ret;
	ret = enc_cmd(dev, ENC_TXREQ);
	if (ret)
		return ret;
	ret = enc_wait(dev, ENC_ECON1, ENC_TXRTS, false, 100);
	if (ret) {
		int reset = enc_tx_reset(dev);

		dev_err(dev, "transmission failed: %d\n", ret);
		if (reset)
			dev_err(dev, "TX reset failed: %d\n", reset);
		return ret;
	}
	ret = enc_read(dev, ENC_EIR, &flags);
	if (ret)
		return ret;
	if (flags & ENC_TXABTIF) {
		dev_err(dev, "transmission aborted\n");
		ret = enc_tx_reset(dev);
		return ret ? ret : -EIO;
	}
	return 0;
}

static int enc_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct encx24j600_priv *priv = dev_get_priv(dev);
	u8 cmd = ENC_RRXDATA;
	u8 rsv[ENC_RSV_SIZE];
	u16 status, next, length, tail;
	int ret;

	if (priv->rx_fault)
		return -EIO;
	ret = enc_read(dev, ENC_ESTAT, &status);
	if (ret)
		return ret;
	if (!(status & 0xff))
		return -EAGAIN;
	ret = enc_pointer(dev, ENC_WRXRDPT, priv->next_packet);
	if (ret)
		return ret;
	ret = enc_spi(dev, &cmd, 1, NULL, rsv, sizeof(rsv));
	if (ret)
		return ret;
	next = rsv[0] | (rsv[1] << 8);
	length = rsv[2] | (rsv[3] << 8);
	if (next < ENC_RX_START || next >= ENC_SRAM_SIZE || (next & 1) ||
	    length > ENC_MAX_FRAME) {
		dev_err(dev, "invalid RX metadata: next=%04x len=%u\n",
			next, length);
		priv->rx_fault = true;
		return -EIO;
	}
	if ((rsv[4] & ENC_RXOK) && length >= 18) {
		ret = enc_spi(dev, &cmd, 1, NULL, net_rx_packets[0], length - 4);
		if (ret)
			return ret;
		*packetp = net_rx_packets[0];
	}
	tail = next == ENC_RX_START ? ENC_SRAM_SIZE - 2 : next - 2;
	ret = enc_write(dev, ENC_ERXTAIL, tail);
	if (ret) {
		priv->rx_fault = true;
		return ret;
	}
	ret = enc_cmd(dev, ENC_PKTDEC);
	if (ret) {
		priv->rx_fault = true;
		return ret;
	}
	priv->next_packet = next;
	if (!(rsv[4] & ENC_RXOK) || length < 18)
		return -EAGAIN;
	return length - 4;
}

static const struct eth_ops enc_ops = {
	.start = enc_start,
	.send = enc_send,
	.recv = enc_recv,
	.stop = enc_stop,
	.read_rom_hwaddr = enc_read_rom_hwaddr,
	.write_hwaddr = enc_write_hwaddr,
};

static const struct udevice_id enc_ids[] = {
	{ .compatible = "microchip,encx24j600" },
	{ }
};

U_BOOT_DRIVER(encx24j600) = {
	.name = "encx24j600",
	.id = UCLASS_ETH,
	.of_match = enc_ids,
	.probe = enc_probe,
	.ops = &enc_ops,
	.priv_auto = sizeof(struct encx24j600_priv),
	.plat_auto = sizeof(struct eth_pdata),
};
