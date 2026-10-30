// SPDX-License-Identifier: GPL-2.0+

#include <dm.h>
#include <dm/device_compat.h>
#include <net.h>
#include <spi.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>

#define ENC28J60_RCR		0x00
#define ENC28J60_WCR		0x40
#define ENC28J60_BFS		0x80
#define ENC28J60_BFC		0xa0
#define ENC28J60_RBM		0x3a
#define ENC28J60_WBM		0x7a
#define ENC28J60_SRC		0xff

#define ENC28J60_ECON1		0x1f
#define ENC28J60_ECON2		0x1e
#define ENC28J60_EIR		0x1c
#define ENC28J60_ESTAT		0x1d
#define ENC28J60_EIE		0x1b
#define ENC28J60_EPKTCNT	0x39
#define ENC28J60_ERDPTL		0x00
#define ENC28J60_EWRPTL		0x02
#define ENC28J60_ETXSTL		0x04
#define ENC28J60_ETXNDL		0x06
#define ENC28J60_ERXSTL		0x08
#define ENC28J60_ERXNDL		0x0a
#define ENC28J60_ERXRDPTL	0x0c
#define ENC28J60_ERXFCON	0x38
#define ENC28J60_MACON1		0xc0
#define ENC28J60_MACON3		0xc2
#define ENC28J60_MACON4		0xc3
#define ENC28J60_MABBIPG	0xc4
#define ENC28J60_MAIPGL		0xc6
#define ENC28J60_MAMXFLL	0xca
#define ENC28J60_MIREGADR	0xd4
#define ENC28J60_MIWRL		0xd6
#define ENC28J60_MISTAT		0xea
#define ENC28J60_MAADR0		0xe1
#define ENC28J60_MAADR1		0xe0
#define ENC28J60_MAADR2		0xe3
#define ENC28J60_MAADR3		0xe2
#define ENC28J60_MAADR4		0xe5
#define ENC28J60_MAADR5		0xe4
#define ENC28J60_EREVID		0x72
#define ENC28J60_PHCON1		0x00
#define ENC28J60_PHCON2		0x10
#define ENC28J60_PHLCON		0x14
#define ENC28J60_PHSTAT2	0x11
#define ENC28J60_MICMD		0xd2
#define ENC28J60_MIRDL		0xd8
#define ENC28J60_PHSTAT2_LSTAT	BIT(10)

#define ENC28J60_ECON1_TXRTS	BIT(3)
#define ENC28J60_ECON1_RXEN	BIT(2)
#define ENC28J60_ECON1_RXRST	BIT(6)
#define ENC28J60_ECON1_TXRST	BIT(7)
#define ENC28J60_ECON1_BSEL	GENMASK(1, 0)
#define ENC28J60_ECON2_AUTOINC	BIT(7)
#define ENC28J60_ECON2_PKTDEC	BIT(6)
#define ENC28J60_ECON2_VRPS	BIT(3)
#define ENC28J60_EIR_TXERIF	BIT(1)
#define ENC28J60_EIR_RXERIF	BIT(0)
#define ENC28J60_ESTAT_TXABRT	BIT(1)
#define ENC28J60_MISTAT_BUSY	BIT(0)
#define ENC28J60_MACON1_MARXEN	BIT(0)
#define ENC28J60_MACON1_TXPAUS	BIT(3)
#define ENC28J60_MACON1_RXPAUS	BIT(2)
#define ENC28J60_MACON3_PADCFG0	BIT(5)
#define ENC28J60_MACON3_TXCRCEN	BIT(4)
#define ENC28J60_MACON3_FRMLNEN	BIT(1)
#define ENC28J60_MACON3_FULDPX	BIT(0)
#define ENC28J60_PHCON1_PDPXMD	BIT(8)
#define ENC28J60_ERXFCON_UCEN	BIT(7)
#define ENC28J60_ERXFCON_CRCEN	BIT(5)
#define ENC28J60_ERXFCON_BCEN	BIT(0)

#define ENC28J60_RX_START	0x0000
#define ENC28J60_RX_END		0x19ff
#define ENC28J60_TX_START	0x1a00
#define ENC28J60_TX_END		0x1fff
#define ENC28J60_RSV_SIZE	6
#define ENC28J60_MAX_FRAME	1518

struct enc28j60_priv {
	u8 bank;
	u16 next_packet;
};

static int enc28j60_spi_xfer(struct udevice *dev, const u8 *tx, uint tx_len,
			     u8 *rx, uint rx_len)
{
	int ret;

	ret = dm_spi_claim_bus(dev);
	if (ret)
		return ret;

	/* A zero-length END transfer makes the MXS driver clock an extra byte */
	if (!rx_len) {
		ret = dm_spi_xfer(dev, tx_len * 8, tx, NULL,
				  SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret)
			goto release_cs;
		dm_spi_release_bus(dev);
		return 0;
	}

	ret = dm_spi_xfer(dev, tx_len * 8, tx, NULL, SPI_XFER_BEGIN);
	if (ret)
		goto release_cs;

	ret = dm_spi_xfer(dev, rx_len * 8, NULL, rx, SPI_XFER_END);
	if (ret)
		goto release_cs;

	dm_spi_release_bus(dev);
	return 0;

release_cs:
	dm_spi_xfer(dev, 0, NULL, NULL, SPI_XFER_END);
	dm_spi_release_bus(dev);
	return ret;
}

static int enc28j60_write_op(struct udevice *dev, u8 op, u8 reg, u8 value)
{
	u8 cmd[] = { op | (reg & 0x1f), value };

	return enc28j60_spi_xfer(dev, cmd, sizeof(cmd), NULL, 0);
}

static int enc28j60_read_buf(struct udevice *dev, u8 *buf, uint len)
{
	u8 cmd = ENC28J60_RBM;

	return enc28j60_spi_xfer(dev, &cmd, 1, buf, len);
}

static int enc28j60_write_buf(struct udevice *dev, const u8 *buf, uint len)
{
	int ret;
	u8 cmd = ENC28J60_WBM;

	ret = dm_spi_claim_bus(dev);
	if (ret)
		return ret;

	ret = dm_spi_xfer(dev, 8, &cmd, NULL, SPI_XFER_BEGIN);
	if (!ret)
		ret = dm_spi_xfer(dev, len * 8, buf, NULL, SPI_XFER_END);
	if (ret) {
		dm_spi_xfer(dev, 0, NULL, NULL, SPI_XFER_END);
		dm_spi_release_bus(dev);
		return ret;
	}

	dm_spi_release_bus(dev);
	return 0;
}

static int enc28j60_read_reg(struct udevice *dev, struct enc28j60_priv *priv,
			     u8 reg, u8 *value)
{
	u8 cmd = ENC28J60_RCR | (reg & 0x1f);
	u8 rx[2] = { 0 };
	uint len = (reg & 0x80) ? 2 : 1;
	int ret;
	u8 bank = (reg >> 5) & 3;

	if (reg < 0x1b || reg > ENC28J60_ECON1) {
		if (bank != priv->bank) {
			ret = enc28j60_write_op(dev, ENC28J60_BFC,
						ENC28J60_ECON1,
						ENC28J60_ECON1_BSEL);
			if (ret)
				return ret;
			ret = enc28j60_write_op(dev, ENC28J60_BFS,
						ENC28J60_ECON1, bank);
			if (ret)
				return ret;
			priv->bank = bank;
		}
	}

	ret = enc28j60_spi_xfer(dev, &cmd, 1, rx, len);
	if (ret)
		return ret;

	*value = rx[len - 1];
	return 0;
}

static int enc28j60_write_reg(struct udevice *dev, struct enc28j60_priv *priv,
			      u8 reg, u8 value)
{
	u8 bank = (reg >> 5) & 3;
	int ret;

	if (reg < 0x1b || reg > ENC28J60_ECON1) {
		if (bank != priv->bank) {
			ret = enc28j60_write_op(dev, ENC28J60_BFC,
						ENC28J60_ECON1,
						ENC28J60_ECON1_BSEL);
			if (ret)
				return ret;
			ret = enc28j60_write_op(dev, ENC28J60_BFS,
						ENC28J60_ECON1, bank);
			if (ret)
				return ret;
			priv->bank = bank;
		}
	}

	return enc28j60_write_op(dev, ENC28J60_WCR, reg, value);
}

static int enc28j60_write_word(struct udevice *dev,
			       struct enc28j60_priv *priv, u8 reg, u16 value)
{
	int ret;

	ret = enc28j60_write_reg(dev, priv, reg, value & 0xff);
	if (ret)
		return ret;

	return enc28j60_write_reg(dev, priv, reg + 1, value >> 8);
}

static int enc28j60_set_bits(struct udevice *dev, u8 op, u8 reg, u8 mask)
{
	return enc28j60_write_op(dev, op, reg, mask);
}

static int enc28j60_reset(struct udevice *dev, struct enc28j60_priv *priv)
{
	u8 cmd = ENC28J60_SRC;
	int ret;

	ret = enc28j60_spi_xfer(dev, &cmd, 1, NULL, 0);
	if (ret)
		return ret;

	priv->bank = 0;
	priv->next_packet = ENC28J60_RX_START;
	mdelay(2);
	return 0;
}

static int enc28j60_phy_write(struct udevice *dev,
			      struct enc28j60_priv *priv, u8 reg, u16 value)
{
	int i, ret;
	u8 status;

	ret = enc28j60_write_reg(dev, priv, ENC28J60_MIREGADR, reg);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MIWRL, value & 0xff);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MIWRL + 1, value >> 8);
	if (ret)
		return ret;

	for (i = 0; i < 100; i++) {
		ret = enc28j60_read_reg(dev, priv, ENC28J60_MISTAT, &status);
		if (ret)
			return ret;
		if (!(status & ENC28J60_MISTAT_BUSY))
			return 0;
		udelay(10);
	}

	return -ETIMEDOUT;
}

static int enc28j60_phy_read(struct udevice *dev,
			     struct enc28j60_priv *priv, u8 reg, u16 *value)
{
	u8 lo, hi, status;
	int i, ret;

	ret = enc28j60_write_reg(dev, priv, ENC28J60_MIREGADR, reg);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MICMD, BIT(0));
	if (ret)
		return ret;

	for (i = 0; i < 100; i++) {
		udelay(10);
		ret = enc28j60_read_reg(dev, priv, ENC28J60_MISTAT, &status);
		if (ret)
			return ret;
		if (!(status & ENC28J60_MISTAT_BUSY))
			break;
	}
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MICMD, 0);
	if (ret)
		return ret;
	if (i == 100)
		return -ETIMEDOUT;

	ret = enc28j60_read_reg(dev, priv, ENC28J60_MIRDL, &lo);
	if (ret)
		return ret;
	ret = enc28j60_read_reg(dev, priv, ENC28J60_MIRDL + 1, &hi);
	if (ret)
		return ret;

	*value = lo | (hi << 8);
	return 0;
}

static int enc28j60_hw_init(struct udevice *dev, struct enc28j60_priv *priv)
{
	struct eth_pdata *plat = dev_get_plat(dev);
	u8 revision;
	u16 phstat2;
	int ret, i;
	const u8 mac_regs[] = {
		ENC28J60_MAADR5, ENC28J60_MAADR4, ENC28J60_MAADR3,
		ENC28J60_MAADR2, ENC28J60_MAADR1, ENC28J60_MAADR0,
	};

	if (!is_valid_ethaddr(plat->enetaddr)) {
		dev_err(dev, "a valid ethaddr must be configured\n");
		return -EINVAL;
	}

	ret = enc28j60_reset(dev, priv);
	if (ret)
		return ret;

	ret = enc28j60_write_op(dev, ENC28J60_WCR, ENC28J60_ECON1, 0);
	if (ret)
		return ret;
	priv->bank = 0;

	ret = enc28j60_read_reg(dev, priv, ENC28J60_EREVID, &revision);
	if (ret)
		return ret;
	if (!revision || revision == 0xff) {
		dev_err(dev, "ENC28J60 not detected (revision 0x%02x)\n",
			revision);
		return -ENODEV;
	}

	ret = enc28j60_write_reg(dev, priv, ENC28J60_ECON2,
				 ENC28J60_ECON2_AUTOINC |
				 ENC28J60_ECON2_VRPS);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXSTL,
				  ENC28J60_RX_START);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXNDL,
				  ENC28J60_RX_END);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXRDPTL,
				  ENC28J60_RX_END);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERDPTL,
				  ENC28J60_RX_START);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ETXSTL,
				  ENC28J60_TX_START);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ETXNDL,
				  ENC28J60_TX_END);
	if (ret)
		return ret;

	ret = enc28j60_write_reg(dev, priv, ENC28J60_ERXFCON,
				 ENC28J60_ERXFCON_UCEN |
				 ENC28J60_ERXFCON_CRCEN |
				 ENC28J60_ERXFCON_BCEN);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MACON1,
				 ENC28J60_MACON1_MARXEN |
				 ENC28J60_MACON1_TXPAUS |
				 ENC28J60_MACON1_RXPAUS);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MACON3,
				 ENC28J60_MACON3_PADCFG0 |
				 ENC28J60_MACON3_TXCRCEN |
				 ENC28J60_MACON3_FRMLNEN |
				 ENC28J60_MACON3_FULDPX);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MACON4, 0);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MAIPGL, 0x12);
	if (ret)
		return ret;
	ret = enc28j60_write_reg(dev, priv, ENC28J60_MABBIPG, 0x15);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_MAMXFLL,
				  ENC28J60_MAX_FRAME);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(mac_regs); i++) {
		ret = enc28j60_write_reg(dev, priv, mac_regs[i],
					 plat->enetaddr[i]);
		if (ret)
			return ret;
	}

	ret = enc28j60_phy_write(dev, priv, ENC28J60_PHLCON, 0x3476);
	if (ret)
		return ret;
	ret = enc28j60_phy_write(dev, priv, ENC28J60_PHCON1,
				 ENC28J60_PHCON1_PDPXMD);
	if (ret)
		return ret;
	ret = enc28j60_phy_write(dev, priv, ENC28J60_PHCON2, 0x0100);
	if (ret)
		return ret;

	priv->next_packet = ENC28J60_RX_START;
	ret = enc28j60_set_bits(dev, ENC28J60_BFS, ENC28J60_ECON1,
				ENC28J60_ECON1_RXEN);
	if (ret)
		return ret;

	if (!enc28j60_phy_read(dev, priv, ENC28J60_PHSTAT2, &phstat2))
		dev_info(dev, "link %s (PHSTAT2 0x%04x)\n",
			 phstat2 & ENC28J60_PHSTAT2_LSTAT ? "up" : "down",
			 phstat2);

	return 0;
}

static int enc28j60_probe(struct udevice *dev)
{
	struct enc28j60_priv *priv = dev_get_priv(dev);
	u8 revision;
	int ret;

	ret = enc28j60_reset(dev, priv);
	if (ret)
		return ret;

	ret = enc28j60_read_reg(dev, priv, ENC28J60_EREVID, &revision);
	if (ret)
		return ret;
	if (!revision || revision == 0xff)
		return -ENODEV;

	dev_info(dev, "ENC28J60 revision %u\n", revision);
	return 0;
}

static int enc28j60_start(struct udevice *dev)
{
	struct enc28j60_priv *priv = dev_get_priv(dev);

	return enc28j60_hw_init(dev, priv);
}

static void enc28j60_stop(struct udevice *dev)
{
	int ret;

	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_ECON1,
				 ENC28J60_ECON1_RXEN);
	if (ret)
		dev_err(dev, "failed to stop receiver (%d)\n", ret);
}

static int enc28j60_tx_reset(struct udevice *dev)
{
	int ret;

	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_ECON1,
				 ENC28J60_ECON1_TXRTS);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFS, ENC28J60_ECON1,
				 ENC28J60_ECON1_TXRST);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_ECON1,
				 ENC28J60_ECON1_TXRST);
	if (ret)
		return ret;

	return enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_EIR,
				 ENC28J60_EIR_TXERIF);
}

static int enc28j60_send(struct udevice *dev, void *packet, int length)
{
	struct enc28j60_priv *priv = dev_get_priv(dev);
	u8 ctrl = 0;
	u8 value;
	int ret, i;

	if (length <= 0 || length > ENC28J60_MAX_FRAME - 4)
		return -EINVAL;

	ret = enc28j60_read_reg(dev, priv, ENC28J60_ECON1, &value);
	if (ret)
		return ret;
	for (i = 0; value & ENC28J60_ECON1_TXRTS && i < 10000; i++) {
		udelay(10);
		ret = enc28j60_read_reg(dev, priv, ENC28J60_ECON1, &value);
		if (ret)
			return ret;
	}
	if (value & ENC28J60_ECON1_TXRTS) {
		ret = enc28j60_tx_reset(dev);
		if (ret)
			return ret;
		return -ETIMEDOUT;
	}

	ret = enc28j60_write_word(dev, priv, ENC28J60_EWRPTL,
				  ENC28J60_TX_START);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ETXNDL,
				  ENC28J60_TX_START + length);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_EIR,
				 ENC28J60_EIR_TXERIF);
	if (ret)
		return ret;
	ret = enc28j60_write_buf(dev, &ctrl, 1);
	if (ret)
		return ret;
	ret = enc28j60_write_buf(dev, packet, length);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFS, ENC28J60_ECON1,
				 ENC28J60_ECON1_TXRTS);
	if (ret)
		return ret;

	for (i = 0; i < 10000; i++) {
		ret = enc28j60_read_reg(dev, priv, ENC28J60_ECON1, &value);
		if (ret)
			return ret;
		if (!(value & ENC28J60_ECON1_TXRTS))
			break;
		udelay(10);
	}
	if (i == 10000) {
		ret = enc28j60_tx_reset(dev);
		if (ret)
			return ret;
		return -ETIMEDOUT;
	}

	ret = enc28j60_read_reg(dev, priv, ENC28J60_ESTAT, &value);
	if (ret)
		return ret;
	if (value & ENC28J60_ESTAT_TXABRT) {
		ret = enc28j60_tx_reset(dev);
		if (ret)
			return ret;
		return -EIO;
	}

	return 0;
}

static int enc28j60_reset_rx(struct udevice *dev, struct enc28j60_priv *priv)
{
	int ret;

	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_ECON1,
				 ENC28J60_ECON1_RXEN);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFS, ENC28J60_ECON1,
				 ENC28J60_ECON1_RXRST);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_ECON1,
				 ENC28J60_ECON1_RXRST);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXSTL,
				  ENC28J60_RX_START);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXNDL,
				  ENC28J60_RX_END);
	if (ret)
		return ret;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXRDPTL,
				  ENC28J60_RX_END);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFC, ENC28J60_EIR,
				 ENC28J60_EIR_RXERIF);
	if (ret)
		return ret;
	priv->next_packet = ENC28J60_RX_START;

	return enc28j60_set_bits(dev, ENC28J60_BFS, ENC28J60_ECON1,
				 ENC28J60_ECON1_RXEN);
}

static int enc28j60_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct enc28j60_priv *priv = dev_get_priv(dev);
	u8 rsv[ENC28J60_RSV_SIZE];
	u8 count;
	u16 next, length, status, rdpt;
	u16 data_start;
	int ret;

	(void)flags;
	ret = enc28j60_read_reg(dev, priv, ENC28J60_EPKTCNT, &count);
	if (ret)
		return ret;
	if (!count)
		return -EAGAIN;
	if (priv->next_packet > ENC28J60_RX_END) {
		ret = enc28j60_reset_rx(dev, priv);
		return ret ? ret : -EIO;
	}

	ret = enc28j60_write_word(dev, priv, ENC28J60_ERDPTL,
				  priv->next_packet);
	if (ret)
		return ret;
	ret = enc28j60_read_buf(dev, rsv, sizeof(rsv));
	if (ret)
		return ret;

	next = rsv[0] | (rsv[1] << 8);
	length = rsv[2] | (rsv[3] << 8);
	status = rsv[4] | (rsv[5] << 8);
	if (next > ENC28J60_RX_END || length > ENC28J60_MAX_FRAME) {
		dev_err(dev, "invalid RX packet metadata\n");
		ret = enc28j60_reset_rx(dev, priv);
		return ret ? ret : -EIO;
	}

	data_start = priv->next_packet + ENC28J60_RSV_SIZE;
	if (data_start > ENC28J60_RX_END)
		data_start -= ENC28J60_RX_END + 1;

	if ((status & BIT(7)) && length >= 14) {
		ret = enc28j60_write_word(dev, priv, ENC28J60_ERDPTL,
					  data_start);
		if (ret)
			return ret;
		ret = enc28j60_read_buf(dev, net_rx_packets[0], length);
		if (ret)
			return ret;
		*packetp = net_rx_packets[0];
	}

	rdpt = next ? next - 1 : ENC28J60_RX_END;
	ret = enc28j60_write_word(dev, priv, ENC28J60_ERXRDPTL, rdpt);
	if (ret)
		return ret;
	ret = enc28j60_set_bits(dev, ENC28J60_BFS, ENC28J60_ECON2,
				 ENC28J60_ECON2_PKTDEC);
	if (ret)
		return ret;
	priv->next_packet = next;

	if (!(status & BIT(7)) || length < 14)
		return -EAGAIN;

	return length;
}

static int enc28j60_read_rom_hwaddr(struct udevice *dev)
{
	(void)dev;
	return -ENOSYS;
}

static const struct eth_ops enc28j60_ops = {
	.start = enc28j60_start,
	.send = enc28j60_send,
	.recv = enc28j60_recv,
	.stop = enc28j60_stop,
	.read_rom_hwaddr = enc28j60_read_rom_hwaddr,
};

static const struct udevice_id enc28j60_ids[] = {
	{ .compatible = "microchip,enc28j60" },
	{ }
};

U_BOOT_DRIVER(enc28j60) = {
	.name = "enc28j60",
	.id = UCLASS_ETH,
	.of_match = enc28j60_ids,
	.probe = enc28j60_probe,
	.ops = &enc28j60_ops,
	.priv_auto = sizeof(struct enc28j60_priv),
	.plat_auto = sizeof(struct eth_pdata),
};
