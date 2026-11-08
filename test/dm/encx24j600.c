// SPDX-License-Identifier: GPL-2.0-only

#include <dm.h>
#include <net.h>
#include <spi.h>
#include <dm/test.h>
#include <test/test.h>
#include <test/ut.h>

#define RX_START	0x600
#define SRAM_SIZE	0x6000
#define ESTAT		0x1a
#define EIR		0x1c
#define ECON1		0x1e
#define ERXTAIL		0x06
#define MIREGADR	0x54
#define MICMD		0x52
#define MIRD		0x68
#define MIWR		0x66
#define EIDLED		0x74

struct enc_emul {
	u8 reg[128];
	u16 phy[32];
	u8 ram[SRAM_SIZE];
	u8 bank;
	u8 cmd;
	u16 offset;
	u16 rxptr;
	u16 txptr;
	bool fail;
	bool tx_abort;
	bool autoneg_complete;
};

static void enc_emul_word(struct enc_emul *emul, int reg, u16 value)
{
	emul->reg[reg] = value;
	emul->reg[reg + 1] = value >> 8;
}

static u16 enc_emul_get_word(struct enc_emul *emul, int reg)
{
	return emul->reg[reg] | (emul->reg[reg + 1] << 8);
}

static void enc_emul_reset(struct enc_emul *emul)
{
	memset(emul->reg, 0, sizeof(emul->reg));
	memset(emul->phy, 0, sizeof(emul->phy));
	emul->bank = 0;
	enc_emul_word(emul, ESTAT, BIT(12) | BIT(10) | BIT(8));
	enc_emul_word(emul, EIDLED, 0x21);
	enc_emul_word(emul, 0x64, 0x1102);
	enc_emul_word(emul, 0x62, 0x3322);
	enc_emul_word(emul, 0x60, 0x5544);
	emul->phy[1] = emul->autoneg_complete ? BIT(5) : 0;
	emul->phy[0x1f] = BIT(3);
}

static int enc_emul_xfer(struct udevice *dev, uint bitlen, const void *dout,
			 void *din, ulong flags)
{
	struct enc_emul *emul = dev_get_priv(dev);
	const u8 *tx = dout;
	u8 *rx = din;
	uint count = bitlen / 8;
	int i;

	if (emul->fail) {
		emul->fail = false;
		return -EIO;
	}
	if (bitlen % 8)
		return -EINVAL;
	if (flags & SPI_XFER_BEGIN) {
		if (!count || !tx)
			return -EINVAL;
		emul->cmd = *tx++;
		emul->offset = 0;
		count--;
		switch (emul->cmd) {
		case 0xc0:
		case 0xc2:
		case 0xc4:
		case 0xc6:
			emul->bank = (emul->cmd & 6) >> 1;
			return 0;
		case 0xca:
			enc_emul_reset(emul);
			return 0;
		case 0xcc:
			if (emul->reg[ESTAT])
				emul->reg[ESTAT]--;
			return 0;
		case 0xd4:
			emul->reg[EIR] |= emul->tx_abort ? BIT(2) : BIT(3);
			emul->reg[ECON1] &= ~BIT(1);
			return 0;
		case 0xe8:
			emul->reg[ECON1] |= BIT(0);
			return 0;
		case 0xea:
			emul->reg[ECON1] &= ~BIT(0);
			return 0;
		}
	}

	for (i = 0; i < count; i++, emul->offset++) {
		u8 cmd = emul->cmd;
		int reg = cmd & 0x1f;

		if (cmd == 0x64 || cmd == 0x6c) {
			u16 *ptr = cmd == 0x64 ? &emul->rxptr : &emul->txptr;

			if (!tx || emul->offset > 1)
				return -EINVAL;
			if (!emul->offset)
				*ptr = tx[i];
			else
				*ptr |= tx[i] << 8;
		} else if (cmd == 0x2a) {
			if (!tx || emul->txptr >= SRAM_SIZE)
				return -EINVAL;
			emul->ram[emul->txptr++] = tx[i];
		} else if (cmd == 0x2c) {
			if (!rx || emul->rxptr >= SRAM_SIZE)
				return -EINVAL;
			rx[i] = emul->ram[emul->rxptr++];
			if (emul->rxptr == SRAM_SIZE)
				emul->rxptr = RX_START;
		} else {
			if (reg < 0x16)
				reg += emul->bank * 0x20;
			reg += emul->offset;
			if (reg >= sizeof(emul->reg))
				return -EINVAL;
			switch (cmd & 0xe0) {
			case 0x00:
				if (!rx)
					return -EINVAL;
				rx[i] = emul->reg[reg];
				break;
			case 0x40:
				if (!tx)
					return -EINVAL;
				emul->reg[reg] = tx[i];
				break;
			case 0x80:
				if (!tx)
					return -EINVAL;
				emul->reg[reg] |= tx[i];
				break;
			case 0xa0:
				if (!tx)
					return -EINVAL;
				emul->reg[reg] &= ~tx[i];
				break;
			default:
				return -EINVAL;
			}
		}
	}
	if ((flags & SPI_XFER_END) && (emul->cmd & 0xe0) == 0x40) {
		if (emul->bank == 2 && (emul->cmd & 0x1f) == 0x12 &&
		    enc_emul_get_word(emul, MICMD) & 1)
			enc_emul_word(emul, MIRD,
				      emul->phy[emul->reg[MIREGADR] & 31]);
		if (emul->bank == 3 && (emul->cmd & 0x1f) == 0x06) {
			emul->phy[emul->reg[MIREGADR] & 31] =
				enc_emul_get_word(emul, MIWR);
			if ((emul->reg[MIREGADR] & 31) == 0) {
				u16 control = emul->phy[0];

				if (!(control & BIT(12))) {
					emul->reg[ESTAT + 1] &= ~BIT(2);
					if (control & BIT(8))
						emul->reg[ESTAT + 1] |= BIT(2);
				} else if (emul->autoneg_complete) {
					emul->phy[1] = BIT(5);
				}
			}
		}
	}
	return 0;
}

static int enc_emul_probe(struct udevice *dev)
{
	struct enc_emul *emul = dev_get_priv(dev);

	emul->autoneg_complete = true;
	enc_emul_reset(emul);
	return 0;
}

static const struct dm_spi_emul_ops enc_emul_ops = {
	.xfer = enc_emul_xfer,
};

static const struct udevice_id enc_emul_ids[] = {
	{ .compatible = "sandbox,encx24j600" },
	{ }
};

U_BOOT_DRIVER(sandbox_encx24j600) = {
	.name = "sandbox_encx24j600",
	.id = UCLASS_SPI_EMUL,
	.of_match = enc_emul_ids,
	.probe = enc_emul_probe,
	.priv_auto = sizeof(struct enc_emul),
	.ops = &enc_emul_ops,
};

static void enc_emul_packet(struct enc_emul *emul, u16 pos, u16 next,
			    const u8 *data, u16 len, bool valid)
{
	u8 rsv[8] = { next, next >> 8, len, len >> 8,
		      valid ? BIT(7) : 0 };
	int i;

	for (i = 0; i < sizeof(rsv) + len; i++) {
		emul->ram[pos++] = i < sizeof(rsv) ? rsv[i] : data[i - sizeof(rsv)];
		if (pos == SRAM_SIZE)
			pos = RX_START;
	}
	emul->reg[ESTAT] = 1;
}

static int dm_test_encx24j600(struct unit_test_state *uts)
{
	const u8 frame[64] = { 0x12, 0x34, 0x56, 0x78 };
	const struct eth_ops *ops;
	struct udevice *dev, *emuldev;
	struct enc_emul *emul;
	uchar *packet;

	ut_assertok(net_init());
	ut_assertok(uclass_get_device_by_name(UCLASS_ETH, "ethernet@2", &dev));
	ut_assertok(uclass_get_device_by_name(UCLASS_SPI_EMUL, "encx24j600-emul",
					      &emuldev));
	emul = dev_get_priv(emuldev);
	ops = eth_get_ops(dev);
	ut_assertok(ops->start(dev));
	ut_asserteq(0x5ffe, enc_emul_get_word(emul, ERXTAIL));
	ut_asserteq(-EAGAIN, ops->recv(dev, 0, &packet));
	ut_asserteq(-EINVAL, ops->send(dev, (void *)frame, 0));
	ut_assertok(ops->send(dev, (void *)frame, sizeof(frame)));
	ut_asserteq_mem(frame, emul->ram, sizeof(frame));

	enc_emul_packet(emul, RX_START, 0x5ffc, frame, sizeof(frame), true);
	ut_asserteq(60, ops->recv(dev, 0, &packet));
	ut_asserteq_mem(frame, packet, 60);
	ut_asserteq(0x5ffa, enc_emul_get_word(emul, ERXTAIL));
	enc_emul_packet(emul, 0x5ffc, RX_START, frame, sizeof(frame), true);
	ut_asserteq(60, ops->recv(dev, 0, &packet));
	ut_asserteq_mem(frame, packet, 60);
	ut_asserteq(0x5ffe, enc_emul_get_word(emul, ERXTAIL));
	enc_emul_packet(emul, RX_START, 0x650, frame, sizeof(frame), false);
	ut_asserteq(-EAGAIN, ops->recv(dev, 0, &packet));
	ut_asserteq(0, emul->reg[ESTAT]);
	emul->tx_abort = true;
	ut_asserteq(-EIO, ops->send(dev, (void *)frame, sizeof(frame)));
	emul->fail = true;
	ut_asserteq(-EIO, ops->send(dev, (void *)frame, sizeof(frame)));
	emul->tx_abort = false;
	ut_assertok(ops->send(dev, (void *)frame, sizeof(frame)));
	enc_emul_packet(emul, 0x650, 0x400, frame, sizeof(frame), true);
	ut_asserteq(-EIO, ops->recv(dev, 0, &packet));
	ut_asserteq(-EIO, ops->recv(dev, 0, &packet));
	ut_assertok(ops->start(dev));
	ut_asserteq(-EAGAIN, ops->recv(dev, 0, &packet));
	emul->autoneg_complete = false;
	ut_assertok(ops->start(dev));
	ut_assert(!(emul->phy[0] & BIT(12)));
	ut_assert(emul->phy[0] & BIT(13));
	ut_assert(!(emul->phy[0] & BIT(8)));
	ut_asserteq(0x12, enc_emul_get_word(emul, 0x44));
	emul->autoneg_complete = true;
	emul->reg[EIR + 1] |= BIT(3);
	ut_assertok(ops->send(dev, (void *)frame, sizeof(frame)));
	ut_assert(emul->phy[0] & BIT(12));
	ops->stop(dev);
	return 0;
}

DM_TEST(dm_test_encx24j600, UTF_SCAN_PDATA | UTF_SCAN_FDT);
