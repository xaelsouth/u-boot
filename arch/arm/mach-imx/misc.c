// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2013 Stefan Roese <sr@denx.de>
 */

#include <lmb.h>
#include <log.h>
#include <asm/arch/sys_proto.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/mach-imx/regs-common.h>

/* 1 second delay should be plenty of time for block reset. */
#define	RESET_MAX_TIMEOUT	1000000

#define	MXS_BLOCK_SFTRST	(1 << 31)
#define	MXS_BLOCK_CLKGATE	(1 << 30)

int mxs_wait_mask_set(struct mxs_register_32 *reg, uint32_t mask, unsigned
								int timeout)
{
	while (--timeout) {
		if ((readl(&reg->reg) & mask) == mask)
			break;
		udelay(1);
	}

	return !timeout;
}

int mxs_wait_mask_clr(struct mxs_register_32 *reg, uint32_t mask, unsigned
								int timeout)
{
	while (--timeout) {
		if ((readl(&reg->reg) & mask) == 0)
			break;
		udelay(1);
	}

	return !timeout;
}

/*
 * The current mxs_reset_block() will do two things:
 *  [1] enable the module.
 *  [2] reset the module.
 *
 * In most of the cases, it's ok.
 * But in MX23, there is a hardware bug in the BCH block (see erratum #2847).
 * If you try to soft reset the BCH block, it becomes unusable until
 * the next hard reset. This case occurs in the NAND boot mode. When the board
 * boots by NAND, the ROM of the chip will initialize the BCH blocks itself.
 * So If the driver tries to reset the BCH again, the BCH will not work anymore.
 * You will see a DMA timeout in this case. The bug has been fixed
 * in the following chips, such as MX28.
 *
 * To avoid this bug, just add a new parameter `just_enable` for
 * the mxs_reset_block(), and rewrite it here.
 */
int gpmi_mxs_reset_block(struct mxs_register_32 *reg, bool just_enable)
{
	/* Clear SFTRST */
	writel(MXS_BLOCK_SFTRST, &reg->reg_clr);

	if (mxs_wait_mask_clr(reg, MXS_BLOCK_SFTRST, RESET_MAX_TIMEOUT))
		return 1;

	/* Clear CLKGATE */
	writel(MXS_BLOCK_CLKGATE, &reg->reg_clr);

	if (!just_enable) {
		/* Set SFTRST */
		writel(MXS_BLOCK_SFTRST, &reg->reg_set);

		/* Wait for CLKGATE being set */
		if (mxs_wait_mask_set(reg, MXS_BLOCK_CLKGATE, RESET_MAX_TIMEOUT))
			return 1;
	}

	/* Clear SFTRST */
	writel(MXS_BLOCK_SFTRST, &reg->reg_clr);

	if (mxs_wait_mask_clr(reg, MXS_BLOCK_SFTRST, RESET_MAX_TIMEOUT))
		return 1;

	/* Clear CLKGATE */
	writel(MXS_BLOCK_CLKGATE, &reg->reg_clr);

	if (mxs_wait_mask_clr(reg, MXS_BLOCK_CLKGATE, RESET_MAX_TIMEOUT))
		return 1;

	return 0;
}

int mxs_reset_block(struct mxs_register_32 *reg)
{
	/* Clear SFTRST */
	writel(MXS_BLOCK_SFTRST, &reg->reg_clr);

	if (mxs_wait_mask_clr(reg, MXS_BLOCK_SFTRST, RESET_MAX_TIMEOUT))
		return 1;

	/* Clear CLKGATE */
	writel(MXS_BLOCK_CLKGATE, &reg->reg_clr);

	/* Set SFTRST */
	writel(MXS_BLOCK_SFTRST, &reg->reg_set);

	/* Wait for CLKGATE being set */
	if (mxs_wait_mask_set(reg, MXS_BLOCK_CLKGATE, RESET_MAX_TIMEOUT))
		return 1;

	/* Clear SFTRST */
	writel(MXS_BLOCK_SFTRST, &reg->reg_clr);

	if (mxs_wait_mask_clr(reg, MXS_BLOCK_SFTRST, RESET_MAX_TIMEOUT))
		return 1;

	/* Clear CLKGATE */
	writel(MXS_BLOCK_CLKGATE, &reg->reg_clr);

	if (mxs_wait_mask_clr(reg, MXS_BLOCK_CLKGATE, RESET_MAX_TIMEOUT))
		return 1;

	return 0;
}
