// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale MX23EVK board
 *
 * (C) Copyright 2013 O.S. Systems Software LTDA.
 *
 * Author: Otavio Salvador <otavio@ossystems.com.br>
 *
 * Based on m28evk.c:
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 */

#include <init.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux-mx23.h>
#include <asm/arch/sys_proto.h>
#include <linux/libfdt.h>

#define	MUX_CONFIG_SSP1	(MXS_PAD_8MA | MXS_PAD_PULLUP)
#define	MUX_CONFIG_LCD	(MXS_PAD_4MA | MXS_PAD_NOPULL)
#define	MUX_CONFIG_GPMI_NAND		(MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_NOPULL)
#define	MUX_CONFIG_GPMI_NAND_FIXUP	(MXS_PAD_3V3 | MXS_PAD_12MA | MXS_PAD_NOPULL)
#define MUX_CONFIG_ENC28J60_INT		(MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP)

/* Except for EMI pins below is the copy of iomux_setup from mx23evk/spl_boot.c. */
static const iomux_cfg_t iomux_setup[] = {
	/* DUART */
	MX23_PAD_PWM0__DUART_RX,
	MX23_PAD_PWM1__DUART_TX,

	/* MMC 0 */
	MX23_PAD_SSP1_DATA0__SSP1_DATA0 | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_DATA1__SSP1_DATA1 | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_DATA2__SSP1_DATA2 | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_DATA3__SSP1_DATA3 | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_CMD__SSP1_CMD | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_DETECT__SSP1_DETECT |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	MX23_PAD_SSP1_SCK__SSP1_SCK |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	/* I2C bus (bit-banged) */
	MX23_PAD_I2C_SCL__GPIO_0_30 |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	MX23_PAD_I2C_SDA__GPIO_0_31 |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	/* Write Protect Pin */
	MX23_PAD_PWM4__GPIO_1_30 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	/* Slot Power Enable */
	MX23_PAD_PWM3__GPIO_1_29 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	/* LCD */
	MX23_PAD_LCD_D00__LCD_D00 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D01__LCD_D01 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D02__LCD_D02 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D03__LCD_D03 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D04__LCD_D04 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D05__LCD_D05 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D06__LCD_D06 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D07__LCD_D07 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D08__LCD_D08 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D09__LCD_D09 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D10__LCD_D10 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D11__LCD_D11 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D12__LCD_D12 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D13__LCD_D13 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D14__LCD_D14 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D15__LCD_D15 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D16__LCD_D16 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_D17__LCD_D17 | MUX_CONFIG_LCD,
	MX23_PAD_GPMI_D08__LCD_D18 | MUX_CONFIG_LCD,
	MX23_PAD_GPMI_D09__LCD_D19 | MUX_CONFIG_LCD,
	MX23_PAD_GPMI_D10__LCD_D20 | MUX_CONFIG_LCD,
	MX23_PAD_GPMI_D11__LCD_D21 | MUX_CONFIG_LCD,
	MX23_PAD_GPMI_D12__LCD_D22 | MUX_CONFIG_LCD,
	MX23_PAD_GPMI_D13__LCD_D23 | MUX_CONFIG_LCD,
	MX23_PAD_LCD_DOTCK__LCD_DOTCK | MUX_CONFIG_LCD,
	MX23_PAD_LCD_ENABLE__LCD_ENABLE | MUX_CONFIG_LCD,
	MX23_PAD_LCD_HSYNC__LCD_HSYNC | MUX_CONFIG_LCD,
	MX23_PAD_LCD_VSYNC__LCD_VSYNC | MUX_CONFIG_LCD,
	MX23_PAD_LCD_RESET__GPIO_1_18 | MUX_CONFIG_LCD, /* LCD power */
	MX23_PAD_PWM2__GPIO_1_28 | MUX_CONFIG_LCD, /* LCD contrast */

#ifdef CONFIG_NAND_MXS
	/* GPMI NAND */
	MX23_PAD_GPMI_D00__GPMI_D00 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D01__GPMI_D01 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D02__GPMI_D02 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D03__GPMI_D03 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D04__GPMI_D04 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D05__GPMI_D05 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D06__GPMI_D06 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_D07__GPMI_D07 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_CLE__GPMI_CLE | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_ALE__GPMI_ALE | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_RDY0__GPMI_RDY0 | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_WPN__GPMI_WPN | MUX_CONFIG_GPMI_NAND_FIXUP,
	MX23_PAD_GPMI_WRN__GPMI_WRN | MUX_CONFIG_GPMI_NAND_FIXUP,
	MX23_PAD_GPMI_RDN__GPMI_RDN | MUX_CONFIG_GPMI_NAND_FIXUP,
	MX23_PAD_GPMI_CE0N__GPMI_CE0N | MUX_CONFIG_GPMI_NAND,
	MX23_PAD_GPMI_CE1N__GPIO_2_27 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	MX23_PAD_GPMI_CE2N__GPIO_0_18 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	/* GPMI_CE3N */
	MX23_PAD_ROTARYB__GPIO_2_8 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	MX23_PAD_GPMI_RDY1__GPIO_0_20 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	MX23_PAD_GPMI_RDY2__GPIO_0_21 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	MX23_PAD_GPMI_RDY3__GPIO_0_22 |
		(MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
#endif
};

DECLARE_GLOBAL_DATA_PTR;

static bool mx23evk_use_enc28j60(void)
{
	uint8_t mode = MXS_SPL_DATA->boot_mode_idx;
	uint8_t pads;

	if (mode >= ARRAY_SIZE(mxs_boot_modes))
		return false;

	pads = mxs_boot_modes[mode].boot_pads;
	return pads == MXS_BM_USB || pads == MXS_BM_NAND_3V3 ||
	       pads == MXS_BM_NAND_1V8;
}

int board_fdt_blob_setup(void **fdtp)
{
	void *fdt;
	fdt32_t pinctrl[2];
	int ssp, enc, spi_pins, irq_pins, ret;

	if (!fdtp || !*fdtp)
		return -EINVAL;
	fdt = *fdtp;
	ssp = fdt_path_offset(fdt, "/apb@80000000/apbh@80000000/spi@80010000");
	if (ssp < 0)
		return ssp;

	enc = fdt_subnode_offset(fdt, ssp, "ethernet@0");
	if (enc < 0)
		return enc;

	if (!mx23evk_use_enc28j60())
		return fdt_setprop_inplace(fdt, enc, "status", "nope",
					   sizeof("nope"));

	spi_pins = fdt_path_offset(fdt,
		"/apb@80000000/apbh@80000000/pinctrl@80018000/enc28j60-spi@0");
	irq_pins = fdt_path_offset(fdt,
		"/apb@80000000/apbh@80000000/pinctrl@80018000/enc28j60-irq@0");
	if (spi_pins < 0 || irq_pins < 0)
		return -FDT_ERR_NOTFOUND;

	if (!fdt_get_phandle(fdt, spi_pins) ||
	    !fdt_get_phandle(fdt, irq_pins))
		return -FDT_ERR_NOTFOUND;
	pinctrl[0] = cpu_to_fdt32(fdt_get_phandle(fdt, spi_pins));
	pinctrl[1] = cpu_to_fdt32(fdt_get_phandle(fdt, irq_pins));
	ret = fdt_setprop_inplace(fdt, ssp, "compatible", "fsl,imx23-spi",
				  sizeof("fsl,imx23-spi"));
	if (ret)
		return ret;
	ret = fdt_setprop_inplace(fdt, ssp, "pinctrl-0", pinctrl,
				  sizeof(pinctrl));
	if (ret)
		return ret;

	return fdt_setprop_inplace(fdt, enc, "status", "okay", sizeof("okay"));
}

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);

	/* Power on LCD */
	gpio_direction_output(MX23_PAD_LCD_RESET__GPIO_1_18, 1);

	/* Set contrast to maximum */
	gpio_direction_output(MX23_PAD_PWM2__GPIO_1_28, 1);

#ifdef CONFIG_NAND_MXS
	mxs_set_gpmiclk(22000, 1);
#endif

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
	static const iomux_cfg_t enc28j60_iomux[] = {
		MX23_PAD_SSP1_DATA0__SSP1_DATA0 | MUX_CONFIG_SSP1,
		MX23_PAD_SSP1_DATA3__SSP1_DATA3 | MUX_CONFIG_SSP1,
		MX23_PAD_SSP1_CMD__SSP1_CMD | MUX_CONFIG_SSP1,
		MX23_PAD_SSP1_SCK__SSP1_SCK | MUX_CONFIG_SSP1,
		MX23_PAD_SSP1_DATA1__GPIO_2_3 | MUX_CONFIG_ENC28J60_INT,
		MX23_PAD_SSP1_DATA2__GPIO_2_4 | MUX_CONFIG_ENC28J60_INT,
	};

	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	mxs_iomux_setup_multiple_pads(iomux_setup, ARRAY_SIZE(iomux_setup));
	if (mx23evk_use_enc28j60())
		mxs_iomux_setup_multiple_pads(enc28j60_iomux,
					      ARRAY_SIZE(enc28j60_iomux));

	return 0;
}
