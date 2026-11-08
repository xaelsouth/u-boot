// SPDX-License-Identifier: GPL-2.0+
/*
 * Olimex MX23 Olinuxino board
 *
 * Copyright (C) 2013 Marek Vasut <marex@denx.de>
 */

#include <init.h>
#include <dm.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/iomux-mx23.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);

	return 0;
}

#ifdef CONFIG_CMD_USB
int board_ehci_hcd_init(int port)
{
	/* Enable LAN9512 (Maxi) or GL850G (Mini) USB HUB power. */
	mxs_iomux_setup_pad(MX23_PAD_GPMI_ALE__GPIO_0_17);
	gpio_direction_output(MX23_PAD_GPMI_ALE__GPIO_0_17, 1);
	udelay(1000);
	return 0;
}

int board_ehci_hcd_exit(int port)
{
	/* Enable LAN9512 (Maxi) or GL850G (Mini) USB HUB power. */
	gpio_direction_output(MX23_PAD_GPMI_ALE__GPIO_0_17, 0);
	return 0;
}
#endif

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
	int  ret = 0;

	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	if (IS_ENABLED(CONFIG_ENCX24J600)) {
		static const iomux_cfg_t uext_spi_pads[] = {
			MX23_PAD_GPMI_WRN__SSP2_SCK | MXS_PAD_8MA | MXS_PAD_PULLUP,
			MX23_PAD_GPMI_RDY1__SSP2_CMD | MXS_PAD_8MA | MXS_PAD_PULLUP,
			MX23_PAD_GPMI_D00__SSP2_DATA0 | MXS_PAD_8MA | MXS_PAD_PULLUP,
			MX23_PAD_GPMI_D03__SSP2_DATA3 | MXS_PAD_8MA | MXS_PAD_PULLUP,
		};

		struct gpio_desc irq;

		mxs_iomux_setup_multiple_pads(uext_spi_pads,
					      ARRAY_SIZE(uext_spi_pads));
		ret = gpio_request_by_name_nodev(ofnode_path("/"),
						 "u-boot,enc-int-gpios", 0,
						 &irq, GPIOD_IS_IN);
		if (ret)
			printf("ENC624J600 interrupt pin setup failed: %d\n", ret);
		else {
			mxs_iomux_setup_pad(MX23_PAD_LCD_ENABLE__GPIO_1_23 |
					    MXS_PAD_4MA | MXS_PAD_PULLUP);
			/* Return 0 anyway. */
			ret = 0;
		}
	}

	return ret;
}
