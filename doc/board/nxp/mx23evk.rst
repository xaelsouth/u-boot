.. SPDX-License-Identifier: GPL-2.0+

i.MX23 EVK
==========

LCD
---

The EVK configuration enables the MXS LCDIF driver and displays the built-in
U-Boot logo during U-Boot proper. The serial console remains available. The
U-Boot device tree uses the 480x272, 9 MHz panel timings from the EVK Linux
configuration; a different panel may need different timings.

The built-in image is ``drivers/video/u_boot_logo.bmp``. The video device is
probed during late board initialization even when the console is configured
only for serial output. A failure to initialize the LCD is reported on the
console and does not prevent booting.

U-Boot enables the LCD's 3.3 V supply and drives the backlight's PWM2 pin
high as a GPIO, without adjustable brightness.

The LCD and NAND should be tested together on the target board, along with
the handover to Linux. An illuminated backlight alone does not confirm that
the LCDIF is displaying the framebuffer.
