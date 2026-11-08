.. SPDX-License-Identifier: GPL-2.0+

.. index::
   single: usbnet (command)

usbnet command
==============

Synopsis
--------

::

    usbnet

Description
-----------

The usbnet command binds the USB Ethernet gadget and selects ``usb_ether``
as the active Ethernet interface by setting the ``ethact`` environment
variable. U-Boot acts as a USB Ethernet device connected to a host computer.

The command does not itself configure an IP address or transfer data. Use a
network command such as ``dhcp`` or ``ping`` afterwards. The host must have
a compatible USB Ethernet driver and network configuration. The selected
gadget protocol and the host's network setup determine whether DHCP is
available.

The gadget uses the USB controller in device mode. Once it is active, other
gadgets, such as fastboot, cannot use the same controller simultaneously.
Switching between gadget and host modes or between gadgets on MXS requires
appropriate removal and reinitialization; the sequence has not been verified.

Example
-------

With a DHCP server configured on the connected host-side network:

::

    => usbnet
    => dhcp

Configuration
-------------

The command is provided by the MXS EHCI driver and requires
``CONFIG_USB_ETHER`` and the corresponding USB gadget and network support.
Select either ``CONFIG_USB_ETH_RNDIS`` or ``CONFIG_USB_ETH_CDC`` for a
protocol supported by the host. The device and host MAC addresses are set
by ``CONFIG_USBNET_DEV_ADDR`` and ``CONFIG_USBNET_HOST_ADDR``, respectively.

Return value
------------

The command returns 0 when the USB Ethernet device was bound, and 1 if
binding fails. A successful return does not guarantee that the host has
enumerated the gadget or that a network connection is available.
