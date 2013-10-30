/*
 * Ingenic burner Gadget Code
 *
 * Copyright (c) 2013 cli <cli@ingenic.cn>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <errno.h>
#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <part.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/compiler.h>
#include <linux/usb/composite.h>
#include "gadget_chips.h"
#include "composite.c"

#define MANUFACTURER_IDX	0
#define PRODUCT_IDX		1
#define SERIALNUMBER_IDX	2

static const char shortname[] = {"jz_usb_burner_"};
static const char manufacturer[] = {"Ingenic"};
static const char product[] = {"Usb burntool Gadget"};
static const char serialnumber[] = {"Version 1.7"};

struct usb_device_descriptor device_desc= {
	.bLength = sizeof(device_desc),
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.idVendor = __constant_cpu_to_le16(CONFIG_USB_VENDOR_ID),
	.idProduct = __constant_cpu_to_le16(CONFIG_USB_PRODUCT_ID),
	.bNumConfigurations = 1,
};

struct usb_string g_bt_string_defs[] = {
	[MANUFACTURER_IDX].s = manufacturer,
	[PRODUCT_IDX].s = product,
	[SERIALNUMBER_IDX].s = serialnumber,
	{}
};

struct usb_gadget_strings g_bt_string = {
	.language = 0x0409, /* en-us */
	.strings = g_bt_string_defs,
};

struct usb_gadget_strings *g_bt_string_tab[] = {
	&g_bt_string,
	NULL,
};

extern int jz_vendor_burner_add(struct usb_configuration *c);
static int g_do_burntool_config(struct usb_configuration *c)
{
	int ret = -ENODEV;
	const char *s = c->cdev->driver->name;
	debug("%s: configuration: 0x%p composite dev: 0x%p\n",
			__func__, c, c->cdev);
	if (!strcmp(s, "jz_usb_burner_vdr")) {
		 ret = jz_vendor_burner_add(c);
	}
	return ret;
}

static void g_do_burntool_unconfig(struct usb_configuration *cdev)
{
	return;
}

static int g_burntool_config(struct usb_composite_dev *cdev)
{
	static struct usb_configuration config = {
		.label = "usb_burntool",
		.bConfigurationValue = 1,
		.bmAttributes	=  USB_CONFIG_ATT_ONE,
		.bMaxPower	=  0xFA,
		.bind	=	g_do_burntool_config,
		.unbind =	g_do_burntool_unconfig,
	};

	return usb_add_config(cdev,&config);
}

static int burntool_unbind(struct usb_composite_dev * cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	debug("%s: calling usb_gadget_disconnect for "
			"controller '%s'\n", shortname, gadget->name);
	return usb_gadget_disconnect(gadget);
}

static int burntool_bind(struct usb_composite_dev * cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int id = -ENODEV;
	int ret,gcnum;

	debug("%s: gadget: 0x%p cdev: 0x%p\n", __func__, gadget, cdev);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	g_bt_string_defs[MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	g_bt_string_defs[PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	g_bt_string_defs[SERIALNUMBER_IDX].id = id;
	device_desc.iSerialNumber = id;

	ret = g_burntool_config(cdev);
	if (ret < 0)
		goto error;

	gcnum = usb_gadget_controller_number(gadget);
	debug("gcnum: %d\n", gcnum);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	 debug("%s: calling usb_gadget_connect for "
			 "controller '%s'\n", shortname, gadget->name);

	usb_gadget_connect(gadget);
error:
	burntool_unbind(cdev);
	return -ENOMEM;
}

static struct usb_composite_driver g_burntool_driver = {
	.name = NULL,
	.dev = &device_desc,
	.strings = g_bt_string_tab,
	.bind = burntool_bind,
	.unbind = burntool_unbind,
};

int g_burntool_register(const char *type)
{
	static char name[sizeof(shortname) + 3];
	int ret;

	if (!strcmp(type, "vdr")) {//vendor
		strcpy(name, shortname);
		strcat(name, type);
	} else {
		printf("%s: unknown command: %s\n", __func__, type);
	}
	g_burntool_driver.name = name;
	debug("%s: g_dnl_driver.name: %s\n", __func__, g_burntool_driver.name);

	ret = usb_composite_register(&g_burntool_driver);
	if (ret) {
		printf("%s: failed!, error: %d\n", __func__, ret);
		return ret;
	}

#if defined(CONFIG_CMD_BURN) || defined(CONFIG_CMD_FASTBOOT)
#else
	while(1) {
		usb_gadget_handle_interrupts();
	}
#endif

	return 0;
}

void g_burntool_unregister(void)
{
	usb_composite_unregister(&g_burntool_driver);
	return;
}
