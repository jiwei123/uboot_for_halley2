/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * BOE 1.54 320*320 TFT LCD Driver (driver's operation part)
 *
 * Model : BV015Z2M-N00-2B00
 *
 * Author: MaoLei.Wang <maolei.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

//#define DEBUG
#include <config.h>
#include <serial.h>
#include <common.h>
#include <lcd.h>
#include <linux/list.h>
#include <linux/fb.h>
#include <asm/types.h>
#include <asm/arch/tcu.h>
#include <asm/arch/lcdc.h>
#include <asm/arch/gpio.h>
#include <regulator.h>

#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/boe_tft320320.h>

#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"

void panel_pin_init(void)
{
#ifdef DEBUG
	int ret= 0;

	ret = gpio_request(boe_tft320320_pdata.gpio_rest, "lcd mipi panel rest");
	if (!ret) {
		debug("request boe_tft320320 lcd mipi panel rest gpio failed\n");
		return;
	}

	ret = gpio_request(boe_tft320320_pdata.pwm_lcd_brightness,
			"lcd mipi brightness power eanble");
	if (!ret) {
		debug("request boe_tft320320 lcd brightness power eanble gpio failed\n");
		return;
	}
#endif

	debug("boe_tft320320 panel display pin init\n");

	return;
}

void panel_power_on(void)
{
	gpio_direction_output(boe_tft320320_pdata.gpio_rest, 1);
	mdelay(50);
	gpio_direction_output(boe_tft320320_pdata.gpio_rest, 0);  //reset active low
	mdelay(50);
	gpio_direction_output(boe_tft320320_pdata.gpio_rest, 1);
	udelay(600);
	debug("boe_tft320320 panel display on\n");

	return;
}

void orise_otm3210a_sleep_in(struct dsi_device *dsi) /* enter sleep */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};

	write_command(dsi, data_to_send);
}

void orise_otm3210a_sleep_out(struct dsi_device *dsi) /* exit sleep */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};

	write_command(dsi, data_to_send);
}

void orise_otm3210a_display_on(struct dsi_device *dsi) /* display on */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};

	write_command(dsi, data_to_send);
}

void orise_otm3210a_display_off(struct dsi_device *dsi) /* display off */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * display inversion on
 */
void orise_otm3210a_display_inversion_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x21, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * display inversion off
 */
void orise_otm3210a_display_inversion_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x20, 0x00};

	write_command(dsi, data_to_send);
}

/**
 * lcd_open_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_open_backlight(void)
{
	gpio_direction_output(boe_tft320320_pdata.pwm_lcd_brightness, 1);

	return;
}

/**
 * lcd_close_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_close_backlight(void)
{
	gpio_direction_output(boe_tft320320_pdata.pwm_lcd_brightness, 0);

	return;
}

void panel_init_set_sequence(struct dsi_device *dsi)
{
	int  i;
	struct dsi_cmd_packet boe_tft320320_cmd_list[] = {
		{0x39, 0x05, 0x00, {0x2A, 0x00, 0x00, 0x01, 0x3F}},
		{0x39, 0x05, 0x00, {0x2B, 0x00, 0x00, 0x01, 0x3F}},
		{0x39, 0x02, 0x00, {0x3A, 0x66}},
		{0x39, 0x03, 0x00, {0xB1, 0xA0}},
		{0x39, 0x05, 0x00, {0xB5, 0x02, 0x02, 0x0A, 0x04}},
		{0x39, 0x02, 0x00, {0xB4, 0x02}},
		{0x39, 0x04, 0x00, {0xB6, 0x02, 0x02, 0x27}},

		{0x39, 0x02, 0x00, {0x35, 0x00}},
		{0x39, 0x02, 0x00, {0x36, 0x48}},

		{0x05, 0x21, 0x00},

		{0x39, 0x04, 0x00, {0xFC, 0x00, 0x05, 0x08}},
		{0x39, 0x0B, 0x00, {0xF2, 0x58, 0x7E, 0x12, 0x02, 0x12, 0x12, 0xFF, 0x0A, 0x90, 0x10}},
		{0x39, 0x03, 0x00, {0xC0, 0x0F, 0x0F}},
		{0x39, 0x02, 0x00, {0xC1, 0x41}},
		{0x39, 0x05, 0x00, {0xC5, 0x00, 0x2F, 0x00, 0x40}},
		{0x39, 0x02, 0x00, {0xC2, 0x22}},
		{0x39, 0x10, 0x00, {0xE0, 0x00, 0x0D, 0x13, 0x03, 0x0F, 0x05, 0x40, 0x67, 0x57, 0x06, 0x0C, 0x0C, 0x32, 0x36, 0x0F}},
		{0x39, 0x10, 0x00, {0xE1, 0x00, 0x0A, 0x0F, 0x03, 0x0F, 0x06, 0x2B, 0x33, 0x41, 0x05, 0x0D, 0x0B, 0x29, 0x30, 0x0F}},
		{0x05, 0x11, 0x00},
	};

	for(i = 0; i < ARRAY_SIZE(boe_tft320320_cmd_list); i++) {
		write_command(dsi, boe_tft320320_cmd_list[i]);
	}

	mdelay(120);

	orise_otm3210a_display_on(dsi);
	debug("init BOE 1.54 320*320 TFT LCD...\n");

	return;
}
