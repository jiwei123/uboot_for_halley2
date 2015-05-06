/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * AUO 1.39 400*400 MIPI LCD Driver
 *
 * Model : H139BLN01.1
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
#include <jz_lcd/auo_h139bln01.h>

#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"

void panel_pin_init(void)
{
#ifdef DEBUG
	int ret= 0;

	ret = gpio_request(auo_h139bln01_pdata.gpio_rest, "lcd mipi panel rest");
	if (!ret) {
		debug("request auo_h139bln01 lcd mipi panel rest gpio failed\n");
		return;
	}

	ret = gpio_request(auo_h139bln01_pdata.gpio_lcd_bl,
			"lcd mipi brightness power eanble");
	if (!ret) {
		debug("request auo_h139bln01 lcd brightness power eanble gpio failed\n");
		return;
	}
#endif

	debug("auo_h139bln01 panel display pin init\n");

	return;
}

void panel_power_on(void)
{
	gpio_direction_output(auo_h139bln01_pdata.gpio_rest, 1);
	mdelay(50);
	gpio_direction_output(auo_h139bln01_pdata.gpio_rest, 0);  //reset active low
	mdelay(50);
	gpio_direction_output(auo_h139bln01_pdata.gpio_rest, 1);
	gpio_direction_output(auo_h139bln01_pdata.gpio_lcd_bl, 1);
	udelay(600);
	debug("auo_h139bln01 panel power on\n");

	return;
}

void auo_h139bln01_sleep_in(struct dsi_device *dsi) /* enter sleep */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x10, 0x00};

	write_command(dsi, data_to_send);
}

void auo_h139bln01_sleep_out(struct dsi_device *dsi) /* exit sleep */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x11, 0x00};

	write_command(dsi, data_to_send);
}

void auo_h139bln01_display_on(struct dsi_device *dsi) /* display on */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x29, 0x00};

	write_command(dsi, data_to_send);
}

void auo_h139bln01_display_off(struct dsi_device *dsi) /* display off */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x28, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * display inversion on
 */
void auo_h139bln01_display_inversion_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x21, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * display inversion off
 */
void auo_h139bln01_display_inversion_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x20, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * brightness setting
 */
void auo_h139bln01_set_brightness(struct dsi_device *dsi, unsigned int brightness)
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x51, (unsigned char)brightness};

	if(brightness >= 255) {
		debug("the max brightness is 255, set it 255\n");
		brightness = 255;
	}

	write_command(dsi, data_to_send);
}

/**
 * lcd_open_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_open_backlight(void)
{
	//gpio_direction_output(auo_h139bln01_pdata.gpio_lcd_bl, 1);

	return;
}

/**
 * lcd_close_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_close_backlight(void)
{
	//gpio_direction_output(auo_h139bln01_pdata.gpio_lcd_bl, 0);

	return;
}

void panel_init_set_sequence(struct dsi_device *dsi)
{
	int  i;
	struct dsi_cmd_packet auo_h139bln01_cmd_list[] = {
		{0x15, 0xFE, 0x07},
		{0x15, 0x07, 0x4F},
		{0x15, 0xFE, 0x0A},
		{0x15, 0x1C, 0x1B},
		{0x15, 0xFE, 0x05},
		{0x15, 0x2B, 0xF8},
		{0x15, 0xFE, 0x00},
		{0x15, 0x35, 0x00},
	};

	for(i = 0; i < ARRAY_SIZE(auo_h139bln01_cmd_list); i++) {
		write_command(dsi, auo_h139bln01_cmd_list[i]);
	}

	auo_h139bln01_sleep_out(dsi);
	mdelay(120);
	auo_h139bln01_display_on(dsi);

	debug("init AUO 1.39 400*400 LCD...\n");

	return;
}

