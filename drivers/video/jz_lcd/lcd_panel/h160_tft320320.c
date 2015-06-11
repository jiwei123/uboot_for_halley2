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
#include <jz_lcd/h160_tft320320.h>

#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"

void panel_pin_init(void)
{
#ifdef DEBUG
	int ret= 0;

	ret = gpio_request(h160_tft320320_pdata.gpio_rest, "lcd mipi panel rest");
	if (!ret) {
		debug("request h160_tft320320 lcd mipi panel rest gpio failed\n");
		return;
	}

	ret = gpio_request(h160_tft320320_pdata.pwm_lcd_brightness,
			"lcd mipi brightness power eanble");
	if (!ret) {
		debug("request h160_tft320320 lcd brightness power eanble gpio failed\n");
		return;
	}
#endif

	debug("h160_tft320320 panel display pin init\n");

	return;
}

void panel_power_on(void)
{
	gpio_direction_output(h160_tft320320_pdata.gpio_rest, 1);
	mdelay(10);//10
	gpio_direction_output(h160_tft320320_pdata.gpio_rest, 0);  //reset active low
	mdelay(1);//1
	gpio_direction_output(h160_tft320320_pdata.gpio_rest, 1);
	mdelay(120);//120
	debug("h160_tft320320 panel display on\n");

	return;
}

void h160_tft320320_sleep_in(struct dsi_device *dsi) /* enter sleep */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};

	write_command(dsi, data_to_send);
}

void h160_tft320320_sleep_out(struct dsi_device *dsi) /* exit sleep */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};

	write_command(dsi, data_to_send);
}

void h160_tft320320_display_on(struct dsi_device *dsi) /* display on */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};

	write_command(dsi, data_to_send);
}

void h160_tft320320_display_off(struct dsi_device *dsi) /* display off */
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * display inversion on
 */
void h160_tft320320_display_inversion_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x21, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * display inversion off
 */
void h160_tft320320_display_inversion_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x20, 0x00};

	write_command(dsi, data_to_send);
}

/**
 * lcd_open_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_open_backlight(void)
{
	gpio_direction_output(h160_tft320320_pdata.pwm_lcd_brightness, 1);

	return;
}

/**
 * lcd_close_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_close_backlight(void)
{
	gpio_direction_output(h160_tft320320_pdata.pwm_lcd_brightness, 0);

	return;
}

void panel_init_set_sequence(struct dsi_device *dsi)
{
	int  i;
	struct dsi_cmd_packet h160_tft320320_cmd_list[] = {
		{0x39, 0x02, 0x00, {0x36, 0xc0}},
		{0x39, 0x05, 0x00, {0x2a, 0x00,     0x00,       319 >> 8, 319 & 0xff}},
		{0x39, 0x05, 0x00, {0x2b, 160 >> 8, 160 & 0xff, 479 >> 8, 479 & 0xff}},
		{0x39, 0x03, 0x00, {0x44, 320 >> 8, 320 & 0xff}},
		{0x05, 0x20, 0x00},
		{0x39, 0x02, 0x00, {0xe1, 0x02}},
		{0x05, 0x35, 0x00},
		{0x05, 0x29, 0x00},
	};

	h160_tft320320_sleep_out(dsi);
	mdelay(10);
	for(i = 0; i < ARRAY_SIZE(h160_tft320320_cmd_list); i++) {
		write_command(dsi, h160_tft320320_cmd_list[i]);
	}


	debug("init Rise h160 1.6 320*320 TFT LCD...\n");

	return;
}
