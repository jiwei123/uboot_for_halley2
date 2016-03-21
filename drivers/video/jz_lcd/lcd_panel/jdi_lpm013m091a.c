/*
 * Copyright (C) 2016 Ingenic Electronics
 *
 * JDI 1.34" 320*300 MIPI LCD Driver
 *
 * Model : LPM013M091A
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

#include <common.h>
#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/jdi_lpm013m091a.h>

#define DEBUG
void panel_pin_init(void)
{
#ifdef DEBUG
	int ret= 0;

	ret = gpio_request(jdi_lpm013m091a_pdata.gpio_rest, "lcd mipi panel rest");
	if (!ret) {
		debug("request jdi_lpm013m091a lcd mipi panel rest gpio failed\n");
		return;
	}

	ret = gpio_request(jdi_lpm013m091a_pdata.gpio_lcd_bl,
			"lcd mipi brightness power eanble");
	if (!ret) {
		debug("request jdi_lpm013m091a lcd brightness power eanble gpio failed\n");
		return;
	}
#endif

	debug("jdi_lpm013m091a panel display pin init\n");

	return;
}

void panel_power_on(void)
{
	//gpio_direction_output(jdi_lpm013m091a_pdata.gpio_rest, 1);
	//udelay(5);
	mdelay(10);
	gpio_direction_output(jdi_lpm013m091a_pdata.gpio_rest, 0);  //reset active low
	mdelay(10);
	gpio_direction_output(jdi_lpm013m091a_pdata.gpio_rest, 1);
	//gpio_direction_output(jdi_lpm013m091a_pdata.gpio_lcd_bl, 1);
	mdelay(50);
	debug("jdi_lpm013m091a panel power on\n");

	return;
}

void jdi_lpm013m091a_sleep_out(struct dsi_device *dsi) /* exit sleep */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x11, 0x00};

	write_command(dsi, data_to_send);
}

void jdi_lpm013m091a_display_on(struct dsi_device *dsi) /* display on */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x29, 0x00};

	write_command(dsi, data_to_send);
}

void jdi_lpm013m091a_soft_reset(struct dsi_device *dsi) /* soft reset */
{
	struct dsi_cmd_packet data_to_send = {0x15, 0x01, 0x00};

	write_command(dsi, data_to_send);
}

/*
 * lcd_open_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_open_backlight(void)
{
	gpio_direction_output(jdi_lpm013m091a_pdata.gpio_lcd_bl, 1);
	return;
}

/*
 * lcd_close_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_close_backlight(void)
{
	gpio_direction_output(jdi_lpm013m091a_pdata.gpio_lcd_bl, 0);
	return;
}

void panel_init_set_sequence(struct dsi_device *dsi)
{
	int  i;
	struct dsi_cmd_packet jdi_lpm013m091a_cmd_list[] = {
		{0x15, 0xB3, 0x02},
		{0x15, 0xBB, 0x10},
		/* Optinal setting (Register list) */
		{0x15, 0x3A, 0x06},
		{0x05, 0x11, 0x00},
		/* O-1 AM write new image by MIPI */
		{0x15, 0xFF, 0x10},
		{0x15, 0x3A, 0x06},
		{0x39, 0x05, 0x00, {0x2A, 0x00, 0x00, 0x01, 0x3F}},
		{0x39, 0x05, 0x00, {0x2B, 0x00, 0x00, 0x01, 0x2B}},
	};

	jdi_lpm013m091a_soft_reset(dsi);
	mdelay(10); /* Wait more than 10 ms */

	for(i = 0; i < ARRAY_SIZE(jdi_lpm013m091a_cmd_list); i++) {
		write_command(dsi, jdi_lpm013m091a_cmd_list[i]);
	}

	jdi_lpm013m091a_sleep_out(dsi);
	mdelay(10);
	jdi_lpm013m091a_display_on(dsi);
	mdelay(120);

	debug("init JDI 1.342' 320*300 LCD...\n");

	return;
}


