/*
 * Copyright (C) 2014 Ingenic Electronics
 *
 * Orise OTM3201A Driver (driver's operation part)
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
#include <jz_lcd/orise_otm3201a.h>

#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"

void board_set_lcd_power_on(void)
{
	const char *id_4 = "RICOH619_LDO4";
	const char *id_6 = "RICOH619_LDO6";
	const char *id_5 = "RICOH619_DC5";
	struct regulator *lcd_regulator_4 = regulator_get(id_4);
	struct regulator *lcd_regulator_6 = regulator_get(id_6);
	struct regulator *lcd_regulator_5 = regulator_get(id_5);

	regulator_set_voltage(lcd_regulator_4, 3000000, 3000000);
	regulator_set_voltage(lcd_regulator_4, 1800000, 1800000);
	regulator_set_voltage(lcd_regulator_6, 2800000, 2800000);

	regulator_enable(lcd_regulator_5);
	regulator_enable(lcd_regulator_4);
	regulator_enable(lcd_regulator_6);

	return;
}

void panel_pin_init(void)
{
	int ret= 0;

	ret = gpio_request(orise_otm3201a_pdata.gpio_rest, "lcd mipi panel rest");
	if (ret) {
		printf("orise_otm3201 lcd mipi panel rest failed\n");
		return;
	}

	printf("orise_otm3201 panel display pin init\n");

	return;
}

void panel_power_on(void)
{
	debug("--------------------%s\n", __func__);
	gpio_direction_output(orise_otm3201a_pdata.gpio_rest, 1);
	mdelay(300);
	gpio_direction_output(orise_otm3201a_pdata.gpio_rest, 0);  //reset active low
	mdelay(10);
	gpio_direction_output(orise_otm3201a_pdata.gpio_rest, 1);
	mdelay(50);
	printf("rise_otm3201a_pdata panel display on\n");

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
	gpio_direction_output(orise_otm3201a_pdata.pwm_lcd_brightness, 1);
	return;
}

/**
 * lcd_close_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_close_backlight(void)
{
	gpio_direction_output(orise_otm3201a_pdata.pwm_lcd_brightness, 0);
	return;
}

/*
	DCS_write_1A_2P(0xF0,0x54,0x47);				// Enable CMD2
	DCS_write_1A_1P(0xA0,0x00);
	DCS_write_1A_1P(0xB1,0x22);
	DCS_write_1A_5P(0xB3,0x02,0x0a,0x1A,0x2a,0x2a); //0x02,0x0a,0x1A,0x2a,0x2a
	DCS_write_1A_3P(0xBD,0x00,0x11,0x31);
	DCS_write_1A_4P(0xBA,0x05,0x15,0x2B,0x01);		//Modify by eric 20140307
	DCS_write_1A_1P(0xE9,0x46);					// Set RTN
	DCS_write_1A_1P(0xE2,0xf5);

	//Gamma 2.4" From INX setting
	DCS_write_1A_4P(0xB5,0x5C,0x5C,0x7A,0xFA);
	DCS_write_1A_17P(0xC0,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f);
	DCS_write_1A_17P(0xC1,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f);
	DCS_write_1A_17P(0xC2,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f);
	DCS_write_1A_17P(0xC3,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f);
	DCS_write_1A_17P(0xC4,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f);
	DCS_write_1A_17P(0xC5,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f);
	printf("init okay...\n");

	DCS_write_1A_0P(0x11);
	Delay(120);
	DCS_write_1A_0P(0x29);
	Delay(120);
	the pseudo-code above comes from FAE XiaoMing.Zhang(zhangxiaoming@focaltech-systems.com)
 */
void panel_init_set_sequence(struct dsi_device *dsi)
{
	int  i;
	struct dsi_cmd_packet orise_otm3201a_cmd_list[] = {
		{0x39, 0x03, 0x00, {0xF0,0x54,0x47}}, // Enable CMD2
		{0x39, 0x02, 0x00, {0xA0,0x00}}, // Read Mode Disable
		{0x39, 0x02, 0x00, {0xB1,0x22}},
		{0x39, 0x06, 0x00, {0xB3,0x02,0x0a,0x1A,0x2a,0x2a}},
		{0x39, 0x04, 0x00, {0xBD,0x00,0x11,0x31}},
		{0x39, 0x05, 0x00, {0xBA,0x05,0x15,0x2B,0x01}},
		{0x39, 0x02, 0x00, {0xE9,0x46}}, // Set RTN
		{0x39, 0x02, 0x00, {0xE2,0xf5}},

		//Gamma 2.4" From INX setting CHECK Analog Gamma(C0h ~ C5h)
		{0x39, 0x05, 0x00, {0xB5,0x5C,0x5C,0x7A,0xFA}},
		{0x39, 0x12, 0x00, {0xC0,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f}},
		{0x39, 0x12, 0x00, {0xC1,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f}},
		{0x39, 0x12, 0x00, {0xC2,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f}},
		{0x39, 0x12, 0x00, {0xC3,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f}},
		{0x39, 0x12, 0x00, {0xC4,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f}},
		{0x39, 0x12, 0x00, {0xC5,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f}},
	};

	for(i = 0; i < ARRAY_SIZE(orise_otm3201a_cmd_list); i++) {
		write_command(dsi, orise_otm3201a_cmd_list[i]);
	}

	printf("init Orise OTM3201A okay...\n");

	orise_otm3210a_sleep_out(dsi);
	mdelay(120);

	orise_otm3210a_display_on(dsi);
	mdelay(120);
}