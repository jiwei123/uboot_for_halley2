/*
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Huddy <hyli@ingenic.cn>
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
 *
 * Author :   Kznan <Derrick.kznan@ingenic.com>
 * Description :  lcd driver for ecx336af
 *
 */

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
#include <jz_lcd/ecx336af.h>

extern void Initial_IC(void);
extern void ECX336AF_POWER_ON(void);
extern void ECX336AF_POWER_OFF(void);
extern struct ecx336af_data ecx336af_pdata;

vidinfo_t panel_info = { 640, 400, LCD_BPP, };

void panel_pin_init(void)
{
	gpio_direction_output(ecx336af_pdata.gpio_reset, 1);
	gpio_direction_output(ecx336af_pdata.gpio_spi_cs, 1);
	gpio_direction_output(ecx336af_pdata.gpio_spi_clk, 1);
	gpio_direction_output(ecx336af_pdata.gpio_spi_sdi, 1);
	gpio_direction_input(ecx336af_pdata.gpio_spi_sdo);
}

void panel_power_on(void)
{
	ECX336AF_POWER_ON();
	mdelay(80);
	Initial_IC();
}

void panel_power_off(void)
{
	ECX336AF_POWER_OFF();
}

struct fb_videomode jzfb1_videomode = {
	.name = "640x400",
	.refresh = 60,
	.xres = 640,
	.yres = 400,
	.pixclock = KHZ2PICOS(27027),
	.left_margin = 58,
	.right_margin = 96,
	.upper_margin = 32,
	.lower_margin = 87,
	.hsync_len = 64,
	.vsync_len = 6,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
};
