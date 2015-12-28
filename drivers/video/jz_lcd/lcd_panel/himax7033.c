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
 * Description :  lcd driver for himax7033
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

extern void himax7033_init(void);

vidinfo_t panel_info = { 320, 240, LCD_BPP, };

void panel_pin_init(void)
{
}

void panel_power_on(void)
{
	mdelay(80);
	himax7033_init();
}

void panel_power_off(void)
{
}

struct fb_videomode jzfb1_videomode = {
	.name = "320x240",
	.refresh = 120,
	.xres = 320,
	.yres = 240,
	.pixclock = KHZ2PICOS(14400),
	.left_margin = 40,
	.right_margin = 40,
	.upper_margin = 30,
	.lower_margin = 30,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

