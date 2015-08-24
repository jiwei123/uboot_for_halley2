/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * ARS-Y1300A Driver (driver's data part)
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

#include <regulator.h>
#include <lcd.h>
#include <asm/gpio.h>

#include <jz_lcd/jz_lcd_v1_2.h>
#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/ars_nt35350.h>

vidinfo_t panel_info = { 360, 360, LCD_BPP, };

struct ars_nt35350_platform_data ars_nt35350_pdata =
{
#if defined(CONFIG_CW004)
	.gpio_rest = GPIO_PC(23),
	.pwm_lcd_brightness = GPIO_PE(2),
#elif defined(CONFIG_IWOP)
	.gpio_rest = GPIO_PC(19),
	.pwm_lcd_brightness = GPIO_PC(23),
#elif defined(CONFIG_ACRAB)
	.gpio_rest = GPIO_PC(16),
	.pwm_lcd_brightness = GPIO_PC(17),
#endif
};

struct fb_videomode jzfb1_videomode = {
	.name = "ars_nt35350-lcd",
	.refresh = 60,
	.xres = 360,
	.yres = 360,
	.pixclock = KHZ2PICOS(7776),/* 7.776MHz 360*360*60/1000 */
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

struct video_config jz_dsi_video_config={
	.no_of_lanes = 1,
	.virtual_channel = 0,
	.color_coding = COLOR_CODE_24BIT,
	.receive_ack_packets = 0,	/* enable receiving of ack packets */
	.is_18_loosely = 0,
	.data_en_polarity = 1,
};

struct dsi_config jz_dsi_config={
	.max_lanes = 1,
	.max_hs_to_lp_cycles = 100,
	.max_lp_to_hs_cycles = 40,
	.max_bta_cycles = 4095,
	.min_mbps = 280, /* 280Mbps */
	.color_mode_polarity = 1,
	.shut_down_polarity = 1,
	.auto_clklane_ctrl = 0,
};

struct dsi_device jz_dsi = {
	.dsi_config = &jz_dsi_config,
	.video_config = &jz_dsi_video_config,
};

struct jzfb_config_info jzfb1_init_data = {
	.modes = &jzfb1_videomode,

	.lcd_type = LCD_TYPE_SLCD,
	.bpp = 24,

	.smart_config.smart_type = SMART_LCD_TYPE_PARALLEL,
	.smart_config.clkply_active_rising = 0,
	.smart_config.rsply_cmd_high    = 0,
	.smart_config.csply_active_high = 0,
	.smart_config.bus_width = 8,
	.dither_enable = 1,
	.dither.dither_red   = 1,	/* 6bit */
	.dither.dither_green = 1,	/* 6bit */
	.dither.dither_blue  = 1,	/* 6bit */
};

