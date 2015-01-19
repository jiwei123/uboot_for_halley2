/*
 * Copyright (C) 2014 Ingenic Electronics
 *
 * Orise OTM3201A Driver (driver's data part)
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
#include <jz_lcd/orise_otm3201a.h>

#if defined(CONFIG_CW004)
#define PWM_LCD_BLK_EN GPIO_PE(2)
#endif

#if defined(CONFIG_CW004)
#define MIPI_RST_N GPIO_PC(23)
#endif

vidinfo_t panel_info = { 320, 320, LCD_BPP, };

struct orise_otm3201a_platform_data orise_otm3201a_pdata =
{
	.gpio_rest = MIPI_RST_N,
#if defined(CONFIG_CW004)
	.pwm_lcd_brightness = PWM_LCD_BLK_EN,
#endif
};

/**
#define LCD_XSIZE_TFT    (320)
#define LCD_YSIZE_TFT    (320)
#define LCD_VBPD         (10)
#define LCD_VFPD         (2)
#define LCD_VSPW         (4)
#define LCD_HBPD         (42)
#define LCD_HFPD         (26)
#define LCD_HSPW         (14)
data above LCD_xxx comes from FAE XiaoMing.Zhang(zhangxiaoming@focaltech-systems.com)
* .pixclock comes from DataSheet(DATA SHEET_OTM3201A_V03_Truly.pdf)
* .left_margin = LCD_HFPD
* .right_margin= LCD_HBPD
* .upper_margin= LCD_VFPD
* .lower_margin= LCD_VBPD
* .hsync_len = LCD_HSPW
* .vsync_len = LCD_VSPW
*/
struct fb_videomode jzfb1_videomode = {
	.name = "orise_otm3201a-lcd",
	.refresh = 60,
	.xres = 320,
	.yres = 320,
	.pixclock = KHZ2PICOS(5310), //PCLK Frequency: 5.31MHz
	.left_margin = 0x1a,
	.right_margin = 0x2a,
	.upper_margin = 0x02,
	.lower_margin = 0x0a,
	.hsync_len = 0x0e,
	.vsync_len = 0x04,
	.sync = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
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
	.color_mode_polarity = 1,
	.shut_down_polarity = 1,
};

struct dsi_device jz_dsi = {
	.dsi_config = &jz_dsi_config,
	.video_config = &jz_dsi_video_config,
};

struct jzfb_config_info jzfb1_init_data = {
	.modes = &jzfb1_videomode,

	.lcd_type = LCD_TYPE_GENERIC_24_BIT,
	.bpp = 24,

	.pixclk_falling_edge = 0,
	.date_enable_active_low = 0,
};

