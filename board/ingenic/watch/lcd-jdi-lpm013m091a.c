/*
 * Copyright (C) 2016 Ingenic Electronics
 *
 * JDI 1.34" 320*300  MIPI LCD Driver (driver's data part)
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

#include <lcd.h>
#include <asm/gpio.h>
#include <jz_lcd/jz_lcd_v1_2.h>
#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/jdi_lpm013m091a.h>

#define GPIO_LCD_BLK_EN -1
#define MIPI_RST_N     GPIO_PC(19)
#define LCD_VDDI_1V8   "SM5007_LDO2"
#define BUCK4_3V1      "SM5007_BUCK4"

vidinfo_t panel_info = { 320, 300, LCD_BPP,};

struct jdi_lpm013m091a_platform_data jdi_lpm013m091a_pdata = {
	.gpio_rest = GPIO_PD(3),
	.gpio_lcd_bl = GPIO_PC(8),
};

void board_set_lcd_power_on(void)
{

	struct regulator *lcd_vddio_1v8 = NULL;
	struct regulator *buck4_3v1    = NULL;

	lcd_vddio_1v8 = regulator_get(LCD_VDDI_1V8);
	if (lcd_vddio_1v8 == NULL)
		return;

	buck4_3v1 = regulator_get(BUCK4_3V1);
	if (buck4_3v1 == NULL)
		return;

	regulator_set_voltage(buck4_3v1, 3100000, 3100000);
	regulator_set_voltage(lcd_vddio_1v8, 1800000, 1800000);

	regulator_enable(buck4_3v1);
	regulator_enable(lcd_vddio_1v8);
	gpio_direction_output(jdi_lpm013m091a_pdata.gpio_rest, 0);

}

struct dsi_config jz_dsi_config={
	.max_lanes = 1,
	.max_hs_to_lp_cycles = 100,
	.max_lp_to_hs_cycles = 40,
	.max_bta_cycles = 4095,
	.min_mbps = 344, /* 344Mbps */
	.color_mode_polarity = 1,
	.shut_down_polarity  = 1,
	.auto_clklane_ctrl = 0,
};

struct video_config jz_dsi_video_config={
	.no_of_lanes = 1,
	.virtual_channel = 0,
	.color_coding = COLOR_CODE_24BIT,
	.video_mode   = VIDEO_BURST_WITH_SYNC_PULSES,
	.receive_ack_packets = 0,	/* enable receiving of ack packets */
	.is_18_loosely    = 0,
	.data_en_polarity = 1,
};

struct dsi_device jz_dsi = {
	.dsi_config   = &jz_dsi_config,
	.video_config = &jz_dsi_video_config,
};

struct fb_videomode jzfb1_videomode = {
	.name = "jdi_lpm013m091a-lcd",
	.refresh = 60,
	.xres = 320,
	.yres = 300,
	.pixclock = KHZ2PICOS(9600), //PCLK Frequency: 9.6MHz
	.left_margin  = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
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
