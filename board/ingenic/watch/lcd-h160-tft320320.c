/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * BOE 1.54 320*320 TFT LCD Driver (driver's data part)
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

#include <regulator.h>
#include <lcd.h>
#include <asm/gpio.h>

#include <jz_lcd/jz_lcd_v1_2.h>
#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/h160_tft320320.h>

#ifdef CONFIG_ACRAB
#define PWM_LCD_BLK_EN GPIO_PC(17)
#define MIPI_RST_N     GPIO_PC(16)
#define LCD_VDD_1V8   "RICOH619_LDO9"
#define LCD_VCI_2V8   "RICOH619_LDO10"

#elif defined(CONFIG_X3)
#define PWM_LCD_BLK_EN GPIO_PC(23)
#define MIPI_RST_N     GPIO_PC(19)
#define LCD_VDD_1V8   "RICOH619_LDO4"
#define LCD_VCI_2V8   "RICOH619_LDO6"
#define BUCK5_3V      "RICOH619_DC5"

#else
#define PWM_LCD_BLK_EN -1
#define MIPI_RST_N     -1
#define LCD_VDD_1V8     NULL
#define LCD_VCI_2V8     NULL
#endif


vidinfo_t panel_info = { 320, 320, LCD_BPP,};

void board_set_lcd_power_on(void)
{
	struct regulator *lcd_vdd_1v8 = NULL;
	struct regulator *lcd_vci_2v8 = NULL;
	struct regulator *buck5_3v    = NULL;

	lcd_vdd_1v8 = regulator_get(LCD_VDD_1V8);
	if (lcd_vdd_1v8 == NULL)
		return;

	lcd_vci_2v8 = regulator_get(LCD_VCI_2V8);
	if (lcd_vci_2v8 == NULL)
		return;

	buck5_3v = regulator_get(BUCK5_3V);
	if (buck5_3v == NULL)
		return;

	regulator_set_voltage(buck5_3v, 3000000, 3000000);
	regulator_set_voltage(lcd_vci_2v8, 2800000, 2800000);
	regulator_set_voltage(lcd_vdd_1v8, 1800000, 1800000);

	regulator_enable(buck5_3v);
	regulator_enable(lcd_vdd_1v8);
	mdelay(10);
	regulator_enable(lcd_vci_2v8);
	mdelay(1);
}

struct h160_tft320320_platform_data h160_tft320320_pdata =
{
	.gpio_rest = MIPI_RST_N,
#if defined(CONFIG_ACRAB) || defined(CONFIG_X3)
	.pwm_lcd_brightness = PWM_LCD_BLK_EN,
#endif
};

struct fb_videomode jzfb1_videomode = {
	.name = "h160_tft320320-lcd",
	.refresh = 60,
	.xres = 320,
	.yres = 320,
	.pixclock = KHZ2PICOS(6144), //PCLK Frequency: 6.144MHz 320*320*60/1000
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

struct video_config jz_dsi_video_config={
	.no_of_lanes = 1,
	.virtual_channel = 0,
	.color_coding = COLOR_CODE_24BIT,
	.video_mode   = VIDEO_BURST_WITH_SYNC_PULSES,
	.receive_ack_packets = 0,	/* enable receiving of ack packets */
	.is_18_loosely    = 0,
	.data_en_polarity = 1,
};

struct dsi_config jz_dsi_config={
	.max_lanes = 1,
	.max_hs_to_lp_cycles = 100,
	.max_lp_to_hs_cycles = 40,
	.max_bta_cycles = 4095,
	.min_mbps = 360, /* 360Mbps */
	.color_mode_polarity = 1,
	.shut_down_polarity  = 1,
	.auto_clklane_ctrl = 0,
};

struct dsi_device jz_dsi = {
	.dsi_config   = &jz_dsi_config,
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

