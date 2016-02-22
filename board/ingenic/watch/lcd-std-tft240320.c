/*
 * Copyright (c) 2016 Ingenic Semiconductor Co.,Ltd
 * Author: Harvis <maolei.wang@ingenic.com>
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
 */

#include <regulator.h>
#include <lcd.h>
#include <asm/gpio.h>
#include <jz_lcd/jz_lcd_v1_2.h>
#include <jz_lcd/std_tft240320.h>

void board_set_lcd_power_on(void)
{
    const char *id_vio  = "RICOH619_LDO4";
    const char *id_vdd  = "RICOH619_LDO6";
    const char *id_led  = "RICOH619_LDO7";

    struct regulator *lcd_vio = regulator_get(id_vio);
    struct regulator *lcd_vdd = regulator_get(id_vdd);
    struct regulator *lcd_led = regulator_get(id_led);
    regulator_set_voltage(lcd_vio, 1800000, 1800000);
    regulator_set_voltage(lcd_vdd, 2800000, 2800000);
    regulator_set_voltage(lcd_led, 3300000, 3300000);
    regulator_enable(lcd_vdd);
    mdelay(5);
    regulator_enable(lcd_vio);
    regulator_enable(lcd_led);
}

struct smart_lcd_data_table std_tft240320_data_table[] = {
    {SMART_CONFIG_UDELAY, 20},

    {SMART_CONFIG_CMD, 0xCF},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xC1},
    {SMART_CONFIG_DATA, 0x30},

    {SMART_CONFIG_CMD, 0xED},
    {SMART_CONFIG_DATA, 0x64},
    {SMART_CONFIG_DATA, 0x03},
    {SMART_CONFIG_DATA, 0x12},
    {SMART_CONFIG_DATA, 0x81},

    {SMART_CONFIG_CMD, 0xE8},
    {SMART_CONFIG_DATA, 0x85},
    {SMART_CONFIG_DATA, 0x10},
    {SMART_CONFIG_DATA, 0x7A},

    {SMART_CONFIG_CMD, 0xCB},
    {SMART_CONFIG_DATA, 0x39},
    {SMART_CONFIG_DATA, 0x2C},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x34},
    {SMART_CONFIG_DATA, 0x02},

    {SMART_CONFIG_CMD, 0xF7},
    {SMART_CONFIG_DATA, 0x20},

    {SMART_CONFIG_CMD, 0xEA},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},

    {SMART_CONFIG_CMD, 0xC0}, //Power control
    {SMART_CONFIG_DATA, 0x1B},//VRH[5:0]

    {SMART_CONFIG_CMD, 0xC1}, //Power control
    {SMART_CONFIG_DATA, 0x01},//SAP[2:0];BT[3:0

    {SMART_CONFIG_CMD, 0xC5}, //VCM control
    {SMART_CONFIG_DATA, 0x30},
    {SMART_CONFIG_DATA, 0x30},

    {SMART_CONFIG_CMD, 0xC7}, //VCM control2
    {SMART_CONFIG_DATA, 0x9f},

    {SMART_CONFIG_CMD, 0x36}, //Memory Access Control
    {SMART_CONFIG_DATA, 0x48},

    {SMART_CONFIG_CMD, 0x3A},
    {SMART_CONFIG_DATA, 0x55},

    {SMART_CONFIG_CMD, 0xB1},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x1A},

    {SMART_CONFIG_CMD, 0xB6}, //Display Function Control
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0xA2},

    {SMART_CONFIG_CMD, 0xF2}, //3Gamma Function Disable
    {SMART_CONFIG_DATA, 0x00},

    {SMART_CONFIG_CMD, 0x26}, //Gamma curve selected
    {SMART_CONFIG_DATA, 0x01},


    {SMART_CONFIG_CMD, 0xE0}, //Set Gamma
    {SMART_CONFIG_DATA, 0x0F},
    {SMART_CONFIG_DATA, 0x2A},
    {SMART_CONFIG_DATA, 0x28},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x0E},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x54},
    {SMART_CONFIG_DATA, 0xA9},
    {SMART_CONFIG_DATA, 0x43},
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0x0F},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},

    {SMART_CONFIG_CMD, 0xE1}, //Set Gamma
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x17},
    {SMART_CONFIG_DATA, 0x07},
    {SMART_CONFIG_DATA, 0x11},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x2B},
    {SMART_CONFIG_DATA, 0x56},
    {SMART_CONFIG_DATA, 0x3C},
    {SMART_CONFIG_DATA, 0x05},
    {SMART_CONFIG_DATA, 0x10},
    {SMART_CONFIG_DATA, 0x0F},
    {SMART_CONFIG_DATA, 0x3F},
    {SMART_CONFIG_DATA, 0x3F},
    {SMART_CONFIG_DATA, 0x0F},

    {SMART_CONFIG_CMD, 0x2A}, // row 320
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x01},
    {SMART_CONFIG_DATA, 0x3f},

    {SMART_CONFIG_CMD, 0x2B}, // column 240
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

    {SMART_CONFIG_CMD, 0x11}, //Exit Sleep
    {SMART_CONFIG_UDELAY, 40},

    {SMART_CONFIG_CMD, 0x29}, //display on

    {SMART_CONFIG_CMD, 0x36}, //display mode
    {SMART_CONFIG_DATA, 0x68},
};

struct fb_videomode jzfb1_videomode = {
    .name = "320x240",
    .refresh = 60,
    .xres = 320,
    .yres = 240,
    .pixclock = KHZ2PICOS(3000000),
    .left_margin = 0,
    .right_margin = 0,
    .upper_margin = 0,
    .lower_margin = 0,
    .hsync_len = 0,
    .vsync_len = 0,
    .sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
    .vmode = FB_VMODE_NONINTERLACED,
    .flag = 0,
};

unsigned long std_tft240320_cmd_buf[]= {
    0x2C2C2C2C,
};

struct jzfb_config_info jzfb1_init_data = {
	.num_modes = 1,
	.modes = &jzfb1_videomode,
	.lcd_type = LCD_TYPE_SLCD,
	.bpp    = 18,
	.pinmd  = 0,

	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.newcfg_fmt_conv =  1,
	.smart_config.write_gram_cmd = std_tft240320_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(std_tft240320_cmd_buf),
	.smart_config.bus_width = 16,
	.smart_config.length_data_table =  ARRAY_SIZE(std_tft240320_data_table),
	.smart_config.data_table = std_tft240320_data_table,
	.dither_enable = 0,
};

#if defined(CONFIG_AW808)
struct std_tft240320_data std_tft240320_pdata = {
	.gpio_lcd_rd  = GPIO_PC(24),
	.gpio_lcd_rst = GPIO_PC(23),
	.gpio_lcd_cs  = GPIO_PC(27),
	.gpio_lcd_bl  = GPIO_PC(22),
};
#else
struct std_tft240320_data std_tft240320_pdata = {
	.gpio_lcd_rd  = -1,
	.gpio_lcd_rst = GPIO_PC(19),
	.gpio_lcd_cs  = GPIO_PC(18),
	.gpio_lcd_bl  = GPIO_PD(28),
};
#endif

vidinfo_t panel_info = { 320, 240, LCD_BPP};
