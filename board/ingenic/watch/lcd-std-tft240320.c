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

#if defined(CONFIG_W561A)
#define LCD_VDD_1V8   "RICOH619_LDO4"
#define LCD_VCI_2V8   "RICOH619_LDO6"
#define LCD_LED_3V0   "RICOH619_LDO7"
#else
#define LCD_VDD_1V8   NULL
#define LCD_VCI_2V8   NULL
#define LCD_LED_3V0   NULL
#error "not support your configure, please fix it!"
#endif

void board_set_lcd_power_on(void)
{
   const char *id_vio = LCD_VDD_1V8;
   const char *id_vdd = LCD_VCI_2V8;
   const char *id_led = LCD_LED_3V0;

	struct regulator *lcd_vio = regulator_get(id_vio);
	struct regulator *lcd_vdd = regulator_get(id_vdd);
	struct regulator *lcd_led = regulator_get(id_led);
	regulator_set_voltage(lcd_vio, 1800000, 1800000);
	regulator_set_voltage(lcd_vdd, 2800000, 2800000);
	regulator_set_voltage(lcd_led, 3000000, 3000000);
	regulator_enable(lcd_vdd);
	mdelay(5);
	regulator_enable(lcd_vio);
	regulator_enable(lcd_led);
}

struct smart_lcd_data_table std_tft240320_data_table[] = {
	{SMART_CONFIG_UDELAY, 20},

	{SMART_CONFIG_CMD,0xef},
	{SMART_CONFIG_DATA,0x03},
	{SMART_CONFIG_DATA,0x80},
	{SMART_CONFIG_DATA,0x02},

	{SMART_CONFIG_CMD,0xcf},
	{SMART_CONFIG_DATA,0x00},
	{SMART_CONFIG_DATA,0xc1},
	{SMART_CONFIG_DATA,0xb0},

	{SMART_CONFIG_CMD,0xed},
	{SMART_CONFIG_DATA,0x67},
	{SMART_CONFIG_DATA,0x03},
	{SMART_CONFIG_DATA,0x12},
	{SMART_CONFIG_DATA,0x81},

	{SMART_CONFIG_CMD,0xcb},
	{SMART_CONFIG_DATA,0x39},
	{SMART_CONFIG_DATA,0x2c},
	{SMART_CONFIG_DATA,0x00},
	{SMART_CONFIG_DATA,0x34},
	{SMART_CONFIG_DATA,0x02},

	{SMART_CONFIG_CMD,0xea},
	{SMART_CONFIG_DATA,0x00},
	{SMART_CONFIG_DATA,0x00},

	{SMART_CONFIG_CMD,0xe8},
	{SMART_CONFIG_DATA,0x85},
	{SMART_CONFIG_DATA,0x0a},
	{SMART_CONFIG_DATA,0x78},

	{SMART_CONFIG_CMD,0xC0}, //Power control
	{SMART_CONFIG_DATA,0x26}, //VRH[5:0]

	{SMART_CONFIG_CMD,0xC1}, //Power control
	{SMART_CONFIG_DATA,0x01}, //SAP[2:0];BT[3:0]

	{SMART_CONFIG_CMD,0xC5}, //VCM control
	{SMART_CONFIG_DATA,0x2b},
	{SMART_CONFIG_DATA,0x2C},

	{SMART_CONFIG_CMD,0xc7},
	{SMART_CONFIG_DATA,0xb8},// c3

	{SMART_CONFIG_CMD,0x3A},
	{SMART_CONFIG_DATA,0x55},

	{SMART_CONFIG_CMD,0x36}, // Memory Access Control
	{SMART_CONFIG_DATA,0x68},

	{SMART_CONFIG_CMD,0xB1}, // Frame Rate Control 17
	{SMART_CONFIG_DATA,0x00},
	{SMART_CONFIG_DATA,0x12},

	{SMART_CONFIG_CMD,0xB6}, // Display Function Control
	{SMART_CONFIG_DATA,0x0a},
	{SMART_CONFIG_DATA,0xa2},

	{SMART_CONFIG_CMD,0xF2}, // 3Gamma Function Disable
	{SMART_CONFIG_DATA,0x00},

	{SMART_CONFIG_CMD,0x26}, //Gamma curve selected
	{SMART_CONFIG_DATA,0x01},

	{SMART_CONFIG_CMD,0xE0}, //Set Gamma
	{SMART_CONFIG_DATA,0x0f},
	{SMART_CONFIG_DATA,0x1d},
	{SMART_CONFIG_DATA,0x1a},
	{SMART_CONFIG_DATA,0x09},
	{SMART_CONFIG_DATA,0x0f},
	{SMART_CONFIG_DATA,0x09},
	{SMART_CONFIG_DATA,0x46},
	{SMART_CONFIG_DATA,0x88},
	{SMART_CONFIG_DATA,0x39},
	{SMART_CONFIG_DATA,0x05},
	{SMART_CONFIG_DATA,0x0f},
	{SMART_CONFIG_DATA,0x03},
	{SMART_CONFIG_DATA,0x07},
	{SMART_CONFIG_DATA,0x05},
	{SMART_CONFIG_DATA,0x00},

	{SMART_CONFIG_CMD,0XE1}, //Set Gamma
	{SMART_CONFIG_DATA,0x00},
	{SMART_CONFIG_DATA,0x22},
	{SMART_CONFIG_DATA,0x25},
	{SMART_CONFIG_DATA,0x06},
	{SMART_CONFIG_DATA,0x10},
	{SMART_CONFIG_DATA,0x06},
	{SMART_CONFIG_DATA,0x39},
	{SMART_CONFIG_DATA,0x22},
	{SMART_CONFIG_DATA,0x4a},
	{SMART_CONFIG_DATA,0x0a},
	{SMART_CONFIG_DATA,0x10},
	{SMART_CONFIG_DATA,0x0c},
	{SMART_CONFIG_DATA,0x38},
	{SMART_CONFIG_DATA,0x3a},
	{SMART_CONFIG_DATA,0x0F},

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

	{SMART_CONFIG_CMD,0x11}, //Exit Sleep
	{SMART_CONFIG_UDELAY, 12000},
	{SMART_CONFIG_CMD,0x29}, //display on
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
	.bpp    = NBITS(LCD_BPP),
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

#if defined(CONFIG_W561A)
struct std_tft240320_data std_tft240320_pdata = {
	.gpio_lcd_rd  = GPIO_PC(24),
	.gpio_lcd_rst = GPIO_PC(23),
	.gpio_lcd_cs  = GPIO_PC(27),
	.gpio_lcd_bl  = GPIO_PE(0),
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
