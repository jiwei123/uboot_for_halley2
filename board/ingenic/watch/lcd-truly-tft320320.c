/*
 * Ingenic dorado lcd code
 *
 * Copyright (c) 2014 Ingenic Semiconductor Co.,Ltd
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
 */

#include <regulator.h>
#include <asm/gpio.h>
#include <jz_lcd/jz_lcd_v1_2.h>
#include <jz_lcd/truly_tft240240_2_e.h>

//#define CONFIG_SLCD_TRULY_18BIT

#define CONFIG_LCD_REGULATOR_9          "RICOH619_LDO9"
#define CONFIG_LCD_REGULATOR_10         "RICOH619_LDO10"

unsigned long truly_cmd_buf[]= {
    0x2C2C2C2C,
};

void board_set_lcd_power_on(void)
{
        char *id_9  = CONFIG_LCD_REGULATOR_9;
        char *id_10 = CONFIG_LCD_REGULATOR_10;
        struct regulator *lcd_regulator_9 = regulator_get(id_9);
        struct regulator *lcd_regulator_10 = regulator_get(id_10);
        regulator_set_voltage(lcd_regulator_9, 1800000, 1800000);
        regulator_set_voltage(lcd_regulator_10, 3000000, 3000000);
        regulator_enable(lcd_regulator_9);
        regulator_enable(lcd_regulator_10);

}

struct smart_lcd_data_table truly_tft240240_data_table[] = {
	/* LCD init code */

	{SMART_CONFIG_CMD, 0x11},
	{SMART_CONFIG_UDELAY, 250000},

	{SMART_CONFIG_CMD, 0xB9},         // Set EXTC
	{SMART_CONFIG_DATA, 0xFF},
	{SMART_CONFIG_DATA, 0x83},
	{SMART_CONFIG_DATA, 0x57},

	{SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_DATA, 0x48},

	{SMART_CONFIG_CMD, 0x3A},
	{SMART_CONFIG_DATA, 0x55},//¸ù¾ÝÖ÷°å¸ü¸Ä


	{SMART_CONFIG_CMD, 0xB1},                //
	{SMART_CONFIG_DATA, 0x00},                //DP
	{SMART_CONFIG_DATA, 0x12},                //BT     12
	{SMART_CONFIG_DATA, 0x1c},                //VSPR  1c
	{SMART_CONFIG_DATA, 0x1c},                //VSNR  1c
	{SMART_CONFIG_DATA, 0x45},                //AP   45
	{SMART_CONFIG_DATA, 0x44},                //FS   44


	{SMART_CONFIG_CMD, 0xB6},                //
	{SMART_CONFIG_DATA, 0x60},                //VCOMDC   62


	{SMART_CONFIG_CMD, 0xB5},                //
	{SMART_CONFIG_DATA, 0x0f},                //DP  0f
	{SMART_CONFIG_DATA, 0x0f},                //BT  0f
	{SMART_CONFIG_DATA, 0x67},                //VSP=vsn  67


	{SMART_CONFIG_CMD, 0xB4},                //
	{SMART_CONFIG_DATA, 0x02},                //NW
	{SMART_CONFIG_DATA, 0x40},                //RTN
	{SMART_CONFIG_DATA, 0x00},                //DIV
	{SMART_CONFIG_DATA, 0x3a},                //DUM   3a
	{SMART_CONFIG_DATA, 0x40},                //DUM
	{SMART_CONFIG_DATA, 0x0a},                //GDON    0a
	{SMART_CONFIG_DATA, 0x3e},                //GDOFF 3e


	{SMART_CONFIG_CMD, 0xC0},                //STBA
	{SMART_CONFIG_DATA, 0x60},                //OPON   60
	{SMART_CONFIG_DATA, 0x50},                //OPON   50
	{SMART_CONFIG_DATA, 0x01},                //01
	{SMART_CONFIG_DATA, 0x3c},                // 3c
	{SMART_CONFIG_DATA, 0x1e},                // c8
	{SMART_CONFIG_DATA, 0x08},                //GEN 08


	{SMART_CONFIG_CMD, 0xE0},
	{SMART_CONFIG_DATA, 0x04},//v0
	{SMART_CONFIG_DATA, 0x12},// v1
	{SMART_CONFIG_DATA, 0x18}, //v2
	{SMART_CONFIG_DATA, 0x21}, //v4
	{SMART_CONFIG_DATA, 0x28},//v6
	{SMART_CONFIG_DATA, 0x3a},//v13
	{SMART_CONFIG_DATA, 0x44},//v20 45
	{SMART_CONFIG_DATA, 0x4f},//v27
	{SMART_CONFIG_DATA, 0x48},//v36
	{SMART_CONFIG_DATA, 0x42}, //v43
	{SMART_CONFIG_DATA, 0x3A},//v50
	{SMART_CONFIG_DATA, 0x27}, //v57
	{SMART_CONFIG_DATA, 0x1B},//v59
	{SMART_CONFIG_DATA, 0x08},// v61
	{SMART_CONFIG_DATA, 0x09}, //v62
	{SMART_CONFIG_DATA, 0x03}, //v63

	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_DATA, 0x12},
	{SMART_CONFIG_DATA, 0x18},
	{SMART_CONFIG_DATA, 0x21},
	{SMART_CONFIG_DATA, 0x28},
	{SMART_CONFIG_DATA, 0x3a},
	{SMART_CONFIG_DATA, 0x44},
	{SMART_CONFIG_DATA, 0x4f},
	{SMART_CONFIG_DATA, 0x48},
	{SMART_CONFIG_DATA, 0x42},
	{SMART_CONFIG_DATA, 0x3A},
	{SMART_CONFIG_DATA, 0x27},
	{SMART_CONFIG_DATA, 0x1B},
	{SMART_CONFIG_DATA, 0x08},
	{SMART_CONFIG_DATA, 0x09},
	{SMART_CONFIG_DATA, 0x03},

	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},


	{SMART_CONFIG_CMD, 0x29},
	{SMART_CONFIG_UDELAY, 250000},

	{SMART_CONFIG_CMD, 0x2C},
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
	.smart_config.write_gram_cmd = truly_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(truly_cmd_buf),
	.smart_config.bus_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(truly_tft240240_data_table),
	.smart_config.data_table = truly_tft240240_data_table,
	.dither_enable = 0,
};

struct truly_tft240240_2_e_data truly_tft240240_2_e_pdata = {
	.gpio_lcd_rd  = -1,
	.gpio_lcd_rst = GPIO_PC(19),
	.gpio_lcd_cs  = GPIO_PC(18),
	.gpio_lcd_bl  = GPIO_PC(15),
};
