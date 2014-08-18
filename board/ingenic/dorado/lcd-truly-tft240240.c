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

unsigned long truly_cmd_buf[]= {
    0x2C2C2C2C,
};

void board_set_lcd_power_on(void)
{
	return;
	char *id = "out11";
	struct regulator *lcd_regulator = regulator_get(id);
	regulator_set_voltage(lcd_regulator, 3300000, 1800000);
	regulator_enable(lcd_regulator);
}

struct smart_lcd_data_table truly_tft240240_data_table[] = {
    /* LCD init code */
    {0, 0x01},  //soft reset, 120 ms = 120 000 us
    {2, 120000},
    {0, 0x11},
    {2, 5000},	  /* sleep out 5 ms  */

    {0, 0x36},
#ifdef	CONFIG_TRULY_240X240_ROTATE_180
    /*{0x36, 0xc0, 2, 0}, //40*/
    {1, 0xd0}, //40
#else
    {1, 0x00}, //40
#endif

    {0, 0x2a},
    {1, 0x00},
    {1, 0x00},
    {1, 0x00},
    {1, 0xef},

    {0, 0x2b},
    {1, 0x00},
    {1, 0x00},
    {1, 0x00},
    {1, 0xef},


    {0, 0x3a},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
    {1, 0x06}, //6-6-6
#else
    {1, 0x05}, //5-6-5
#endif
    //	{1, 0x55},

    {0, 0xb2},
    {1, 0x7f},
    {1, 0x7f},
    {1, 0x01},
    {1, 0xde},
    {1, 0x33},

    {0, 0xb3},
    {1, 0x10},
    {1, 0x05},
    {1, 0x0f},

    {0, 0xb4},
    {1, 0x0b},

    {0, 0xb7},
    {1, 0x35},

    {0, 0xbb},
    {1, 0x28}, //23

    {0, 0xbc},
    {1, 0xec},

    {0, 0xc0},
    {1, 0x2c},

    {0, 0xc2},
    {1, 0x01},

    {0, 0xc3},
    {1, 0x1e}, //14

    {0, 0xc4},
    {1, 0x20},

    {0, 0xc6},
    {1, 0x14},

    {0, 0xd0},
    {1, 0xa4},
    {1, 0xa1},

    {0, 0xe0},
    {1, 0xd0},
    {1, 0x00},
    {1, 0x00},
    {1, 0x08},
    {1, 0x07},
    {1, 0x05},
    {1, 0x29},
    {1, 0x54},
    {1, 0x41},
    {1, 0x3c},
    {1, 0x17},
    {1, 0x15},
    {1, 0x1a},
    {1, 0x20},

    {0, 0xe1},
    {1, 0xd0},
    {1, 0x00},
    {1, 0x00},
    {1, 0x08},
    {1, 0x07},
    {1, 0x04},
    {1, 0x29},
    {1, 0x44},
    {1, 0x42},
    {1, 0x3b},
    {1, 0x16},
    {1, 0x15},
    {1, 0x1b},
    {1, 0x1f},

    {0, 0x35}, // TE on
    {1, 0x00}, // TE mode: 0, mode1; 1, mode2
    //	{0, 0x34}, // TE off

    {0, 0x29}, //Display ON

    /* set window size*/
    //	{0, 0xcd},
    {0, 0x2a},
    {1, 0},
    {1, 0},
    {1, (239>> 8) & 0xff},
    {1, 239 & 0xff},
#ifdef	CONFIG_TRULY_240X240_ROTATE_180
    {0, 0x2b},
    {1, ((320-240)>>8)&0xff},
    {1, ((320-240)>>0)&0xff},
    {1, ((320-1)>>8) & 0xff},
    {1, ((320-1)>>0) & 0xff},
#else
    {0, 0x2b},
    {1, 0},
    {1, 0},
    {1, (239>> 8) & 0xff},
    {1, 239 & 0xff},
#endif
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
	.gpio_lcd_rd  = GPIO_PC(17),
	.gpio_lcd_rst = GPIO_PA(12),
	.gpio_lcd_cs  = GPIO_PC(14),
	.gpio_lcd_bl  = GPIO_PC(18),
};
