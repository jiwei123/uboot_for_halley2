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

#define CONFIG_SLCD_TRULY_18BIT

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
        {0x01, 0x01, 1, 120000},  //soft reset, 120 ms = 120 000 us
        {0x11, 0x11, 1, 5000},    /* sleep out 5 ms  */

        {0x36, 0x36, 1, 0},
#ifdef  CONFIG_TRULY_240X240_ROTATE_180
        /*{0x36, 0xc0, 2, 0}, //40*/
        {0x36, 0xd0, 2, 0}, //40
#else
        {0x36, 0x00, 2, 0}, //40
#endif

        {0x2a, 0x2a, 1, 0},
        {0x2a, 0x00, 2, 0},
        {0x2a, 0x00, 2, 0},
        {0x2a, 0x00, 2, 0},
        {0x2a, 0xef, 2, 0},

        {0x2b, 0x2b, 1, 0},
        {0x2b, 0x00, 2, 0},
        {0x2b, 0x00, 2, 0},
        {0x2b, 0x00, 2, 0},
        {0x2b, 0xef, 2, 0},


        {0x3a, 0x3a, 1, 0},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
        {0x3a, 0x06, 2, 0}, //6-6-6
#else
        {0x3a, 0x05, 2, 0}, //5-6-5
#endif
//      {0x3a, 0x55, 2, 0},

        {0xb2, 0xb2, 1, 0},
        {0xb2, 0x7f, 2, 0},
        {0xb2, 0x7f, 2, 0},
        {0xb2, 0x01, 2, 0},
        {0xb2, 0xde, 2, 0},
        {0xb2, 0x33, 2, 0},

        {0xb3, 0xb3, 1, 0},
        {0xb3, 0x10, 2, 0},
        {0xb3, 0x05, 2, 0},
        {0xb3, 0x0f, 2, 0},

        {0xb4, 0xb4, 1, 0},
        {0xb4, 0x0b, 2, 0},
        {0xb7, 0xb7, 1, 0},
        {0xb7, 0x35, 2, 0},

        {0xbb, 0xbb, 1, 0},
        {0xbb, 0x28, 2, 0}, //23

        {0xbc, 0xbc, 1, 0},
        {0xbc, 0xec, 2, 0},

        {0xc0, 0xc0, 1, 0},
        {0xc0, 0x2c, 2, 0},

        {0xc2, 0xc2, 1, 0},
        {0xc2, 0x01, 2, 0},

        {0xc3, 0xc3, 1, 0},
        {0xc3, 0x1e, 2, 0}, //14

        {0xc4, 0xc4, 1, 0},
        {0xc4, 0x20, 2, 0},

        {0xc6, 0xc6, 1, 0},
        {0xc6, 0x14, 2, 0},

        {0xd0, 0xd0, 1, 0},
        {0xd0, 0xa4, 2, 0},
        {0xd0, 0xa1, 2, 0},

        {0xe0, 0xe0, 1, 0},
        {0xe0, 0xd0, 2, 0},
        {0xe0, 0x00, 2, 0},
        {0xe0, 0x00, 2, 0},
        {0xe0, 0x08, 2, 0},
        {0xe0, 0x07, 2, 0},
        {0xe0, 0x05, 2, 0},
        {0xe0, 0x29, 2, 0},
        {0xe0, 0x54, 2, 0},
        {0xe0, 0x41, 2, 0},
        {0xe0, 0x3c, 2, 0},
        {0xe0, 0x17, 2, 0},
        {0xe0, 0x15, 2, 0},
        {0xe0, 0x1a, 2, 0},
        {0xe0, 0x20, 2, 0},

        {0xe1, 0xe1, 1, 0},
        {0xe1, 0xd0, 2, 0},
        {0xe1, 0x00, 2, 0},
        {0xe1, 0x00, 2, 0},
        {0xe1, 0x08, 2, 0},
        {0xe1, 0x07, 2, 0},
        {0xe1, 0x04, 2, 0},
        {0xe1, 0x29, 2, 0},
        {0xe1, 0x44, 2, 0},
        {0xe1, 0x42, 2, 0},
        {0xe1, 0x3b, 2, 0},
        {0xe1, 0x16, 2, 0},
        {0xe1, 0x15, 2, 0},
        {0xe1, 0x1b, 2, 0},
        {0xe1, 0x1f, 2, 0},

        {0x35, 0x35, 1, 0}, // TE on
        {0x35, 0x00, 2, 0}, // TE mode: 0, mode1; 1, mode2
//      {0x34, 0x34, 1, 0}, // TE off

        {0x29, 0x29, 1, 0}, //Display ON

        /* set window size*/
//      {0xcd, 0xcd, 1, 0},
        {0x2a, 0x2a, 1, 0},
        {0x2a, 0, 2, 0},
        {0x2a, 0, 2, 0},
        {0x2a, (239>> 8) & 0xff, 2, 0},
        {0x2a, 239 & 0xff, 2, 0},
#ifdef  CONFIG_TRULY_240X240_ROTATE_180
        {0x2b, 0x2b, 1, 0},
        {0x2b, ((320-240)>>8)&0xff, 2, 0},
        {0x2b, ((320-240)>>0)&0xff, 2, 0},
        {0x2b, ((320-1)>>8) & 0xff, 2, 0},
        {0x2b, ((320-1)>>0) & 0xff, 2, 0},
#else
	{0x2b, 0x2b, 1, 0},
	{0x2b, 0, 2, 0},
	{0x2b, 0, 2, 0},
	{0x2b, (239>> 8) & 0xff, 2, 0},
	{0x2b, 239 & 0xff, 2, 0},
#endif

	//{0xcd, 0xcd, 1, 0},
	//{0x2c, 0x2c, 1, 0},
};

struct jzfb_config_info jzfb1_init_data = {
	.num_modes = 1,
	.modes = &jzfb1_videomode,
	.lcd_type = LCD_TYPE_LCM,
	.bpp    = 18,
	.pinmd  = 0,
	.pixclk_falling_edge    = 0,
	.date_enable_active_low = 0,

	.smart_config.smart_type      = SMART_LCD_TYPE_PARALLEL,
	.smart_config.cmd_width       = SMART_LCD_CWIDTH_8_BIT_ONCE,           //8bit, according to the 8bit command
	.smart_config.data_width      = SMART_LCD_DWIDTH_24_BIT_ONCE_PARALLEL, //due to new slcd mode, must be 24bit
	.smart_config.data_new_width  = SMART_LCD_NEW_DWIDTH_8_BIT,          //8bit for lcd init
	.smart_config.data_new_times  = SMART_LCD_NEW_DTIMES_ONCE,   //8bit once, for 8bit command

#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
	.smart_config.data_new_times2 = SMART_LCD_NEW_DTIMES_THICE, //18bit three times, for 6-6-6
#else
	.smart_config.data_new_times2 = SMART_LCD_NEW_DTIMES_TWICE,//16bit two times,for 5-6-5
#endif
	.smart_config.clkply_active_rising = 0,
	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.newcfg_fmt_conv =  1,
	.smart_config.write_gram_cmd = 0x2c2c2c2c,
	.smart_config.bus_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(truly_tft240240_data_table),
	.smart_config.data_table = truly_tft240240_data_table,
	.smart_config.init = 0,
	.dither_enable = 0,
};

struct truly_tft240240_2_e_data truly_tft240240_2_e_pdata = {
	.gpio_lcd_rd  = GPIO_PC(17),
	.gpio_lcd_rst = GPIO_PA(12),
	.gpio_lcd_cs  = GPIO_PC(14),
	.gpio_lcd_bl  = GPIO_PC(18),
};
