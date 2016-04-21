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

#include <config.h>
#include <serial.h>
#include <common.h>
#include <linux/list.h>
#include <linux/fb.h>
#include <asm/types.h>
#include <asm/arch-m200/tcu.h>
#include <asm/arch-m200/lcdc.h>
#include <asm/arch-m200/gpio.h>
#include <regulator.h>
#include <jz_lcd/std_tft240320.h>

void panel_pin_init(void)
{
	int ret = 0;
	ret = gpio_request(std_tft240320_pdata.gpio_lcd_cs, "lcd_cs");
	if(ret){
		printf("canot request gpio lcd_cs\n");
	}

	ret = gpio_request(std_tft240320_pdata.gpio_lcd_rd, "lcd_rd");
	if(ret){
		printf("canot request gpio lcd_rd\n");
	}

    ret = gpio_request(std_tft240320_pdata.gpio_lcd_rst, "lcd_rst");
	if(ret){
		printf("canot request gpio lcd_rst\n");
	}

	ret = gpio_request(std_tft240320_pdata.gpio_lcd_bl, "lcd_bl");
	if(ret){
		printf("canot request gpio lcd_bl\n");
	}

#ifdef CONFIG_LCD_GPIO_FUNC2_SLCD
	gpio_set_func(GPIO_PORT_C, GPIO_FUNC_2, 0x0e0ff3fc);
#endif
	serial_puts("std_tft240320 panel display pin init\n");
}

void panel_power_on(void)
{
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_cs, 1);
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_rd, 1);

	/*power reset*/
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_rst, 0);
	mdelay(20);
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_rst, 1);
	mdelay(10);
	mdelay(5);
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_cs, 0);
	mdelay(5);
	serial_puts("std_tft240320 panel display on\n");
}

/**
 * lcd_open_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_open_backlight(void)
{
//	gpio_direction_output(std_tft240320_pdata.gpio_lcd_bl, 1);
	return;
}

/**
 * lcd_close_backlight() - Overwrite the weak function defined at common/lcd.c
 */
void lcd_close_backlight(void)
{
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_bl, 0);
	return;
}

void panel_power_off(void)
{
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_cs, 0);
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_rd, 0);
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_rst, 0);
	gpio_direction_output(std_tft240320_pdata.gpio_lcd_bl, 0);
	serial_puts("std_tft240320 panel display off\n");
}

