/* linux/drivers/video/exynos/himax7069.c
 *
 * MIPI-DSI based himax7069 AMOLED lcd 4.65 inch panel driver.
 *
 * Inki Dae, <inki.dae@samsung.com>
 * Donghwa Lee, <dh09.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <config.h>
#include <serial.h>
#include <common.h>
#include <lcd.h>
#include <linux/list.h>
#include <linux/fb.h>
#include <asm/types.h>
#include <asm/arch/tcu.h>
#include <asm/arch/lcdc.h>
#include <asm/arch/gpio.h>
#include <regulator.h>

#include <jz_lcd/jz_dsim.h>
#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"

#define GPIO_MIPI_VDDA  GPIO_PC(9)
#define GPIO_LCD_RST  GPIO_PC(8) 

vidinfo_t panel_info = {640, 360, LCD_BPP, };

struct dsi_cmd_packet himax7069_cmd_list[] = {

	{0x39, 0x04, 0x00,  {0xFF, 0x70, 0x69, 0x00}}, 	//unlock panel

	{0x39, 0x19, 0x00,  {0xB5,
		0x01, 0xFF, 0x01, 0xA3, 0x01, 0x6A, 0x01, 0x3B, 0x01, 0x0E, 0x01, 0x0C,
		0x00, 0x00, 0x00, 0x5C, 0x00, 0x95, 0x00, 0xC4, 0x00, 0xF1, 0x00, 0xFF }
	}, 	//Gamma for R

	{0x39, 0x19, 0x00,  {0xB6,
		0x01, 0xFF, 0x01, 0xA3, 0x01, 0x6A, 0x01, 0x3B, 0x01, 0x0E, 0x01, 0x0C,
		0x00, 0x00, 0x00, 0x5C, 0x00, 0x95, 0x00, 0xC4, 0x00, 0xF1, 0x00, 0xFF }
	}, 	//Gamma for G

	{0x39, 0x19, 0x00,  {0xB7,
		0x01, 0xFF, 0x01, 0xA3, 0x01, 0x6A, 0x01, 0x3B, 0x01, 0x0E, 0x01, 0x0C,
		0x00, 0x00, 0x00, 0x5C, 0x00, 0x95, 0x00, 0xC4, 0x00, 0xF1, 0x00, 0xFF }
	}, 	//Gamma for B
	{0x39, 0x06, 0x00, {0xB2, 0x01, 0x0B, 0x00, 0xFF, 0x31} }, 	//VCOM
	{0x39, 0x03, 0x00, {0xB4, 0xF9, 0x05}},		      		//Vring
	{0x05, 0xD2, 0x00},

};

static void himax7069_sleep_in(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};

	write_command(dsi, data_to_send);
}

static void himax7069_sleep_out(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x01};

	write_command(dsi, data_to_send);
}

static void himax7069_display_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x01};

	write_command(dsi, data_to_send);
}

static void himax7069_display_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};

	write_command(dsi, data_to_send);
}

static void himax7069_panel_init(struct dsi_device *dsi)
{
	int  i;

	for(i = 0; i < ARRAY_SIZE(himax7069_cmd_list); i++) {
		write_command(dsi,  himax7069_cmd_list[i]);
	}
}

static int himax7069_set_power(struct lcd_device *ld, int power)
{
	int ret = 0;
	return ret;
}

void panel_pin_init(void)
{
	int ret= 0;

	ret = gpio_request(GPIO_LCD_RST, "lcd mipi panel rst");
	if (ret < 0){
		printf("request rst gpio fail!\n");
	}
	ret = gpio_request(GPIO_MIPI_VDDA, "lcd mipi panel vdda");
	if (ret < 0){
		printf("request vdda gpio fail!\n");
	}
}

void panel_init_set_sequence(struct dsi_device *dsi)
{
       	himax7069_panel_init(dsi);
	himax7069_sleep_out(dsi);
	mdelay(300);	//datesheet requirement
	himax7069_display_on(dsi);
}

void panel_power_on(void)
{
	gpio_direction_output(GPIO_MIPI_VDDA, 1);
	mdelay(30);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(100);
	gpio_direction_output(GPIO_LCD_RST, 0);  //reset active low
	mdelay(10);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(5);
}

void panel_power_off(void)
{
	gpio_direction_output(GPIO_MIPI_VDDA, 0);
}

struct fb_videomode jzfb1_videomode = {
	.name = "himax7069",
	.refresh = 60,
	.xres = 640,
	.yres = 360,
	.pixclock = KHZ2PICOS(15504),
	.left_margin = 20,
	.right_margin = 19,
	.upper_margin = 9,
	.lower_margin = 10,
	.hsync_len = 1,
	.vsync_len = 1,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
