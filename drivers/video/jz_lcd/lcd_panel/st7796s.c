/*******************************************************************************
 *               Copyright (C) 2015, GYENNO Tech. Co., Ltd.
 *                      ALL RIGHTS RESERVED
 *******************************************************************************/

/** @defgroup st7796s.c

 *  @author  lingyun
 *  @version 1.0
 *  @date    2015/1/31 \n

 *  history:\n
 *          1. 2015/1/31, lingyun, create this file\n\n
 *
 * @{
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
#include <jz_lcd/st7796s.h>
#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /*  __cpluscplus */
#endif /*  __cpluscplus */

#define LCM_HIGH_FR      1

vidinfo_t panel_info = { 256, 320, LCD_BPP, };

void st7796s_regulator_enable(struct dsi_device *dsi)
{
}

void st7796s_regulator_disable(struct dsi_device *dsi)
{
}

/* enter sleep */
void st7796s_sleep_in(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};
	write_command(dsi, data_to_send);
    printf("st7796s set sleep in\n");
}

/* exit sleep */
void st7796s_sleep_out(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};
	write_command(dsi, data_to_send);
    printf("st7796s set sleep out\n");
}

/* display on */
void st7796s_display_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};
	write_command(dsi, data_to_send);
    printf("st7796s set display on\n");
}

/* display off */
void st7796s_display_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};
	write_command(dsi, data_to_send);
    printf("st7796s set display off\n");
}

/* TBD: set pixels off */
void st7796s_set_pixel_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x39, 0x02, 0x00, {0x22,0x00}};
	write_command(dsi, data_to_send);
    printf("st7796s set pixel off\n");
}

/* TBD: set pixels on */
void st7796s_set_pixel_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x39, 0x02, 0x00, {0x23,0x00}};
	write_command(dsi, data_to_send);
    printf("st7796s set pixel on\n");
}

/* set brightness */
void st7796s_set_brightness(struct dsi_device *dsi, unsigned int brightness)
{
	if(brightness >= 255) {
		debug("the max brightness is 255, set it 255\n");
		brightness = 255;
	}
	struct dsi_cmd_packet data_to_send[] = {
        {0x39, 0x02, 0x00, {0x51, brightness}},
    };
    int  i;
	for(i = 0; i < ARRAY_SIZE(data_to_send); i++) {
		write_command(dsi,  data_to_send[i]);
	}
    printf("st7796s set brightness to %d\n", brightness);
}

void panel_power_on(void)
{
	gpio_direction_output(st7796s_pdata.gpio_rst, 1);
	mdelay(10);
	gpio_direction_output(st7796s_pdata.gpio_rst, 0);
	mdelay(10);
	gpio_direction_output(st7796s_pdata.gpio_rst, 1);
	mdelay(140);
	printf("st7796s panel power on\n");
}

void lcd_open_backlight(void)
{
	gpio_direction_output(st7796s_pdata.gpio_lcd_bl, 1);
    serial_puts("st7796s panel open backlight\n");
	return;
}

void lcd_close_backlight(void)
{
	gpio_direction_output(st7796s_pdata.gpio_lcd_bl, 0);
    printf("st7796s panel close backlight\n");
	return;
}

struct dsi_cmd_packet st7796s_init_cmd_list[] = {
    {0x39, 0x02, 0x00, {0x3A, 0x77}},        					/**< Interface pixel format */
    {0x39, 0x02, 0x00, {0xF0, 0xC3}},
    {0x39, 0x02, 0x00, {0xF0, 0x96}},
    {0x39, 0x04, 0x00, {0xB6, 0x89, 0x47, 0x3B}},               /**< Display function control */
    {0x39, 0x05, 0x00, {0xB5, 0x35, 0x35, 0x00, 0x5A}},         /**< Blanking Porch Control */
#ifdef LCM_HIGH_FR
    {0x39, 0x03, 0x00, {0xB1, 0xC0, 0x10}},                     /**< Frame Rate control */
#else
    {0x39, 0x03, 0x00, {0xB1, 0x80, 0x10}},                     /**< Frame Rate control */
#endif
    {0x39, 0x02, 0x00, {0x36, 0x08}},                           /**< Memory Data Access Control */
    {0x39, 0x02, 0x00, {0xB4, 0x01}},                           /**< Display Inversion Control */
    {0x39, 0x09, 0x00, {0xE8, 0x40, 0x84, 0x08, 0x08, 0x10, 0x03, 0x38, 0x33}},
    {0x05, 0x21, 0x00},                                         /**< Display Inversion On */
    {0x39, 0x03, 0x00, {0xB9, 0x00, 0xC0}},
    {0x39, 0x02, 0x00, {0xC5, 0x25}},        					/**< VCMP Control */
    {0x39, 0x02, 0x00, {0xC1, 0x1C}},        					/**< Power Control 2 */
    {0x05, 0x12, 0x00},                                         /**< Partial Display Mode On */
    {0x39, 0x05, 0x00, {0x30, 0x00, 0xA0, 0x01, 0xDF}},        	/**< Partial Area 320 */
    {0x39, 0x0F, 0x00, {0xE0, 0xF0, 0x00, 0x02, 0x04, 0x03, 0x13, 0x2A, 0x65, 0x46, 0x2B, 0x17, 0x15, 0x1D, 0x24}},        /**< Positive gamma control */
    {0x39, 0x0F, 0x00, {0xE1, 0xF0, 0x00, 0x01, 0x04, 0x03, 0x22, 0x2C, 0x33, 0x44, 0x2C, 0x17, 0x16, 0x1C, 0x1E}},        /**< Negative gamma control */
    {0x39, 0x02, 0x00, {0xF0, 0x3C}},
    {0x39, 0x02, 0x00, {0xF0, 0x69}},
    {0x39, 0x02, 0x00, {0x51, 0xFF}},        					/**< Write Display Brightness */
    {0x39, 0x02, 0x00, {0x53, 0x24}},        					/**< Write CTRL Display */
    {0x39, 0x02, 0x00, {0x55, 0x00}},        					/**< Write Adaptive Brightness Control */
    {0x39, 0x02, 0x00, {0x35, 0x00}},        					/**< Tearing Effect Line On */
    {0x39, 0x05, 0x00, {0x2A, 0x00, 0x20, 0x01, 0x1F}},        	/**< Column address set 32~287 */
    {0x39, 0x05, 0x00, {0x2B, 0x00, 0xA0, 0x01, 0xDF}},        	/**< Row address set 0~319 */
    /** workaround for stronix */
    {0x05, 0x11, 0x00},                                         /**< Sleep out */
    {0xFF, 120, 0},
};


static void st7796s_panel_condition_setting(struct dsi_device *dsi)
{
    int  i;
	for(i = 0; i < ARRAY_SIZE(st7796s_init_cmd_list); i++) {
		write_command(dsi,  st7796s_init_cmd_list[i]);
	}
}

void panel_init_set_sequence(struct dsi_device *dsi)
{
	st7796s_panel_condition_setting(dsi);
	mdelay(360);
    printf("st7796s panel initialization\n");
}

void panel_pin_init(void)
{
	int ret = 0;
	ret = gpio_request(st7796s_pdata.gpio_rst, "lcd mipi panel rst");
	printf("st7796s panel display pin init\n");
}

struct fb_videomode jzfb1_videomode = {
	.name = "st7796s-lcd",
	.refresh = 60,
	.xres = 256,
	.yres = 320,
	.pixclock = KHZ2PICOS(4916),
#ifdef ST7796S_IN_VIDEO_MODE
	.left_margin = 90, // HBP
	.right_margin = 90, // HFP
	.upper_margin = 32,// VBP
	.lower_margin = 32, // VFP
	.hsync_len = 32,// HSA
	.vsync_len = 32,// VSA
#else
    .left_margin = 0, // HBP
    .right_margin = 0, // HFP
    .upper_margin = 0,// VBP
    .lower_margin = 0, // VFP
    .hsync_len = 0,// HSA
    .vsync_len = 0,// VSA
#endif
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};



#ifdef __cplusplus
#if __cplusplus
}
#endif /*  __cpluscplus */
#endif /*  __cpluscplus */


/** @}*/
