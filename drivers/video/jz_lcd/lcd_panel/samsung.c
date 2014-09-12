/* linux/drivers/video/exynos/samsung.c
 *
 * MIPI-DSI based samsung AMOLED lcd 4.65 inch panel driver.
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

vidinfo_t panel_info = { 320, 320, LCD_BPP, };
#define MIPI_RST_N GPIO_PC(19)


void samsung_regulator_enable(struct dsi_device *dsi)
{
#if 0
	int ret = 0;
	struct lcd_platform_data *pd = NULL;

	pd = lcd->ddi_pd;
	mutex_lock(&lcd->lock);
	if (!lcd->enabled) {
		regulator_enable(lcd->lcd_vcc_reg);
		regulator_enable(lcd->lcd_vcc_reg1);
		if (ret)
			goto out;

		lcd->enabled = true;
	}
	mdelay(pd->power_on_delay);
out:
	mutex_unlock(&lcd->lock);
#endif
}

void samsung_regulator_disable(struct dsi_device *dsi)
{
#if 0
	int ret = 0;

	mutex_lock(&lcd->lock);
	if (lcd->enabled) {
		regulator_disable(lcd->lcd_vcc_reg);
		regulator_disable(lcd->lcd_vcc_reg);
		if (ret)
			goto out;

		lcd->enabled = false;
	}
out:
	mutex_unlock(&lcd->lock);
#endif
}

void samsung_sleep(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};

	write_command(dsi, data_to_send);
}

void samsung_sleep_out(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};

	write_command(dsi, data_to_send);
}

void samsung_memory_access(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x2c, 0x00};

	write_command(dsi, data_to_send);
}


void samsung_display_on(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};

	write_command(dsi, data_to_send);
}

void samsung_display_off(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};

	write_command(dsi, data_to_send);
}


int samsung_set_power(struct lcd_device *ld, int power)
{
    return 0;
}

void panel_power_on(void)
{
	debug("--------------------%s\n", __func__);
	gpio_direction_output(MIPI_RST_N, 1);
	mdelay(300);
	gpio_direction_output(MIPI_RST_N, 0);  //reset active low
	mdelay(10);
	gpio_direction_output(MIPI_RST_N, 1);
	mdelay(5);
	serial_puts("lh155 panel display on\n");

#if 0
	struct samsung *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (power)
		samsung_regulator_enable(lcd);
	else
		samsung_regulator_disable(lcd);

	/* lcd power on */
	if (lcd->ddi_pd->power_on)
		lcd->ddi_pd->power_on(lcd->ld, power);
	/* lcd reset */
	if (lcd->ddi_pd->reset)
		lcd->ddi_pd->reset(lcd->ld);
#endif
}
struct dsi_cmd_packet samsung_cmd_list1[] = {
    {0x39, 0x03, 0x00, {0xF2, 0x1C, 0x28}},
    {0x39, 0x04, 0x00, {0xB5, 0x0A, 0x01, 0x00}},/* Frame Freq = 60Hz, ALPM CTL Boosting */
    {0x39, 0x05, 0x00, {0x2A, 0x00,0x14,0x01,0x53}},/* column address */
    {0x39, 0x05, 0x00, {0x2B, 0x01,0x40,0x00,0x01}},/* Page address */
    {0x39, 0x0E, 0x00, {0xF8, 0x08, 0x08, 0x08, 0x17, 0x00, 0x2A, 0x02, 0x26, 0x00, 0x00, 0x02, 0x00, 0x00}},
    {0x39, 0x02, 0x00, {0xF7, 0x02}},/* Normal2 + BICTL0*/
};
struct dsi_cmd_packet samsung_cmd_list2[] = {
    {0x39, 0x02, 0x00, {0x51, 0xEA}},/* Write Display Brightness */
    {0x39, 0x02, 0x00, {0x53, 0x20}},/* Write ControlDisplay */
};
struct dsi_cmd_packet samsung_cmd_list3[] = {
    {0x39, 0x03, 0x00, {0xB1, 0x00, 0x09}},/* ELVSS Condition SET */
    {0x39, 0x02, 0x00, {0x36, 0x80}},/* reversal left and right */
};


static void samsung_panel_test_key_enable(struct dsi_device *dsi)
{
	struct dsi_cmd_packet data_to_send0 = {0x39, 0x03, 0x00, {0xf0, 0x5a, 0x5a}};
	struct dsi_cmd_packet data_to_send1 = {0x39, 0x03, 0x00, {0xf1, 0x5a, 0x5a}};

	write_command(dsi, data_to_send0);
	write_command(dsi, data_to_send1);
}


static void samsung_panel_test_key_disable(struct dsi_device *dsi)
{
    int  i;
	struct dsi_cmd_packet data_to_send = {0x15, 0xa5, 0xa5};

	write_command(dsi, data_to_send);
}

static void samsung_panel_condition_setting(struct dsi_device *dsi)
{
    int  i;
	for(i = 0; i < ARRAY_SIZE(samsung_cmd_list1); i++) {
		write_command(dsi,  samsung_cmd_list1[i]);
	}

}

static void samsung_brightness_setting(struct dsi_device *dsi)
{
    int  i;
	for(i = 0; i < ARRAY_SIZE(samsung_cmd_list2); i++) {
		write_command(dsi,  samsung_cmd_list2[i]);
	}

}

static void samsung_etc_setting(struct dsi_device *dsi)
{
    int  i;
	for(i = 0; i < ARRAY_SIZE(samsung_cmd_list3); i++) {
		write_command(dsi,  samsung_cmd_list3[i]);
	}

}

void panel_init_set_sequence(struct dsi_device *dsi)
{
    samsung_panel_test_key_enable(dsi);
    samsung_panel_condition_setting(dsi);
    samsung_sleep_out(dsi);
    mdelay(120);
    samsung_brightness_setting(dsi);
    samsung_etc_setting(dsi);
    samsung_memory_access(dsi);
    samsung_panel_test_key_disable(dsi);
    samsung_display_on(dsi);
    samsung_memory_access(dsi);
}

void panel_pin_init(void)
{
	debug("--------------------%s\n", __func__);
	int ret= 0;
	ret = gpio_request(MIPI_RST_N, "lcd mipi panel rst");

	serial_puts("byd_9177aa panel display pin init\n");
}


struct fb_videomode jzfb1_videomode = {
	.name = "samsung-lcd",
	.refresh = 60,
	.xres = 320,
	.yres = 320,
	.pixclock = KHZ2PICOS(6144),
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

