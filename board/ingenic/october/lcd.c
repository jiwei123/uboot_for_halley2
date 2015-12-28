/*
 * Ingenic dorado lcd code
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
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

#ifdef CONFIG_VIDEO_ECX336AF
#include <jz_lcd/ecx336af.h>
struct ecx336af_data ecx336af_pdata = {
	.gpio_reset = GPIO_PC(9),
	.gpio_spi_cs = GPIO_PE(2),
	.gpio_spi_clk = GPIO_PE(1),
	.gpio_spi_sdi = GPIO_PE(3),
	.gpio_spi_sdo = GPIO_PA(13),
	.power_1v8 = NULL,
};
#endif

#if defined(CONFIG_VIDEO_HIMAX7069)
#include <jz_lcd/jz_dsim.h>
unsigned long himax7069_cmd_buf[]= {
    0x2C2C2C2C,
};

struct dsi_config jz_dsi_config={
    .max_lanes = 2,
    .max_hs_to_lp_cycles = 100,
    .max_lp_to_hs_cycles = 40,
    .max_bta_cycles = 4095,
    .color_mode_polarity = 1,
    .shut_down_polarity = 1,
};

struct video_config jz_dsi_video_config={
    .no_of_lanes = 2,
    .virtual_channel = 0,
    .color_coding = COLOR_CODE_24BIT,
    .receive_ack_packets = 0,	/* enable receiving of ack packets */
    .is_18_loosely = 0, /*loosely: R0R1R2R3R4R5__G0G1G2G3G4G5G6__B0B1B2B3B4B5B6,
			  not loosely: R0R1R2R3R4R5G0G1G2G3G4G5B0B1B2B3B4B5*/
    .data_en_polarity = 1,
};

struct dsi_device jz_dsi = {
    .dsi_config = &jz_dsi_config,
    .video_config = &jz_dsi_video_config,
};
#endif

void board_set_lcd_power_on(void)
{
	char *id = "RICOH619_DC5";

	struct regulator *lcd_regulator = regulator_get(id);
	if (lcd_regulator) {
		regulator_set_voltage(lcd_regulator, 1800000, 1800000);
		regulator_enable(lcd_regulator);

		gpio_request(GPIO_PD(27), "lcd_pwm_en");
		gpio_direction_output(GPIO_PD(27), 1);
	} else
		printf("Enable RICOH619_DC5 FAIL!\n");
#ifdef CONFIG_VIDEO_ECX336AF
	ecx336af_pdata.power_1v8 = lcd_regulator;
#endif
}

void board_set_lcd_power_off(void)
{
	char *id = "RICOH619_DC5";

	struct regulator *lcd_regulator = regulator_get(id);
	if (lcd_regulator) {
		regulator_disable(lcd_regulator);

		gpio_request(GPIO_PD(27), "lcd_pwm_en");
		gpio_direction_output(GPIO_PD(27), 0);
	} else
		printf("GET RICOH619_DC5 FAIL!\n");
#ifdef CONFIG_VIDEO_ECX336AF
	ecx336af_pdata.power_1v8 = lcd_regulator;
#endif
}

void board_set_backlight_init(int num)
{
    int i = 0;

   lcd_init_backlight(0);
    for(; i <= num; i += 10){
	mdelay(20);
	lcd_set_backlight_level(i);
    }
}

struct jzfb_config_info jzfb1_init_data = {
#if defined(CONFIG_VIDEO_HIMAX7097)
	.modes = &jzfb1_videomode,
	.lcd_type = LCD_TYPE_GENERIC_24_BIT,
	.bpp = 16,

	.pixclk_falling_edge = 1,
	.date_enable_active_low = 0,

	.dither_enable = 0,
#elif defined(CONFIG_VIDEO_HIMAX7069)
	.modes = &jzfb1_videomode,
	.lcd_type =  LCD_TYPE_SLCD,
	.bpp = 24,

	.smart_config.smart_type      = SMART_LCD_TYPE_PARALLEL,
	.smart_config.clkply_active_rising = 0,
	.smart_config.rsply_cmd_high = 0,
	.smart_config.csply_active_high = 0,
	.smart_config.write_gram_cmd = himax7069_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(himax7069_cmd_buf),
	.smart_config.bus_width = 8,
#else 
#error "Please add the board data!!!"
#endif
};
