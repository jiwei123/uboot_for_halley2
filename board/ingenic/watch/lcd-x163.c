/*
 * Copyright (c) 2014 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 *  dorado board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <common.h>
#include <regulator.h>
//#include <include/regulator.h>
#include <asm/gpio.h>
#include <jz_lcd/jz_lcd_v1_2.h>
#include <linux/err.h>
#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/auo_x163.h>

struct lcd_power_regulator {
    char *name;
    int  voltage;
    int  mdelay;
    struct regulator *regulator;
};

#define LCD_REGULATOR_REG(_name, _voltage, _mdelay) \
{ \
    .name    = _name, \
    .voltage = _voltage, \
    .mdelay  = _mdelay, \
}

static struct lcd_power_regulator lcd_power_regulator[] = {
#ifdef CONFIG_PMU_RICOH6x
#if defined(CONFIG_AW808)
    LCD_REGULATOR_REG("RICOH619_DC5",   3400000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO4",  1800000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO6",  2800000, 5),
#elif defined(CONFIG_X3)
    LCD_REGULATOR_REG("RICOH619_DC5",   3400000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO4",  1800000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO6",  2800000, 5),
#elif defined(CONFIG_F1)
    LCD_REGULATOR_REG("RICOH619_DC5",   3400000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO4",  1800000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO6",  2950000, 5),
#else
    LCD_REGULATOR_REG("RICOH619_LDO9",  1800000, 5),
    LCD_REGULATOR_REG("RICOH619_LDO10", 3300000, 5),
#endif
#elif defined CONFIG_PMU_D2041
#elif defined CONFIG_PMU_SM5007
#if defined(CONFIG_SOLAR)
    LCD_REGULATOR_REG("SM5007_BUCK4", 3300000, 5),
    LCD_REGULATOR_REG("SM5007_LDO2",  1800000, 5),
    LCD_REGULATOR_REG("SM5007_LDO4",  2800000, 5),
#endif
#endif
};

#if defined(CONFIG_ACRAB)
#define GPIO_LCD_BLK_EN GPIO_PC(9)
#define MIPI_RST_N GPIO_PC(16)
#elif defined(CONFIG_AW808)
#define GPIO_LCD_BLK_EN GPIO_PC(23)
#define MIPI_RST_N GPIO_PC(19)
#elif defined(CONFIG_X3) || defined(CONFIG_F1)
#define GPIO_LCD_BLK_EN GPIO_PC(23)
#define MIPI_RST_N GPIO_PC(19)
#elif defined(CONFIG_NEWTON2)
#define MIPI_RST_N GPIO_PC(19)
#elif defined(CONFIG_SOLAR)
#define GPIO_LCD_BLK_EN GPIO_PD(0)
#define MIPI_RST_N GPIO_PD(3)
#endif

static inline struct regulator *lcd_power_regulator_init(const char *id, int voltage, int delay)
{
    struct regulator *lcd_regulator;
    if (voltage < 0 || id == NULL ) {
        printf("lcd power regulator init args wrong\n");
        return -1;
    }

    lcd_regulator = regulator_get(id);
    if (lcd_regulator) {
        regulator_set_voltage(lcd_regulator, voltage, voltage);
        regulator_enable(lcd_regulator);
    } else {
        printf("%s regulator get failed\n", id);
        return NULL;
    }

    if (delay);
        mdelay(delay);

    return lcd_regulator;
}

void board_set_lcd_power_on(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(lcd_power_regulator); i++) {
		struct regulator *reg = NULL;
        reg = lcd_power_regulator_init(lcd_power_regulator[i].name,
                lcd_power_regulator[i].voltage,
                lcd_power_regulator[i].mdelay);
		if (!reg) {
			printf("init lcd power %s error:%d\n", lcd_power_regulator[i].name, PTR_ERR(reg));
		} else {
			lcd_power_regulator[i].regulator = reg;
		}
    }
}

void board_set_lcd_power_off(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(lcd_power_regulator); i++) {
		regulator_disable(lcd_power_regulator[i].regulator);
		mdelay(lcd_power_regulator[i].mdelay);
    }
}

struct auo_x163_platform_data auo_x163_pdata = {
    .gpio_rst = MIPI_RST_N,
#if (defined(CONFIG_ACRAB) || defined(CONFIG_AW808) ||defined(CONFIG_X3) ||defined(CONFIG_SOLAR)) || defined(CONFIG_F1)
	.gpio_lcd_bl = GPIO_LCD_BLK_EN,
#endif
};

struct dsi_config jz_dsi_config={
    .max_lanes = 1,
    .max_hs_to_lp_cycles = 100,
    .max_lp_to_hs_cycles = 40,
    .max_bta_cycles = 4095,
    .min_mbps = 224, /* 224Mbps */
    .color_mode_polarity = 1,
    .shut_down_polarity = 1,
    .auto_clklane_ctrl = 0,
};

struct video_config jz_dsi_video_config={
    .no_of_lanes = 1,
    .virtual_channel = 0,
    .color_coding = COLOR_CODE_24BIT,
    //.color_coding = COLOR_CODE_18BIT_CONFIG1,
    //.byte_clock = ( CONFIG_DEFAULT_BYTE_CLOCK * 1000) / 8,
    .video_mode = VIDEO_BURST_WITH_SYNC_PULSES,

    .receive_ack_packets = 0,	/* enable receiving of ack packets */
    .is_18_loosely = 0, /*loosely: R0R1R2R3R4R5__G0G1G2G3G4G5G6__B0B1B2B3B4B5B6,
    not loosely: R0R1R2R3R4R5G0G1G2G3G4G5B0B1B2B3B4B5*/
    .data_en_polarity = 1,
};

struct dsi_device jz_dsi = {
    .dsi_config = &jz_dsi_config,
    .video_config = &jz_dsi_video_config,
};

struct jzfb_config_info jzfb1_init_data = {
    //.num_modes = 1,
    .modes = &jzfb1_videomode,

    .lcd_type = LCD_TYPE_SLCD,
    .bpp = 24,

    .smart_config.smart_type      = SMART_LCD_TYPE_PARALLEL,
    .smart_config.clkply_active_rising = 0,
    .smart_config.rsply_cmd_high = 0,
    .smart_config.csply_active_high = 0,
    .smart_config.bus_width = 8,
    .dither_enable = 1,
    .dither.dither_red   = 1,	/* 6bit */
    .dither.dither_green = 1,	/* 6bit */
    .dither.dither_blue  = 1,	/* 6bit */
};
