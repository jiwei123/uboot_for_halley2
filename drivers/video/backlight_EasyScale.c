/*
 *  Copyright (C) 2015 Wu Jiao <jiao.wu@ingenic.com wujiaososo@qq.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <config.h>
#include <lcd.h>
#include <asm/types.h>
#include <asm/arch/tcu.h>
#include <asm/arch/gpio.h>
#include <common.h>

void EasyScale_board_set_power(void *data, int power_on);

#ifndef CONFIG_EASYSCALE_MAX_STEPS
#define CONFIG_EASYSCALE_CTRL_STEPS 32
#endif

#ifndef CONFIG_SYS_BACKLIGHT_LEVELS
#define CONFIG_SYS_BACKLIGHT_LEVELS 256
#endif

#define set_ctrl(level) \
	gpio_direction_output(gpio, level)

struct EasyScale_one_wire {
	/* dc-dc chip power on */
	void (*set_power) (void *data, int power_on);
	void *data;

	/* CTRL pin */
	unsigned int gpio_ctrl;
	unsigned int gpio_pwr_en;

	unsigned int power;

	unsigned int cur_step;
	unsigned int cur_brightness;

	unsigned int max_steps;
	unsigned int max_brightness;
};

static struct EasyScale_one_wire mEasyScale = {
	.set_power = EasyScale_board_set_power,
	.max_steps = CONFIG_EASYSCALE_CTRL_STEPS - 1,
	.max_brightness = CONFIG_SYS_BACKLIGHT_LEVELS - 1,
	.gpio_pwr_en = CONFIG_GPIO_EASYSCALE_PWR_EN,
	.gpio_ctrl = CONFIG_GPIO_EASYSCALE_CTRL,
	.data = &mEasyScale,
};

/**
 * those parameters below are takes from TPS61165.
 * if you have a new EasyScale dc-dc device, add your config here
 *
 * Device support list:
 * @TPS61165
 *
 */
#define t_es_dect  300			/* in us */
#define t_es_delay 150			/* in us */
#define t_es_win   2			/* in ms */
#define t_off      3			/* in ms */

#define t_start 3
#define t_eos   3

#define t_h_lb  3
#define t_l_lb  8
#define t_h_hb  8
#define t_l_hb  3

static void enter_es_mode(unsigned int gpio) {
	set_ctrl(1);
	udelay(t_es_delay);
	set_ctrl(0);
	udelay(t_es_dect);
	set_ctrl(1);
	mdelay(t_es_win);
}

static void send_start(unsigned int gpio) {
	set_ctrl(1);
	udelay(t_start);
}

static void send_high(unsigned int gpio) {
	set_ctrl(0);
	udelay(t_h_lb);
	set_ctrl(1);
	udelay(t_h_hb);
}

static void send_low(unsigned int gpio) {
	set_ctrl(0);
	udelay(t_l_lb);
	set_ctrl(1);
	udelay(t_l_hb);
}

static void send_eos(unsigned int gpio) {
	set_ctrl(0);
	udelay(t_eos);
	set_ctrl(1);
}

static void send_shutdown(unsigned int gpio) {
	set_ctrl(0);
	mdelay(t_off);
}

static void send_byte(unsigned int gpio, unsigned char byte) {
	int i;

	send_start(gpio);
	for (i = 7; i >= 0; --i) {
		if (byte & (1 << i))
			send_high(gpio);
		else
			send_low(gpio);
	}
	send_eos(gpio);
}

static void EasyScale_set_power(struct EasyScale_one_wire *es, int power_on) {
	if (power_on && !es->power) {
		gpio_direction_output(es->gpio_ctrl, 0);
		if (es->set_power)
			es->set_power(es->data, 1);
		gpio_direction_output(es->gpio_ctrl, 1);
		enter_es_mode(es->gpio_ctrl);
	} else if (!power_on && es->power) {
		send_shutdown(es->gpio_ctrl);
		if (es->set_power)
			es->set_power(es->data, 0);
	}
	es->power = !!power_on;
}

/**
 * EasyScale_set_brightness() - set brightness by EasyScale
 *
 * @brightness - the brightness relative to CONFIG_SYS_BACKLIGHT_LEVELS
 */
void EasyScale_set_brightness(unsigned int brightness) {
	unsigned int level;
	unsigned int address = 0x72;
	struct EasyScale_one_wire *es = &mEasyScale;

	if (brightness > es->max_brightness)
		brightness = es->max_brightness;

	if (brightness != 0) {
		level = (es->max_steps * brightness) / es->max_brightness;
		if (level < es->max_steps)
			level = level + 1;
		EasyScale_set_power(&mEasyScale, 1);
		send_byte(es->gpio_ctrl, address);
		send_byte(es->gpio_ctrl, level);
	} else {
		EasyScale_set_power(&mEasyScale, 0);
	}
}

/**
 * EasyScale_board_set_power() - default board defined EasyScale set power function
 *
 * board may use regulator or other way to control DC-DC's power suppply,
 * if that is true, define your function somewhere. or rebuilt this function
 */
__weak void EasyScale_board_set_power(void *data, int power_on) {
	struct EasyScale_one_wire *es = data;

	if (es->gpio_pwr_en < 0)
		return;

	if (power_on) {
		gpio_direction_output(es->gpio_pwr_en, 1);
		mdelay(10);
	} else {
		gpio_direction_output(es->gpio_pwr_en, 0);
	}
}
