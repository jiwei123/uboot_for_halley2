/*
 * Copyright (C) 2014 Ingenic Electronics
 *
 * Orise OTM3201A Driver (driver's Header File part)
 *
 * Author: MaoLei.Wang <maolei.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _ORISE_OTM3201A_H_
#define _ORISE_OTM3201A_H_

#include <jz_lcd/jz_dsim.h>

struct orise_otm3201a_platform_data {
	unsigned int gpio_rest;
	unsigned int pwm_lcd_brightness;
};

extern struct orise_otm3201a_platform_data orise_otm3201a_pdata;
extern void orise_otm3201a_sleep_in(struct dsi_device *dsi);
extern void orise_otm3201a_sleep_out(struct dsi_device *dsi);
extern void orise_otm3201a_display_on(struct dsi_device *dsi);
extern void orise_otm3201a_display_off(struct dsi_device *dsi);
extern void orise_otm3201a_set_pixel_off(struct dsi_device *dsi); /* set_pixels_off */
extern void orise_otm3201a_set_pixel_on(struct dsi_device *dsi); /* set_pixels_on */
extern void orise_otm3201a_set_brightness(struct dsi_device *dsi, unsigned int brightness); /* set brightness */


#endif /* _ORISE_OTM3201A_H_ */
