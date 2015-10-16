/*******************************************************************************
 *               Copyright (C) 2015, GYENNO Tech. Co., Ltd.
 *                      ALL RIGHTS RESERVED
 *******************************************************************************/

/** @defgroup st7796s.h

 *  @author  lingyun
 *  @version 1.0
 *  @date    2015/2/5 \n

 *  history:\n
 *          1. 2015/2/5, lingyun, create this file\n\n
 *
 * @{
 */

#ifndef __ST7796S_H__
#define __ST7796S_H__

#include <jz_lcd/jz_dsim.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /*  __cpluscplus */
#endif /*  __cpluscplus */



struct st7796s_platform_data {
	unsigned int gpio_rst;
	unsigned int gpio_lcd_bl;
};

extern struct st7796s_platform_data st7796s_pdata;
extern void st7796s_sleep_in(struct dsi_device *dsi);
extern void st7796s_sleep_out(struct dsi_device *dsi);
extern void st7796s_display_on(struct dsi_device *dsi);
extern void st7796s_display_off(struct dsi_device *dsi);
extern void st7796s_set_pixel_off(struct dsi_device *dsi); /* set_pixels_off */
extern void st7796s_set_pixel_on(struct dsi_device *dsi); /* set_pixels_on */
extern void st7796s_set_brightness(struct dsi_device *dsi, unsigned int brightness); /* set brightness */



#ifdef __cplusplus
#if __cplusplus
}
#endif /*  __cpluscplus */
#endif /*  __cpluscplus */


#endif  /* __ST7796S_H__ */

/** @}*/
