 /*
 * JZ4780 LCDC DRIVER
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

#include <asm/io.h>
#include <config.h>
#include <serial.h>
#include <common.h>
#include <lcd.h>
#include <asm/arch/lcdc.h>
#include "jz4780_lcd.h"
#include <asm/arch/gpio.h>

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;
/* Frame buffer memory information */
void *lcd_base;			/* Start of framebuffer memory	*/
void *lcd_console_address;	/* Start of console buffer	*/
short console_col;
short console_row;
struct jzfb_config_info lcd_config_info;
static int lcd_enable_state = 0;
void board_lcd_init(void);
void flush_cache_all(void);
void lcd_close_backlight(void);
void lcd_set_backlight_level(int num);
#define reg_write(addr,config) \
	writel(config,lcd_config_info.lcdbaseoff+addr)
#define reg_read(addr)	\
	readl(lcd_config_info.lcdbaseoff+addr)

vidinfo_t panel_info = {
#if defined(CONFIG_VIDEO_KR070LA0S_270_65HZ)
	1024,600,LCD_BPP,
#elif defined(CONFIG_VIDEO_KR070LA0S_270)
	1024,600,LCD_BPP,
#elif defined(CONFIG_VIDEO_KR080LA4S_250)
	1024,768,LCD_BPP,
#elif defined(CONFIG_VIDEO_SL007DC18B05)
	1024,600,LCD_BPP,
#elif defined(CONFIG_VIDEO_EK070TN93)
	800,480,LCD_BPP,
#elif defined(CONFIG_VIDEO__AT065TN14)
	800,480,LCD_BPP,
#elif defined(CONFIG_VIDEO_JCMT070T115A18)
	800,480,LCD_BPP,
#elif  defined(CONFIG_VIDEO_HSD101PWW1)
	1280,800,LCD_BPP,
#elif defined(CONFIG_VIDEO_LP101WX1_SLN2)
	1280,800,LCD_BPP,
#elif defined(CONFIG_VIDEO_KD50G2_40NM_A2)
	800,480,LCD_BPP,
#elif defined(CONFIG_VIDEO_TFT_HSD070IDW1)
	800,480,LCD_BPP,
#elif defined(CONFIG_VIDEO__HHX070ML208CP21)
	1024,600,LCD_BPP,
#elif defined(CONFIG_VIDEO_BYD_BM8766U)
	800,480,LCD_BPP,
#elif defined(CONFIG_VIDEO_KFM701A21_1A)
	400,240,LCD_BPP,
#elif  defined(CONFIG_VIDEO_TFT_720P)
	1280,720,LCD_BPP,
#elif defined(CONFIG_VIDEO_TM080TDH01)
	1024,768,LCD_BPP,
#elif defined(CONFIG_VIDEO_S369FG06)   /* driver IC: TL2796  */
	480,800,LCD_BPP,
#else
#error "Select LCD panel first!!!"
#endif
};

int jzfb_get_controller_bpp(unsigned int bpp)
{
	switch (bpp) {
	case 18:
	case 24:
		return 32;
	case 15:
		return 16;
	default:
		return bpp;
	}
}

static void jzfb_config_fg0(struct jzfb_config_info *info)
{
	unsigned int rgb_ctrl, cfg;

	/* OSD mode enable and alpha blending is enabled */
	cfg = LCDC_OSDC_OSDEN | LCDC_OSDC_ALPHAEN ;//|	LCDC_OSDC_PREMULTI0;
	cfg |= 1 << 16; /* once transfer two pixels */
	cfg |= LCDC_OSDC_COEF_SLE0_1;
	/* OSD control register is read only */

	if (info->fmt_order == FORMAT_X8B8G8R8) {
		rgb_ctrl = LCDC_RGBC_RGBFMT | LCDC_RGBC_ODD_BGR |
			LCDC_RGBC_EVEN_BGR;
	} else {
		/* default: FORMAT_X8R8G8B8*/
		rgb_ctrl = LCDC_RGBC_RGBFMT | LCDC_RGBC_ODD_RGB |
			LCDC_RGBC_EVEN_RGB;
	}
	reg_write(LCDC_OSDC,cfg);
	reg_write(LCDC_RGBC,rgb_ctrl);
}

static void jzfb_config_tft_lcd_dma(struct jzfb_config_info *info)
{
	struct jz_fb_dma_descriptor *framedesc = info->dmadesc_fbhigh;

#define BYTES_PER_PANEL	 (((info->modes->xres * jzfb_get_controller_bpp(info->bpp) / 8 + 3) >> 2 << 2) * info->modes->yres)
	framedesc->fdadr = virt_to_phys((void*)info->dmadesc_fbhigh);
	framedesc->fsadr = virt_to_phys((void *)info->screen);
	framedesc->fidr = 0xda0;
#ifdef CONFIG_VIDEO_JZ4775
	framedesc->ldcmd = LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN;
#endif
#ifdef CONFIG_VIDEO_JZ4780
	framedesc->ldcmd = LCDC_CMD_SOFINT | LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN;
#endif
	framedesc->ldcmd |= BYTES_PER_PANEL/4;
	framedesc->offsize = 0;
	framedesc->page_width = 0;
	info->fdadr0 = virt_to_phys((void*)info->dmadesc_fbhigh);

	switch (jzfb_get_controller_bpp(info->bpp)) {
	case 16:
		framedesc->cmd_num = LCDC_CPOS_RGB_RGB565
			| LCDC_CPOS_BPP_16;
		break;
	case 30:
		framedesc->cmd_num = LCDC_CPOS_BPP_30;
		break;
	default:
		framedesc->cmd_num = LCDC_CPOS_BPP_18_24;
		break;
	}
	/* global alpha mode */
	framedesc->cmd_num |= 0;
	/* data has not been premultied */
	framedesc->cmd_num |= LCDC_CPOS_PREMULTI;
	/* coef_sle 0 use 1 */
	framedesc->cmd_num |= LCDC_CPOS_COEF_SLE_1;

	/* fg0 alpha value */
	framedesc->desc_size = 0xff << LCDC_DESSIZE_ALPHA_BIT;
	framedesc->desc_size |= (((info->modes->yres - 1) << LCDC_DESSIZE_HEIGHT_BIT & LCDC_DESSIZE_HEIGHT_MASK) |
			((info->modes->xres - 1) << LCDC_DESSIZE_WIDTH_BIT & LCDC_DESSIZE_WIDTH_MASK));
}

static void jzfb_config_smart_lcd_dma(struct jzfb_config_info *info)
{
	unsigned long bypes_per_panel;
	struct jz_fb_dma_descriptor *framedesc = info->dmadesc_fbhigh;
	struct jz_fb_dma_descriptor *framedesc_cmd[2];

	framedesc_cmd[0] = info->dmadesc_cmd;
	framedesc_cmd[1] = info->dmadesc_cmd_tmp;

	bypes_per_panel = (((info->modes->xres * jzfb_get_controller_bpp(info->bpp)
			     / 8 + 3) >> 2 << 2) * info->modes->yres);
	framedesc->fdadr = virt_to_phys((void *)info->dmadesc_cmd);
	framedesc->fsadr = virt_to_phys((void *)info->screen);
	framedesc->fidr = 0xda0da0;
	framedesc->ldcmd = LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN;
	framedesc->ldcmd |= bypes_per_panel / 4;
	framedesc->offsize = 0;
	framedesc->page_width = 0;

	switch (jzfb_get_controller_bpp(info->bpp)) {
	case 16:
		framedesc->cmd_num = LCDC_CPOS_RGB_RGB565
			| LCDC_CPOS_BPP_16;
		break;
	case 30:
		framedesc->cmd_num = LCDC_CPOS_BPP_30;
		break;
	default:
		framedesc->cmd_num = LCDC_CPOS_BPP_18_24;
		break;
	}
	/* global alpha mode */
	framedesc->cmd_num |= 0;
	/* data has not been premultied */
	framedesc->cmd_num |= LCDC_CPOS_PREMULTI;
	/* coef_sle 0 use 1 */
	framedesc->cmd_num |= LCDC_CPOS_COEF_SLE_1;

	/* fg0 alpha value */
	framedesc->desc_size = 0xff << LCDC_DESSIZE_ALPHA_BIT;
	framedesc->desc_size |= (((info->modes->yres - 1) << LCDC_DESSIZE_HEIGHT_BIT & LCDC_DESSIZE_HEIGHT_MASK) |
			((info->modes->xres - 1) << LCDC_DESSIZE_WIDTH_BIT & LCDC_DESSIZE_WIDTH_MASK));

	framedesc_cmd[0]->fdadr = virt_to_phys((void *)info->dmadesc_fbhigh);
	framedesc_cmd[0]->fsadr = 0;
	framedesc_cmd[0]->fidr = 0xda0da1;
	framedesc_cmd[0]->ldcmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 0;
	framedesc_cmd[0]->offsize = 0;
	framedesc_cmd[0]->page_width = 0;
	framedesc_cmd[0]->cmd_num = 0;
	framedesc_cmd[0]->desc_size = 0;

	framedesc_cmd[1]->fdadr = virt_to_phys((void *)info->dmadesc_fbhigh);
	framedesc_cmd[1]->fsadr = virt_to_phys((void *)info->dma_cmd_buf);
	framedesc_cmd[1]->fidr = 0xda0da2;
	framedesc_cmd[1]->ldcmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 4;
	framedesc_cmd[1]->offsize = 0;
	framedesc_cmd[1]->page_width = 0;
	framedesc_cmd[1]->cmd_num = 4;
	framedesc_cmd[1]->desc_size = 0;
	info->fdadr0 = virt_to_phys((void *)info->dmadesc_cmd_tmp);

	flush_cache_all();
}

static void jzfb_config_fg1_dma(struct jzfb_config_info *info)
{
	struct jz_fb_dma_descriptor *framedesc = info->dmadesc_fblow;

	/*
	 * the descriptor of DMA 1 just init once
	 * and generally no need to use it
	 */

#define BYTES_PER_PANEL	 (((info->modes->xres * jzfb_get_controller_bpp(info->bpp) / 8 + 3) >> 2 << 2) * info->modes->yres)
	framedesc->fsadr = virt_to_phys((void *)(info->screen + BYTES_PER_PANEL));
	framedesc->fdadr = (unsigned)virt_to_phys((void *)info->dmadesc_fblow);
	info->fdadr1 = (unsigned)virt_to_phys((void *)info->dmadesc_fblow);

	framedesc->fidr = 0xda1;

	framedesc->ldcmd = (LCDC_CMD_EOFINT & ~LCDC_CMD_FRM_EN)
		| (BYTES_PER_PANEL/4);
	framedesc->offsize = 0;
	framedesc->page_width = 0;

	/* global alpha mode, data has not been premultied, COEF_SLE is 11 */
	framedesc->cmd_num = LCDC_CPOS_BPP_18_24 | LCDC_CPOS_COEF_SLE_3 | LCDC_CPOS_PREMULTI;
	framedesc->desc_size |= (((info->modes->yres - 1) << LCDC_DESSIZE_HEIGHT_BIT & LCDC_DESSIZE_HEIGHT_MASK) |
			((info->modes->xres - 1) << LCDC_DESSIZE_WIDTH_BIT & LCDC_DESSIZE_WIDTH_MASK));

	framedesc->desc_size |= 0xff <<
		LCDC_DESSIZE_ALPHA_BIT;

	flush_cache_all();
	reg_write( LCDC_DA1, framedesc->fdadr);
}

static int jzfb_prepare_dma_desc(struct jzfb_config_info *info)
{
	info->dmadesc_fblow = (struct jz_fb_dma_descriptor *)((unsigned long)info->palette - 2*32);
	info->dmadesc_fbhigh = (struct jz_fb_dma_descriptor *)((unsigned long)info->palette - 1*32);
#ifdef CONFIG_VIDEO_JZ4775
	info->dmadesc_cmd = (struct jz_fb_dma_descriptor *)((unsigned long)info->palette - 3*32);
	info->dmadesc_cmd_tmp = (struct jz_fb_dma_descriptor *)((unsigned long)info->palette - 4*32);
#endif
	if (info->lcd_type != LCD_TYPE_LCM) {
		jzfb_config_tft_lcd_dma(info);
	} else {
		jzfb_config_smart_lcd_dma(info);
	}
	jzfb_config_fg1_dma(info);
	return 0;
}

/* Sent a command without data (18-bit bus, 16-bit index) */
static void slcd_send_mcu_command(struct jzfb_config_info *info, unsigned long cmd)
{
	int count = 10000;

	switch (info->smart_config.bus_width) {
	case 18:
		cmd = ((cmd & 0xff) << 1) | ((cmd & 0xff00) << 2);
		break;
	case 16:
		cmd &= 0xffff;
		break;
	case 9:
	case 8:
		while ((reg_read(SLCDC_STATE) & SLCDC_STATE_BUSY) && count--) {
			udelay(10);
		}
		if (count < 0) {
			serial_puts("SLCDC wait busy state wrong\n");
		}
		reg_write(SLCDC_DATA, SLCDC_DATA_RS_COMMAND |
			  (cmd & 0xff00) >> 8);
		count = 10000;
		while ((reg_read(SLCDC_STATE) & SLCDC_STATE_BUSY) && count--) {
			udelay(10);
		}
		if (count < 0) {
			serial_puts("SLCDC wait busy state wrong\n");
		}
		reg_write(SLCDC_DATA, SLCDC_DATA_RS_COMMAND |
			  (cmd & 0xff) >> 0);
		return;
	default:
		serial_puts("Don't support %d bit bus");
		break;
	}
	count = 10000;
	while ((reg_read(SLCDC_STATE) & SLCDC_STATE_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		serial_puts("SLCDC wait busy state wrong\n");
	}
	reg_write(SLCDC_DATA, SLCDC_DATA_RS_COMMAND | cmd);
}

static void slcd_send_mcu_data(struct jzfb_config_info *info, unsigned long data)
{
	int count = 10000;

	switch (info->smart_config.bus_width) {
	case 18:
	case 9:
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		data = ((data << 6) & 0xfc0000) | ((data << 4) & 0xfc00)
			| ((data << 2) & 0xfc);
		break;
	case 16:
	case 8:
		data &= 0xffff;
		break;
	default:
		serial_puts("Don't support %d bit bus");
		break;
	}

	while ((reg_read(SLCDC_STATE) & SLCDC_STATE_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		serial_puts("SLCDC wait busy state wrong\n");
	}
	reg_write(SLCDC_DATA, SLCDC_DATA_RS_DATA | data);
}

/* Sent a command with data (18-bit bus, 16-bit index, 16-bit register value) */
static void slcd_set_mcu_register(struct jzfb_config_info *info, unsigned long cmd,
					unsigned long data)
{
	slcd_send_mcu_command(info, cmd);
	slcd_send_mcu_data(info, data);
}

static void jzfb_slcd_mcu_init(struct jzfb_config_info *info)
{
	unsigned int is_enabled, i;
	unsigned long tmp;

	if (info->lcd_type != LCD_TYPE_LCM)
		return;

	is_enabled = lcd_enable_state;
	if (!is_enabled) {
		lcd_enable();
	}

	if (info->smart_config.length_data_table &&
	    info->smart_config.data_table) {
		for (i = 0; i < info->smart_config.length_data_table; i++) {
			switch (info->smart_config.data_table[i].type) {
			case 0:
				slcd_set_mcu_register(
					info,
					info->smart_config.data_table[i].reg,
					info->smart_config.data_table[i].value);
				break;
			case 1:
				slcd_send_mcu_command(
					info,
					info->smart_config.data_table[i].value);
				break;
			case 2:
				slcd_send_mcu_data(
					info,
					info->smart_config.data_table[i].value);
				break;
			default:
				serial_puts("Unknow SLCD data type\n");
				break;
			}
			udelay(info->smart_config.data_table[i].udelay);
		}
	}
	/* SLCD DMA mode select 0 */
	tmp = reg_read(SLCDC_CTRL);
	tmp &= ~SLCDC_CTRL_DMA_MODE;
	reg_write(SLCDC_CTRL, tmp);

	if (!is_enabled) {
		lcd_disable();
	}
}

void lcd_enable(void)
{
	unsigned ctrl;
	if (lcd_enable_state == 0) {
		reg_write( LCDC_STATE, 0);
		reg_write( LCDC_DA0, lcd_config_info.fdadr0);
		ctrl = reg_read( LCDC_CTRL);
		ctrl |= LCDC_CTRL_ENA;
		ctrl &= ~LCDC_CTRL_DIS;
		reg_write( LCDC_CTRL, ctrl);
		serial_puts("dump_lcdc_registers\n");
	}
	lcd_enable_state= 1;
}

void lcd_disable(void)
{
	unsigned ctrl;
	if (lcd_enable_state == 1) {
		if (lcd_config_info.lcd_type != LCD_TYPE_LCM) {
			ctrl = reg_read( LCDC_CTRL);
			ctrl |= LCDC_CTRL_DIS;
			reg_write(LCDC_CTRL, ctrl);
			while(!(reg_read(LCDC_STATE) & LCDC_STATE_LDD));
		} else {
			/* SLCD and TVE only support quick disable */
			ctrl = reg_read(LCDC_CTRL);
			ctrl &= ~LCDC_CTRL_ENA;
			reg_write(LCDC_CTRL, ctrl);
		}
	}
	lcd_enable_state = 0;
}

static int jzfb_set_par(struct jzfb_config_info *info)
{
	struct fb_videomode *mode = info->modes;
	unsigned short hds, vds;
	unsigned short hde, vde;
	unsigned short ht, vt;
	unsigned cfg, ctrl;
	unsigned size0,size1;
	unsigned smart_cfg = 0, smart_ctrl = 0;;
	unsigned pcfg;

	hds = mode->hsync_len + mode->left_margin;
	hde = hds + mode->xres;
	ht = hde + mode->right_margin;

	vds = mode->vsync_len + mode->upper_margin;
	vde = vds + mode->yres;
	vt = vde + mode->lower_margin;

	/*
	 * configure LCDC config register
	 * use 8words descriptor, not use palette
	 * ! JZ4780 JZ4780 NOT SUPPORT PALETTE FUNCTION, DO NOT SET LCDC_CFG_PALBP(BIT27), IT CAUGHT BPP16 COLOR ERROR.
	 */
	cfg = LCDC_CFG_NEWDES | LCDC_CFG_RECOVER;
	cfg |= info->lcd_type;

	if (!(mode->sync & FB_SYNC_HOR_HIGH_ACT))
		cfg |= LCDC_CFG_HSP;

	if (!(mode->sync & FB_SYNC_VERT_HIGH_ACT))
		cfg |= LCDC_CFG_VSP;

	if (lcd_config_info.pixclk_falling_edge)
		cfg |= LCDC_CFG_PCP;

	if (lcd_config_info.date_enable_active_low)
		cfg |= LCDC_CFG_DEP;

	/* configure LCDC control register */
	ctrl = LCDC_CTRL_BST_64 | LCDC_CTRL_OFUM;
	if (lcd_config_info.pinmd)
		ctrl |= LCDC_CTRL_PINMD;

	ctrl |= LCDC_CTRL_BPP_18_24;
	/* configure smart LCDC registers */
	if(info->lcd_type == LCD_TYPE_LCM) {
		smart_cfg = lcd_config_info.smart_config.smart_type |
			lcd_config_info.smart_config.cmd_width |
			lcd_config_info.smart_config.data_width;

		if (lcd_config_info.smart_config.clkply_active_rising)
			smart_cfg |= SLCDC_CFG_CLK_ACTIVE_RISING;
		if (lcd_config_info.smart_config.rsply_cmd_high)
			smart_cfg |= SLCDC_CFG_RS_CMD_HIGH;
		if (lcd_config_info.smart_config.csply_active_high)
			smart_cfg |= SLCDC_CFG_CS_ACTIVE_HIGH;
		/* SLCD DMA mode select 0 */
		smart_ctrl = SLCDC_CTRL_DMA_MODE | SLCDC_CTRL_DMA_EN;
	}
	if(info->lcd_type != LCD_TYPE_LCM) {
		reg_write( LCDC_VAT, (ht << 16) | vt);
		reg_write( LCDC_DAH, (hds << 16) | hde);
		reg_write( LCDC_DAV, (vds << 16) | vde);

		reg_write( LCDC_HSYNC, mode->hsync_len);
		reg_write( LCDC_VSYNC, mode->vsync_len);
	} else {
		reg_write( LCDC_VAT, (mode->xres << 16) | mode->yres);
		reg_write( LCDC_DAH, mode->xres);
		reg_write( LCDC_DAV, mode->yres);

		reg_write( LCDC_HSYNC, 0);
		reg_write( LCDC_VSYNC, 0);

		reg_write( SLCDC_CFG, smart_cfg);
		reg_write(SLCDC_CTRL, smart_ctrl);
	}

	reg_write( LCDC_CFG, cfg);
	reg_write( LCDC_CTRL, ctrl);

	pcfg = 0xC0000000 | (511<<18) | (400<<9) | (256<<0) ;
	reg_write( LCDC_PCFG, pcfg);

	size0 = (info->modes->xres << LCDC_SIZE_WIDTH_BIT) & LCDC_SIZE_WIDTH_MASK;
	size0 |= ((info->modes->yres << LCDC_SIZE_HEIGHT_BIT) & LCDC_SIZE_HEIGHT_MASK);
	size1 = size0;
	reg_write( LCDC_SIZE0, size0);
	reg_write( LCDC_SIZE1, size1);

	jzfb_config_fg0(info);

	jzfb_prepare_dma_desc(info);

	if (info->lcd_type == LCD_TYPE_LCM) {
		jzfb_slcd_mcu_init(info);
	}
	return 0;
}

static int jz_lcd_init_mem(void *lcdbase, struct jzfb_config_info *info)
{
	unsigned long palette_mem_size;
	int fb_size = (info->modes->xres *(jzfb_get_controller_bpp(info->bpp) / 8))* info->modes->yres;

	info->screen = (unsigned long)lcdbase;
	info->palette_size = 256;
	palette_mem_size = info->palette_size * sizeof(u16);

	/* locate palette and descs at end of page following fb */
	info->palette = (unsigned long)lcdbase + fb_size + PAGE_SIZE - palette_mem_size;
#ifdef CONFIG_VIDEO_JZ4775
	info->dma_cmd_buf = (((unsigned long)lcdbase + fb_size + PAGE_SIZE)
			     + PAGE_SIZE -1) & ~(PAGE_SIZE -1);
	if (info->lcd_type == LCD_TYPE_LCM) {
		int i;
		unsigned long cmd[2], *ptr;

		ptr = (unsigned long *)info->dma_cmd_buf;
		cmd[0] = info->smart_config.write_gram_cmd;
		switch (info->smart_config.bus_width) {
		case 18:
			cmd[0] = (cmd[0] & 0xff) << 1 | (cmd[0] & 0xff00) << 2;
			cmd[1] = cmd[0];
			break;
		case 16:
			cmd[0] &= 0xffff;
			cmd[1] = cmd[0];
			break;
		case 9:
		case 8:
			cmd[1] = (cmd[0] & 0xff00) >> 8;
			cmd[0] = (cmd[0] & 0xff) >> 0;
			break;
		default:
			serial_puts("Don't support %d bit bus");
			break;
		}
		for (i = 0; i < 4; i += 2) {
			ptr[i] = cmd[0];
			ptr[i + 1] = cmd[1];
		}
		flush_cache_all();
	}
#endif
	return 0;
}

void lcd_ctrl_init(void *lcd_base)
{
	/* init registers base address */
	lcd_config_info = jzfb1_init_data;
#if defined(CONFIG_VIDEO_JZ4775) || defined(CONFIG_FB_JZ4780_LCDC0)
	lcd_config_info.lcdbaseoff = 0;
#elif defined(CONFIG_FB_JZ4780_LCDC1)
	lcd_config_info.lcdbaseoff = LCDC1_BASE - LCDC0_BASE;
#else
	printf("error, LCDC init data is NULL\n");
	return;
#endif
	lcd_close_backlight();
	lcd_display_pin_init();

#ifdef CONFIG_LCD_FORMAT_X8B8G8R8
	lcd_config_info.fmt_order = FORMAT_X8B8G8R8;
#else
	lcd_config_info.fmt_order = FORMAT_X8R8G8B8;
#endif

	jz_lcd_init_mem(lcd_base, &lcd_config_info);

	board_lcd_init();

	jzfb_set_par(&lcd_config_info);

	lcd_display_on();
	flush_cache_all();

#ifdef DEFAULT_BACKLIGHT_LEVEL
        lcd_set_backlight_level(CONFIG_SYS_BACKLIGHT_LEVEL);
#else
        lcd_set_backlight_level(80);
        puts("80");
#endif
	return;
}

void lcd_show_board_info(void)
{
	return ;
}

void lcd_setcolreg(ushort regno, ushort red, ushort green, ushort blue)
{
	return ;
}
