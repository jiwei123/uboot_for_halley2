/*
 * Ingenic watch setup code
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
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

#include <common.h>
#include <nand.h>
#include <net.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/cpm.h>
#include <asm/arch/nand.h>
#include <asm/arch/mmc.h>
#include <asm/arch/clk.h>

struct cgu_clk_src cgu_clk_src[] = {
	{VPU, MPLL},
	{OTG, EXCLK},
	{LCD, MPLL},
	{MSC, MPLL},
	{SSI, MPLL},
	{CIM, MPLL},
	{PCM, MPLL},
	{GPU, MPLL},
	{ISP, MPLL},
	{BCH, MPLL},
	{I2S, MPLL},
	{SRC_EOF,SRC_EOF}
};

#ifdef CONFIG_BOOT_ANDROID
extern void boot_mode_select(void);
#endif

#ifdef CONFIG_PMU_RICOH6x
extern int ricoh61x_regulator_init(void);
#endif

#ifdef CONFIG_MMC
extern int mmc_backup_save(void);
extern int mmc_backup_restore(void);
#endif

void image_backup_restore(void)
{
#ifdef CONFIG_MMC
     //mmc_backup_save();
     //mmc_backup_restore();
#endif
}
int board_early_init_f(void)
{
	return 0;
}

#ifdef CONFIG_REGULATOR
int regulator_init(void)
{
	int ret;

#ifdef CONFIG_PMU_RICOH6x
	ret = ricoh61x_regulator_init();
#endif
	return ret;
}
#endif /* CONFIG_REGULATOR */

#ifdef CONFIG_GPIO_EARLY_INIT
int gpio_early_init(void)
{
    int ret = 0;

#if defined(CONFIG_SENSORS_PIXART_PAH8001)
    ret = gpio_request(GPIO_PAH8001_RESET, "pah8001_reset");
    if (ret < 0) {
        printf("%s : Request GPIO %d failed----------\n", __FUNCTION__,
                GPIO_PAH8001_RESET);
        return -1;
    } else {
        gpio_direction_output(GPIO_PAH8001_RESET, 1);
    }

    ret = gpio_request(GPIO_PAH8001_INT, "pah8001_int");
    if (ret < 0) {
        printf("%s : Request GPIO %d failed----------\n", __FUNCTION__,
                GPIO_PAH8001_INT);
        gpio_free(GPIO_PAH8001_RESET);
        return -1;
    } else {
        gpio_direction_output(GPIO_PAH8001_INT, 1);
    }
#endif

#ifdef CONFIG_GPIO_PRE_TEST
    ret = gpio_request(CONFIG_GPIO_PRE_TEST, "pre_test");
    if (ret < 0) {
        printf("gpio request pre_test failed\n");
    } else
        gpio_direction_input(CONFIG_GPIO_PRE_TEST);
#endif
    return 0;
}
#endif

int board_early_init_r(void)
{
#ifdef CONFIG_GPIO_EARLY_INIT
	gpio_early_init();
#endif

#ifdef CONFIG_REGULATOR
	regulator_init();
#endif
	return 0;
}

#ifdef CONFIG_USB_GADGET
int jz_udc_probe(void);
void board_usb_init(void)
{
	printf("USB_udc_probe\n");
	jz_udc_probe();
}
#endif /* CONFIG_USB_GADGET */

int misc_init_r(void)
{
#if 0 /* TO DO */
	uint8_t mac[6] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc };

	/* set MAC address */
	eth_setenv_enetaddr("ethaddr", mac);
#endif
#ifdef CONFIG_BOOT_ANDROID
	boot_mode_select();
#endif

#if defined(CONFIG_CMD_BATTERYDET) && defined(CONFIG_BATTERY_INIT_GPIO)
	battery_init_gpio();
#endif
	image_backup_restore();
	return 0;
}

int board_nand_init(struct nand_chip *nand)
{
	return 0;
}


#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bd)
{
	jz_mmc_init();
	return 0;
}
#endif

int board_eth_init(bd_t *bis)
{
	return 0;
}

#ifdef CONFIG_SPL_SPI_SUPPORT
void spl_spi_load_image(void)
{

}
#endif

#ifdef CONFIG_SPL_NAND_SUPPORT
void nand_init(void)
{
}

int nand_spl_load_image(uint32_t offs, unsigned int size, void *dst)
{
	return 0;
}

void nand_deselect(void)
{
}
#endif

#ifdef CONFIG_SPL_NOR_SUPPORT
int spl_start_uboot(void)
{
	return 1;
}
#endif
/* U-Boot common routines */
int checkboard(void)
{
	puts("Board: watch (Ingenic XBurst M200 SoC)\n");

	if (poweron_key_pressed()) {
	    cpm_set_scrpad(VIBRATION_SIGNATURE);
	}

	return 0;
}

#ifdef CONFIG_SPL_BUILD

void spl_board_init(void)
{
}

#endif /* CONFIG_SPL_BUILD */
