/*
 * Ingenic grus boot mode select
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Martin <czhu@ingenic.cn>
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
#include <config.h>
#include <boot_img.h>
#include <asm/arch/cpm.h>
#include <asm/gpio.h>
#include <asm/io.h>

#define FAST_BOOT                       0
#define NORMAL_BOOT                     1
#define RECOVERY_BOOT                   2
#define FASTBOOT_RECOVERY_BOOT          3

/*
 *KEY_UNPRESS:  key is not pressed
 *KEY_PRESS:	key is pressed
*/
enum {
	KEY_UNPRESS = 0,
	KEY_PRESS,
};
/*
 *Get the status of gpio.
 *Ret: 0 or 1
 */
static int get_key_status(unsigned pin)
{
#define GPIO_DEBOUNCE  20
	int cnt = GPIO_DEBOUNCE,v = 0, t = 0;

	while (cnt--) {
		t = !!gpio_get_value(pin);
		if (v != t) {
			cnt = GPIO_DEBOUNCE;
			mdelay(1);
		}
		v = t;
	}
	return v;
}

/*
 * Get USB boot keys states.
 */
#if defined(CONFIG_FAST_BOOT_SUPPORT)
static int is_fast_boot_keys_pressed(void)
{
	gpio_direction_input(CONFIG_RECOVERY_KEY);
	gpio_direction_input(CONFIG_GPIO_USB_DETECT);
	gpio_disable_pull(CONFIG_RECOVERY_KEY);
	gpio_disable_pull(CONFIG_GPIO_USB_DETECT);

	if ((CONFIG_RECOVERY_ENLEVEL == get_key_status(CONFIG_RECOVERY_KEY))
	    && (CONFIG_USB_DETECT_ENLEVEL == get_key_status(CONFIG_GPIO_USB_DETECT)))
		return KEY_PRESS;
	else
		return KEY_UNPRESS;
}
#endif

/*
 *Get the status of recovery boot key.
 */
static int is_recovery_keys_pressed(void)
{
	gpio_direction_input(CONFIG_RECOVERY_KEY);
	gpio_disable_pull(CONFIG_RECOVERY_KEY);

	if (CONFIG_RECOVERY_ENLEVEL == get_key_status(CONFIG_RECOVERY_KEY))
		return KEY_PRESS;
	else
		return KEY_UNPRESS;
}

/*
 * Get recovery signature and reset it.
 */
static int get_recovery_signature(void)
{
	unsigned int flag = cpm_get_scrpad();

	if ((flag & 0xffff) == RECOVERY_SIGNATURE) {
		/*
	 	* Clear the signature,
	 	* reset the signature to force into normal boot after factory reset
	 	*/
		cpm_set_scrpad(flag & ~(0xffff));
		return KEY_PRESS;
	} else {
		return KEY_UNPRESS;
	}
}

/*
 *Get the status of usb.
 */
static int is_usb_detected(void)
{
	gpio_direction_input(CONFIG_GPIO_USB_DETECT);
	gpio_disable_pull(CONFIG_GPIO_USB_DETECT);

	if (CONFIG_USB_DETECT_ENLEVEL == get_key_status(CONFIG_GPIO_USB_DETECT))
		return KEY_PRESS;
	else
		return KEY_UNPRESS;
}

/*
 * Get boot keys.
 * ret: 0: USB boot  1: normal boot  2: recovery boot
 */
static int get_boot_keys(void)
{
	/* Fast boot keys */
#if defined(CONFIG_FAST_BOOT_SUPPORT)
	if (is_fast_boot_keys_pressed()){
		return FASTBOOT_RECOVERY_BOOT;
	}
#endif
	/* Recovery signature */
	if (get_recovery_signature()) {
		return RECOVERY_BOOT;
	}

	/* Recovery boot keys */
	if (is_recovery_keys_pressed()) {
		return RECOVERY_BOOT;
	}
	return NORMAL_BOOT;
}

/* Select boot mode */
void boot_mode_select(void)
{
	int boot_select, rc, count = 0, flag = 0;
#ifdef CONFIG_MSC_BURN
	printf("Mod:   Normal boot mode.\n");
	setenv("bootcmd", CONFIG_NORMAL_BOOT);
	return;
#endif
	/* First, handle boot keys. */
	boot_select = get_boot_keys();
	switch (boot_select) {
#if defined(CONFIG_FAST_BOOT_SUPPORT)
	case FASTBOOT_RECOVERY_BOOT:
		while (!is_usb_detected()) {
		mdelay(500);
			if (count++ == 10) {
				printf("Mod:   Recovery boot mode.\n");
				setenv("bootcmd", CONFIG_RECOVERY_BOOT);
				return;
			}
		}
		printf("Mod:   Fast boot mode.\n");
		rc = run_command("fastboot", flag);
		if (rc < 0) {
			printf("fastboot:command run error!");
		}
		return;
#endif
	case RECOVERY_BOOT:
		printf("Mod:   Recovery boot mode.\n");
		setenv("bootcmd", CONFIG_RECOVERY_BOOT);
		return;
	default:
		printf("Mod:   Normal boot mode.\n");
		setenv("bootcmd", CONFIG_NORMAL_BOOT);
	};
}
