/*
 * Ingenic mensa boot mode select
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
#include <boot_msg.h>

#define FAST_BOOT                       0
#define NORMAL_BOOT                     1
#define RECOVERY_BOOT                   2
#define FASTBOOT_RECOVERY_BOOT          3
#define REBOOT_BOOT                     4
#define FTEST_BOOT                      5

#define UPDATE_UBOOT                    0
#define UPDATE_RADIO                    1

struct bootloader_message g_boot_msg;
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
static int get_key_level(unsigned pin)
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

static int get_key_status(int pin, int en_level)
{
       if(pin < 0) {
                printf("There is no GPIO configured for RECOVERY/FASTBOOT/USB_DETECT.\n");
                return 0;
	 }

	gpio_direction_input(pin);
	gpio_disable_pull(pin);

	return en_level == get_key_level(pin) ? KEY_PRESS : KEY_UNPRESS;
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
 * Get boot keys.
 * ret: 0: USB boot  1: normal boot  2: recovery boot
 */
static int get_boot_sel(void)
{
	/* Fast boot keys */
#if defined(CONFIG_CMD_FASTBOOT)
        if (get_key_status(CONFIG_GPIO_FASTBOOT, CONFIG_GPIO_FASTBOOT_ENLEVEL)
#ifdef CONFIG_GPIO_USB_DETECT
                        && get_key_status(CONFIG_GPIO_USB_DETECT, CONFIG_GPIO_USB_DETECT_ENLEVEL)
#endif
           ) {
                return FASTBOOT_RECOVERY_BOOT;
        }
#endif
        /* Recovery signature */
        if (get_recovery_signature()) {
                return RECOVERY_BOOT;
        }

        /* Recovery boot keys */
        if (get_key_status(CONFIG_GPIO_RECOVERY, CONFIG_GPIO_RECOVERY_ENLEVEL)) {
                return RECOVERY_BOOT;
        }
        return NORMAL_BOOT;
}

/*
 * Handle the command that speified in bootloader_message and return boot select.
 * Command:     "update firmware";
 * "boot into recovery";
 * any other commands we have already defined.
 * Ret: 0       Fast boot
 *      1       Normal boot
 *      2       Recovery boot
 */
static int handle_bootloader_command(void)
{
        /* Command: boot-recovery-ftest */
        if ( !memcmp(g_boot_msg.command, "boot-recovery-ftest", strlen("boot-recovery-ftest")) ) {
                printf("In handle_bootloader_command ... boot-recovery-ftest...\n");
                return FTEST_BOOT;
        }

        /* Command: boot-recovery */
        if ( !memcmp(g_boot_msg.command, "boot-recovery", strlen("boot-recovery")) ) {
                printf("In handle_bootloader_command ... boot-recovery ...\n");
                return RECOVERY_BOOT;
        }

        /* Command: update-radio */
        if ( !memcmp(g_boot_msg.command, "update-radio", strlen("update-radio")) ) {
                printf("In handle_bootloader_command ... update-radio ...\n");
                //update_firmware(UPDATE_RADIO);
                return RECOVERY_BOOT;
        }

        /* Command: update-xboot */
        if ( !memcmp(g_boot_msg.command, "update-xboot", strlen("update-xboot")) ) {
                printf("In handle_bootloader_command ... update-xboot ...\n");
                //update_firmware(UPDATE_UBOOT);
                return RECOVERY_BOOT;
        }
        if (g_boot_msg.command[0] != '\0') {
                printf("WARNING: bootloader_message -> command [0] is not '\\0' !\n");
        }

        printf("In handle_bootloader_command ... default ...\n");

        //dump_ram(&g_boot_msg, sizeof(struct bootloader_message));

        return NORMAL_BOOT;
}


int get_bootloader_message(struct bootloader_message *out)
{
#if defined(CONFIG_NAND_X_BOOT)
        unsigned char data[CFG_NAND_MAX_PAGE_SIZE];

        memset(data, '\0', CFG_NAND_MAX_PAGE_SIZE);
        do_nand (PTN_MISC_OFFSET, nandparam->nandinfo.pagesize, data);
#elif defined(CONFIG_MSC_U_BOOT)
        unsigned char data[sizeof(struct bootloader_message)];
        memset(data, '\0', sizeof(struct bootloader_message));
        do_msc (PTN_MISC_OFFSET, data, sizeof(struct bootloader_message));
#endif

        memcpy(out, data, sizeof(struct bootloader_message));
        return 0;
}

int set_bootloader_message(const struct bootloader_message *in)
{
#if defined(CONFIG_NAND_X_BOOT)
        unsigned char data[CFG_NAND_MAX_PAGE_SIZE];

        memset(data, '\0', CFG_NAND_MAX_PAGE_SIZE);
        memcpy(data, in, sizeof(struct bootloader_message));

        /* Clear MISC partition, and write bootloader_message. */
        if (nand_erase_block(64) || nand_erase_block(64 + 1)) {
                printf("NAND erase failed!!!\n");
                return 1;
        }

        if (nand_program_page(data, PTN_MISC_OFFSET / nandparam->nandinfo.pagesize)) {
                printf("NAND program failed !!!\n");
                return 1;
        }

        printf("set_bootloader_message finish ...\n");
#elif defined(CONFIG_MSC_X_BOOT)
        printf("set_bootloader_message not support ! ...\n");
#endif

        return 0;
}

/* Select boot mode */
void boot_mode_select(void)
{
	int boot_select, rc;
        int boot_cmd = NORMAL_BOOT;

        /* Second, handle boot message (MISC partition). */
        memset(&g_boot_msg, '\0', sizeof(struct bootloader_message));
        if ( get_bootloader_message(&g_boot_msg) ) {
                printf("Got bootloader message failed !\n");
        } else {
                boot_cmd = handle_bootloader_command();
                if (boot_cmd == FTEST_BOOT) {
                        printf("Bootloader message: boot-recovery-ftest\n");
                        boot_select = RECOVERY_BOOT;
                        setenv("bootcmd", CONFIG_RECOVERY_BOOT);
                        return;
                }
        }

	/* First, handle boot keys. */
	boot_select = get_boot_sel();
	switch (boot_select) {
#if defined(CONFIG_CMD_FASTBOOT)
	case FASTBOOT_RECOVERY_BOOT:
		printf("Mod:   Fast boot mode.\n");
		rc = run_command("fastboot", 0);
		if (rc < 0) {
			printf("fastboot:command run error!");
		}
                break;
#endif
	case RECOVERY_BOOT:
		printf("Mod:   Recovery boot mode.\n");
		setenv("bootcmd", CONFIG_RECOVERY_BOOT);
		break;
	case NORMAL_BOOT:
	default:
		printf("Mod:   Normal boot mode.\n");
		setenv("bootcmd", CONFIG_NORMAL_BOOT);
                break;
	};

        if(boot_cmd == RECOVERY_BOOT){
                printf("Bootloader message: boot-recovery\n");
                setenv("bootcmd", CONFIG_RECOVERY_BOOT);
                boot_select = RECOVERY_BOOT;
                return;
        }
}
