/*
 * cmd_vibrator_shock.c
 *
 *  Created on: Mar 31, 2015
 *      Author: xblin
 */

#include <command.h>
#include <common.h> /* simple_strtol need */

static int vibrate_at_machine_begin(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#if defined(VIBRATOR_EN) && defined(ACTIVE_LEVEL)
	long time = 200; /* time = 200ms default */
	if(argc != 1) {
		if(argc > 2) {
			printf("your param is error\n");
			return 0;
		} else {
			time = simple_strtol(argv[1], '\0', 0);
		}
	}

	if(gpio_request(VIBRATOR_EN, "vibrator_en") >= 0)
		gpio_direction_output(VIBRATOR_EN, !(ACTIVE_LEVEL));
	else
		printf("vibrator gpio request error\n");

	mdelay(time);

	gpio_direction_output(VIBRATOR_EN, (ACTIVE_LEVEL));
#endif
	return 0;
}

U_BOOT_CMD(
	vibrate, 5, 1, vibrate_at_machine_begin,
	"vibrator shock 200ms default\n",
	"example: vibrator_shock 1000 (you shock 1s)"
);
