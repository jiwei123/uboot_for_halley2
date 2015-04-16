/*
 * cmd_vibrator_shock.c
 *
 *  Created on: Mar 31, 2015
 *      Author: xblin
 */

#include <command.h>
#include <common.h> /* simple_strtol need */
#include <i2c.h>

#if defined(CONFIG_VIBRATE_DRV2605)
static int drv2605_select_mode(struct client_i2c_bus *bus)
{
    int err = 0;
    uchar chip = DRV2605_I2C_ADDR;
    uchar buf = 0;

    /* select Internal Trigger mode */
    buf = 0x0;
    err = mutiple_i2c_write(bus, chip, 0x1, 1, &buf, 1);
    if (err != 0) {
        printf("%s, fail to write (%d)\n", __FUNCTION__, err);
        return -1;
    }

    buf = 0x0a;
    err = mutiple_i2c_write(bus, chip, 0x4, 1, &buf, 1);
    if (err != 0) {
        printf("%s, fail to write (%d)\n", __FUNCTION__, err);
        return -1;
    }
    buf = 0x84;
    err = mutiple_i2c_write(bus, chip, 0x5, 1, &buf, 1);
    if (err != 0) {
        printf("%s, fail to write (%d)\n", __FUNCTION__, err);
        return -1;
    }
    buf = 0x0;
    err = mutiple_i2c_write(bus, chip, 0x6, 1, &buf, 1);
    if (err != 0) {
        printf("%s, fail to write (%d)\n", __FUNCTION__, err);
        return -1;
    }

    return 0;
}

static int drv2605_set_go_bit(struct client_i2c_bus *bus)
{
    int err = 0;
    uchar chip = DRV2605_I2C_ADDR;
    uchar buf = 0x01;

    //set GO bit
    err = mutiple_i2c_write(bus, chip, 0xc, 1, &buf, 1);
    if (err != 0) {
        printf("%s, fail to write (%d)\n", __FUNCTION__, err);
        return -1;
    }

    return 0;
}

static int do_drv2605_vibrate()
{
    struct client_i2c_bus *drv2605_bus = NULL;

    drv2605_bus = mutiple_i2c_probe(DRV2605_I2C_ADDR);
    if (drv2605_bus == NULL) {
        printf("%s : Can not get i2c_bus\n", __FUNCTION__);
        return -1;
    }

    /* Enable power to the chip */
    if (gpio_request(DRV2605_ENABLE, "vibrator_en") < 0) {
        printf("%s : Request GPIO %d failed\n", __FUNCTION__, DRV2605_ENABLE);
        return -1;
    } else {
        gpio_direction_output(DRV2605_ENABLE, DRV2605_ACTIVE_LEVEL);
    }
    /* Wait 30 us */
    udelay(30);

    drv2605_select_mode(drv2605_bus);
    drv2605_set_go_bit(drv2605_bus);

    return 0;
}
#endif

static int vibrate_at_machine_begin(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#if defined(CONFIG_VIBRATE_GPIO)
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

#if defined(CONFIG_VIBRATE_DRV2605)
	do_drv2605_vibrate();
#endif
	return 0;
}

U_BOOT_CMD(
	vibrate, 5, 1, vibrate_at_machine_begin,
	"vibrator shock 200ms default\n",
	"example: vibrator_shock 1000 (you shock 1s)"
);
