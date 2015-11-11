/*
 * detect battery and show charge logo
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: yqfu <yqfu@ingenic.cn>
 * Based on: xboot/boot/common/pm.c
 *           Written by Aaron <qyang@ingenic.cn>
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
//#define DEBUG
#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <linux/err.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rtc.h>
#include <asm/arch/cpm.h>
#include <asm/arch/sadc.h>
#include <lcd.h>
#include <rle_charge_logo.h>
#include <malloc.h>
#include <regulator.h>

#ifdef CONFIG_PMU_SM5007
#include <power/sm5007_api.h>
#endif

#include "../drivers/gpio/fixed_gpio.h"

extern int boot_mode_is_show_charging_logo(void);

DECLARE_GLOBAL_DATA_PTR;
#define LOGO_CHARGE_SIZE    (0xffffffff)	//need to fixed!
#define RLE_LOGO_BASE_ADDR  (0x00000000)	//need to fixed!

#ifdef CONFIG_PMU_RICOH6x
#define BATTERY_DEFAULT_MIN         (3600000)
#define BATTERY_DEFAULT_MAX         (4200000)
#define BATTERY_DEFAULT_SCALE       (100000)
#else
#define BATTERY_DEFAULT_MIN         (3600)
#define BATTERY_DEFAULT_MAX         (4200)
#define BATTERY_DEFAULT_SCALE       (100)
#endif

#define CONFIG_CAPA_MEASURE_FG

/*
extern void board_powerdown_device(void);
*/

#ifndef CONFIG_GPIO_USB_DETECT
#define CONFIG_GPIO_USB_DETECT -1
#define CONFIG_GPIO_USB_DETECT_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_DC_DETECT
#define CONFIG_GPIO_DC_DETECT -1
#define CONFIG_GPIO_DC_DETECT_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_CHARGE_DETECT
#define CONFIG_GPIO_CHARGE_DETECT -1
#define CONFIG_GPIO_CHARGE_DETECT_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_VOL_SUB
#define CONFIG_GPIO_VOL_SUB -1
#define CONFIG_GPIO_VOL_SUB_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_VOL_ADD
#define CONFIG_GPIO_VOL_ADD -1
#define CONFIG_GPIO_VOL_ADD_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_BACK
#define CONFIG_GPIO_BACK -1
#define CONFIG_GPIO_BACK_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_MENU
#define CONFIG_GPIO_MENU -1
#define CONFIG_GPIO_MENU_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_HOME
#define CONFIG_GPIO_HOME -1
#define CONFIG_GPIO_HOME_ENLEVEL 0
#endif

#ifndef CONFIG_GPIO_PWR_WAKE
#define CONFIG_GPIO_PWR_WAKE -1
#define CONFIG_GPIO_PWR_WAKE_ENLEVEL 0
#endif

enum {
	GPIO_USB = 0,
	GPIO_DC,
	GPIO_CHARGE,
	GPIO_VOL_SUB,
	GPIO_VOL_ADD,
	GPIO_BACK,
	GPIO_MENU,
	GPIO_HOME,
	GPIO_PWR_WAKE,
	GPIO_NUMS,
};

struct fixed_gpio gpio_arr[GPIO_NUMS] = {
	[GPIO_USB] = {CONFIG_GPIO_USB_DETECT, CONFIG_GPIO_USB_DETECT_ENLEVEL},
	[GPIO_DC] = {CONFIG_GPIO_DC_DETECT, CONFIG_GPIO_DC_DETECT_ENLEVEL},
	[GPIO_CHARGE] = {CONFIG_GPIO_CHARGE_DETECT, CONFIG_GPIO_CHARGE_DETECT_ENLEVEL},
	[GPIO_VOL_SUB] = {CONFIG_GPIO_VOL_SUB, CONFIG_GPIO_VOL_SUB_ENLEVEL},
	[GPIO_VOL_ADD] = {CONFIG_GPIO_VOL_ADD, CONFIG_GPIO_VOL_ADD_ENLEVEL},
	[GPIO_BACK] = {CONFIG_GPIO_BACK, CONFIG_GPIO_BACK_ENLEVEL},
	[GPIO_MENU] = {CONFIG_GPIO_MENU, CONFIG_GPIO_MENU_ENLEVEL},
	[GPIO_HOME] = {CONFIG_GPIO_HOME, CONFIG_GPIO_HOME_ENLEVEL},
	[GPIO_PWR_WAKE] = {CONFIG_GPIO_PWR_WAKE, CONFIG_GPIO_PWR_WAKE_ENLEVEL},
};

#define __poweron_key_pressed() fixed_gpio_is_active(gpio_arr[GPIO_PWR_WAKE])
#define __usb_detected()        fixed_gpio_is_active(gpio_arr[GPIO_USB])
#define __dc_detected()         fixed_gpio_is_active(gpio_arr[GPIO_DC])
#define __battery_is_charging() fixed_gpio_is_active(gpio_arr[GPIO_CHARGE])
#define __charge_detect()      (__battery_is_charging() || __dc_detected() || __usb_detected())

#define __vol_add_key_pressed() fixed_gpio_is_active(gpio_arr[GPIO_VOL_ADD])
#define __vol_sub_key_pressed() fixed_gpio_is_active(gpio_arr[GPIO_VOL_SUB])
#define __back_key_pressed()    fixed_gpio_is_active(gpio_arr[GPIO_BACK])
#define __menu_key_pressed()    fixed_gpio_is_active(gpio_arr[GPIO_MENU])
#define __home_key_pressed()    fixed_gpio_is_active(gpio_arr[GPIO_HOME])
#define keys_pressed()         (__vol_add_key_pressed() || \
                                __vol_sub_key_pressed() || \
                                __back_key_pressed() || \
                                __menu_key_pressed() || \
                                __home_key_pressed())

static long slop = 0;
static long cut = 0;
static	unsigned char  *logo_addr;

static unsigned int battery_voltage_min;
static unsigned int battery_voltage_max;
static unsigned int battery_voltage_scale;

static void inline reg_bit_set(unsigned int reg_addr, int bit)
{
	unsigned long val;
	val = readl(reg_addr);
	val |= bit;
	writel(val, reg_addr);
}

static void inline reg_bit_clr(unsigned int reg_addr, int bit)
{
	unsigned long val;
	val = readl(reg_addr);
	val &= ~bit;
	writel(val, reg_addr);
}

static void key_init_gpio(void)
{
	fixed_gpio_direction_input(gpio_arr[GPIO_VOL_SUB]);
	fixed_gpio_direction_input(gpio_arr[GPIO_VOL_ADD]);
	fixed_gpio_direction_input(gpio_arr[GPIO_BACK]);
	fixed_gpio_direction_input(gpio_arr[GPIO_MENU]);
	fixed_gpio_direction_input(gpio_arr[GPIO_HOME]);
}

int poweron_key_pressed(void)
{
    return __poweron_key_pressed();
}

#define RD_ADJ		15
#define RD_STROBE	7
#define ADDRESS		0x10
#define LENGTH		0x3
#define RD_EN		0x01
#define RD_DONE		0x1
/* JZ4780 adc adjust*/
static void sadc_power_on(void)
{
	/* Enable the SADC clock ,so we can access the SACD registers */
	reg_bit_clr(CPM_BASE + CPM_CLKGR, CPM_CLKGR_SADC);

	/* Clear the SADC_ADENA_POWER bit to turn on SADC,just once */
	reg_bit_clr(SADC_BASE + SADC_ADENA, SADC_ADENA_POWER);
}
static void sadc_power_off(void)
{
	/* set the SADC_ADENA_POWER bit to turn off SADC,just once */
	reg_bit_set(SADC_BASE + SADC_ADENA, SADC_ADENA_POWER);

	/* disable the SADC clock ,then we can't access the SACD registers */
	reg_bit_set(CPM_BASE + CPM_CLKGR, CPM_CLKGR_SADC);
}
static void get_cpu_id(void)
{
#ifdef CONFIG_ADC_SUPPORT_ADJUST
	/* jz4780 support adc adjust */
	unsigned int tmp;
	char slop_r;
	char cut_r;

	tmp = readl(EFUSE_BASE + EFUSE_EFUCFG);
	tmp &= ~(EFUSE_EFUCFG_RD_ADJ_MASK | EFUSE_EFUCFG_RD_STROBE_MASK);
	tmp |= (RD_ADJ << EFUSE_EFUCFG_RD_ADJ_BIT) |
	    (RD_STROBE << EFUSE_EFUCFG_RD_STROBE_BIT);
	writel(tmp, EFUSE_BASE + EFUSE_EFUCFG);
	tmp = readl(EFUSE_BASE + EFUSE_EFUCFG);
	if (tmp & EFUSE_EFUCFG_RD_STROBE_MASK == EFUSE_EFUCFG_RD_STROBE_MASK) {
		tmp = readl(EFUSE_BASE + EFUSE_EFUCFG);
		tmp &=
		    ~(EFUSE_EFUCFG_RD_ADJ_MASK | EFUSE_EFUCFG_RD_STROBE_MASK);
		tmp |=
		    (RD_ADJ << EFUSE_EFUCFG_RD_ADJ_BIT) | (RD_STROBE <<
							   EFUSE_EFUCFG_RD_STROBE_BIT);
		writel(tmp, EFUSE_BASE + EFUSE_EFUCFG);
	}

	tmp = readl(EFUSE_BASE + EFUSE_EFUCTRL);
	tmp &= ~(EFUSE_EFUCTRL_ADDRESS_MASK | EFUSE_EFUCTRL_LENGTH_MASK |
		 EFUSE_EFUCTRL_RD_EN);
	tmp |= ((ADDRESS << EFUSE_EFUCTRL_ADDRESS_BIT) |
		(LENGTH << EFUSE_EFUCTRL_LENGTH_BIT) |
		(RD_EN << EFUSE_EFUCTRL_RD_EN_BIT));
	writel(tmp, EFUSE_BASE + EFUSE_EFUCTRL);

	do {
		tmp = readl(EFUSE_BASE, EFUSE_EFUSTATE);
	} while (!(tmp & RD_DONE));

	tmp = readl(EFUSE_EFUDATA0);

	slop_r = (tmp >> 24) & 0xff;
	cut_r = (tmp >> 16) & 0xff;

	slop = 2 * slop_r + 2895;
	cut = 1000 * cut_r + 90000;
#endif
}

/* Basic RTC ops */
static unsigned int rtc_read_reg(unsigned int offset)
{
	unsigned int data;
	do {
		data = readl(RTC_BASE + offset);
	} while (readl(RTC_BASE + offset) != data);
	return data;
}

/* Waiting for the RTC register writing finish */
static void __wait_write_ready(void)
{
	unsigned int timeout = 10000;
	while (!(rtc_read_reg(RTC_RTCCR) & RTC_RTCCR_WRDY) && timeout++) ;
}

/* Waiting for the RTC register writable */
static void __wait_writable(void)
{
	unsigned int timeout = 1;
	__wait_write_ready();
	writel(RTC_WENR_WENPAT_WRITABLE, RTC_BASE + RTC_WENR);
	__wait_write_ready();
	while (!(rtc_read_reg(RTC_WENR) & RTC_WENR_WEN) && timeout++) ;
}

static void rtc_write_reg(unsigned int offset, unsigned int data)
{
	__wait_writable();
	writel(data, RTC_BASE + offset);
	__wait_write_ready();
}

static int ppreset_occurred(void)
{
	return (rtc_read_reg(RTC_HWRSR) & RTC_HWRSR_PPR);
}

static int jz_pm_do_hibernate(void)
{
	int a = 1000;
	printf("jz_do_hibernate...\n");

	lcd_close_backlight();
	/*
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */
	writel(0x0, CPM_BASE + CPM_RSR);

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 1000ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(1000));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	/* set wake up valid level as low */
	rtc_write_reg(RTC_HWCR, 0x8);

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, RTC_HCR_PD);

#ifdef CONFIG_PMU_D2041
	d2041_shutdown();
#endif

#ifdef CONFIG_PMU_RICOH6x
	printf("RICOH619 power down\n");
	ricoh619_power_off();
#endif

#ifdef CONFIG_PMU_SM5007
	printf("SM5007 power down\n");
	sm5007_shutdown();
#endif
	mdelay(100);
	printf("We should not come here, please check the PMU config!!!\n");

	/* We can't get here */
	return 0;
}

static void jz_pm_do_idle(void)
{
	/* set wait to sleep */
	unsigned int regval;
	regval = readl(CPM_BASE + CPM_LCR);
	regval = (regval & ~LCR_LPM_MASK) | 0;
	writel(regval, CPM_BASE + CPM_LCR);	/* set wait  to sleep */
	gpio_as_irq_low_level(CONFIG_GPIO_PWR_WAKE);
	/* unmask IRQ_GPIOn depends on GPIO_WAKEUP */
	intc_unmask_gpio_irq(CONFIG_GPIO_PWR_WAKE);

	fixed_gpio_set_active_irq(gpio_arr[GPIO_DC]);
	fixed_gpio_set_active_irq(gpio_arr[GPIO_USB]);

	printf("enter sleep mode\n");
	mdelay(50);
	__asm__ volatile (".set mips32\n\t"
			  "sync\n\t"
			  "wait\n\t"
			  "nop\n\t"
			  "nop\n\t" "nop\n\t" "nop\n\t" ".set mips32");
	printf("out  sleep mode\n");

	fixed_gpio_direction_input(gpio_arr[GPIO_USB]);
	fixed_gpio_direction_input(gpio_arr[GPIO_DC]);
}

static unsigned int read_adc_vbat(void)
{
	debug("read battery voltage by adc\n");

	unsigned int timeout = 0xfff;
	unsigned long long bat = 0;

	/* Set the ADCLK register bit[7:0],SACD work at 100Khz */
	writel(120 - 1, SADC_BASE + SADC_ADCLK);

	/* Set the ADCFG register bit[1:0],select the measure VBAT_ER/AUX2 voltage mode */
	writel(0x2, SADC_BASE + SADC_ADCFG);

	/* Set the ADENA register bit[1],Enable the VBAT control */
	reg_bit_set(SADC_BASE + SADC_ADENA, SADC_ADENA_VBATEN);

	/* Wait for Battery value ready */
	while ((!(readl(SADC_BASE + SADC_ADSTATE) & SADC_ADSTATE_VRDY))
	       && --timeout) ;
	if (!timeout)
		printf("Reading vbat timeout!\n");

	/* Read VDATA */
	bat = (readl(SADC_BASE + SADC_ADVDAT) & 0xfff);

	/* Clear the SADC_STATE_VRDY bit */
	reg_bit_clr(SADC_BASE + SADC_ADSTATE, SADC_ADSTATE_VRDY);

	/* Clear the SADC_ADVDAT_VDATA bit[11:0] */
	writel(0x0, SADC_BASE + SADC_ADVDAT);

	return bat;
}

static void bat_voltage_filter(unsigned int *voltage)
{
#ifdef CONFIG_BATTERY_CHARGE_VOLTAGE_WINDAGE
	*voltage -= CONFIG_BATTERY_CHARGE_VOLTAGE_WINDAGE;
#endif
	if (__charge_detect()) {
		if (__usb_detected() && __dc_detected()) {
			debug("charge: usb and dc...\n");
#ifdef CONFIG_BATTERY_CHARGE_DC_USB_FILTER
			*voltage -= DC_USB_FILTER;
#else
			*voltage -= 250;
#endif
			return;
		}
		if (__usb_detected()) {
			debug("charge: usb...\n");
#ifdef CONFIG_BATTERY_CHARGE_USB_FILTER
			*voltage -= CONFIG_BATTERY_CHARGE_USB_FILTER;
#else
			*voltage -= 100;
#endif
			return;
		}
		if (__dc_detected()) {
			debug("charge: dc...\n");
#ifdef CONFIG_BATTERY_CHARGE_DC_FILTER
			*voltage -= CONFIG_BATTERY_CHARGE_DC_FILTER;
#else
			*voltage -= 200;
#endif
			return;
		}
	}
	return;
}

unsigned int read_battery_voltage(void)
{
	unsigned int voltage = 0;
	int min = 0xffff, max = 0, tmp;
	int i;

	sadc_power_on();
	for (i = 0; i < 12; i++) {
		tmp = read_adc_vbat();
		if (tmp < min)
			min = tmp;
		else if (tmp > max)
			max = tmp;
		voltage += tmp;
		mdelay(10);
	}
	sadc_power_off();

	voltage -= min + max;
	voltage /= 10;

	/* not support JZ4760,JZ4760B etc. */
	get_cpu_id();
	if ((slop == 0) && (cut == 0)) {
		voltage = voltage * 1200 / 4096 * 4;
	} else {
		voltage = (voltage * slop + cut) / 10000 * 4;
	}

	/* bat_voltage_filter(&voltage); */
	debug("battery voltage is: %d mV\n", voltage);
	return voltage;
}

static void battery_init_gpio(void)
{
	fixed_gpio_direction_input(gpio_arr[GPIO_USB]);
	fixed_gpio_disable_pull(gpio_arr[GPIO_USB]);
	fixed_gpio_direction_input(gpio_arr[GPIO_DC]);
	fixed_gpio_disable_pull(gpio_arr[GPIO_DC]);
	fixed_gpio_direction_input(gpio_arr[GPIO_CHARGE]);
	fixed_gpio_disable_pull(gpio_arr[GPIO_CHARGE]);
}

static int get_viberation_signature(void)
{
    unsigned int flag = cpm_get_scrpad();

    if ((flag & 0xffff) == VIBRATION_SIGNATURE) {
        /* Clear the signature */
        cpm_set_scrpad(flag & ~(0xffff));
        return 1;
    } else {
        return 0;
    }
}

int detect_boot_state(void)
{
	if (boot_mode_is_show_charging_logo())
		return 0;

	if (readl(CPM_BASE + CPM_RSR) & CPM_RSR_WR)
		return 1;

	if (get_viberation_signature())
	    return 1;

	return 0;
}

int detect_charger_state(void)
{
	int charging = 0;
	int i;

	for (i = 0; i < 5; i++) {
		mdelay(5);
		charging += __charge_detect();
	}

	return charging;
}

static int charge_detect(void)
{
	return detect_charger_state();
}

#define NO_PRESS -1
#define SHORT_PRESS 0
#define LONG_PRESS 1
static int poweron_key_pressed_status(int charge_logo_first_show)
{
	int count = 0;

	while (1) {
		if (__poweron_key_pressed()) {
		    if (charge_logo_first_show == 0) {
		    /* if never show charge logo,boot the system immediately */
		        return LONG_PRESS;
		    } else
		        mdelay(10);

			count++;
		} else {
			if (count == 0)
				return NO_PRESS;
			else {
				debug("count = %d\n", count);
				return SHORT_PRESS;
			}
		}
		if (count >= 100) {
			return LONG_PRESS;
		}
	}
	return NO_PRESS;
}

static int battery_is_low(void)
{
#if defined(CONFIG_PMU_RICOH6x)
	int capa = 0, vsys = 0, first = 0;

	vsys = cmd_measure_vbat_ADC();
	printf ("vsys: %u\n",vsys);
	if (charge_detect()) {

#ifdef LOW_BATTERY_MIN
		if (vsys <= LOW_BATTERY_MIN + 100000)
			return 1;
#else
		if (vsys <= battery_voltage_min + 100000)
			return 1;
#endif
		else
			return 0;

	}
	else {

#ifdef LOW_BATTERY_MIN
		if (vsys <= LOW_BATTERY_MIN)
				return 1;
#else
		if (vsys <= battery_voltage_min)
			return 1;
#endif
		else
			return 0;

	}

#elif defined(CONFIG_PMU_SM5007)
	int capa = 0, vbat = 0;
	capa = fg_get_soc();
	vbat = fg_get_vbat();
	if(capa < 10) {
		return 1;
	}else {
		return 0;
	}
#else
	unsigned int voltage = 0;
	voltage = read_battery_voltage();

#ifdef LOW_BATTERY_MIN
	if (voltage <= LOW_BATTERY_MIN)
		return 1;
#else
	if (voltage <= battery_voltage_min)
		return 1;
#endif
	else
		return 0;
#endif 

}

static void * malloc_charge_logo(int buf_size)
{
	void *addr;
	int logo_charge_num = (battery_voltage_max  - battery_voltage_min) / battery_voltage_scale;
	addr=malloc(buf_size * logo_charge_num);
	memset(addr, 0x00, buf_size * logo_charge_num);
	return addr;
}

static void free_charge_logo(void *addr)
{
	free(addr);
}

static int show_charge_logo_rle(int rle_num)
{
	int logo_charge_num = (battery_voltage_max  - battery_voltage_min) / battery_voltage_scale;
	if (rle_num < 0 && rle_num > logo_charge_num)
		return -EINVAL;
	show_rle_picture_in_fb_middle(rle_charge_logo_addr + rle_num);
	return 0;
}

static int voltage_to_rle_num(void)
{
	int voltage, capa;
	unsigned int voltage_adc;
	int vsys;
	int rle_num_base;
	int first;

#ifdef CONFIG_PMU_RICOH6x
	first = detection_first_poweron();
	if(first){
#ifdef CONFIG_VOL_MEASURE_FG
		voltage = cmd_measure_vbatt_FG();
		if (voltage < battery_voltage_min) {
			rle_num_base = 0;
		} else if (voltage < battery_voltage_max) {
			rle_num_base = (voltage - battery_voltage_min) / battery_voltage_scale - 1;
		} else {
			rle_num_base = 5;
		}
		return rle_num_base;
#endif

#ifdef CONFIG_CAPA_MEASURE_FG
		capa = ricoh61x_get_capacity1();
		if(capa <= 10){
			rle_num_base = 0;
		}else if((capa > 10) && (capa <= 20)){
			rle_num_base = 1;
		}else if((capa > 20) && (capa <= 40)){
			rle_num_base = 2;
		}else if((capa > 40) && (capa <= 60)){
			rle_num_base = 3;
		}else if((capa > 60) && (capa <= 80)){
			rle_num_base = 4;
		}else if((capa > 80) && (capa <= 100)){
			rle_num_base = 5;
		}
		return rle_num_base;
#endif

	}else{
		vsys = cmd_measure_vsys_ADC();
		if (vsys < battery_voltage_min) {
			rle_num_base = 0;
		} else if (vsys < battery_voltage_max) {
			rle_num_base = (vsys - battery_voltage_min) / battery_voltage_scale - 1;
		} else {
			rle_num_base = 5;
		}   
		return rle_num_base;
	}
#else
	voltage_adc = read_battery_voltage();
	if (voltage_adc < battery_voltage_min) {
		rle_num_base = 0;
	} else if (voltage_adc < battery_voltage_max) {
		rle_num_base = (voltage_adc - battery_voltage_min) / battery_voltage_scale - 1;
	} else {
		rle_num_base = 5;
	}
	return rle_num_base;
#endif
}

static inline void wait_lcd_refresh_finish(void)
{
	mdelay(100);
}
/*
 * show charging flash every second(flash means animation)
 * go to idle every FLASH_IDLE_FREQUENCY seconds while in charging state
 * there are three states: idle, charging and hibernate
 * if it is not idle state, we think it as charging state
 */
static void show_charging_logo_normal(void)
{
/* Show time for the charge flash */
#define FLASH_IDLE_FREQUENCY 12


	int kpressed = 0;
	int rle_num = 0;
	int rle_num_base = 0;
	unsigned long start_time;
	unsigned int charge_logo_num = (battery_voltage_max - battery_voltage_min) / battery_voltage_scale;
	int is_out_from_idle = 0;
	int go_to_idle = 0;
	ulong sec_current = 0;
	ulong sec_last = 0;
	ulong charge_logo_cnt = 0;
	int charge_logo_first_show = 0;
	int poweron_key;
	int bat_is_low = 0;

	/* Shut some modules power down,cdma,gsm e.g. */
	/* board_powerdown_device(); */
	key_init_gpio();
	rle_num_base = voltage_to_rle_num();
	rle_num = rle_num_base;
	start_time = get_timer(0);

	while (1) {
		if (kpressed == 0) {
			kpressed = keys_pressed();
			if (kpressed) {
				debug("---key pressed!\n");
			}
		}
		poweron_key = poweron_key_pressed_status(charge_logo_first_show);
		bat_is_low = battery_is_low();
		if (poweron_key == LONG_PRESS && !bat_is_low) {
			wait_lcd_refresh_finish();
			lcd_clear_black();
			debug("poweron long pressed \n");
			return;
		}
		else if (poweron_key == SHORT_PRESS || (poweron_key == LONG_PRESS && bat_is_low)) {
			debug("poweron short pressed \n");
			/* press power on key while in charging flash then go to idle */
			if (!is_out_from_idle) {
				go_to_idle = 1;
			}
		}
		// During the charge process ,User extract the USB cable ,Enter hibernate mode
		if (!(__usb_detected() || __dc_detected())) {
			printf("Charge is stop, do not Power on System, so shutdown!\n");
//			show_charge_logo_rle(rle_num_base);
//			wait_lcd_refresh_finish();
			wait_lcd_refresh_finish();
			lcd_clear_black();
			wait_lcd_refresh_finish();
			jz_pm_do_hibernate();
		}

		start_time = get_timer(0);
		sec_current = start_time / 1000;

		/* current second is not the same as last, it's a new second */
		if ((sec_current != sec_last)) {
			debug("sec_current = %d sec_last = %d rle_num = %d\n",
					sec_current, sec_last, rle_num);
			charge_logo_cnt++;
			charge_logo_first_show = 1;
			show_charge_logo_rle(rle_num);
			rle_num++;
			if (rle_num >= charge_logo_num)
				rle_num = rle_num_base;

			/* go to idle every FLASH_IDLE_FREQUENCY seconds */
			if (charge_logo_cnt == FLASH_IDLE_FREQUENCY) {
				go_to_idle = 1;
			}
			sec_last = sec_current;
		}

		is_out_from_idle = 0;
		if (go_to_idle) {
			wait_lcd_refresh_finish();
			lcd_clear_black();
			lcd_close_backlight();
			jz_pm_do_idle();
			lcd_open_backlight();
			rle_num_base = voltage_to_rle_num();
			rle_num = rle_num_base;

			go_to_idle = 0;
			charge_logo_cnt = 0;
			is_out_from_idle = 1;
		}
		mdelay(10);
	}
}

#ifndef CONFIG_CHARGE_LOGO_TICK_TIME_MS
#define CONFIG_CHARGE_LOGO_TICK_TIME_MS 800
#endif

static inline unsigned long get_charging_tick(void) {
	return get_timer(0) / CONFIG_CHARGE_LOGO_TICK_TIME_MS;
}

static void show_charging_logo_tencent_os(void)
{
	unsigned long last_time = 0xffff0000;
	unsigned long current_time;
	unsigned int lcd_flush_count = 0;
	void *lcd_base = (void *)gd->fb_base;
	unsigned short *src_picture_addr;
	while (1) {
		if (!battery_is_low()) {
			printf ("show_charging_logo: battery is not low now, boot\n");
			break;
		}

		if (!charge_detect()) {
			printf ("show_charging_logo: battery is low now, shutdown\n");
			wait_lcd_refresh_finish();
			lcd_clear_black();
			wait_lcd_refresh_finish();
			jz_pm_do_hibernate();
		}

		current_time = get_charging_tick();
		if (current_time != last_time) {
			show_rle_picture_in_fb_middle(rle_charge_logo_addr + lcd_flush_count % 2);
			lcd_flush_count++;
			last_time = current_time;
		}
	}

	wait_lcd_refresh_finish();
	lcd_clear_black();
}

static void show_battery_low_logo(void)
{
	lcd_clear_black();
	show_charge_logo_rle(0);
	mdelay(5000);
	lcd_close_backlight();
}

/* 腾讯的开机流程 */
static void battery_detect_tencent_os(void) {
	if (detect_boot_state() && !battery_is_low()) {
		printf ("is in reboot process\n");
		return;
	}

	if (charge_detect()) {
		show_charging_logo_tencent_os();
	} else if (battery_is_low()) {
		printf("The battery voltage is too low. Please charge\n");
		printf("Battery low level,Into hibernate mode ... \n");
		jz_pm_do_hibernate();
	} else if (boot_mode_is_show_charging_logo()) {
		printf("Not show charging logo because not charing\n");
		jz_pm_do_hibernate();
	}
}

/* 一般的开机流程 */
static void battery_detect_normal(void) {
	if (detect_boot_state() && !battery_is_low()) {
		printf ("is in reboot process\n");
		return;
	}

	if (charge_detect()) {
		show_charging_logo_normal();
	} else if (battery_is_low()) {
		show_battery_low_logo();
		printf("The battery voltage is too low. Please charge\n");
		printf("Battery low level,Into hibernate mode ... \n");
		jz_pm_do_hibernate();
	} else if (boot_mode_is_show_charging_logo()) {
		printf("Not show charging logo because not charing\n");
		jz_pm_do_hibernate();
	}
}

static void battery_detect(void)
{
#if defined(CONFIG_BOOT_PROGRESS_NORMAL)
	battery_detect_normal();
#else
	battery_detect_tencent_os();
#endif
}
static int  voltage_argument_init(int argc, char *const argv[])
{
	if(argc == 1) {
#ifdef CONFIG_BATTERY_VOLTAGE_MIN
		battery_voltage_min = CONFIG_BATTERY_VOLTAGE_MIN;
#else
		battery_voltage_min = BATTERY_DEFAULT_MIN;
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_MAX
		battery_voltage_max = CONFIG_BATTERY_VOLTAGE_MAX;
#else
		battery_voltage_max = BATTERY_DEFAULT_MAX;
#endif
#ifdef CONFIG_BATTERY_VOLTAGE_SCALE
		battery_voltage_scale = CONFIG_BATTERY_VOLTAGE_SCALE;
#else
		battery_voltage_scale = BATTERY_DEFAULT_SCALE;
#endif
	}else if(argc == 4) {
		battery_voltage_min = simple_strtoul(argv[1], NULL, 10);
		battery_voltage_max = simple_strtoul(argv[2], NULL, 10);
		battery_voltage_scale = simple_strtoul(argv[3], NULL, 10);
		printf("battery_voltage_min = %d\nbattery_voltage_max = %d\nbattery_voltage_scale = %d\ncharge_logo_num = %d\n", battery_voltage_min, battery_voltage_max, battery_voltage_scale, (battery_voltage_max - battery_voltage_min) / battery_voltage_scale );
	}else{
		return -1;
	}
	return 0;
}

extern unsigned char rle_default_logo_addr [ 59824 ];
static int do_battery_detect(cmd_tbl_t * cmdtp, int flag, int argc,
			     char *const argv[])
{
	int ret = 0;

	ret = voltage_argument_init(argc, argv);
	if(ret != 0){
		return ret;
	}

#ifndef CONFIG_BATTERY_INIT_GPIO
	/* IF default battery_init_gpio function is not suitable for actual board,
	 * define and call the init function in board setup file. For example ,
	 * init gpio in borad/ingenic/mensa/mensa.c */
	battery_init_gpio();
#endif

#ifdef CONFIG_PMU_RICOH6x
	ricoh619_limit_current_init();
#endif

#ifdef CONFIG_PMU_SM5007
		sm5007_enable_chgen(1);
#endif

	battery_detect();
	return ret;
}

U_BOOT_CMD(batterydet, 4, 1, do_battery_detect,
	   "battery and show charge logo", "[<battery_min> <battery_max>, <battery_scale>]");

static int do_idle(cmd_tbl_t * cmdtp, int flag, int argc,
			     char *const argv[])
{
	int ret = 0;

	jz_pm_do_idle();

	return ret;
}

U_BOOT_CMD(idle, 4, 1, do_idle,
		   "idle: cpu will going to idle mode",
	   "idle: cpu will going to idle mode");


static int do_shutdown(cmd_tbl_t * cmdtp, int flag, int argc,
			     char *const argv[])
{
	int ret = 0;

	jz_pm_do_hibernate();

	return ret;
}

U_BOOT_CMD(shutdown, 4, 1, do_shutdown,
		   "idle: cpu will going to shutdown mode",
	   "idle: cpu will going to shutdown mode");


static int do_regulator(cmd_tbl_t * cmdtp, int flag, int argc,
			     char *const argv[])
{
	int ret = 0;
	int cmd = 0;
	struct regulator *reg;

	if (argc != 3) {
		printf ("invalid args!\n");
		return 0;
	}

	if (!strcmp(argv[1], "enable")) {
		cmd = 1;
	} else if (!strcmp(argv[1], "disable")) {
		cmd = 0;
	} else {
		printf ("invalid cmd: %s!\n", argv[1]);
		return 0;
	}

	reg = regulator_get(argv[2]);
	if (reg == NULL) {
		printf ("invalid regulator: %s!\n", argv[2]);
		return 0;
	}

	switch(cmd) {
	case 0:
		regulator_disable(reg);
		break;
	case 1:
		regulator_enable(reg);
		break;
	default:
		break;
	}

	return ret;
}

U_BOOT_CMD(regulator, 4, 1, do_regulator,
		   "idle: cpu will going to idle mode",
	   "idle: cpu will going to idle mode");
