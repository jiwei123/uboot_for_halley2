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

DECLARE_GLOBAL_DATA_PTR;
#define LOGO_CHARGE_SIZE    (0xffffffff)	//need to fixed!
#define RLE_LOGO_BASE_ADDR  (0x00000000)	//need to fixed!

#define BATTERY_DEFAULT_MIN         (3600)
#define BATTERY_DEFAULT_MAX         (4200)
#define BATTERY_DEFAULT_SCALE       (120)
/*
extern void board_powerdown_device(void);
*/

#define __is_gpio_en(GPIO)	\
	(GPIO##_ENLEVEL ? gpio_get_value(GPIO) : !gpio_get_value(GPIO))

#define __poweron_key_pressed()	__is_gpio_en(CONFIG_GPIO_PWR_WAKE)


#ifdef CONFIG_PMU_RICOH6x

#define __usb_detected()	ricoh61x_reg_charge_status(0xbd, 7)
#define __dc_detected()		ricoh61x_reg_charge_status(0xbd, 6)
#define __charge_detect()	( __dc_detected() || __usb_detected())

#else
#if defined(CONFIG_GPIO_USB_DETECT) && defined(CONFIG_GPIO_USB_DETECT_ENLEVEL)
#define __usb_detected()	__is_gpio_en(CONFIG_GPIO_USB_DETECT)
#else
#define __usb_detected()	0
#endif

#if defined(CONFIG_GPIO_DC_DETECT) && defined(CONFIG_GPIO_DC_DETECT_ENLEVEL)
#define __dc_detected()		__is_gpio_en(CONFIG_GPIO_DC_DETECT)
#else
#define __dc_detected()		0
#endif

#if defined(CONFIG_GPIO_CHARGE_DETECT) && defined(CONFIG_GPIO_CHARGE_DETECT_ENLEVEL)
#define __battery_is_charging()		__is_gpio_en(CONFIG_GPIO_DC_DETECT)
#else
#define __battery_is_charging()		0
#endif

#define __charge_detect()	(__battery_is_charging() || __dc_detected() || __usb_detected())

#endif


static long slop = 0;
static long cut = 0;
static	unsigned char  *logo_addr;
static	unsigned char  logo_id;

static unsigned int battery_voltage_min;
static unsigned int battery_voltage_max;
static unsigned int battery_voltage_scale;
static unsigned int read_battery_voltage(void);

#ifdef CONFIG_RTC_POWER_DETECT
void check_power_status(int cur_rle_num, int full_rle_num);
#endif

#ifdef CONTROLL_BACKLIGHT_WITH_LCD_POWER
void panel_power_on(void);
void board_set_lcd_power_on(void);
void board_set_lcd_power_off(void);
#endif

static void lcd_close_backlight(void)
{
	gpio_direction_output(CONFIG_GPIO_LCD_PWM, 0);

#ifdef CONTROLL_BACKLIGHT_WITH_LCD_POWER
	board_set_lcd_power_off();
#endif
}

static void lcd_open_backlight(void)
{
	unsigned port = CONFIG_GPIO_LCD_PWM / 32;
	unsigned pin = CONFIG_GPIO_LCD_PWM % 32;
	gpio_set_func(port, GPIO_FUNC_0, 1 << pin);

#ifdef CONTROLL_BACKLIGHT_WITH_LCD_POWER
	board_set_lcd_power_on();
	panel_power_on();
#endif
}

#define INTC_ICMCR0	0x0C
#define INTC_ICMR0_GPIO0_BIT	17
static void intc_unmask_gpio_irq(unsigned gpio)
{
	unsigned port = gpio / 32;
	writel(1 << (INTC_ICMR0_GPIO0_BIT - port), INTC_BASE + INTC_ICMCR0);
}

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
#ifdef CONFIG_GPIO_VOL_SUB
	gpio_direction_input(CONFIG_GPIO_VOL_SUB);
#endif
#ifdef CONFIG_GPIO_VOL_ADD
	gpio_direction_input(CONFIG_GPIO_VOL_ADD);
#endif
#ifdef CONFIG_GPIO_BACK
	gpio_direction_input(CONFIG_GPIO_BACK);
#endif
#ifdef CONFIG_GPIO_MENU
	gpio_direction_input(CONFIG_GPIO_MENU);
#endif
#ifdef CONFIG_GPIO_HOME
	gpio_direction_input(CONFIG_GPIO_HOME);
#endif
	return;
}

static int keys_pressed(void)
{
	int pressed = 0;
#if defined(CONFIG_GPIO_VOL_ADD) && defined(CONFIG_GPIO_VOL_ADD_ENLEVEL)
	pressed |= __is_gpio_en(CONFIG_GPIO_VOL_ADD);
#endif
#if defined(CONFIG_GPIO_VOL_SUB) && defined(CONFIG_GPIO_VOL_SUB_ENLEVEL)
	pressed |= __is_gpio_en(CONFIG_GPIO_VOL_SUB);
#endif
#if defined(CONFIG_GPIO_BACK) && defined(CONFIG_GPIO_BACK_ENLEVEL)
	pressed |= __is_gpio_en(CONFIG_GPIO_BACK);
#endif
#if defined(CONFIG_GPIO_MENU) && defined(CONFIG_GPIO_MENU_ENLEVEL)
	pressed |= __is_gpio_en(CONFIG_GPIO_MENU);
#endif
#if defined(CONFIG_GPIO_HOME) && defined(CONFIG_GPIO_HOME_ENLEVEL)
	pressed |= __is_gpio_en(CONFIG_GPIO_HOME);
#endif
	return pressed;
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
	ricoh619_power_off();
#endif

	printf("\n\nWe should not come here, please check the jz4760rtc.h!!!\n\n\n");

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

#ifdef CONFIG_GPIO_DC_DETECT
	gpio_as_irq_fall_edge(CONFIG_GPIO_DC_DETECT);
	/* unmask IRQ_GPIOn depends on GPIO_WAKEUP */
	intc_unmask_gpio_irq(CONFIG_GPIO_DC_DETECT);
	gpio_ack_irq(CONFIG_GPIO_DC_DETECT);
#endif

#ifdef CONFIG_GPIO_USB_DETECT
	gpio_as_irq_fall_edge(CONFIG_GPIO_USB_DETECT);
	/* unmask IRQ_GPIOn depends on GPIO_WAKEUP */
	intc_unmask_gpio_irq(CONFIG_GPIO_USB_DETECT);
	gpio_ack_irq(CONFIG_GPIO_USB_DETECT);
#endif

#ifdef CONFIG_RTC_POWER_DETECT
	/* Only for cruise now */
#define WRITE_CP0_REG(value,cn,slt)				\
	__asm__ __volatile__(					\
			     "mtc0\t%0, $%1, %2\n"		\
			     : /* no output */			\
			     :"r"(value),"i"(cn),"i"(slt))

	{
		unsigned int rtccr = rtc_read_reg(RTC_RTCCR) & 0x6F;
		unsigned int rtcsr = rtc_read_reg(RTC_RTCSR);
		unsigned int rtcsar = rtc_read_reg(RTC_RTCSAR);
		rtc_write_reg(RTC_RTCSAR, (rtcsr + 60 * 10)); // 10min
		rtccr |= (0x3 << 2); // Open RTC Alarm interrupt generate
		rtc_write_reg(RTC_RTCCR, rtccr);
		// open rtc interrupt
		writel(0xFFFDFFFF, INTC_BASE + 0x4);
		writel(0xFFFFFFFE, INTC_BASE + 0x24);
		WRITE_CP0_REG(0x0040FC05, 12, 0);
		printf("[ RTC ] CR : 0x%08x, SR : 0x%08x, SAR : 0x%08x\n",
		       rtc_read_reg(RTC_RTCCR), rtc_read_reg(RTC_RTCSR), rtc_read_reg(RTC_RTCSAR));
	}
#endif

	printf("enter sleep mode\n");
	mdelay(50);
	__asm__ volatile (".set mips32\n\t"
			  "sync\n\t"
			  "wait\n\t"
			  "nop\n\t"
			  "nop\n\t" "nop\n\t" "nop\n\t" ".set mips32");
	printf("out  sleep mode\n");

#ifdef CONFIG_RTC_POWER_DETECT
	/* Only for cruise now */
	{
		// close INT
		WRITE_CP0_REG(0x0040FC04, 12, 0);
		printf("[ RTC ] CR : 0x%08x, SR : 0x%08x, SAR : 0x%08x\n",
		       rtc_read_reg(RTC_RTCCR), rtc_read_reg(RTC_RTCSR), rtc_read_reg(RTC_RTCSAR));
	}
#endif

#ifdef CONFIG_GPIO_USB_DETECT
	gpio_direction_input(CONFIG_GPIO_USB_DETECT);
#endif
#ifdef CONFIG_GPIO_DC_DETECT
	gpio_direction_input(CONFIG_GPIO_DC_DETECT);
#endif
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
	printf("battery voltage is: %d mV\n", voltage);
	return voltage;
}

static void battery_init_gpio(void)
{
#if defined(CONFIG_GPIO_USB_DETECT) && defined(CONFIG_GPIO_USB_DETECT_ENLEVEL)
	gpio_direction_input(CONFIG_GPIO_USB_DETECT);
	gpio_disable_pull(CONFIG_GPIO_USB_DETECT);
#endif
#if defined(CONFIG_GPIO_DC_DETECT) && defined(CONFIG_GPIO_DC_DETECT_ENLEVEL)
	gpio_direction_input(CONFIG_GPIO_DC_DETECT);
	gpio_disable_pull(CONFIG_GPIO_DC_DETECT);
#endif
#if defined(CONFIG_GPIO_CHARGE_DETECT) && defined(CONFIG_GPIO_CHARGE_DETECT_ENLEVEL)
	gpio_direction_input(CONFIG_GPIO_CHARGE_DETECT);
#endif
}

static int charge_detect(void)
{
	int ret = 0;
	int i;

	if (readl(CPM_BASE + CPM_RSR) & CPM_RSR_WR)
		return ret;
#ifndef CONFIG_BATTERY_INIT_GPIO
	/* IF default battery_init_gpio function is not suitable for actual board,
	 * define and call the init function in board setup file. For example ,
	 * init gpio in borad/ingenic/mensa/mensa.c */
	battery_init_gpio();
#endif
	for (i = 0; i < 5; i++) {
		mdelay(10);
		ret += __charge_detect();
	}
	if (__dc_detected())
		printf("now use DC charge!!!\n");

	if (__usb_detected())
		printf("now use USB charge!!!\n");
	printf("ret = %d\n", ret);
	return ret;
}

static int poweron_key_long_pressed(void)
{
	int count = 0;
	if (__poweron_key_pressed()) {
		while (1) {
			if (__poweron_key_pressed()) {
				mdelay(10);
				count++;
			} else {
				count = 0;
				return 0;
			}
			if (count >= 100) {
				return 1;
			}
		}
	}
	return 0;
}

static int battery_is_low(void)
{
	unsigned int voltage = 0;

#ifdef PASS_CHECK_BATTERY_LOW
	return 0;
#endif

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

static void fb_fill(void *logo_addr, void *fb_addr, int count)
{
	//memcpy(logo_buf, fb_addr, count);
	int i;
	int *dest_addr = (int *)fb_addr;
	int *src_addr = (int *)logo_addr;
	for(i = 0; i < count; i = i + 4){
		*dest_addr =  *src_addr;
		src_addr++;
		dest_addr++;
	}
}

static int show_charge_logo_rle(int rle_num)
{
	void *lcd_base = (void *)gd->fb_base;
	int vm_width = panel_info.vl_col;
	int vm_height = panel_info.vl_row;
	int bpp = NBITS(panel_info.vl_bpix);
	int buf_size = vm_height * vm_width * bpp / 8;
	int logo_charge_num = (battery_voltage_max  - battery_voltage_min) / battery_voltage_scale;


	if (rle_num < 0 && rle_num > logo_charge_num)
		return -EINVAL;
	//rle_plot(rle_num * LOGO_CHARGE_SIZE + RLE_LOGO_BASE_ADDR, lcd_base);
	if(logo_id < logo_charge_num ){
		if(logo_addr == NULL) {
			logo_addr = (unsigned char *)malloc_charge_logo(buf_size);
			if(logo_addr == NULL){
				printf("famebuffer malloc failed\n");
				goto orig;
			}
		}
		logo_id += 1;
		debug("logo_id == %d\n", logo_id);
		rle_plot(rle_charge_logo_addr[rle_num], logo_addr + rle_num * buf_size);
		fb_fill(logo_addr + rle_num * buf_size, lcd_base, buf_size);
		lcd_sync();
	}else if(logo_addr != NULL){
		fb_fill(logo_addr + rle_num * buf_size, lcd_base, buf_size);
		lcd_sync();
	}else{
		goto orig;
	}
	return 0;
orig:
	rle_plot(rle_charge_logo_addr[rle_num], lcd_base);
	lcd_sync();
	return 0;
}

extern int calc_battery_capacity();
static int get_rle_num(void)
{
	int capacity;
	int rle_num_base;
	int logo_charge_num = (battery_voltage_max  - battery_voltage_min) / battery_voltage_scale;
	unsigned int voltage;

	capacity = calc_battery_capacity();

	if(capacity < 0){
		voltage = read_battery_voltage();
		if (voltage < battery_voltage_min) {
			rle_num_base = 0;
		} else if (voltage < battery_voltage_max) {
			rle_num_base = (voltage - battery_voltage_min) / battery_voltage_scale - 1;
		} else {
			rle_num_base = logo_charge_num - 1;
		}
	}else{
		if (capacity < 10) {
			rle_num_base = 0;
		} else if (capacity > 90) {
			rle_num_base = 4;
		} else if (logo_charge_num > 2) {
			rle_num_base = 1 + (capacity - 10) * (logo_charge_num - 2) / 81;
		}
	}
#ifdef CONFIG_RTC_POWER_DETECT
	/* Do something while battery capacity changes */
	check_power_status(rle_num_base, logo_charge_num);
#endif
	return rle_num_base;	
}

static void show_charging_logo(void)
{
/* Show time for the charge flash */
#define FLASH_INTERVAL_TIME	500	/* 500 ms */
#define FLASH_SHOW_TIME	12000	/* 12S */

	int show_flash;
	int kpressed = 0;
	int rle_num = 0;
	int rle_num_base = 0;
	unsigned long start_time;
	unsigned long timeout = FLASH_INTERVAL_TIME;
	unsigned int charge_logo_num = (battery_voltage_max - battery_voltage_min) / battery_voltage_scale;

	/* Shut some modules power down,cdma,gsm e.g. */
	/* board_powerdown_device(); */

	key_init_gpio();
	rle_num_base = get_rle_num();
	rle_num = rle_num_base;
	start_time = get_timer(0);
	show_flash = 1;

	mdelay(2000);
	lcd_clear_black();
	while (1) {
		if (kpressed == 0) {
			kpressed = keys_pressed();
			if (kpressed) {
				show_flash = 1;
				timeout = FLASH_INTERVAL_TIME;
				start_time = get_timer(0);
				debug("---key pressed!\n");
			}
		}
		if (__poweron_key_pressed()) {
			show_flash = 1;
			timeout = FLASH_INTERVAL_TIME;
			start_time = get_timer(0);
			debug("---poweron pressed!\n");
			if (poweron_key_long_pressed()) {
				lcd_clear_black();
				debug("poweron long pressed \n");
				return;
			}
		}
		// During the charge process ,User extract the USB cable ,Enter hibernate mode
		if (!(__usb_detected() || __dc_detected())) {
			debug("charge is stop\n");
			show_charge_logo_rle(rle_num_base);
			mdelay(200);
			jz_pm_do_hibernate();
		}

		/* show the charge flash */
		if (show_flash && (get_timer(start_time) > timeout)) {
			show_charge_logo_rle(rle_num);
			mdelay(1000);
			kpressed = 0;
			rle_num++;
			if (rle_num >= charge_logo_num)
				rle_num = rle_num_base;

			timeout += FLASH_INTERVAL_TIME;
			if (timeout > FLASH_SHOW_TIME) {
				show_flash = 0;
				lcd_clear_black();
				lcd_close_backlight();
				jz_pm_do_idle();
				mdelay(10);
				lcd_open_backlight();

				rle_num_base = get_rle_num();
				rle_num = rle_num_base;
				debug("rle_num_base = %d\n", rle_num_base);
				show_flash = 1;
				timeout = FLASH_INTERVAL_TIME;
				start_time = get_timer(0);
			}
		}
	}
}

static void show_battery_low_logo(void)
{
	lcd_clear_black();
	show_charge_logo_rle(0);
	mdelay(5000);
	lcd_close_backlight();
}

static void battery_detect(void)
{
	if (charge_detect()) {
		show_charging_logo();
	} else if (battery_is_low()) {
		show_battery_low_logo();
		printf("The battery voltage is too low. Please charge\n");
		printf("Battery low level,Into hibernate mode ... \n");
#ifdef CONFIG_PMU_RICOH6x
		ricoh619_set_pswr(1); // 1% capacity
#endif
		jz_pm_do_hibernate();
	}

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
int __is_burner_reset(void)
{
	return 0;
}
int is_burner_reset(void) __attribute__((weak, alias("__is_burner_reset")));

extern unsigned char rle_default_logo_addr [ 59824 ];
static int do_battery_detect(cmd_tbl_t * cmdtp, int flag, int argc,
			     char *const argv[])
{
	int ret = 0;

#ifdef CONFIG_GPIO_PMU_SLEEP
	gpio_direction_output(CONFIG_GPIO_PMU_SLEEP, 0);
#endif

	if (is_burner_reset())
		jz_pm_do_hibernate();
	ret = voltage_argument_init(argc, argv);
	if(ret != 0){
		return ret;
	}
	battery_detect();
	return ret;
}

U_BOOT_CMD(batterydet, 4, 1, do_battery_detect,
	   "battery and show charge logo", "[<battery_min> <battery_max>, <battery_scale>]");
