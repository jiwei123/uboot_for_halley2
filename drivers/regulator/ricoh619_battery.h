
/*
 * include/linux/power/ricoh619_battery.h
 *
 * RICOH RC5T619 Charger Driver
 *
 * Copyright (C) 2012-2014 RICOH COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef __LINUX_POWER_RICOH619_H_
#define __LINUX_POWER_RICOH619_H_

/* #include <linux/power_supply.h> */
/* #include <linux/types.h> */

/* Defined battery information */
#define	ADC_VDD_MV	2800
#define	MIN_VOLTAGE	3100
#define	MAX_VOLTAGE	4200
#define	B_VALUE		3435
#define CAPA_MIN 	5

/* RICOH61x Register information */
/* bank 0 */
#define PSWR_REG		0x07
#define VINDAC_REG		0x03
/* for ADC */
#define	INTEN_REG		0x9D
#define	EN_ADCIR3_REG		0x8A
#define	ADCCNT3_REG		0x66
#define	VBATDATAH_REG		0x6A
#define	VBATDATAL_REG		0x6B

#define VSYSDATAH_REG	0x70
#define VSYSDATAL_REG	0x71

#define CHGCTL1_REG		0xB3
#define	REGISET1_REG	0xB6
#define	REGISET2_REG	0xB7
#define	CHGISET_REG		0xB8
#define	TIMSET_REG		0xB9
#define	BATSET1_REG		0xBA
#define	BATSET2_REG		0xBB

#define CHGSTATE_REG		0xBD

#define	FG_CTRL_REG		0xE0
#define	SOC_REG			0xE1
#define	RE_CAP_H_REG		0xE2
#define	RE_CAP_L_REG		0xE3
#define	FA_CAP_H_REG		0xE4
#define	FA_CAP_L_REG		0xE5
#define	TT_EMPTY_H_REG		0xE7
#define	TT_EMPTY_L_REG		0xE8
#define	TT_FULL_H_REG		0xE9
#define	TT_FULL_L_REG		0xEA
#define	VOLTAGE_1_REG		0xEB
#define	VOLTAGE_2_REG		0xEC
#define	TEMP_1_REG		0xED
#define	TEMP_2_REG		0xEE

#define	CC_CTRL_REG		0xEF
#define	CC_SUMREG3_REG		0xF3
#define	CC_SUMREG2_REG		0xF4
#define	CC_SUMREG1_REG		0xF5
#define	CC_SUMREG0_REG		0xF6
#define	CC_AVERAGE1_REG		0xFB
#define	CC_AVERAGE0_REG		0xFC

/* bank 1 */
/* Top address for battery initial setting */
#define	BAT_INIT_TOP_REG	0xBC
#define	TEMP_GAIN_H_REG		0xD6
#define	TEMP_OFF_H_REG		0xD8
#define	BAT_REL_SEL_REG		0xDA
#define	BAT_TA_SEL_REG		0xDB
/**************************/

#define CONFIG_RICOH61X_CHARGE_DONE_LIMIT 3

/* detailed status in CHGSTATE (0xBD) */
enum ChargeState {
	CHG_STATE_CHG_OFF = 0,
	CHG_STATE_CHG_READY_VADP,
	CHG_STATE_CHG_TRICKLE,
	CHG_STATE_CHG_RAPID,
	CHG_STATE_CHG_COMPLETE,
	CHG_STATE_SUSPEND,
	CHG_STATE_VCHG_OVER_VOL,
	CHG_STATE_BAT_ERROR,
	CHG_STATE_NO_BAT,
	CHG_STATE_BAT_OVER_VOL,
	CHG_STATE_BAT_TEMP_ERR,
	CHG_STATE_DIE_ERR,
	CHG_STATE_DIE_SHUTDOWN,
	CHG_STATE_NO_BAT2,
	CHG_STATE_CHG_READY_VUSB,
};

enum SupplyState {
	SUPPLY_STATE_BAT = 0,
	SUPPLY_STATE_ADP,
	SUPPLY_STATE_USB,
} ;

struct ricoh619_battery_type_data {
	int	ch_vfchg;
	int	ch_vrchg;
	int	ch_vbatovset;
	int	ch_ichg;
	int	ch_icchg;
	int	ch_ilim_adp;
	int	ch_ilim_usb;
	int	fg_target_vsys;
	int	fg_target_ibat;
	int	fg_poff_vbat;
	int	fg_rsense_val;
	int	jt_en;
	int	jt_hw_sw;
	int	jt_temp_h;
	int	jt_temp_l;
	int	jt_vfchg_h;
	int	jt_vfchg_l;
	int	jt_ichg_h;
	int	jt_ichg_l;
};

#define BATTERY_TYPE_NUM 6
struct ricoh619_battery_platform_data {
	int	irq;
	int	alarm_vol_mv;
	int	multiple;
	unsigned long	monitor_time;
	struct ricoh619_battery_type_data type[BATTERY_TYPE_NUM];
};
#endif


struct ricoh619_battery_platform_data pdata = {
	.irq 		= 262,
//	.irq 		= RICOH619_IRQ_BASE,
	.alarm_vol_mv 	= 3300,
	// .adc_channel = RICOH619_ADC_CHANNEL_VBAT,
	.multiple	= 100, //100%
	.monitor_time 	= 60,
		/* some parameter is depend of battery type */
	/*the battery fof 4000mA 2000mA*/
	.type[0] = {
		.ch_vfchg 	= 0x02,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x4,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x1,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x09,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x1D,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x1D,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= (CONFIG_RICOH61X_CHARGE_DONE_LIMIT),	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3500,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 200, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x0D,	/* VFCHG Low  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_l 	= 0x09,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},
	/*the battery for 260mA 500mA*/
	.type[1] = {
		.ch_vfchg 	= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x01,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x1,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x03,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x1D,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x1D,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= (CONFIG_RICOH61X_CHARGE_DONE_LIMIT),	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3500,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 100,	/* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x04,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},
	/*the battery for 300mAh or 320mAh*/
	.type[2] = {
		.ch_vfchg 	= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x04,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x1,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x04,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x1D,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x1D,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= (CONFIG_RICOH61X_CHARGE_DONE_LIMIT),	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3500,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 50,	/* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x04,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},
	/*the battery for 200mAh*/
	.type[3] = {
		.ch_vfchg 	= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x04,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x1,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x04,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x1D,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x1D,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= 0x03,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3500,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 50,	/* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x04,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},
	/*the battery for wakeup 320mah 4.2v, aw808 320mah 4.2v*/
	.type[4] = {
		.ch_vfchg 	= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x04,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x1,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x04,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x1D,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x1D,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= 0x00,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3500,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 50,	/* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x04,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},
	/*the battery for oband 300mah 4.35v; oband 310mah 4.35v; inwatch 310mah 4.35v  */
	.type[5] = {
		.ch_vfchg 	= 0x04,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x04,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x1,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x04,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x1D,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x1D,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= 0x00,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3500,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 50,	/* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x04,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	}
};

#if defined CONFIG_BATTERY_WAKEUP_320MAH_4200MV_100MA
uint8_t battery_init_para[][32] = {
    {
		0x0A, 0x0A, 0x0B, 0xEF, 0x0C, 0x0F, 0x0C, 0x0D,
		0x0C, 0x1E, 0x0C, 0x36, 0x0C, 0x5C, 0x0C, 0x93,
		0x0C, 0xC7, 0x0D, 0x0C, 0x0D, 0x64, 0x01, 0x32,
		0x01, 0x09, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#elif defined CONFIG_BATTERY_WAKEUP_320MAH_4200MV
uint8_t battery_init_para[][32] = {
    {
		0x0A, 0x0F, 0x0B, 0xD3, 0x0B, 0xFA, 0x0C, 0x13,
		0x0C, 0x24, 0x0C, 0x3C, 0x0C, 0x62, 0x0C, 0x99,
		0x0C, 0xCE, 0x0D, 0x16, 0x0D, 0x72, 0x01, 0x2C,
		0x01, 0x36, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#elif defined CONFIG_BATTERY_AW808_320MAH_4200MV
uint8_t battery_init_para[][32] = {
    {
		0x09, 0xEA, 0x0B, 0xAA, 0x0B, 0xEF, 0x0C, 0x0A,
		0x0C, 0x1B, 0x0C, 0x31, 0x0C, 0x56, 0x0C, 0x92,
		0x0C, 0xCE, 0x0D, 0x19, 0x0D, 0x72, 0x01, 0x48,
		0x01, 0x26, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#elif defined CONFIG_BATTERY_OBAND_300MAH_4350MV
uint8_t battery_init_para[][32] = {
    {
		0x0A, 0x1D, 0x0B, 0xD5, 0x0B, 0xF7, 0x0C, 0x15,
		0x0C, 0x33, 0x0C, 0x59, 0x0C, 0x8D, 0x0C, 0xD7,
		0x0D, 0x30, 0x0D, 0x86, 0x0D, 0xE7, 0x01, 0x25,
		0x01, 0x7C, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#elif defined CONFIG_BATTERY_OBAND_310MAH_4350MV
uint8_t battery_init_para[][32] = {
    {
		0x0A, 0x7F, 0x0B, 0xCA, 0x0B, 0xFC, 0x0C, 0x14,
		0x0C, 0x29, 0x0C, 0x4B, 0x0C, 0x84, 0x0C, 0xCD,
		0x0D, 0x1B, 0x0D, 0x75, 0x0D, 0xE6, 0x01, 0x4C,
		0x01, 0x63, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#elif defined CONFIG_BATTERY_INWATCH_310MAH_4350MV
uint8_t battery_init_para[][32] = {
    {
		0x0A, 0x2E, 0x0B, 0xD5, 0x0B, 0xF9, 0x0C, 0x15,
		0x0C, 0x33, 0x0C, 0x55, 0x0C, 0x88, 0x0C, 0xCE,
		0x0D, 0x25, 0x0D, 0x7C, 0x0D, 0xE8, 0x01, 0x31,
		0x01, 0x27, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#else
uint8_t battery_init_para[][32] = {
    {
		0x09, 0xEA, 0x0B, 0xAA, 0x0B, 0xEF, 0x0C, 0x0A,
		0x0C, 0x1B, 0x0C, 0x31, 0x0C, 0x56, 0x0C, 0x92,
		0x0C, 0xCE, 0x0D, 0x19, 0x0D, 0x72, 0x01, 0x48,
		0x01, 0x26, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
    }
};
#endif
