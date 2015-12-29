/*
 * include/linux/mfd/ricoh619.h
 *
 * Core driver interface to access RICOH R5T619 power management chip.
 *
 * Copyright (C) 2012-2014 RICOH COMPANY,LTD
 *
 * Based on code
 *	Copyright (C) 2011 NVIDIA Corporation
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __LINUX_MFD_RICOH619_H
#define __LINUX_MFD_RICOH619_H


/* Maximum number of main interrupts */
#define MAX_INTERRUPT_MASKS	13
#define MAX_MAIN_INTERRUPT	7
#define MAX_GPEDGE_REG		2

/* Power control register */
#define RICOH61x_PWR_WD			0x0B
#define RICOH61x_PWR_WD_COUNT		0x0C
#define RICOH61x_PWR_FUNC		0x0D
#define RICOH61x_PWR_SLP_CNT		0x0E
#define RICOH61x_PWR_REP_CNT		0x0F
#define RICOH61x_PWR_ON_TIMSET		0x10
#define RICOH61x_PWR_NOE_TIMSET		0x11
#define RICOH61x_PWR_IRSEL		0x15

/* Interrupt enable register */
#define RICOH61x_INT_EN_SYS		0x12
#define RICOH61x_INT_EN_DCDC		0x40
#define RICOH61x_INT_EN_RTC		0xAE
#define RICOH61x_INT_EN_ADC1		0x88
#define RICOH61x_INT_EN_ADC2		0x89
#define RICOH61x_INT_EN_ADC3		0x8A
#define RICOH61x_INT_EN_GPIO		0x94
#define RICOH61x_INT_EN_GPIO2		0x94 /* dummy */
#define RICOH61x_INT_MSK_CHGCTR		0xBE
#define RICOH61x_INT_MSK_CHGSTS1	0xBF
#define RICOH61x_INT_MSK_CHGSTS2	0xC0
#define RICOH61x_INT_MSK_CHGERR		0xC1
#define RICOH61x_INT_MSK_CHGEXTIF	0xD1

/* Interrupt select register */
#define RICOH61x_CHG_CTRL_DETMOD1	0xCA
#define RICOH61x_CHG_CTRL_DETMOD2	0xCB
#define RICOH61x_CHG_STAT_DETMOD1	0xCC
#define RICOH61x_CHG_STAT_DETMOD2	0xCD
#define RICOH61x_CHG_STAT_DETMOD3	0xCE


/* interrupt status registers (monitor regs)*/
#define RICOH61x_INTC_INTPOL		0x9C
#define RICOH61x_INTC_INTEN		0x9D
#define RICOH61x_INTC_INTMON		0x9E

#define RICOH61x_INT_MON_SYS		0x14
#define RICOH61x_INT_MON_DCDC		0x42
#define RICOH61x_INT_MON_RTC		0xAF

#define RICOH61x_INT_MON_CHGCTR		0xC6
#define RICOH61x_INT_MON_CHGSTS1	0xC7
#define RICOH61x_INT_MON_CHGSTS2	0xC8
#define RICOH61x_INT_MON_CHGERR		0xC9
#define RICOH61x_INT_MON_CHGEXTIF	0xD3

/* interrupt clearing registers */
#define RICOH61x_INT_IR_SYS		0x13
#define RICOH61x_INT_IR_DCDC		0x41
#define RICOH61x_INT_IR_RTC		0xAF
#define RICOH61x_INT_IR_ADCL		0x8C
#define RICOH61x_INT_IR_ADCH		0x8D
#define RICOH61x_INT_IR_ADCEND		0x8E
#define RICOH61x_INT_IR_GPIOR		0x95
#define RICOH61x_INT_IR_GPIOF		0x96
#define RICOH61x_INT_IR_CHGCTR		0xC2
#define RICOH61x_INT_IR_CHGSTS1		0xC3
#define RICOH61x_INT_IR_CHGSTS2		0xC4
#define RICOH61x_INT_IR_CHGERR		0xC5
#define RICOH61x_INT_IR_CHGEXTIF	0xD2

/* GPIO register base address */
#define RICOH61x_GPIO_IOSEL		0x90
#define RICOH61x_GPIO_IOOUT		0x91
#define RICOH61x_GPIO_GPEDGE1		0x92
#define RICOH61x_GPIO_GPEDGE2		0x93
/* #define RICOH61x_GPIO_EN_GPIR	0x94 */
/* #define RICOH61x_GPIO_IR_GPR		0x95 */
/* #define RICOH61x_GPIO_IR_GPF		0x96 */
#define RICOH61x_GPIO_MON_IOIN		0x97
#define RICOH61x_GPIO_LED_FUNC		0x98

#define RICOH61x_REG_BANKSEL		0xFF

/* Charger Control register */
#define RICOH61x_CHG_CTL1		0xB3
#define	TIMSET_REG			0xB9

/* ADC Control register */
#define RICOH61x_ADC_CNT1		0x64
#define RICOH61x_ADC_CNT2		0x65
#define RICOH61x_ADC_CNT3		0x66
#define RICOH61x_ADC_VADP_THL		0x7C
#define RICOH61x_ADC_VSYS_THL		0x80

#define	RICOH61x_FG_CTRL		0xE0
#define	RICOH61x_PSWR			0x07

/* RICOH61x IRQ definitions */
enum {
	RICOH61x_IRQ_POWER_ON,
	RICOH61x_IRQ_EXTIN,
	RICOH61x_IRQ_PRE_VINDT,
	RICOH61x_IRQ_PREOT,
	RICOH61x_IRQ_POWER_OFF,
	RICOH61x_IRQ_NOE_OFF,
	RICOH61x_IRQ_WD,
	RICOH61x_IRQ_CLK_STP,

	RICOH61x_IRQ_DC1LIM,
	RICOH61x_IRQ_DC2LIM,
	RICOH61x_IRQ_DC3LIM,
	RICOH61x_IRQ_DC4LIM,
	RICOH61x_IRQ_DC5LIM,

	RICOH61x_IRQ_ILIMLIR,
	RICOH61x_IRQ_VBATLIR,
	RICOH61x_IRQ_VADPLIR,
	RICOH61x_IRQ_VUSBLIR,
	RICOH61x_IRQ_VSYSLIR,
	RICOH61x_IRQ_VTHMLIR,
	RICOH61x_IRQ_AIN1LIR,
	RICOH61x_IRQ_AIN0LIR,

	RICOH61x_IRQ_ILIMHIR,
	RICOH61x_IRQ_VBATHIR,
	RICOH61x_IRQ_VADPHIR,
	RICOH61x_IRQ_VUSBHIR,
	RICOH61x_IRQ_VSYSHIR,
	RICOH61x_IRQ_VTHMHIR,
	RICOH61x_IRQ_AIN1HIR,
	RICOH61x_IRQ_AIN0HIR,

	RICOH61x_IRQ_ADC_ENDIR,

	RICOH61x_IRQ_GPIO0,
	RICOH61x_IRQ_GPIO1,
	RICOH61x_IRQ_GPIO2,
	RICOH61x_IRQ_GPIO3,
	RICOH61x_IRQ_GPIO4,

	RICOH61x_IRQ_CTC,
	RICOH61x_IRQ_DALE,

	RICOH61x_IRQ_FVADPDETSINT,
	RICOH61x_IRQ_FVUSBDETSINT,
	RICOH61x_IRQ_FVADPLVSINT,
	RICOH61x_IRQ_FVUSBLVSINT,
	RICOH61x_IRQ_FWVADPSINT,
	RICOH61x_IRQ_FWVUSBSINT,

	RICOH61x_IRQ_FONCHGINT,
	RICOH61x_IRQ_FCHGCMPINT,
	RICOH61x_IRQ_FBATOPENINT,
	RICOH61x_IRQ_FSLPMODEINT,
	RICOH61x_IRQ_FBTEMPJTA1INT,
	RICOH61x_IRQ_FBTEMPJTA2INT,
	RICOH61x_IRQ_FBTEMPJTA3INT,
	RICOH61x_IRQ_FBTEMPJTA4INT,

	RICOH61x_IRQ_FCURTERMINT,
	RICOH61x_IRQ_FVOLTERMINT,
	RICOH61x_IRQ_FICRVSINT,
	RICOH61x_IRQ_FPOOR_CHGCURINT,
	RICOH61x_IRQ_FOSCFDETINT1,
	RICOH61x_IRQ_FOSCFDETINT2,
	RICOH61x_IRQ_FOSCFDETINT3,
	RICOH61x_IRQ_FOSCMDETINT,

	RICOH61x_IRQ_FDIEOFFINT,
	RICOH61x_IRQ_FDIEERRINT,
	RICOH61x_IRQ_FBTEMPERRINT,
	RICOH61x_IRQ_FVBATOVINT,
	RICOH61x_IRQ_FTTIMOVINT,
	RICOH61x_IRQ_FRTIMOVINT,
	RICOH61x_IRQ_FVADPOVSINT,
	RICOH61x_IRQ_FVUSBOVSINT,

	RICOH61x_IRQ_FGCDET,
	RICOH61x_IRQ_FPCDET,
	RICOH61x_IRQ_FWARN_ADP,

	/* Should be last entry */
	RICOH61x_NR_IRQS,
};

/* RICOH61x gpio definitions */
enum {
	RICOH61x_GPIO0,
	RICOH61x_GPIO1,
	RICOH61x_GPIO2,
	RICOH61x_GPIO3,
	RICOH61x_GPIO4,

	RICOH61x_NR_GPIO,
};

enum ricoh61x_sleep_control_id {
	RICOH619_DS_DC1,
	RICOH619_DS_DC2,
	RICOH619_DS_DC3,
	RICOH619_DS_DC4,
	RICOH619_DS_DC5,
	RICOH619_DS_LDO1,
	RICOH619_DS_LDO2,
	RICOH619_DS_LDO3,
	RICOH619_DS_LDO4,
	RICOH619_DS_LDO5,
	RICOH619_DS_LDO6,
	RICOH619_DS_LDO7,
	RICOH619_DS_LDO8,
	RICOH619_DS_LDO9,
	RICOH619_DS_LDO10,
	RICOH619_DS_LDORTC1,
	RICOH619_DS_LDORTC2,
	RICOH619_DS_PSO0,
	RICOH619_DS_PSO1,
	RICOH619_DS_PSO2,
	RICOH619_DS_PSO3,
	RICOH619_DS_PSO4,
};

#ifdef CONFIG_320MAH_CAPACITY_BATTERY
/*for 320mA*/
uint8_t battery_init_para[][32] = { 
        {   
	        0x0B, 0x20, 0x0B, 0x59, 0x0B, 0x9B, 0x0B, 0xE2,
		0x0C, 0x07, 0x0C, 0x15, 0x0C, 0x3D, 0x0C, 0x7A,
		0x0C, 0xBA, 0x0D, 0x04, 0x0D, 0x66, 0x01, 0x4F,
		0x01, 0xB3, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
	}   
};
#endif

#ifdef CONFIG_400MAH_PARAMOUNT_CAPACITY_BATTERY
/*for paramount 400mA*/
uint8_t battery_init_para[][32] = {
	{
		0x0B, 0x3B, 0x0B, 0xCA, 0x0B, 0xF7, 0x0C, 0x13,
		0x0C, 0x30, 0x0C, 0x55, 0x0C, 0x91, 0x0C, 0xDB,
		0x0D, 0x29, 0x0D, 0x7F, 0x0D, 0xCE, 0x01, 0x8F,
		0x01, 0x45, 0x0F, 0xC8, 0x05, 0x29, 0x22, 0x56
	}
};
#endif

#ifdef CONFIG_400MAH_WESEEING_CAPACITY_BATTERY
/*for weseeing 400mA*/
uint8_t battery_init_para[][32] = {
	{
		0x0B, 0x1A, 0x0B, 0xD2, 0x0B, 0xF9, 0x0C, 0x11, 
		0x0C, 0x24, 0x0C, 0x3B, 0x0C, 0x64, 0x0C, 0x9F, 
		0x0C, 0xD6, 0x0D, 0x1A, 0x0D, 0x6F, 0x01, 0xAD, 
		0x00, 0xF0, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
	}
};
#endif

#ifdef CONFIG_430MAH_CAPACITY_BATTERY
/*for cruise 430mA*/
uint8_t battery_init_para[][32] = {
	{
		0x0B, 0x19, 0x0B, 0xCD, 0x0B, 0xF9, 0x0C, 0x11, 
		0x0C, 0x24, 0x0C, 0x3E, 0x0C, 0x67, 0x0C, 0xA5, 
		0x0C, 0xDD, 0x0D, 0x1E, 0x0D, 0x6F, 0x01, 0xB8, 
		0x00, 0xFA, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56		
	}
};
#endif

#ifdef CONFIG_500MAH_CAPACITY_BATTERY
/*for cruise 500mA*/
uint8_t battery_init_para[][32] = {
	{
		0x0B, 0x33, 0x0B, 0xD1, 0x0B, 0xF8, 0x0C, 0x0E, 
		0x0C, 0x21, 0x0C, 0x3B, 0x0C, 0x61, 0x0C, 0x98, 
		0x0C, 0xCE, 0x0D, 0x15, 0x0D, 0x6E, 0x01, 0xFC, 
		0x01, 0x1D, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
	}
};
#endif

#ifdef CONFIG_600MAH_CAPACITY_BATTERY
/*for regularity 600mA*/
uint8_t battery_init_para[][32] = {
	{
		0x0B, 0x63, 0x0B, 0xD3, 0x0B, 0xFC, 0x0C, 0x14, 
		0x0C, 0x22, 0x0C, 0x38, 0x0C, 0x62, 0x0C, 0x96, 
		0x0C, 0xCA, 0x0D, 0x09, 0x0D, 0x6B, 0x02, 0x76, 
		0x00, 0xE1, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
	}
};
#endif

#ifdef CONFIG_LARGE_CAPACITY_BATTERY
/*for 4000mA*/
uint8_t battery_init_para[][32] = {
	{
		0x0B, 0x0D, 0x0B, 0xC3, 0x0B, 0xF1, 0x0C, 0x0E,
		0x0C, 0x20, 0x0C, 0x38, 0x0C, 0x5C, 0x0C, 0x98,
		0x0C, 0xCD, 0x0D, 0x0A, 0x0D, 0x5A, 0x0E, 0x38,
		0x00, 0x4B, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
	}
};
#endif

/* Defined battery information */
#define	ADC_VDD_MV	2800
#define	MIN_VOLTAGE	3100
#define	MAX_VOLTAGE	4200
#define	B_VALUE		3435


/* RICOH61x Register information */
/* bank 0 */
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
/* / */

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

#define BATTERY_TYPE_NUM 2
struct ricoh619_battery_platform_data {
	int	alarm_vol_mv;
	int	multiple;
	unsigned long	monitor_time;
	struct ricoh619_battery_type_data type[BATTERY_TYPE_NUM];
};

struct ricoh619_battery_platform_data pdata = {
	.alarm_vol_mv 	= 3300,
	// .adc_channel = RICOH619_ADC_CHANNEL_VBAT,
	.multiple	= 100, //100%
	.monitor_time 	= 60,
	/* some parameter is depend of battery type */
	/*the battery for 260mA 500mA*/
	.type[0] = {
		.ch_vfchg 	= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x01,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x0,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x09,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x0e,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x0e,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= 0x00,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3000,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100,  /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 100,  /* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x04,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0x02,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x09,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},

	.type[1] = {
		.ch_vfchg 	= 0x04,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg 	= 0x04,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset 	= 0x01,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 	= 0x09,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x0e,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x0e,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg 	= 0x01,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys = 3000,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat = 100, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat 	= 0, 	/* setting value of 0 per Vbat */
		.fg_rsense_val	= 100,  /* setting value of R Sense */
		.jt_en 		= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw 	= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h 	= 50,	/* degree C */
		.jt_temp_l 	= 12,	/* degree C */
		.jt_vfchg_h 	= 0x04,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0x02,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h 	= 0x09,	/* ICHG Hi   	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l 	= 0x01,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},

};

#endif
