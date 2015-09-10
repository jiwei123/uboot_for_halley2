/*
 * drivers/regulator/ricoh619-regulator.c
 *
 * Regulator driver for RICOH R5T619 power management chip.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/
#include <config.h>
#include <common.h>
#include <linux/err.h>
#include <linux/list.h>
#include <regulator.h>
#include <i2c.h>
#include <malloc.h>
#include "ricoh619_battery.h"
#ifndef CONFIG_SPL_BUILD
#include <power/ricoh619.h>
#include <power/ricoh619-regulator.h>
#endif

#define RICOH61x_I2C_ADDR    0x32

#define RICOH619_DC1  0x36
#define RICOH619_DC2  0x37
#define RICOH619_DC3  0x38
#define RICOH619_DC4  0x39
#define	RICOH619_DC5  0x3A
#define	RICOH619_LDO1 0x4c
#define	RICOH619_LDO2 0x4d
#define	RICOH619_LDO3 0x4e
#define	RICOH619_LDO4 0x4f
#define	RICOH619_LDO5 0x50
#define	RICOH619_LDO6 0x51
#define	RICOH619_LDO7 0x52
#define	RICOH619_LDO8 0x53
#define	RICOH619_LDO9 0x54
#define	RICOH619_LDO10 0x55
#define	RICOH619_LDORTC1 0x56
#define	RICOH619_LDORTC2 0x57
#define RICOH61x_INTC_INTEN 0x9D
#ifndef CONFIG_SPL_BUILD
/* FG setting */
#define RICOH61x_REL1_SEL_VALUE         64
#define RICOH61x_REL2_SEL_VALUE         0

#define RICOH61x_GET_CHARGE_NUM		10

#define ENABLE_FG_KEEP_ON_MODE
#define ENABLE_LOW_BATTERY_DETECTION

int g_full_flag;
int charger_irq;
int g_soc;
int g_fg_on_mode;
int bank_num  = 1;

enum {
	RICOH61x_SOCA_START,
	RICOH61x_SOCA_UNSTABLE,
	RICOH61x_SOCA_FG_RESET,
	RICOH61x_SOCA_DISP,
	RICOH61x_SOCA_STABLE,
	RICOH61x_SOCA_ZERO,
	RICOH61x_SOCA_FULL,
	RICOH61x_SOCA_LOW_VOL,
};

enum {  
	POWER_SUPPLY_STATUS_UNKNOWN = 0,
	POWER_SUPPLY_STATUS_CHARGING,
	POWER_SUPPLY_STATUS_DISCHARGING,
	POWER_SUPPLY_STATUS_NOT_CHARGING,
	POWER_SUPPLY_STATUS_FULL,
};


struct ricoh61x {
	struct device           *dev;
	struct i2c_client       *client;
	int                     gpio_base;
	int                     irq_base;
	int                     chip_irq;
	unsigned long           group_irq_en[MAX_MAIN_INTERRUPT];

	/* For main interrupt bits in INTC */
	u8                      intc_inten_cache;
	u8                      intc_inten_reg;

	/* For group interrupt bits and address */
	u8                      irq_en_cache[MAX_INTERRUPT_MASKS];
	u8                      irq_en_reg[MAX_INTERRUPT_MASKS];

	/* For gpio edge */
	u8                      gpedge_cache[MAX_GPEDGE_REG];
	u8                      gpedge_reg[MAX_GPEDGE_REG];

	int                     bank_num;
};


struct ricoh61x_battery_info {
	struct device      *dev;
	struct workqueue_struct *monitor_wqueue;
	struct workqueue_struct *workqueue;     /* for Charging & VUSB/VADP */

#ifdef ENABLE_FACTORY_MODE
	struct delayed_work     factory_mode_work;
	struct workqueue_struct *factory_mode_wqueue;
#endif

	//        struct mutex            lock;
	unsigned long           monitor_time;
	int             adc_vdd_mv;
	int             multiple;
	int             alarm_vol_mv;
	int             status;
	int             min_voltage;
	int             max_voltage;
	int             cur_voltage;
	int             capacity;
	int             battery_temp;
	int             time_to_empty;
	int             time_to_full;
	int             chg_ctr;
	int             chg_stat1;
	unsigned        present:1;
	u16             delay;
	struct          ricoh61x_soca_info *soca;
	int             first_pwon;
	bool            entry_factory_mode;
	int             ch_vfchg;
	int             ch_vrchg;
	int             ch_vbatovset;
	int             ch_ichg;
	int             ch_ilim_adp;
	int             ch_ilim_usb;
	int             ch_icchg;
	int             fg_target_vsys;
	int             fg_target_ibat;
	int             fg_poff_vbat;
	int             fg_rsense_val;
	int             jt_en;
	int             jt_hw_sw;
	int             jt_temp_h;
	int             jt_temp_l;
	int             jt_vfchg_h;
	int             jt_vfchg_l;
	int             jt_ichg_h;
	int             jt_ichg_l;
	uint8_t         adp_current_val;
	uint8_t         usb_current_val;

	int             num;
};

//struct ricoh61x_battery_info *info;




struct ricoh61x_soca_info {
	int Rbat;
	int n_cap;
	int ocv_table_def[11];
	int ocv_table[11];
	int ocv_table_low[11];
	int soc;                /* Latest FG SOC value */
	int displayed_soc;
	int suspend_soc;
	int status;             /* SOCA status 0: Not initial; 5: Finished */
	int stable_count;
	int chg_status;         /* chg_status */
	int soc_delta;          /* soc delta for status3(DISP) */
	int cc_delta;
	int cc_cap_offset;
	int last_soc;
	int last_displayed_soc;
	int ready_fg;
	int reset_count;
	int reset_soc[3];
	int chg_cmp_times;
	int dischg_state;
	int Vbat[RICOH61x_GET_CHARGE_NUM];
	int Vsys[RICOH61x_GET_CHARGE_NUM];
	int Ibat[RICOH61x_GET_CHARGE_NUM];
	int Vbat_ave;
	int Vbat_old;
	int Vsys_ave;
	int Ibat_ave;
	int chg_count;
	int full_reset_count;
	int soc_full;
	int fc_cap;
	/* for LOW VOL state */
	int target_use_cap;
	int hurry_up_flg;
	int zero_flg;
	int re_cap_old;
	int cutoff_ocv;
	int Rsys;
	int target_vsys;
	int target_ibat;
	int jt_limit;
	int OCV100_min;
	int OCV100_max;
	int R_low;
	int rsoc_ready_flag;
	int init_pswr;
	int last_cc_sum;
};

enum int_type {
	SYS_INT  = 0x01,
	DCDC_INT = 0x02,
	ADC_INT  = 0x08,
	GPIO_INT = 0x10,
	CHG_INT  = 0x40,
};


struct ricoh61x_regulator {
	int		id;
	int		sleep_id;
	/* Regulator register address.*/
	u8		reg_en_reg;
	u8		en_bit;
	u8		reg_disc_reg;
	u8		disc_bit;
	u8		vout_reg;
	u8		vout_mask;
	u8		vout_reg_cache;
	u8		sleep_reg;
	u8		eco_reg;
	u8		eco_bit;
	u8		eco_slp_reg;
	u8		eco_slp_bit;

	/* chip constraints on regulator behavior */
	int			min_uV;
	int			max_uV;
	int			step_uV;
	int			nsteps;

	/* regulator specific turn-on delay */
	u16			delay;

	/* used by regulator core */
	struct regulator	desc;

	/* Device */
	struct device		*dev;
};

enum regulator_type {
	REGULATOR_VOLTAGE,
	REGULATOR_CURRENT,
};
#endif

static int ricoh61x_write_reg(u8 reg, u8 *val)
{
	unsigned int  ret;

	ret = i2c_write(RICOH61x_I2C_ADDR, reg, 1, val, 1);
	if(ret) {
		debug("ricoh61x write register error\n");
		return -EIO;
	}
	return 0;
}

#ifndef CONFIG_SPL_BUILD
static int ricoh61x_read_reg(u8 reg, u8 *val, u32 len)
{
	int ret;
	ret = i2c_read(RICOH61x_I2C_ADDR, reg, 1, val, len);
	if(ret) {
		printf("ricoh61x read register error\n");
		return -EIO;
	}
	return 0;
}
void *rdev_get_drvdata(struct regulator *rdev)
{
	    return rdev->reg_data;
}

int rdev_set_drvdata(struct regulator *rdev, void *data)
{

	rdev->reg_data = data;
	return 0;
}

int ricoh61x_set_bits(u8 reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;
	ret = set_bank_ricoh61x(0);

	if (!ret) {
		ret = ricoh61x_read_reg(reg, &reg_val, 1);
		if (ret)
			goto out;

		if ((reg_val & bit_mask) != bit_mask) {
			reg_val |= bit_mask;
			ret = ricoh61x_write_reg(reg, &reg_val);
		}
	}
out:
	return ret;
}

int ricoh61x_clr_bits(u8 reg, uint8_t bit_mask)
{

	uint8_t reg_val;
	int ret = 0;

	ret = set_bank_ricoh61x(0);
	if (!ret) {
		ret = ricoh61x_read_reg(reg, &reg_val, 1);
		if (ret)
			goto out;

		if (reg_val & bit_mask) {
			reg_val &= ~bit_mask;
			ret = ricoh61x_write_reg(reg, &reg_val);
		}
	}
out:
	return ret;

}




static int ricoh61x_reg_enable(struct regulator *rdev)
{
	struct ricoh61x_regulator *ri = rdev_get_drvdata(rdev);
	u8 value;
	int ret;

	ret = ricoh61x_set_bits(ri->reg_en_reg, (1 << ri->en_bit));
	if(ret){
		printf("ricoh61x set bit is error\n");
	}
	return ret;
}

static int ricoh61x_reg_disable(struct regulator *rdev)
{
	struct ricoh61x_regulator *ri = rdev_get_drvdata(rdev);
	u8 value;
	int ret;

	ret = ricoh61x_clr_bits(ri->reg_en_reg, (1 << ri->en_bit));
	if(ret){
		printf("ricoh61x set clr is error\n");
	}

return ret;
}


static int __ricoh61x_set_voltage( struct ricoh61x_regulator *ri,
		int min_uV, int max_uV, unsigned *selector)
{
	int vsel;
	int ret;
	uint8_t vout_val;

	if ((min_uV < ri->min_uV) || (max_uV > ri->max_uV))
		return -EDOM;

	vsel = (min_uV - ri->min_uV + ri->step_uV - 1)/ri->step_uV;
	if (vsel > ri->nsteps)
		return -EDOM;

	if (selector)
		*selector = vsel;

	vout_val =  (vsel & ri->vout_mask);
	ret = ricoh61x_write_reg(ri->vout_reg, &vout_val);
	if (ret < 0)
		printf("Error in writing the Voltage register\n");

	return ret;
}

static  int ricoh61x_set_voltage(struct regulator *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct ricoh61x_regulator *ri = rdev_get_drvdata(rdev);

	return __ricoh61x_set_voltage(ri, min_uV, max_uV, selector);
}

void test_richo()
{
	int val =0;
//	spl_ricoh61x_regulator_set_voltage(RICOH619_LDO4, 1200);
	ricoh61x_read_reg(0x2c, &val, 1);
	ricoh61x_read_reg(0xbd, &val, 1);

//	ricoh61x_read_reg(u8 reg, u8 *val, u32 len);
}

static int ricoh61x_reg_is_enabled(struct regulator *regulator)
{
	uint8_t control;
	struct ricoh61x_regulator *ri = rdev_get_drvdata(regulator);

	ricoh61x_read_reg(ri->reg_en_reg, &control,1);
	return (((control >> ri->en_bit) & 1) == 1);
}

static struct regulator_ops ricoh61x_ops = {

	.disable = ricoh61x_reg_disable,
	.enable = ricoh61x_reg_enable,
	.set_voltage = ricoh61x_set_voltage,
	.is_enabled	= ricoh61x_reg_is_enabled,

};

#define RICOH61x_REG(_id, _en_reg, _en_bit, _disc_reg, _disc_bit, _vout_reg, \
		_vout_mask, _ds_reg, _min_mv, _max_mv, _step_uV, _nsteps,    \
		_ops, _delay, _eco_reg, _eco_bit, _eco_slp_reg, _eco_slp_bit) \
{								\
	.reg_en_reg	= _en_reg,				\
	.en_bit		= _en_bit,				\
	.reg_disc_reg	= _disc_reg,				\
	.disc_bit	= _disc_bit,				\
	.vout_reg	= _vout_reg,				\
	.vout_mask	= _vout_mask,				\
	.sleep_reg	= _ds_reg,				\
	.step_uV	= _step_uV,				\
	.nsteps		= _nsteps,				\
	.delay		= _delay,				\
	.id		= RICOH619_ID_##_id,			\
	.sleep_id	= RICOH619_DS_##_id,			\
	.eco_reg	=  _eco_reg,				\
	.eco_bit	=  _eco_bit,				\
	.min_uV		= _min_mv * 1000,			\
	.max_uV		= _max_mv * 1000,			\
	.eco_slp_reg	=  _eco_slp_reg,			\
	.eco_slp_bit	=  _eco_slp_bit,			\
	.desc = {						\
		.name = ricoh619_rails(_id),			\
		.id = RICOH619_ID_##_id,			\
		.min_uV		= _min_mv * 1000,			\
		.max_uV		= _max_mv * 1000,			\
		.n_voltages = _nsteps,				\
		.ops = &_ops,					\
		},							\
}

static struct ricoh61x_regulator ricoh61x_regulator[] = {
	RICOH61x_REG(DC1, 0x2C, 0, 0x2C, 1, 0x36, 0xFF, 0x3B,
			600, 3500, 12500, 0xE8, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(DC2, 0x2E, 0, 0x2E, 1, 0x37, 0xFF, 0x3C,
			600, 3500, 12500, 0xE8, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(DC3, 0x30, 0, 0x30, 1, 0x38, 0xFF, 0x3D,
			600, 3500, 12500, 0xE8, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(DC4, 0x32, 0, 0x32, 1, 0x39, 0xFF, 0x3E,
			600, 3500, 12500, 0xE8, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(DC5, 0x34, 0, 0x34, 1, 0x3A, 0xFF, 0x3F,
			600, 3500, 12500, 0xE8, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(LDO1, 0x44, 0, 0x46, 0, 0x4C, 0x7F, 0x58,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x48, 0, 0x4A, 0),

	RICOH61x_REG(LDO2, 0x44, 1, 0x46, 1, 0x4D, 0x7F, 0x59,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x48, 1, 0x4A, 1),

	RICOH61x_REG(LDO3, 0x44, 2, 0x46, 2, 0x4E, 0x7F, 0x5A,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x48, 2, 0x4A, 2),

	RICOH61x_REG(LDO4, 0x44, 3, 0x46, 3, 0x4F, 0x7F, 0x5B,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x48, 3, 0x4A, 3),

	RICOH61x_REG(LDO5, 0x44, 4, 0x46, 4, 0x50, 0x7F, 0x5C,
			600, 3500, 25000, 0x74, ricoh61x_ops, 500,
			0x48, 4, 0x4A, 4),

	RICOH61x_REG(LDO6, 0x44, 5, 0x46, 5, 0x51, 0x7F, 0x5D,
			600, 3500, 25000, 0x74, ricoh61x_ops, 500,
			0x48, 5, 0x4A, 5),

	RICOH61x_REG(LDO7, 0x44, 6, 0x46, 6, 0x52, 0x7F, 0x5E,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(LDO8, 0x44, 7, 0x46, 7, 0x53, 0x7F, 0x5F,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(LDO9, 0x45, 0, 0x47, 0, 0x54, 0x7F, 0x60,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(LDO10, 0x45, 1, 0x47, 1, 0x55, 0x7F, 0x61,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),


	RICOH61x_REG(LDORTC1, 0x45, 4, 0x00, 0, 0x56, 0x7F, 0x00,
			1700, 3500, 25000, 0x48, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),

	RICOH61x_REG(LDORTC2, 0x45, 5, 0x00, 0, 0x57, 0x7F, 0x00,
			900, 3500, 25000, 0x68, ricoh61x_ops, 500,
			0x00, 0, 0x00, 0),
};


static int get_OCV_init_Data(struct ricoh61x_battery_info *info, int index)
{
	int ret = 0;
	ret =  (battery_init_para[info->num][index*2]<<8) | (battery_init_para[info->num][index*2+1]);
	return ret;
}

int __ricoh61x_write(u8 reg, int val)
{
	int ret;
	u8 val1 = val;

	ret = ricoh61x_write_reg(reg, &val1);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int set_bank_ricoh61x(int bank)
{
	int ret;

	if (bank != (bank & 1))
		return -EINVAL;
	if (bank == bank_num)
		return 0;
	ret = __ricoh61x_write(RICOH61x_REG_BANKSEL, bank);
	if (!ret)
		bank_num = bank;

	return ret;
}

int ricoh61x_write(u8 reg, uint8_t *val)
{
	int ret = 0;

	ret = set_bank_ricoh61x(0);
	if (!ret)
		ret = __ricoh61x_write(reg, val);

	return ret;
}


int ricoh61x_write_bank1(u8 reg, uint8_t *val)
{       
	int ret = 0;

	ret = set_bank_ricoh61x(1);
	if (!ret)
		ret = __ricoh61x_write(reg, val);

	return ret;
}


int __ricoh61x_read(u8 reg, uint8_t *val)
{
	int ret;
	ret = ricoh61x_read_reg(reg, val, 1);
	//        ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		printf("failed reading at 0x%02x\n", reg);
		return ret;
	}
	return 0;
}

int ricoh61x_read_bank1(u8 reg, uint8_t *val)
{
	int ret = 0;
	ret = set_bank_ricoh61x(1);
	if (!ret)
		ret =  __ricoh61x_read(reg, val);

	return ret;
}



int ricoh61x_read(u8 reg, uint8_t *val)
{
	int ret = 0;

	ret = set_bank_ricoh61x(0);
	if (!ret)
		ret = __ricoh61x_read(reg, val);

	return ret;
}

static int get_OCV_voltage(struct ricoh61x_battery_info *info, int index)
{
	int ret = 0;
	ret =  get_OCV_init_Data(info, index);
	/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
	ret = ret * 50000 / 4095;
	/* return unit should be 1uV */
	ret = ret * 100;
	return ret;
}

static void ricoh61x_scaling_OCV_table(struct ricoh61x_battery_info *info, int cutoff_vol, int full_vol, int *start_per, int *end_per)
{
	int		i, j;
	int		temp;
	int		percent_step;
	int		OCV_percent_new[11];


	/* Check Start % */
	if (info->soca->ocv_table_def[0] > cutoff_vol * 1000) {
		*start_per = 0;
	} else {
		for (i = 1; i < 11; i++) {
			if (info->soca->ocv_table_def[i] >= cutoff_vol * 1000) {
				/* unit is 0.001% */
				*start_per = Calc_Linear_Interpolation(
						(i-1)*1000, info->soca->ocv_table_def[i-1], i*1000,
						info->soca->ocv_table_def[i], (cutoff_vol * 1000));
				break;
			}
		}
	}

	/* Check End % */
	for (i = 1; i < 11; i++) {
		if (info->soca->ocv_table_def[i] >= full_vol * 1000) {
			/* unit is 0.001% */
			*end_per = Calc_Linear_Interpolation(
					(i-1)*1000, info->soca->ocv_table_def[i-1], i*1000,
					info->soca->ocv_table_def[i], (full_vol * 1000));
			break;
		}
	}

	/* calc new ocv percent */
	percent_step = ( *end_per - *start_per) / 10;
	//pr_info("PMU : %s : percent_step is %d end per is %d start per is %d\n",__func__, percent_step, *end_per, *start_per);

	for (i = 0; i < 11; i++) {
		OCV_percent_new[i]
		                = *start_per + percent_step*(i - 0);
	}

	/* calc new ocv voltage */
	for (i = 0; i < 11; i++) {
		for (j = 1; j < 11; j++) {
			if (1000*j >= OCV_percent_new[i]) {
				temp = Calc_Linear_Interpolation(
						info->soca->ocv_table_def[j-1], (j-1)*1000,
						info->soca->ocv_table_def[j] , j*1000,
						OCV_percent_new[i]);

				temp = ( (temp/1000) * 4095 ) / 5000;

				battery_init_para[info->num][i*2 + 1] = temp;
				battery_init_para[info->num][i*2] = temp >> 8;

				break;
			}
		}
	}
	for (i = 0; i <= 10; i = i+1) {
		temp = (battery_init_para[info->num][i*2]<<8)
					 | (battery_init_para[info->num][i*2+1]);
		/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
		temp = ((temp * 50000 * 10 / 4095) + 5) / 10;
	}

}

static inline int __ricoh61x_bulk_writes(u8 reg, int len, uint8_t *val)
{       
	int ret;
	int i;

	for (i = 0; i < len; ++i) {
		printf("ricoh61x: reg write  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}

	ret = i2c_write(RICOH61x_I2C_ADDR, reg, 1 , val, len);
	//        ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		printf("failed writings to 0x%02x\n", reg);
		return ret;
	}

	return 0;
} 

int ricoh61x_bulk_writes(u8 reg, u8 len, uint8_t *val)
{
	int ret = 0;
	ret = set_bank_ricoh61x(0);
	if (!ret)
		ret = __ricoh61x_bulk_writes(reg, len, val);

	return ret;
}


int ricoh61x_bulk_writes_bank1(u8 reg, u8 len, uint8_t *val)
{
	int ret = 0;

	ret = set_bank_ricoh61x(1);
	if (!ret)
		ret = __ricoh61x_bulk_writes(reg, len, val);

	return ret;
}


static int ricoh61x_set_OCV_table(struct ricoh61x_battery_info *info)
{
	int		ret = 0;
	int		i;
	int		full_ocv;
	int		available_cap;
	int		available_cap_ori;
	int		temp;
	int		temp1;
	int		start_per = 0;
	int		end_per = 0;
	int		Rbat;
	int		Ibat_min;
	uint8_t val;
	uint8_t val2;
	uint8_t val_temp;


	//get ocv table
	for (i = 0; i <= 10; i = i+1) {
		info->soca->ocv_table_def[i] = get_OCV_voltage(info, i);
	}

	temp =  (battery_init_para[info->num][24]<<8) | (battery_init_para[info->num][25]);
	Rbat = temp * 1000 / 512 * 5000 / 4095;
	info->soca->Rsys = Rbat + 55;

	if ((info->fg_target_ibat == 0) || (info->fg_target_vsys == 0)) {	/* normal version */

		temp =  (battery_init_para[info->num][22]<<8) | (battery_init_para[info->num][23]);
		//fa_cap = get_check_fuel_gauge_reg(info, FA_CAP_H_REG, FA_CAP_L_REG,
		//				0x7fff);

		info->soca->target_ibat = temp*2/10; /* calc 0.2C*/
		temp1 =  (battery_init_para[info->num][0]<<8) | (battery_init_para[info->num][1]);
		//		temp = get_OCV_voltage(info, 0) / 1000; /* unit is 1mv*/
		//		info->soca->cutoff_ocv = info->soca->target_vsys - Ibat_min * info->soca->Rsys / 1000;

		info->soca->target_vsys = temp1 + ( info->soca->target_ibat * info->soca->Rsys ) / 1000;


	} else {
		info->soca->target_ibat = info->fg_target_ibat;
		/* calc min vsys value */
		temp1 =  (battery_init_para[info->num][0]<<8) | (battery_init_para[info->num][1]);
		temp = temp1 + ( info->soca->target_ibat * info->soca->Rsys ) / 1000;
		if( temp < info->fg_target_vsys) {
			info->soca->target_vsys = info->fg_target_vsys;
		} else {
			info->soca->target_vsys = temp;
			printf("PMU : %s : setting value of target vsys(%d) is out of range(%d)\n",__func__, info->fg_target_vsys, temp);
		}
	}

	//for debug
	//pr_info("PMU : %s : target_vsys is %d target_ibat is %d\n",__func__,info->soca->target_vsys,info->soca->target_ibat * 20 / info->fg_rsense_val);

	if ((info->soca->target_ibat == 0) || (info->soca->target_vsys == 0)) {	/* normal version */
	} else {	/*Slice cutoff voltage version. */

		Ibat_min = -1 * info->soca->target_ibat;
		info->soca->cutoff_ocv = info->soca->target_vsys - Ibat_min * info->soca->Rsys / 1000;

		full_ocv = (battery_init_para[info->num][20]<<8) | (battery_init_para[info->num][21]);
		full_ocv = full_ocv * 5000 / 4095;

		ricoh61x_scaling_OCV_table(info, info->soca->cutoff_ocv, full_ocv, &start_per, &end_per);

		/* calc available capacity */
		/* get avilable capacity */
		/* battery_init_para23-24 is designe capacity */
		available_cap = (battery_init_para[info->num][22]<<8)
							 | (battery_init_para[info->num][23]);

		available_cap = available_cap
				* ((10000 - start_per) / 100) / 100 ;


		battery_init_para[info->num][23] =  available_cap;
		battery_init_para[info->num][22] =  available_cap >> 8;

	}
	ret = ricoh61x_clr_bits(FG_CTRL_REG, 0x01);
	if (ret < 0) {
		printf("error in FG_En off\n");
		goto err;
	}
	/////////////////////////////////
	ret = ricoh61x_read_bank1(0xDC, &val);
	if (ret < 0) {
		printf("batterry initialize error\n");
		goto err;
	}

	val_temp = val;
	val	&= 0x0F; //clear bit 4-7
	val	|= 0x10; //set bit 4

	ret = ricoh61x_write_bank1(0xDC, &val);
	if (ret < 0) {
		printf("batterry initialize error\n");
		goto err;
	}

	ret = ricoh61x_read_bank1(0xDC, &val2);
	if (ret < 0) {
		printf("batterry initialize error\n");
		goto err;
	}

	ret = ricoh61x_write_bank1(0xDC, &val_temp);
	if (ret < 0) {
		printf("batterry initialize error\n");
		goto err;
	}


	if (val != val2) {
		ret = ricoh61x_bulk_writes_bank1(BAT_INIT_TOP_REG, 30, &battery_init_para[info->num]);
		if (ret < 0) {
			printf("batterry initialize error\n");
			goto err;
		}
	} else {
		ret = ricoh61x_read_bank1(0xD2, &val);
		if (ret < 0) {
			printf("batterry initialize error\n");
			goto err;
		}

		ret = ricoh61x_read_bank1(0xD3, &val2);
		if (ret < 0) {
			printf("batterry initialize error\n");
			goto err;
		}

		available_cap_ori = val2 + (val << 8);
		available_cap = battery_init_para[info->num][23]
		                                             + (battery_init_para[info->num][22] << 8);

		if (available_cap_ori == available_cap) {
			ret = ricoh61x_bulk_writes_bank1(BAT_INIT_TOP_REG, 22, battery_init_para[info->num]);
			if (ret < 0) {
				printf("batterry initialize error\n");
				return ret;
			}

			for (i = 0; i < 6; i++) {
				ret = ricoh61x_write_bank1(0xD4+i, battery_init_para[info->num][24+i]);
				if (ret < 0) {
					printf("batterry initialize error\n");
					return ret;
				}
			}
		} else {
			ret = ricoh61x_bulk_writes_bank1(BAT_INIT_TOP_REG, 30, battery_init_para[info->num]);
			if (ret < 0) {
				printf("batterry initialize error\n");
				goto err;
			}
		}
	}
	////////////////////////////////

	return 0;
err:
	return ret;
}


/* Initial setting of battery */
static int ricoh61x_init_battery(struct ricoh61x_battery_info *info)
{
	int ret = 0;
	uint8_t val;
	uint8_t val2;
	uint8_t jk;
	int temp1 = 0;
	/* Need to implement initial setting of batery and error */
	/* -------------------------- */
#ifdef ENABLE_FUEL_GAUGE_FUNCTION

	/* set relaxation state */
	if (RICOH61x_REL1_SEL_VALUE > 240)
		val = 0x0F;
	else
		val = RICOH61x_REL1_SEL_VALUE / 16 ;

	/* set relaxation state */
	if (RICOH61x_REL2_SEL_VALUE > 120)
		val2 = 0x0F;
	else
		val2 = RICOH61x_REL2_SEL_VALUE / 8 ;

	val =  val + (val2 << 4);

	ret = ricoh61x_write_bank1(BAT_REL_SEL_REG, &val);
	if (ret < 0) {
		printf("Error in writing BAT_REL_SEL_REG\n");
		return ret;
	}

	ret = ricoh61x_read_bank1(BAT_REL_SEL_REG, &val);
	//pr_info("PMU: -------  BAT_REL_SEL= %xh: =======\n",
	//      val);

	jk = 0;
	ret = ricoh61x_write_bank1(BAT_TA_SEL_REG, &jk);
	if (ret < 0) {
		printf("Error in writing BAT_TA_SEL_REG\n");
		return ret;
	}
	//      ret = ricoh61x_read(info->dev->parent, FG_CTRL_REG, &val);
	//      if (ret < 0) {
	//              printf(  "Error in reading the control register\n");
	//              return ret;
	//      }

	//      val = (val & 0x10) >> 4;
	//      info->first_pwon = (val == 0) ? 1 : 0;
	ret = ricoh61x_read(PSWR_REG, &val);
	if (ret < 0) {
		printf("Error in reading PSWR_REG %d\n", ret);
		return ret;
	}
	info->first_pwon = (val == 0) ? 1 : 0;
	//info->first_pwon = 1;
	g_soc = val & 0x7f;

	info->soca->init_pswr = val & 0x7f;
	printf("PMU FG_RESET : %s : initial pswr = %d\n",__func__,info->soca->init_pswr);

	if(info->first_pwon) {
		info->soca->rsoc_ready_flag = 1;
	}else {
		info->soca->rsoc_ready_flag = 0;
	}

	ret = ricoh61x_set_OCV_table(info);
	if (ret < 0) {
		printf("Error in writing the OCV Tabler\n");
		return ret;
	}

	jk = 0x11;

	ret = ricoh61x_write(FG_CTRL_REG, &jk);
	if (ret < 0) {
		printf("error in %d FG_CTRL_REG \n", __LINE__);
		printf("Error in writing the control register\n");
		return ret;
	}

#endif
	if (ret < 0) {
		printf("error in %d VINDAC_REG \n", __LINE__);
		printf("Error in writing the control register\n");
		return ret;
	}

	if (info->alarm_vol_mv < 2700 || info->alarm_vol_mv > 3400) {
		printf("alarm_vol_mv is out of range!\n");
		return -1;
	}
	return ret;
}

/**/
int cmd_get_check_fuel_gauge_reg(int Reg_h, int Reg_l, int enable_bit)
{
	uint8_t get_data_h, get_data_l;
	int old_data, current_data;
	int i;
	int ret = 0;
	int val;

	old_data = 0;

	for (i = 0; i < 5 ; i++) {
		ret = ricoh61x_read(0xEB, &get_data_h);
		if (ret < 0) {
			printf("H Error in reading the control register\n");
			return ret;
		}

		ret = ricoh61x_read(0xEC, &get_data_l);

		if (ret < 0) {
			printf("L Error in reading the control register\n");
			return ret;
		}

		current_data = ((get_data_h & 0xff) << 8) | (get_data_l & 0xff);
		current_data = (current_data & enable_bit);

		if (current_data == old_data){
			return current_data;
		}else{
			old_data = current_data;
		}
	}
	return current_data;
}
/**/


int get_check_fuel_gauge_reg(struct ricoh61x_battery_info *info,
		int Reg_h, int Reg_l, int enable_bit)
{
	uint8_t get_data_h, get_data_l;
	int old_data, current_data;
	int i;
	int ret = 0;
	uint8_t val;

	old_data = 0;

	for (i = 0; i < 5 ; i++) {
		ret = ricoh61x_read_reg(0xEB, &get_data_h, 1);
		if (ret < 0) {
			printf("H Error in reading the control register\n");
			return ret;
		}

		ret = ricoh61x_read_reg(0xEC, &get_data_l, 1);

		if (ret < 0) {
			printf("L Error in reading the control register\n");
			return ret;
		}

		current_data = ((get_data_h & 0xff) << 8) | (get_data_l & 0xff);
		current_data = (current_data & enable_bit);

		if (current_data == old_data){
			return current_data;
		}else{
			old_data = current_data;
		}
	}
	return current_data;
}
/******************/

int cmd_measure_vbatt_FG()
{
	int vol = 0, volup = 0;
	int ret = 0;

	if(1) {
		vol = cmd_get_check_fuel_gauge_reg(VOLTAGE_1_REG, VOLTAGE_2_REG, 0x0fff);
		if (ret < 0) {
			printf("Error in reading the fuel gauge control register\n");
			return ret;
		}
		volup = vol;
		/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
		volup = volup * 50000 / 4095;
		/* return unit should be 1uV */
		volup = volup * 100;
		//	info->soca->Vbat_old = volup;
	} else {
		//	volup = info->soca->Vbat_old;
	}

	return volup;
}


/***/

int cmd_measure_vsys_ADC()
{
	uint8_t data_l = 0, data_h = 0;
	int ret;
	int current_vsys = 1;

	ret = ricoh61x_read(VSYSDATAH_REG, &data_h);
	if (ret < 0) {
		printf("Error in reading the control register\n");
	}

	ret = ricoh61x_read(VSYSDATAL_REG, &data_l);
	if (ret < 0) {
		printf("Error in reading the control register\n");
	}

	current_vsys = ((data_h & 0xff) << 4) | (data_l & 0x0f);
	current_vsys = current_vsys * 1000 * 3 * 5 / 2 / 4095;
	/* return unit should be 1uV */
	current_vsys = current_vsys * 1000;
	return current_vsys;
}


int detection_first_poweron()
{
	uint8_t first_poweron;
	int ret;
	 ret = ricoh61x_read(PSWR_REG, &first_poweron);
	if(first_poweron == 0){
		return 0;
	}else{
		return 1;
	}
	
}

/* battery voltage is get from Fuel gauge */
int measure_vbatt_FG(struct ricoh61x_battery_info *info, int *data)
{
	int ret = 0;

	info->soca->ready_fg = 1;

	if(info->soca->ready_fg == 1) {
		ret = get_check_fuel_gauge_reg(info, VOLTAGE_1_REG, VOLTAGE_2_REG,
				0x0fff);
		if (ret < 0) {
			printf("Error in reading the fuel gauge control register\n");
			return ret;
		}
		(*data) = ret;
		/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
		*data = (*data) * 50000 / 4095;
		/* return unit should be 1uV */
		*data = *data * 100;
		info->soca->Vbat_old = *data;
	} else {
		*data = info->soca->Vbat_old;
	}

	return ret;
}

static int measure_Ibatt_FG(struct ricoh61x_battery_info *info, int *data)
{
	int ret = 0;

	ret =  get_check_fuel_gauge_reg(info, CC_AVERAGE1_REG,
			CC_AVERAGE0_REG, 0x3fff);
	if (ret < 0) {
		printf("Error in reading the fuel gauge control register\n");
		return ret;
	}

	*data = (ret > 0x1fff) ? (ret - 0x4000) : ret;
	return ret;
}

int ricoh61x_bulk_reads(u8 reg, u8 len, uint8_t *val)
{
	int ret = 0;

	ret = set_bank_ricoh61x(0);
	if (!ret)
		ret = ricoh61x_read_reg(reg, val, len);

	return ret;
}



static int calc_ocv(struct ricoh61x_battery_info *info)
{
	int Ibat = 0;
	int Vbat = 0;
	int ret;
	int ocv;

	ret = measure_vbatt_FG(info, &Vbat);
	ret = measure_Ibatt_FG(info, &Ibat);
	ocv = Vbat - Ibat * info->soca->Rbat;

	return ocv;
}


static int calc_capacity_in_period(struct ricoh61x_battery_info *info,
		int *cc_cap, bool *is_charging, bool cc_rst)
{
	int err;
	uint8_t 	cc_sum_reg[4];
	uint8_t 	cc_clr[4] = {0, 0, 0, 0};
	uint8_t 	fa_cap_reg[2];
	uint16_t 	fa_cap;
	uint32_t 	cc_sum;
	int		cc_stop_flag;
	uint8_t 	status;
	uint8_t 	charge_state;
	int 		Ocv;
	uint32_t 	cc_cap_temp;
	uint32_t 	cc_cap_min;
	int		cc_cap_res;
	uint8_t 	jk;

	*is_charging = true;	/* currrent state initialize -> charging */

	if (info->entry_factory_mode)
		return 0;

	/* get  power supply status */
	err = ricoh61x_read(CHGSTATE_REG, &status);
	if (err < 0)
		goto out;
	charge_state = (status & 0x1F);
	Ocv = calc_ocv(info);
	if (charge_state == CHG_STATE_CHG_COMPLETE) {
		/* Check CHG status is complete or not */
		cc_stop_flag = 0;
		//	} else if (calc_capacity(info) == 100) {
		//		/* Check HW soc is 100 or not */
		//		cc_stop_flag = 0;
	} else if (Ocv < get_OCV_voltage(info, 9)) {
		/* Check VBAT is high level or not */
		cc_stop_flag = 0;
	} else {
		cc_stop_flag = 1;
	}

	if (cc_stop_flag == 1)
	{
		/* Disable Charging/Completion Interrupt */
		err = ricoh61x_set_bits(RICOH61x_INT_MSK_CHGSTS1, 0x01);
		if (err < 0)
			goto out;

		/* disable charging */
		err = ricoh61x_clr_bits(RICOH61x_CHG_CTL1, 0x03);
		if (err < 0)
			goto out;
	}

	/* CC_pause enter */
	jk = 0x01;
	err = ricoh61x_write(CC_CTRL_REG, &jk);
	if (err < 0)
		goto out;

	/* Read CC_SUM */
	err = ricoh61x_bulk_reads(CC_SUMREG3_REG, 4, cc_sum_reg);
	if (err < 0)
		goto out;

	if (cc_rst == true) {
		/* CC_SUM <- 0 */
		err = ricoh61x_bulk_writes(CC_SUMREG3_REG, 4, cc_clr);
		if (err < 0)
			goto out;
	}

	/* CC_pause exist */
	jk = 0;
	err = ricoh61x_write_reg(CC_CTRL_REG, &jk);
	if (err < 0)
		goto out;
	if (cc_stop_flag == 1)
	{

		/* Enable charging */
		err = ricoh61x_set_bits(RICOH61x_CHG_CTL1, 0x03);
		if (err < 0)
			goto out;

		udelay(1000);

		/* Clear Charging Interrupt status */
		err = ricoh61x_clr_bits(RICOH61x_INT_IR_CHGSTS1, 0x01);
		if (err < 0)
			goto out;

		/* ricoh61x_read(info->dev->parent, RICOH61x_INT_IR_CHGSTS1, &val);
//		pr_info("INT_IR_CHGSTS1 = 0x%x\n",val); */

		/* Enable Charging Interrupt */
		err = ricoh61x_clr_bits(RICOH61x_INT_MSK_CHGSTS1, 0x01);
		if (err < 0)
			goto out;
	}
	/* Read FA_CAP */
	err = ricoh61x_bulk_reads(FA_CAP_H_REG, 2, fa_cap_reg);
	if (err < 0)
		goto out;

	/* fa_cap = *(uint16_t*)fa_cap_reg & 0x7fff; */
	fa_cap = ((fa_cap_reg[0] << 8 | fa_cap_reg[1]) & 0x7fff);

	/* cc_sum = *(uint32_t*)cc_sum_reg; */
	cc_sum = cc_sum_reg[0] << 24 | cc_sum_reg[1] << 16 |
			cc_sum_reg[2] << 8 | cc_sum_reg[3];

	/* calculation  two's complement of CC_SUM */
	if (cc_sum & 0x80000000) {
		cc_sum = (cc_sum^0xffffffff)+0x01;
		*is_charging = false;		/* discharge */
	}
	/* (CC_SUM x 10000)/3600/FA_CAP */

	if(fa_cap == 0)
		goto out;
	else
		*cc_cap = cc_sum*25/9/fa_cap;		/* unit is 0.01% */

	//////////////////////////////////////////////////////////////////
	cc_cap_min = fa_cap*3600/100/100/100;	/* Unit is 0.0001% */

	if(cc_cap_min == 0)
		goto out;
	else
		cc_cap_temp = cc_sum / cc_cap_min;

	cc_cap_res = cc_cap_temp % 100;

	//printf("PMU: cc_sum = %d: cc_cap_res= %d: \n", cc_sum, cc_cap_res);


	if(*is_charging) {
		info->soca->cc_cap_offset += cc_cap_res;
		if (info->soca->cc_cap_offset >= 100) {
			*cc_cap += 1;
			info->soca->cc_cap_offset %= 100;
		}
	} else {
		info->soca->cc_cap_offset -= cc_cap_res;
		if (info->soca->cc_cap_offset <= -100) {
			*cc_cap += 1;
			info->soca->cc_cap_offset %= 100;
		}
	}
	//printf("PMU: cc_cap_offset= %d: \n", info->soca->cc_cap_offset);

	//////////////////////////////////////////////////////////////////
	return 0;
out:
	return err;
}

int ricoh61x_get_capacity()
{
	uint8_t capacity;
	
	ricoh61x_read(SOC_REG, &capacity);
	printf("ricoh61x_get_capacity: capacity = %d\n", capacity);
	if(capacity <= CAPA_MIN){
		printf("Is a dead battery, system will power off! \n");
		return 1;
	}else if(capacity > 5 && capacity <= 100){
		return 0;
	}else{
		printf("The battery detection is bad !!!!!!!!\n");
		return 1;
	}
}

int ricoh61x_get_capacity1()
{
	uint8_t capacity;
	
	ricoh61x_read(SOC_REG, &capacity);
	return capacity;	
}


/* Initial setting of FuelGauge SOCA function */
static int ricoh61x_init_fgsoca(struct ricoh61x_battery_info *info)
{
	int i;
	int err;
	uint8_t val;
	int cc_cap = 0;
	bool is_charging = true;
	int jk;

	for (i = 0; i <= 10; i = i+1) {
		info->soca->ocv_table[i] = get_OCV_voltage(info, i);
		//		printf("PMU: %s : * %d0%% voltage = %d uV\n",
		//				 __func__, i, info->soca->ocv_table[i]);
	}

	for (i = 0; i < 3; i = i+1)
		info->soca->reset_soc[i] = 0;
	info->soca->reset_count = 0;


	if (info->first_pwon) {

		err = ricoh61x_read(CHGISET_REG, &val);
		if (err < 0)
			printf("Error in read CHGISET_REG%d\n", err);
		jk = 0;
		err = ricoh61x_write(CHGISET_REG, &jk);
		if (err < 0)
			printf("Error in writing CHGISET_REG%d\n", err);
		/* msleep(1000); */
		if (!info->entry_factory_mode) {
			jk = 0x51;
			err = ricoh61x_write(FG_CTRL_REG, &jk);
			if (err < 0)
				printf("Error in writing the control register\n");
		}

		info->soca->rsoc_ready_flag = 1;

		err = calc_capacity_in_period(info, &cc_cap, &is_charging, true);

		/* msleep(6000); */

		err = ricoh61x_write(CHGISET_REG, &val);
		if (err < 0)
			printf("Error in writing CHGISET_REG%d\n", err);
	}

	/* Rbat : Transfer */

	info->soca->Rbat = get_OCV_init_Data(info, 12) * 1000 / 512
			* 5000 / 4095;
	info->soca->n_cap = get_OCV_init_Data(info, 11);


	info->soca->displayed_soc = 0;
	info->soca->last_displayed_soc = 0;
	info->soca->suspend_soc = 0;
	info->soca->ready_fg = 0;
	info->soca->soc_delta = 0;
	info->soca->full_reset_count = 0;
	info->soca->soc_full = 0;
	info->soca->fc_cap = 0;
	info->soca->status = RICOH61x_SOCA_START;
	/* stable count down 11->2, 1: reset; 0: Finished; */
	info->soca->stable_count = 11;
	info->soca->chg_cmp_times = 0;
	info->soca->dischg_state = 0;
	info->soca->Vbat_ave = 0;
	info->soca->Vbat_old = 0;
	info->soca->Vsys_ave = 0;
	info->soca->Ibat_ave = 0;
	info->soca->chg_count = 0;
	info->soca->target_use_cap = 0;
	info->soca->hurry_up_flg = 0;
	info->soca->re_cap_old = 0;
	info->soca->jt_limit = 0;
	info->soca->zero_flg = 0;
	info->soca->cc_cap_offset = 0;
	info->soca->last_cc_sum = 0;

	for (i = 0; i < 11; i++) {
		info->soca->ocv_table_low[i] = 0;
	}

	for (i = 0; i < RICOH61x_GET_CHARGE_NUM; i++) {
		info->soca->Vbat[i] = 0;
		info->soca->Vsys[i] = 0;
		info->soca->Ibat[i] = 0;
	}

	g_full_flag = 0;

#ifdef ENABLE_FG_KEEP_ON_MODE
	g_fg_on_mode = 1;
	info->soca->rsoc_ready_flag = 1;
#else
	g_fg_on_mode = 0;
#endif

#if 0
	/* Start first Display job */
	queue_delayed_work(info->monitor_wqueue, &info->displayed_work,
			RICOH61x_FG_RESET_TIME*HZ);

	/* Start first Waiting stable job */
	queue_delayed_work(info->monitor_wqueue, &info->charge_stable_work,
			RICOH61x_FG_STABLE_TIME*HZ/10);

	queue_delayed_work(info->monitor_wqueue, &info->charge_monitor_work,
			RICOH61x_CHARGE_MONITOR_TIME * HZ);

	queue_delayed_work(info->monitor_wqueue, &info->get_charge_work,
			RICOH61x_CHARGE_MONITOR_TIME * HZ);
#endif
	if (info->jt_en) {
		if (info->jt_hw_sw) {
			/* Enable JEITA function supported by H/W */
			err = ricoh61x_set_bits(CHGCTL1_REG, 0x04);
			if (err < 0)
				printf("Error in writing the control register\n");
		} else {
			/* Disable JEITA function supported by H/W */
			err = ricoh61x_clr_bits(CHGCTL1_REG, 0x04);
			if (err < 0)
				printf("Error in writing the control register\n");
			//queue_delayed_work(info->monitor_wqueue, &info->jeita_work,
			//			 	 RICOH61x_FG_RESET_TIME * HZ);
		}
	} else {
		/* Disable JEITA function supported by H/W */
		err = ricoh61x_clr_bits(CHGCTL1_REG, 0x04);
		if (err < 0)
			printf("Error in writing the control register\n");
	}

	err = measure_vbatt_FG(info, &jk);

	return 1;
}



static int get_power_supply_status(struct ricoh61x_battery_info *info)
{
	uint8_t status;
	uint8_t supply_state;
	uint8_t charge_state;
	int ret = 0;

	/* get  power supply status */
	ret = ricoh61x_read(CHGSTATE_REG, &status);
	if (ret < 0) {
		printf(  "Error in reading the control register\n");
		return ret;
	}

	charge_state = (status & 0x1F);
	supply_state = ((status & 0xC0) >> 6);

	if (info->entry_factory_mode)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (supply_state == SUPPLY_STATE_BAT) {
		info->soca->chg_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		switch (charge_state) {
		case	CHG_STATE_CHG_OFF:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case	CHG_STATE_CHG_READY_VADP:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_CHG_TRICKLE:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_CHARGING;
			break;
		case	CHG_STATE_CHG_RAPID:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_CHARGING;
			break;
		case	CHG_STATE_CHG_COMPLETE:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_FULL;
			break;
		case	CHG_STATE_SUSPEND:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case	CHG_STATE_VCHG_OVER_VOL:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case	CHG_STATE_BAT_ERROR:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_NO_BAT:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_BAT_OVER_VOL:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_BAT_TEMP_ERR:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_DIE_ERR:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_DIE_SHUTDOWN:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case	CHG_STATE_NO_BAT2:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case	CHG_STATE_CHG_READY_VUSB:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			info->soca->chg_status
			= POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
	}

	return info->soca->chg_status;
}


/* Initial setting of charger */
static int ricoh61x_init_charger(struct ricoh61x_battery_info *info)
{
	int err;
	uint8_t val;
	uint8_t val2;
	uint8_t val3;
	int charge_status;
	int	vfchg_val;
	int	icchg_val;
	int	rbat;
	int	temp;
	int temp1 = 0;

	info->chg_ctr = 0;
	info->chg_stat1 = 0;
	err = ricoh61x_set_bits(RICOH61x_PWR_FUNC, 0x20);
	if (err < 0) {
		printf(  "Error in writing the PWR FUNC register\n");
		goto free_device;
	}

	charge_status = get_power_supply_status(info);

	if (charge_status != POWER_SUPPLY_STATUS_FULL)
	{
		/* Disable charging */
		err = ricoh61x_clr_bits(CHGCTL1_REG, 0x03);
		if (err < 0) {
			printf(  "Error in writing the control register\n");
			goto free_device;
		}
	}


	/* REGISET1:(0xB6) setting */
	if ((info->ch_ilim_adp != 0xFF) || (info->ch_ilim_adp <= 0x1D)) {
		val = info->ch_ilim_adp;

		err = ricoh61x_write(REGISET1_REG,val);
		if (err < 0) {
			printf(  "Error in writing REGISET1_REG %d\n",
					err);
			goto free_device;
		}
		info->adp_current_val = val;
	}
	else info->adp_current_val = 0xff;

	/* REGISET2:(0xB7) setting */
	err = ricoh61x_read(REGISET2_REG, &val);
	if (err < 0) {
		printf( 
				"Error in read REGISET2_REG %d\n", err);
		goto free_device;
	}

	if ((info->ch_ilim_usb != 0xFF) || (info->ch_ilim_usb <= 0x1D)) {
		val2 = info->ch_ilim_usb;
	} else {/* Keep OTP value */
		val2 = (val & 0x1F);
	}

	/* keep bit 5-7 */
	val &= 0xE0;

	val = val + val2;
	info->usb_current_val = val;
	err = ricoh61x_write(REGISET2_REG,val);
	if (err < 0) {
		printf(  "Error in writing REGISET2_REG %d\n",
				err);
		goto free_device;
	}

	err = ricoh61x_read(CHGISET_REG, &val);
	if (err < 0) {
		printf( 
				"Error in read CHGISET_REG %d\n", err);
		goto free_device;
	}

	/* Define Current settings value for charging (bit 4~0)*/
	if ((info->ch_ichg != 0xFF) || (info->ch_ichg <= 0x1D)) {
		val2 = info->ch_ichg;
	} else { /* Keep OTP value */
		val2 = (val & 0x1F);
	}

	/* Define Current settings at the charge completion (bit 7~6)*/
	if ((info->ch_icchg != 0xFF) || (info->ch_icchg <= 0x03)) {
		val3 = info->ch_icchg << 6;
	} else { /* Keep OTP value */
		val3 = (val & 0xC0);
	}

	val = val2 + val3;

	err = ricoh61x_write(CHGISET_REG, val);
	if (err < 0) {
		printf(  "Error in writing CHGISET_REG %d\n",
				err);
		goto free_device;
	}

	//debug messeage
	err = ricoh61x_read(CHGISET_REG,&val);

	//debug messeage
	err = ricoh61x_read(BATSET1_REG,&val);

	/* BATSET1_REG(0xBA) setting */
	err = ricoh61x_read(BATSET1_REG, &val);
	if (err < 0) {
		printf("Error in read BATSET1 register %d\n", err);
		goto free_device;
	}

	/* Define Battery overvoltage  (bit 4)*/
	if ((info->ch_vbatovset != 0xFF) || (info->ch_vbatovset <= 0x1)) {
		val2 = info->ch_vbatovset;
		val2 = val2 << 4;
	} else { /* Keep OTP value */
		val2 = (val & 0x10);
	}

	/* keep bit 0-3 and bit 5-7 */
	val = (val & 0xEF);

	val = val + val2;

	err = ricoh61x_write(BATSET1_REG, val);
	if (err < 0) {
		printf(  "Error in writing BAT1_REG %d\n",
				err);
		goto free_device;
	}
	//debug messeage
	err = ricoh61x_read(BATSET1_REG,&val);

	//debug messeage
	err = ricoh61x_read(BATSET2_REG,&val);


	/* BATSET2_REG(0xBB) setting */
	err = ricoh61x_read(BATSET2_REG, &val);
	if (err < 0) {
		printf("Error in read BATSET2 register %d\n", err);
		goto free_device;
	}

	/* Define Re-charging voltage (bit 2~0)*/
	if ((info->ch_vrchg != 0xFF) || (info->ch_vrchg <= 0x04)) {
		val2 = info->ch_vrchg;
	} else { /* Keep OTP value */
		val2 = (val & 0x07);
	}

	/* Define FULL charging voltage (bit 6~4)*/
	if ((info->ch_vfchg != 0xFF) || (info->ch_vfchg <= 0x04)) {
		val3 = info->ch_vfchg;
		val3 = val3 << 4;
	} else {	/* Keep OTP value */
		val3 = (val & 0x70);
	}

	/* keep bit 3 and bit 7 */
	val = (val & 0x88);

	val = val + val2 + val3;

	err = ricoh61x_write(BATSET2_REG, val);
	if (err < 0) {
		printf(  "Error in writing RICOH61x_RE_CHARGE_VOLTAGE %d\n",
				err);
		goto free_device;
	}

	/* Set rising edge setting ([1:0]=01b)for INT in charging */
	/*  and rising edge setting ([3:2]=01b)for charge completion */
	err = ricoh61x_read(RICOH61x_CHG_STAT_DETMOD1, &val);
	if (err < 0) {
		printf(  "Error in reading CHG_STAT_DETMOD1 %d\n",
				err);
		goto free_device;
	}
	val &= 0xf0;
	val |= 0x05;
	err = ricoh61x_write(RICOH61x_CHG_STAT_DETMOD1, val);
	if (err < 0) {
		printf(  "Error in writing CHG_STAT_DETMOD1 %d\n",
				err);
		goto free_device;
	}

	/* Unmask In charging/charge completion */
	err = ricoh61x_write(RICOH61x_INT_MSK_CHGSTS1, 0xfc);
	if (err < 0) {
		printf(  "Error in writing INT_MSK_CHGSTS1 %d\n",
				err);
		goto free_device;
	}

	/* Set both edge for VUSB([3:2]=11b)/VADP([1:0]=11b) detect */
	err = ricoh61x_read(RICOH61x_CHG_CTRL_DETMOD1, &val);
	if (err < 0) {
		printf(  "Error in reading CHG_CTRL_DETMOD1 %d\n",
				err);
		goto free_device;
	}
	val &= 0xf0;
	val |= 0x0f;
	err = ricoh61x_write(RICOH61x_CHG_CTRL_DETMOD1, val);
	if (err < 0) {
		printf(  "Error in writing CHG_CTRL_DETMOD1 %d\n",
				err);
		goto free_device;
	}

	/* Unmask In VUSB/VADP completion */
	err = ricoh61x_write(RICOH61x_INT_MSK_CHGCTR, 0xfc);
	if (err < 0) {
		printf(  "Error in writing INT_MSK_CHGSTS1 %d\n",
				err);
		goto free_device;
	}


	if (charge_status != POWER_SUPPLY_STATUS_FULL)
	{
		/* Enable charging */
		err = ricoh61x_set_bits(CHGCTL1_REG, 0x03);
		if (err < 0) {
			printf(  "Error in writing the control register\n");
			goto free_device;
		}
	}
	/* get OCV100_min, OCV100_min*/
	temp = (battery_init_para[info->num][24]<<8) | (battery_init_para[info->num][25]);
	rbat = temp * 1000 / 512 * 5000 / 4095;

	/* get vfchg value */
	err = ricoh61x_read(BATSET2_REG, &val);
	if (err < 0) {
		printf(  "Error in reading the batset2reg\n");
		goto free_device;
	}
	val &= 0x70;
	val2 = val >> 4;
	if (val2 <= 3) {
		vfchg_val = 4050 + val2 * 50;
	} else {
		vfchg_val = 4350;
	}

	/* get  value */
	err = ricoh61x_read(CHGISET_REG, &val);
	if (err < 0) {
		printf(  "Error in reading the chgisetreg\n");
		goto free_device;
	}
	val &= 0xC0;
	val2 = val >> 6;
	icchg_val = 50 + val2 * 50;

	info->soca->OCV100_min = ( vfchg_val * 99 / 100 - (icchg_val * (rbat +20))/1000 - 20 ) * 1000;
	info->soca->OCV100_max = ( vfchg_val * 101 / 100 - (icchg_val * (rbat +20))/1000 + 20 ) * 1000;


#ifdef ENABLE_LOW_BATTERY_DETECTION
	/* Set ADRQ=00 to stop ADC */
	ricoh61x_write(RICOH61x_ADC_CNT3, 0x0);
	/* Enable VSYS threshold Low interrupt */
	ricoh61x_write(RICOH61x_INT_EN_ADC1, 0x10);
	/* Set ADC auto conversion interval 250ms */
	ricoh61x_write(RICOH61x_ADC_CNT2, 0x0);
	/* Enable VSYS pin conversion in auto-ADC */
	ricoh61x_write(RICOH61x_ADC_CNT1, 0x10);
	/* Set VSYS threshold low voltage value = (voltage(V)*255)/(3*2.5) */
	val = info->alarm_vol_mv * 255 / 7500;
	ricoh61x_write(RICOH61x_ADC_VSYS_THL, val);
	/* Start auto-mode & average 4-time conversion mode for ADC */
	ricoh61x_write(RICOH61x_ADC_CNT3, 0x28);

#endif

	free_device:
	return err;
}

void ricoh619_power_off(void)
{
	int ret;
	uint8_t reg_val;
	reg_val = g_soc;
	reg_val &= 0x7f;

	if (g_fg_on_mode == 0) {
		/* Clear RICOH61x_FG_CTRL 0x01 bit */
		ret = __ricoh61x_read(RICOH61x_FG_CTRL, &reg_val);
		if (reg_val & 0x01) {
			reg_val &= ~0x01;
			ret = __ricoh61x_write(RICOH61x_FG_CTRL, reg_val);
		}
		if (ret < 0)
			printf("Error in writing FG_CTRL\n");
	}

	/* set rapid timer 300 min */
	ret = __ricoh61x_read(TIMSET_REG, &reg_val);

	reg_val |= 0x03;

	ret = __ricoh61x_write(TIMSET_REG, reg_val);
	if (ret < 0)
		printf("Error in writing the TIMSET_Reg\n");
	/* Disable all Interrupt */
	__ricoh61x_write(RICOH61x_INTC_INTEN, 0);

	/* Not repeat power ON after power off(Power Off/N_OE) */
	__ricoh61x_write(RICOH61x_PWR_REP_CNT, 0x0);

	/* Power OFF */
	__ricoh61x_write(RICOH61x_PWR_SLP_CNT, 0x1);
}


void ricoh619_battery_init(void)
{
	struct ricoh61x_battery_info *info;
	struct ricoh61x *ricoh61x;
	int ret;
	int temp =  0, temp1 = 0;
	int type_n = 0;
	info = kzalloc(sizeof(struct ricoh61x_battery_info), GFP_KERNEL);
	info->fg_rsense_val = 100;

	ricoh61x = kzalloc(sizeof(struct ricoh61x), GFP_KERNEL);
	if (ricoh61x == NULL)
		return -ENOMEM;

	info->soca = kzalloc(sizeof(struct ricoh61x_soca_info), GFP_KERNEL);
	if (!info->soca)
		return -ENOMEM;

	info->alarm_vol_mv = pdata.alarm_vol_mv;
	info->soca->Vbat_old = 0;

	info->fg_rsense_val = 100;

	info->ch_vfchg = pdata.type[type_n].ch_vfchg;
	info->ch_vrchg = pdata.type[type_n].ch_vrchg;
	info->ch_vbatovset = pdata.type[type_n].ch_vbatovset;
	info->ch_ichg = pdata.type[type_n].ch_ichg;
	info->ch_ilim_adp = pdata.type[type_n].ch_ilim_adp;
	info->ch_ilim_usb = pdata.type[type_n].ch_ilim_usb;
	info->ch_icchg = pdata.type[type_n].ch_icchg;

	if (pdata.type[type_n].fg_rsense_val ==0 ) info->fg_rsense_val = 100;
	else info->fg_rsense_val = pdata.type[type_n].fg_rsense_val;

	info->fg_target_vsys = pdata.type[type_n].fg_target_vsys;
	info->fg_target_ibat = pdata.type[type_n].fg_target_ibat * info->fg_rsense_val / 20;
	info->fg_poff_vbat = pdata.type[type_n].fg_poff_vbat;
	info->jt_en = pdata.type[type_n].jt_en;
	info->jt_hw_sw = pdata.type[type_n].jt_hw_sw;
	info->jt_temp_h = pdata.type[type_n].jt_temp_h;
	info->jt_temp_l = pdata.type[type_n].jt_temp_l;
	info->jt_vfchg_h = pdata.type[type_n].jt_vfchg_h;
	info->jt_vfchg_l = pdata.type[type_n].jt_vfchg_l;
	info->jt_ichg_h = pdata.type[type_n].jt_ichg_h;
	info->jt_ichg_l = pdata.type[type_n].jt_ichg_l;


	temp = get_OCV_init_Data(info, 11) * info->fg_rsense_val / 20;
	battery_init_para[info->num][22] = (temp >> 8);
	battery_init_para[info->num][23] = (temp & 0xff);
	temp = get_OCV_init_Data(info, 12) * 20 / info->fg_rsense_val;
	battery_init_para[info->num][24] = (temp >> 8);
	battery_init_para[info->num][25] = (temp & 0xff);



	ret = ricoh61x_clr_bits(RICOH61x_INTC_INTEN, CHG_INT | ADC_INT);

	ret = ricoh61x_init_battery(info);
	if (ret)
		goto out;

	ret = ricoh61x_init_charger(info);

	ret = ricoh61x_init_fgsoca(info);

	ricoh61x_set_bits(RICOH61x_INTC_INTEN, CHG_INT | ADC_INT);
out:
	kfree(info);

}


int ricoh61x_regulator_init(void)
{

	int ret,i;
	ret = i2c_probe(RICOH61x_I2C_ADDR);
	if(ret) {
		printf("probe richo61x error, i2c addr ox%x\n", RICOH61x_I2C_ADDR);
		return -EIO;
	}
	for (i = 0; i < ARRAY_SIZE(ricoh61x_regulator); i++) {
		ret = regulator_register(&ricoh61x_regulator[i].desc, NULL);
		rdev_set_drvdata(&ricoh61x_regulator[i].desc,&ricoh61x_regulator[i]);
		if(ret)
			printf("%s regulator_register error\n",
					ricoh61x_regulator[i].desc.name);
	}

	return 0;
}
#endif

#ifdef CONFIG_SPL_BUILD
int spl_regulator_set_voltage(enum regulator_outnum outnum, int vol_mv)
{
	char reg;
	u8 regvalue;

	switch(outnum) {
	case REGULATOR_CORE:
		reg = RICOH619_DC1;
		if ((vol_mv < 1000) || (vol_mv >1300)) {
			debug("voltage for core is out of range\n");
			return -EINVAL;
		}
		break;
	case REGULATOR_MEM:
		reg = RICOH619_DC2;
		break;
	case REGULATOR_IO:
		reg = RICOH619_DC4;
		break;
	default:return -EINVAL;
	}

	if ((vol_mv < 600) || (vol_mv > 3500)) {
		debug("unsupported voltage\n");
		return -EINVAL;
	} else {
		regvalue = ((vol_mv - 600) * 10)/ 125;
	}

	return ricoh61x_write_reg(reg, &regvalue);
}
#endif
