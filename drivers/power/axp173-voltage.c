#include <power/axp173.h>
#include <config.h>
#include <common.h>

#define CHARGING_ON 	1
#ifdef CONFIG_ASLMOM_BATTERY
#define INTER_RESIST    300
#else
#define INTER_RESIST 	132
#endif
#define SAMPLE_COUNT	10

extern int axp173_read_reg(u8 reg, u8 *val, u32 len);
extern int axp173_write_reg(u8 reg, u8 *val);

#ifdef CONFIG_ASLMOM_BATTERY
struct ocv2soc ocv2soc[] = {
        {4321, 100},
        {4152,  97},
        {4096,  93},
        {4030,  88},
        {3974,  83},
        {3920,  78},
        {3868,  73},
        {3804,  67},
        {3764,  62},
        {3721,  56},
        {3679,  48},
        {3653,  41},
        {3628,  35},
        {3603,  28},
        {3576,  22},
        {3552,  17},
        {3522,  12},
        {3497,   9},
        {3461,   6},
        {3417,   3},
        {3371,   0},
};
#else
struct ocv2soc ocv2soc[] = {
	{4245, 100},
	{4125,  90},
	{4025,  80},
	{3965,  70},
	{3895,  60},
	{3865,  50},
	{3825,  40},
	{3795,  30},
	{3765,  20},
	{3685,  10},
	{3635,   5},
	{3445,   0},
};
#endif
enum adc_type {
        ACIN_VOL = 0,
        ACIN_CUR,
        VBUS_VOL,
        VBUS_CUR,
        BAT_VOL,
        BAT_CUR,
};

static unsigned int axp173_get_single_adc_data(enum adc_type type)
{
	unsigned char tmp[2] = {0,};
	unsigned int val = 0;

#define GET_ADC_VALUE(reg1, reg2)		\
	axp173_read_reg(reg1, tmp, 1);		\
	axp173_read_reg(reg2, tmp+1, 1)

	switch (type) {
	case ACIN_VOL:
		GET_ADC_VALUE(POWER_ACIN_VOL_H8, POWER_ACIN_VOL_L4);
		val = ((tmp[0] << 4) + (tmp[1] & 0x0f)) * 17 / 10;
		break;
	case ACIN_CUR:
		GET_ADC_VALUE(POWER_ACIN_CUR_H8, POWER_ACIN_CUR_L4);
		val = ((tmp[0] << 4) + (tmp[1] & 0x0f)) * 5 / 8;
		break;
	case VBUS_VOL:
		GET_ADC_VALUE(POWER_VBUS_VOL_H8, POWER_VBUS_VOL_L4);
		val = ((tmp[0] << 4) + (tmp[1] & 0x0f)) * 17 / 10;
		break;
	case VBUS_CUR:
		GET_ADC_VALUE(POWER_VBUS_CUR_H8, POWER_VBUS_CUR_L4);
		val = ((tmp[0] << 4) + (tmp[1] & 0x0f)) * 375 / 1000;
		break;
	case BAT_VOL:
		GET_ADC_VALUE(POWER_BAT_AVERVOL_H8, POWER_BAT_AVERVOL_L4);
		val = ((tmp[0] << 4) + ((tmp[1] & 0x0f))) * 11 / 10;
		break;
	case BAT_CUR:
		GET_ADC_VALUE(POWER_BAT_AVERCHGCUR_H8, POWER_BAT_AVERCHGCUR_L5);
			val = ((tmp[0] << 5) + (tmp[1] & 0x1f)) / 2;
		GET_ADC_VALUE(POWER_BAT_AVERDISCHGCUR_H8,
			POWER_BAT_AVERDISCHGCUR_L5);
		val += ((tmp[0] << 5) + (tmp[1] & 0x1f)) / 2;
		break;
	default:
		break;
	}
#undef GET_ADC_VALUE

	return val;
}

static unsigned int axp173_get_adjust_adc_data(enum adc_type type)
{
	int i = 0;
	unsigned int max_val = 0;
	unsigned int min_val = 0;
	unsigned int count = 0;
	unsigned int value = 0;
	int tmp = 0;
	int times = 0;

	for (; i < SAMPLE_COUNT; ++i) {
		tmp = axp173_get_single_adc_data(type);
		if (tmp > 0) {
			if (i == 0)
				max_val = min_val = tmp;
			count += tmp;
			times++;
			if (tmp > max_val)
				max_val = tmp;
			else if (tmp < min_val)
				min_val = tmp;
		}
	}

	switch (times) {
	case 0:
		value = 0;
		break;
	case 1:
	case 2:
		value = count / times;
		break;
	default:
		value = (count-max_val-min_val) / (times-2);
		break;
	}

	return value;
}

static unsigned int axp173_get_adc_data(enum adc_type type, int sw_adjust)
{
	return sw_adjust + axp173_get_adjust_adc_data(type);
}

static int get_pmu_current(void)
{
	return axp173_get_adc_data(BAT_CUR, 0);
}

static int get_pmu_voltage(void)
{
	return axp173_get_adc_data(BAT_VOL, 0);
}

static unsigned int jz_current_battery_voltage()
{
	unsigned int voltage = 0;
	int pmu_charging = 0;
	int pmu_current = 0;
	int value;

	gpio_direction_input(USB_DETE);
	value = gpio_get_value(USB_DETE);
	if(value == 0)
		pmu_charging = 1;
	if(value == 1)
		pmu_charging = 0;

	voltage = get_pmu_voltage();
	pmu_current = get_pmu_current();
	voltage = pmu_charging == CHARGING_ON ? voltage - (pmu_current * INTER_RESIST
			/ 1000) : voltage + (pmu_current * INTER_RESIST / 1000);
//	printf("========++>pmu_charging = %d, pmu_current = %d, voltage = %d\n", 
//		pmu_charging, pmu_current, voltage);

	return voltage;
}

static int jz_current_battery_current_cpt(unsigned int voltage)
{
	int i = 0;
	int cpt = 0;

	for (; i < ARRAY_SIZE(ocv2soc); ++i)
		if (voltage >= ocv2soc[i].vol)
			break;
	if (i == 0)
		cpt = ocv2soc[i].cpt;
	else if (i == ARRAY_SIZE(ocv2soc))
		cpt = ocv2soc[i-1].cpt;
	else {
		int cpt_step = (ocv2soc[i-1].vol - ocv2soc[i].vol) /
				(ocv2soc[i-1].cpt - ocv2soc[i].cpt);
		int vol = ocv2soc[i-1].vol - voltage;

		cpt = ocv2soc[i-1].cpt - vol / cpt_step;
	}

	return cpt;
}

int get_battery_current_cpt(void)
{
	unsigned int voltage = 0;

	mdelay(250);
	voltage = jz_current_battery_voltage();
	return jz_current_battery_current_cpt(voltage);
}
