#include <config.h>
#include <common.h>
#include <linux/err.h>
#include <malloc.h>
#include <vsprintf.h>

struct voltage_pair {
	int uboot_voltage;
	int slpt_voltage;
};

static struct voltage_pair adc_voltage_table_no_charing[] = {
	{3652, 3747},
	{3712, 3776},
	{3768, 3823},
	{3808, 3862},
	{3828, 3873},
	{3844, 3883},
	{3864, 3900},
	{3928, 3947},
	{4060, 4042},
	{4168, 4145},
	{4188, 4188},
};

static struct voltage_pair adc_voltage_table_charing[] = {
	{3740, 3747},
	{3772, 3776},
	{3828, 3823},
	{3868, 3862},
	{3884, 3873},
	{3896, 3883},
	{3912, 3900},
	{3972, 3947},
	{4080, 4042},
	{4196, 4145},
	{4240, 4188},
};

int match_to_slpt_voltage(struct voltage_pair *table, unsigned int size, int voltage) {
	unsigned int i;
	unsigned int start1, end1;
	unsigned int start2, end2;

	for (i = 0; i < size; ++i) {
		if (table[i].uboot_voltage >= voltage) {
			break;
		}
	}

	if (size == 0)
		return voltage;

	if (i == 0)
		return voltage + (table[i].slpt_voltage - table[i].uboot_voltage);

	if (i == size)
		return voltage + (table[i].slpt_voltage - table[i].uboot_voltage);

	start1 = table[i-1].uboot_voltage;
	start2 = table[i-1].slpt_voltage;
	end1 = table[i].uboot_voltage;
	end2 = table[i].slpt_voltage;

	printf ("voltage %d match from [%d %d] to [%d %d]\n", voltage, start1, end1, start2, end2);

	/* if a/b = c/d then c = (a*d)/b */
	return start2 + (((end2 - start2) * (voltage - start1)) / (end1 - start1));
}

int match_to_slpt_voltage_no_charging(int voltage) {
	return match_to_slpt_voltage(adc_voltage_table_no_charing, ARRAY_SIZE(adc_voltage_table_no_charing), voltage);
}

int match_to_slpt_voltage_charging(int voltage) {
	return match_to_slpt_voltage(adc_voltage_table_charing, ARRAY_SIZE(adc_voltage_table_charing), voltage);
}

#ifdef CONFIG_PMU_RICOH6x
extern int ricoh619_get_battery_for_kernel(unsigned int *voltage);
extern int detection_first_poweron(void);
extern void ricoh619_limit_current_set_to_50mA(void);
extern void ricoh619_limit_current_init(void);
extern int detection_first_poweron(void);
#endif

extern unsigned int read_battery_voltage(void);
extern int detect_charger_state(void);

extern int add_cmd_line_arg(char *arg);

static char cmdline_battery_for_kernel[30] = "";

static int do_battery_for_kernel(cmd_tbl_t * cmdtp, int flag, int argc,
								 char *const argv[])
{
	int ret = 0;
	unsigned int voltage;
	int first_poweron = 0;
	int is_charging = 0;

#ifdef CONFIG_PMU_RICOH6x
	first_poweron = !detection_first_poweron();
#endif

	/* 如果是第一次开机则帮助kernel检查电量 */
	if (!first_poweron)
		return 0;

	/* 限制充电电流，这样测得准一点 */
#ifdef CONFIG_PMU_RICOH6x
	ricoh619_limit_current_set_to_50mA();
#endif

	is_charging = detect_charger_state();

	voltage = read_battery_voltage();
	printf ("voltage from adc: %u\n", voltage);

	/* 和slpt测得的数据相匹配，这么做是为了在kernel休眠唤醒后不产生大的跳变 */
	voltage = is_charging ?
		match_to_slpt_voltage_charging(voltage) : match_to_slpt_voltage_no_charging(voltage);

	voltage *= 1000;

	printf ("match to slpt: %s %u uv\n", is_charging ? "charing" : "no charing", voltage);

#ifdef CONFIG_PMU_RICOH6x
/* 	ret = ricoh619_get_battery_for_kernel(&voltage); */
	ricoh619_limit_current_init();
#endif

	if (ret == 0)
		sprintf(cmdline_battery_for_kernel, " u-boot-battery=%d ", voltage);

	/* 加到kernel启动参数中去 */
	add_cmd_line_arg(cmdline_battery_for_kernel);

	return ret;
}

U_BOOT_CMD(battery_for_kernel, 4, 1, do_battery_for_kernel,
		   "measure battery voltage", "measure battery voltage");
