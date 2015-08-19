/*
 * SoC Regulator driver support
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Kage Shen <kkshen@ingenic.cn>
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
#include <linux/list.h>
#include <asm/errno.h>
#include <linux/err.h>
#include <regulator.h>

static LIST_HEAD(regulator_map_list);


/* Internal regulator request function
 * id: name of regulator
 * ignore_case: 0: case sensitive 1: mixed case
 *  */
static struct regulator *_regulator_get(const char *id, int ignore_case)
{
	struct regulator *map;
	int (* str_cmp_function)(const char *s1, const char *s2) = NULL;
	if (id == NULL) {
		printf("get() with no identifier\n");
		return ERR_PTR(-EINVAL);
	}

	if(ignore_case){
		str_cmp_function = strcasecmp;
	}else{
		str_cmp_function = strcmp;
	}
	list_for_each_entry(map, &regulator_map_list, list) {
		if (str_cmp_function(map->name, id) == 0) {
			return map;
		}
	}

	return NULL;
}

/**
 * regulator_get - lookup and obtain a reference to a regulator.
 * @id: Supply name or regulator ID.
 *
 * Returns a struct regulator corresponding to the regulator producer,
 * or IS_ERR() condition containing errno.
 *
 * Use of supply names configured via regulator_set_device_supply() is
 * strongly encouraged.  It is recommended that the supply name used
 * should match the name used for the supply and/or the relevant
 * device pins in the datasheet.
 */
struct regulator *regulator_get(const char *id)
{
	return _regulator_get(id, 1);
}

/**
 * regulator_get_drvdata - get regulator driver data
 * @regulator: regulator
 *
 * Get regulator driver private data. This call can be used in the consumer
 * driver context when non API regulator specific functions need to be called.
 */
void *regulator_get_drvdata(struct regulator *regulator)
{
	return regulator->reg_data;
}

/**
 * regulator_register - register regulator
 *
 * Called by regulator drivers to register a regulator.
 * Returns 0 on success.
 */
int regulator_register(struct regulator *regulator, void *driver_data)
{
	int ret;

	if (regulator == NULL)
		return -EINVAL;

	if (regulator->name == NULL || regulator->ops == NULL)
		return -EINVAL;

	/* preform any regulator specific init */
	if (regulator->regulator_init) {
		ret = regulator->regulator_init(driver_data);
		if (ret < 0)
			return ret;
	}

	list_add(&regulator->list, &regulator_map_list);

	return 0;
}

/**
 * regulator_reg_read - read an reg from pmu chip
 * @regulator: regulator source
 * @reg: register address you want to read
 * @data: data address point to storage
 * Returns negative if the regulator read failed, zero if it hasn't, else a
 * negative errno code.
 */
int regulator_reg_read(struct regulator *regulator, const unsigned char reg, unsigned char * const data){
	if (regulator == NULL)
		return -EINVAL;

	if (!regulator->ops->read)
		return -EPERM;

	return regulator->ops->read(reg, data);
}

/**
 * regulator_reg_read - write an reg to pmu chip
 * @regulator: regulator source
 * @reg: register address you want to write
 * @data: data you want to write
 * Returns negative if the regulator write failed, zero if it hasn't, else a
 * negative errno code.
 */
int regulator_reg_write(struct regulator *regulator, const unsigned char reg, const unsigned char data){
	if (regulator == NULL)
		return -EINVAL;

	if (!regulator->ops->write)
		return -EPERM;

	return regulator->ops->write(reg, data);
}

/**
 * regulator_is_enabled - is the regulator output enabled
 * @regulator: regulator source
 *
 * Returns positive if the regulator driver backing the source/client
 * has requested that the device be enabled, zero if it hasn't, else a
 * negative errno code.
 */
int regulator_is_enabled(struct regulator *regulator)
{
	if (regulator == NULL)
		return -EINVAL;

	/* If we don't know then assume that the regulator is always on */
	if (!regulator->ops->is_enabled)
		return true;

	return regulator->ops->is_enabled(regulator);
}

/**
 * regulator_enable - enable regulator output
 * @regulator: regulator source
 *
 * Request that the regulator be enabled with the regulator output at
 * the predefined voltage or current value.  Calls to regulator_enable()
 * must be balanced with calls to regulator_disable().
 */
int regulator_enable(struct regulator *regulator)
{
	if (regulator == NULL)
		return -EINVAL;

	if(regulator_is_enabled(regulator))
		return 0;

	if (!regulator->ops->is_enabled)
		return -EPERM;

	return regulator->ops->enable(regulator);
}

/**
 * regulator_disable - disable regulator output
 * @regulator: regulator source
 *
 * Disable the regulator output voltage or current.  Calls to
 * regulator_enable() must be balanced with calls to
 * regulator_disable().
 */
int regulator_disable(struct regulator *regulator)
{
	if (regulator == NULL)
		return -EINVAL;

	if (!regulator_is_enabled(regulator))
		return 0;

	if (!regulator->ops->disable)
		return -EPERM;

	return regulator->ops->disable(regulator);
}

/**
 * regulator_set_voltage - set regulator output voltage
 * @regulator: regulator source
 * @min_uV: Minimum required voltage in uV
 * @max_uV: Maximum acceptable voltage in uV
 *
 * Sets a voltage regulator to the desired output voltage. This can be set
 * during any regulator state. IOW, regulator can be disabled or enabled.
 */
int regulator_set_voltage(struct regulator *regulator, int min_uV, int max_uV)
{
	int vol_max, vol_min;

	if (regulator == NULL)
		return -EINVAL;

	/* This is only going to work if we've had a voltage configured. */
	if (!regulator->min_uV && !regulator->max_uV) {
		return -EINVAL;
	}

	if (min_uV > max_uV)
		return -EINVAL;

	if (max_uV < regulator->min_uV || min_uV > regulator->max_uV)
		return -EINVAL;

	vol_max = max_uV;
	vol_min = min_uV;

	if (max_uV > regulator->max_uV)
		vol_max = regulator->max_uV;
	if (min_uV < regulator->min_uV)
		vol_min = regulator->min_uV;

	if (!regulator->ops->set_voltage)
		return -EPERM;

	return regulator->ops->set_voltage(regulator, vol_min, vol_max);
}

/**
 * regulator_get_voltage - get regulator output voltage
 * @regulator: regulator source
 *
 * This returns the current regulator voltage in uV.
 */
int regulator_get_voltage(struct regulator *regulator)
{
	if (regulator == NULL)
		return -EINVAL;

	if (!regulator->ops->get_voltage)
		return -EPERM;

	return regulator->ops->get_voltage(regulator);
}

/**
 * regulator_set_current_limit - set regulator output current limit
 * @regulator: regulator source
 * @min_uA: Minimuum supported current in uA
 * @max_uA: Maximum supported current in uA
 */
int regulator_set_current_limit(struct regulator *regulator,
		int min_uA, int max_uA)
{
	int val_max, val_min;

	if (regulator == NULL)
		return -EINVAL;

	/* This is only going to work if we've had a voltage configured. */
	if (!regulator->min_uA && !regulator->max_uA) {
		return -EINVAL;
	}

	if (min_uA > max_uA)
		return -EINVAL;

	if (max_uA < regulator->min_uA || min_uA > regulator->max_uA)
		return -EINVAL;

	val_max = max_uA;
	val_min = min_uA;

	if (max_uA > regulator->max_uV)
		val_max = regulator->max_uV;
	if (min_uA < regulator->min_uV)
		val_min = regulator->min_uV;

	if (!regulator->ops->set_voltage)
		return -EPERM;

	return regulator->ops->set_current(regulator, val_min, val_max);
}

/**
 * regulator_get_current_limit - get regulator output current
 * @regulator: regulator source
 */
int regulator_get_current_limit(struct regulator *regulator)
{
	if (regulator == NULL)
		return -EINVAL;

	if (!regulator->ops->get_current)
		return -EPERM;

	return regulator->ops->get_current(regulator);
}

/**
 * get_simple_regulator - get a regulator instance
 * used in cmd_pmu, for pmu register read & write
 * @regulator: regulator source
 */
struct regulator *get_simple_regulator(void){
	return list_entry(regulator_map_list.next, struct regulator, list);
}

/**
 * do_powerinfo - show regulator output voltage and status
 * @regulator: regulator source
 */
static int do_powerinfo(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct regulator *map;

	printf("%-20s %-9s %-15s %-15s %-15s %-15s\n",
			"name", "status", "voltage(mV)", "min(mV)", "max(mV)");
	list_for_each_entry(map, &regulator_map_list, list) {
		printf("%-20s %-9s %-15d %-15d %-15d\n",
				map->name,
				(regulator_is_enabled(map)?"on" : "off"),
				regulator_get_voltage(map)/1000,
				map->min_uV/1000,
				map->max_uV/1000);
	}
	return 0;
}

/*
 * Used to display all regulator information.
 * used by pmu cmd
 *
 * */
int show_all_regulator_info(void){
	return do_powerinfo(NULL, 0, 0, NULL);
}

U_BOOT_CMD(
		powerinfo,	1,	1,	do_powerinfo,
		"show regulator output voltage and status",
		"cmd: dpower"
	  );
