/*
 * Control PMU Regulator
 *
 * Copyright (c) 2015 Ingenic Semiconductor Co.,Ltd
 * Author: Schspa <chaohui.shi@ingenic.cn>
 *
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

#include <common.h>
#include <command.h>

#include <asm/gpio.h>
#include <regulator.h>
#include <linux/err.h>

//#define _DEBUG	1

#define STR_VOLTAGE_CMD                 "set"
#define STR_ON                          "up"
#define STR_OFF                         "down"
#define STR_INFO                        "info"
#define STR_READ                        "read"
#define STR_WRITE                       "write"

static enum regulator_cmd{
	REGULATOR_CMD_ON,
	REGULATOR_CMD_OFF,
	REGULATOR_CMD_VOLTAGE,
	REGULATOR_CMD_INFO,
	REGULATOR_CMD_WRITE,
	REGULATOR_CMD_READ,
};

/* show regulator information
 * @regulator: pointer of regulator
 * */
static void show_regulator_info(struct regulator *regulator){
	printf("%s status %s, Voltage = %ldmV\n",regulator->name,
			regulator_is_enabled(regulator)?"on":"off",
			regulator_get_voltage(regulator)/1000);
}

static int do_pmu(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	enum regulator_cmd sub_cmd;
	struct regulator * regulator = NULL;
	unsigned char reg_data = 0, reg = 0;
	ulong value;  //used for set voltage
	char *name = NULL;
	int ret = 0;

	switch(argc){
	case 2:
		if(!strcmp(argv[1], STR_INFO)){
			sub_cmd = REGULATOR_CMD_INFO;
		}else{
			return CMD_RET_USAGE;
		}
		break;
	case 3:
		if(!strcmp(argv[2], STR_ON)){
			sub_cmd = REGULATOR_CMD_ON;
		}else if(!strcmp(argv[2], STR_OFF)){
			sub_cmd = REGULATOR_CMD_OFF;
		}else if(!strcmp(argv[1], STR_READ)){
			sub_cmd = REGULATOR_CMD_READ;
			ret = simple_strtoul(argv[2], NULL, 16);
			if(ret < 0 || ret >0xff){
				printf("Argument illegal should be 0x00 ~ 0xFF\n");
				return CMD_RET_FAILURE;
			}
			reg = ret;
		}else{
			return CMD_RET_USAGE;
		}
		break;
	case 4:
		if(!strcmp(argv[2], STR_VOLTAGE_CMD)){
			sub_cmd = REGULATOR_CMD_VOLTAGE;
			value = simple_strtoul(argv[3], NULL, 10);
		}else if(!strcmp(argv[1], STR_WRITE)){
			sub_cmd = REGULATOR_CMD_WRITE;
			ret = (u_char *)simple_strtoul(argv[2], NULL, 16);
			if(ret < 0 || ret >0xff){
				printf("Argument illegal should be 0x00 ~ 0xFF\n");
				return CMD_RET_FAILURE;
			}
			reg = ret;
			ret = (u_char *)simple_strtoul(argv[3], NULL, 16);
			if(ret < 0 || ret >0xff){
				printf("Argument illegal should be 0x00 ~ 0xFF\n");
				return CMD_RET_FAILURE;
			}
			reg_data = ret;
		}else
			return CMD_RET_USAGE;

		break;
	default:
		return CMD_RET_USAGE;
	}

	if(sub_cmd == REGULATOR_CMD_ON || sub_cmd == REGULATOR_CMD_OFF || sub_cmd == REGULATOR_CMD_VOLTAGE){
		name = argv[1];
		if(name == NULL){
			debug("regulator channel name == NULL\n");
			return CMD_RET_FAILURE;
		}
		debug("try to get %s\n", name);
		regulator = regulator_get(name);
		if(regulator == NULL|| IS_ERR(regulator)){
			printf("get channel %s failed\n", name);
			return CMD_RET_FAILURE;
		}
	}

	switch(sub_cmd){
	case REGULATOR_CMD_INFO:{
			printf("show pmu info\n");
			if(0 != show_all_regulator_info()){
				printf("Error while show pmu info\n");
			}
		}
		break;
	case REGULATOR_CMD_ON:
		ret = regulator_enable(regulator);
		if(ret){
			printf("failed to enable %s", name);
		}
		show_regulator_info(regulator);
		break;
	case REGULATOR_CMD_OFF:
		ret = regulator_disable(regulator);
		if(ret){
			printf("failed to disable %s", name);
		}
		show_regulator_info(regulator);
		break;
	case REGULATOR_CMD_VOLTAGE:
		printf("do set voltage\n");
		ret = regulator_set_voltage(regulator, value*1000, value*1000);
		if(ret){
			printf("failed to set voltage, ret = %d\n", ret);
			printf("make sure your input is beyound %ld ~ %ld\n", regulator->min_uV/1000, regulator->max_uV/1000);
		}
		show_regulator_info(regulator);
		break;
	case REGULATOR_CMD_READ:
		regulator = get_simple_regulator();
		if(!regulator){
			printf("Error get sample regulator\n");
		}
		ret = regulator_reg_read(regulator, reg, &reg_data);
		if(ret){
			printf("READ reg:0x%x Error\n", reg);
			return CMD_RET_FAILURE;
		}
		printf("Read Address:0x%x : 0x%x\n", reg, reg_data);
		break;
	case REGULATOR_CMD_WRITE:
		regulator = get_simple_regulator();
		if(!regulator){
			printf("Error get sample regulator\n");
		}
		ret = regulator_reg_write(regulator, reg, reg_data);
		if(ret){
			printf("Write reg:0x%x Error\n", reg);
			return CMD_RET_FAILURE;
		}
		printf("Write Address:0x%x : 0x%x\n", reg, reg_data);
		break;
	default:
		printf("fault command, should never seen me\n");
		return CMD_RET_FAILURE;
	}
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(pmu, 4, 1, do_pmu,
	"regulator operation set",
	"usage:\n"
	"pmu <channel> <up/down>\n"
	"pmu <channel> set <voltage unit: mV>\n"
	"pmu info\n"
	"pmu read <reg address>\n"
	"pmu write <reg address> <data>\n"
	"<reg address> <data>  -- hex number, 13 = 0x13\n"
	"<voltage unit: mV>  -- dec number"
	);
//#undef _DEBUG
