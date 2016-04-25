/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#include "./floader_m200.h"

#define RICOH61x_I2C_ADDR       0x32

#define RICOH619_DC1            0x36
#define RICOH619_DC1_SLP        0x3b
#define RICOH619_DC1_SLOT       0x16
#define RICOH619_DC2            0x37
#define RICOH619_DC2_SLP        0x3c
#define RICOH619_DC2_SLOT       0x17
#define RICOH619_DC3            0x38
#define RICOH619_DC4            0x39
#define RICOH619_DC5            0x3a
#define RICOH619_DC5CTL         0x34
#define RICOH619_LDO1           0x4c
#define RICOH619_LDO2           0x4d
#define RICOH619_LDO3           0x4e
#define RICOH619_LDO4           0x4f
#define RICOH619_LDO5           0x50
#define RICOH619_LDO6           0x51
#define RICOH619_LDO7           0x52
#define RICOH619_LDO8           0x53
#define RICOH619_LDO9           0x54
#define RICOH619_LDO10          0x55
#define RICOH619_LDORTC1        0x56
#define RICOH619_LDORTC2        0x57
#define RICOH619_INTC_INTEN     0x9d
#define RICOH619_NOETIMSET      0x11
#define RICOH619_POFFHIS        0xa


static struct i2c richo_5t619_i2c = {
        .scl = CONFIG_PMU_GPIO_I2C_SCL,
        .sda = CONFIG_PMU_GPIO_I2C_SDA,
        .name = "RICOH-5T619",
};

static int ricoh61x_write_reg(u8 reg, u8 *val)
{
    unsigned int ret;

    ret = i2c_write(&richo_5t619_i2c, RICOH61x_I2C_ADDR, reg, 1, val, 1);
    if(ret)
        return ret;

    return 0;
}

static int ricoh61x_read_reg(u8 reg, u8 *val, u32 len)
{
    int ret;
    ret = i2c_read(&richo_5t619_i2c, RICOH61x_I2C_ADDR, reg, 1, 1, val, len);
    if(ret)
        return ret;

    return 0;
}

void pmu_5t619_set_core_voltage(int mV)
{
    if (mV < 700 || mV > 1300)
        stop(STOP_ERROR_PMU_WRONG_VALUE);

    u8 value = ((mV - 600) * 10) / 125;

    if (ricoh61x_write_reg(RICOH619_DC1, &value))
        stop(STOP_ERROR_PMU_WRITE_FAILED);
}

void pmu_5t619_set_ddr_vdd2_voltage(int mV)
{
    if (mV < 700 || mV > 1300)
        stop(STOP_ERROR_PMU_WRONG_VALUE);

    u8 value = ((mV - 600) * 10) / 125;

    if (ricoh61x_write_reg(RICOH619_DC2, &value))
        stop(STOP_ERROR_PMU_WRITE_FAILED);
}

void pmu_5t619_set_core_sleep_voltage(int mV)
{
    if (mV < 700 || mV > 1300)
        stop(STOP_ERROR_PMU_WRONG_VALUE);

    u8 value = ((mV - 600) * 10) / 125;
    if (ricoh61x_write_reg(RICOH619_DC1_SLP, &value))
        stop(STOP_ERROR_PMU_WRITE_FAILED);
}

void pmu_5t619_set_ddr_vdd2_sleep_voltage(int mV)
{
    if (mV < 700 || mV > 1300)
        stop(STOP_ERROR_PMU_WRONG_VALUE);

    u8 value = ((mV - 600) * 10) / 125;
    if (ricoh61x_write_reg(RICOH619_DC2_SLP, &value))
        stop(STOP_ERROR_PMU_WRITE_FAILED);
}

void pmu_5t619_set_buck5_for_vibrate(int mV)
{
    /*
     * Enable set buck5 to 3000mV
     *
     * TODO: should set by board config
     */
    u8 value = ((mV - 600) * 10) / 125;
    if (ricoh61x_write_reg(RICOH619_DC5, &value))
        stop(STOP_ERROR_PMU_WRITE_FAILED);

    value = 0x3;
    if (ricoh61x_write_reg(RICOH619_DC5CTL, &value))
        stop(STOP_ERROR_PMU_WRITE_FAILED);
}

int probe_5t619()
{
    int failed = i2c_probe(&richo_5t619_i2c, RICOH61x_I2C_ADDR);
    if (!failed) {
        u8 value;
        if (!ricoh61x_read_reg(RICOH619_POFFHIS, &value, 1))
            debug("PMU-Ricoh619 power off caused by: 0x%08x\n", value);
    }

    return failed;
}
