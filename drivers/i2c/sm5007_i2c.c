#include <config.h>
#include <common.h>
#include <linux/err.h>
#include <linux/list.h>
#include <regulator.h>
#include <i2c.h>
#include <malloc.h>

static inline int __sm5007_read(unsigned char addr, u8 reg, uint8_t *val) {
    int ret;

    ret = i2c_read(addr, reg, 1, val, 1);
    if (ret != 0) {
        return -1;
    }
    return 0;
}

static inline int __sm5007_write(unsigned char addr, u8 reg, uint8_t *val) {
    int ret;

    ret = i2c_write(addr, reg, 1, val, 1);
    if (ret != 0) {
        return -1;
    }

    return 0;
}

int sm5007_set_bits(unsigned char addr, u8 reg, uint8_t bit_mask) {
    uint8_t reg_val;
    int ret = 0;

    ret = __sm5007_read(addr, reg, &reg_val);
    if (ret)
        goto out;

    if ((reg_val & bit_mask) != bit_mask) {
        reg_val |= bit_mask;
        ret = __sm5007_write(addr, reg, &reg_val);
    }
    out: return ret;
}

int sm5007_clr_bits(unsigned char addr, u8 reg, uint8_t bit_mask) {
    uint8_t reg_val;
    int ret = 0;

    ret = __sm5007_read(addr, reg, &reg_val);
    if (ret)
        goto out;

    if (reg_val & bit_mask) {
        reg_val &= ~bit_mask;
        ret = __sm5007_write(addr, reg, &reg_val);
    }
    out: return ret;
}

int sm5007_read(unsigned char addr, u8 reg, uint8_t *val) {
    int ret = 0;

    ret = __sm5007_read(addr, reg, val);

    return ret;
}

int sm5007_write(unsigned char addr, u8 reg, uint8_t val) {
    int ret = 0;

    ret = __sm5007_write(addr, reg, &val);

    return ret;
}
