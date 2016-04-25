/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * Copyright 2008, Freescale Semiconductor, Inc
 * Andy Fleming
 *
 * Based vaguely on the Linux code
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */

#include "./floader_m200.h"

#define SD_VERSION_SD   0x20000
#define SD_VERSION_3    (SD_VERSION_SD | 0x300)
#define SD_VERSION_2    (SD_VERSION_SD | 0x200)
#define SD_VERSION_1_0  (SD_VERSION_SD | 0x100)
#define SD_VERSION_1_10 (SD_VERSION_SD | 0x10a)
#define MMC_VERSION_MMC     0x10000
#define MMC_VERSION_UNKNOWN (MMC_VERSION_MMC)
#define MMC_VERSION_1_2     (MMC_VERSION_MMC | 0x102)
#define MMC_VERSION_1_4     (MMC_VERSION_MMC | 0x104)
#define MMC_VERSION_2_2     (MMC_VERSION_MMC | 0x202)
#define MMC_VERSION_3       (MMC_VERSION_MMC | 0x300)
#define MMC_VERSION_4       (MMC_VERSION_MMC | 0x400)
#define MMC_VERSION_4_1     (MMC_VERSION_MMC | 0x401)
#define MMC_VERSION_4_2     (MMC_VERSION_MMC | 0x402)
#define MMC_VERSION_4_3     (MMC_VERSION_MMC | 0x403)
#define MMC_VERSION_4_41    (MMC_VERSION_MMC | 0x429)
#define MMC_VERSION_4_5     (MMC_VERSION_MMC | 0x405)

#define MMC_MODE_HS     0x001
#define MMC_MODE_HS_52MHz   0x010
#define MMC_MODE_1BIT       0x000
#define MMC_MODE_4BIT       0x100
#define MMC_MODE_8BIT       0x200
#define MMC_MODE_SPI        0x400
#define MMC_MODE_HC     0x800

#define MMC_MODE_MASK_WIDTH_BITS (MMC_MODE_1BIT | MMC_MODE_4BIT | MMC_MODE_8BIT)
#define MMC_MODE_WIDTH_BITS_SHIFT 8

#define SD_DATA_4BIT    0x00040000

#define MMC_DATA_READ       1

#define NO_CARD_ERR     -16 /* No SD/MMC card inserted */
#define UNUSABLE_ERR        -17 /* Unusable Card */
#define COMM_ERR        -18 /* Communications Error */
#define TIMEOUT         -19
#define IN_PROGRESS     -20 /* operation is in progress */

#define MMC_CMD_GO_IDLE_STATE       0
#define MMC_CMD_SEND_OP_COND        1
#define MMC_CMD_ALL_SEND_CID        2
#define MMC_CMD_SET_RELATIVE_ADDR   3
#define MMC_CMD_SET_DSR         4
#define MMC_CMD_SWITCH          6
#define MMC_CMD_SELECT_CARD     7
#define MMC_CMD_SEND_EXT_CSD        8
#define MMC_CMD_SEND_CSD        9
#define MMC_CMD_SEND_CID        10
#define MMC_CMD_STOP_TRANSMISSION   12
#define MMC_CMD_SEND_STATUS     13
#define MMC_CMD_SET_BLOCKLEN        16
#define MMC_CMD_READ_SINGLE_BLOCK   17
#define MMC_CMD_READ_MULTIPLE_BLOCK 18
#define MMC_CMD_WRITE_SINGLE_BLOCK  24
#define MMC_CMD_WRITE_MULTIPLE_BLOCK    25
#define MMC_CMD_ERASE_GROUP_START   35
#define MMC_CMD_ERASE_GROUP_END     36
#define MMC_CMD_ERASE           38
#define MMC_CMD_APP_CMD         55
#define MMC_CMD_SPI_READ_OCR        58
#define MMC_CMD_SPI_CRC_ON_OFF      59
#define MMC_CMD_RES_MAN         62

#define MMC_CMD62_ARG1          0xefac62ec
#define MMC_CMD62_ARG2          0xcbaea7

#define SD_CMD_SEND_RELATIVE_ADDR   3
#define SD_CMD_SWITCH_FUNC      6
#define SD_CMD_SEND_IF_COND     8

#define SD_CMD_APP_SET_BUS_WIDTH    6
#define SD_CMD_ERASE_WR_BLK_START   32
#define SD_CMD_ERASE_WR_BLK_END     33
#define SD_CMD_APP_SEND_OP_COND     41
#define SD_CMD_APP_SEND_SCR     51

/* SCR definitions in different words */
#define SD_HIGHSPEED_BUSY   0x00020000
#define SD_HIGHSPEED_SUPPORTED  0x00020000

#define MMC_HS_TIMING       0x00000100
#define MMC_HS_52MHZ        0x2

#define OCR_BUSY        0x80000000
#define OCR_HCS         0x40000000
#define OCR_VOLTAGE_MASK    0x007FFF80
#define OCR_ACCESS_MODE     0x60000000

#define SECURE_ERASE        0x80000000

#define MMC_STATUS_MASK     (~0x0206BF7F)
#define MMC_STATUS_RDY_FOR_DATA (1 << 8)
#define MMC_STATUS_CURR_STATE   (0xf << 9)
#define MMC_STATUS_ERROR    (1 << 19)

#define MMC_STATE_PRG       (7 << 9)

#define MMC_CSD_PERM_WRITE_PROTECT  (1 << 13)

#define MMC_VDD_165_195     0x00000080  /* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21       0x00000100  /* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22       0x00000200  /* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23       0x00000400  /* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24       0x00000800  /* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25       0x00001000  /* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26       0x00002000  /* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27       0x00004000  /* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28       0x00008000  /* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29       0x00010000  /* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30       0x00020000  /* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31       0x00040000  /* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32       0x00080000  /* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33       0x00100000  /* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34       0x00200000  /* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35       0x00400000  /* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36       0x00800000  /* VDD voltage 3.5 ~ 3.6 */

#define MMC_SWITCH_MODE_CMD_SET     0x00 /* Change the command set */
#define MMC_SWITCH_MODE_SET_BITS    0x01 /* Set bits in EXT_CSD byte
                        addressed by index which are
                        1 in value field */
#define MMC_SWITCH_MODE_CLEAR_BITS  0x02 /* Clear bits in EXT_CSD byte
                        addressed by index, which are
                        1 in value field */
#define MMC_SWITCH_MODE_WRITE_BYTE  0x03 /* Set target byte to value */

#define SD_SWITCH_CHECK     0
#define SD_SWITCH_SWITCH    1

/*
 * EXT_CSD fields
 */
#define EXT_CSD_GP_SIZE_MULT        143 /* R/W */
#define EXT_CSD_PARTITIONING_SUPPORT    160 /* RO */
#define EXT_CSD_RPMB_MULT       168 /* RO */
#define EXT_CSD_ERASE_GROUP_DEF     175 /* R/W */
#define EXT_CSD_BOOT_BUS_WIDTH      177
#define EXT_CSD_PART_CONF       179 /* R/W */
#define EXT_CSD_BUS_WIDTH       183 /* R/W */
#define EXT_CSD_HS_TIMING       185 /* R/W */
#define EXT_CSD_REV         192 /* RO */
#define EXT_CSD_CARD_TYPE       196 /* RO */
#define EXT_CSD_SEC_CNT         212 /* RO, 4 bytes */
#define EXT_CSD_HC_WP_GRP_SIZE      221 /* RO */
#define EXT_CSD_HC_ERASE_GRP_SIZE   224 /* RO */
#define EXT_CSD_BOOT_MULT       226 /* RO */

/*
 * EXT_CSD field definitions
 */

#define EXT_CSD_CMD_SET_NORMAL      (1 << 0)
#define EXT_CSD_CMD_SET_SECURE      (1 << 1)
#define EXT_CSD_CMD_SET_CPSECURE    (1 << 2)

#define EXT_CSD_CARD_TYPE_26    (1 << 0)    /* Card can run at 26MHz */
#define EXT_CSD_CARD_TYPE_52    (1 << 1)    /* Card can run at 52MHz */

#define EXT_CSD_BUS_WIDTH_1 0   /* Card is in 1 bit mode */
#define EXT_CSD_BUS_WIDTH_4 1   /* Card is in 4 bit mode */
#define EXT_CSD_BUS_WIDTH_8 2   /* Card is in 8 bit mode */

#define EXT_CSD_BOOT_ACK_ENABLE         (1 << 6)
#define EXT_CSD_BOOT_PARTITION_ENABLE       (1 << 3)
#define EXT_CSD_PARTITION_ACCESS_ENABLE     (1 << 0)
#define EXT_CSD_PARTITION_ACCESS_DISABLE    (0 << 0)

#define EXT_CSD_BOOT_ACK(x)     (x << 6)
#define EXT_CSD_BOOT_PART_NUM(x)    (x << 3)
#define EXT_CSD_PARTITION_ACCESS(x) (x << 0)

#define R1_ILLEGAL_COMMAND      (1 << 22)
#define R1_APP_CMD          (1 << 5)

#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136 (1 << 1)        /* 136 bit response */
#define MMC_RSP_CRC (1 << 2)        /* expect valid crc */
#define MMC_RSP_BUSY    (1 << 3)        /* card may send busy */
#define MMC_RSP_OPCODE  (1 << 4)        /* response contains opcode */

#define MMC_RSP_NONE    (0)
#define MMC_RSP_R1  (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1b (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE| \
            MMC_RSP_BUSY)
#define MMC_RSP_R2  (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3  (MMC_RSP_PRESENT)
#define MMC_RSP_R4  (MMC_RSP_PRESENT)
#define MMC_RSP_R5  (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R6  (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R7  (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)

/*
 * ============================================================================
 * MMC core driver
 * ============================================================================
 */

struct mmc_cmd {
    ushort cmdidx;
    uint resp_type;
    uint cmdarg;
    uint response[4];
};

struct mmc_data {
    union {
        char *dest;
        const char *src; /* src buffers don't get written to */
    };
    uint flags;
    uint blocks;
    uint blocksize;
};

struct mmc {
    void *priv;
    uint voltages;
    uint has_init;
    uint f_min;
    uint f_max;
    int high_capacity;
    uint bus_width;
    uint clock;
    uint card_caps;
    uint host_caps;
    uint ocr;
    ushort rca;
    uint tran_speed;
    uint read_bl_len;
    int (*send_cmd)(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data);
    void (*set_ios)(struct mmc *mmc);
    int (*init)(struct mmc *mmc);
    uint b_max;
    char op_cond_pending; /* 1 if we are waiting on an op_cond command */
    char init_in_progress; /* 1 if we have done mmc_start_init() */
    uint op_cond_response; /* the response byte from the last op_cond */
};


/* Set block count limit because of 16 bit register limit on some hardware*/
#ifndef CONFIG_SYS_MMC_MAX_BLK_COUNT
#define CONFIG_SYS_MMC_MAX_BLK_COUNT 65535
#endif

static struct mmc *the_device;

static int mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data)
{
    int ret;

    ret = mmc->send_cmd(mmc, cmd, data);
    return ret;
}

static int mmc_send_status(struct mmc *mmc, int timeout)
{
    struct mmc_cmd cmd;
    int err, retries = 5;

    cmd.cmdidx = MMC_CMD_SEND_STATUS;
    cmd.resp_type = MMC_RSP_R1;
    cmd.cmdarg = mmc->rca << 16;

    do {
        err = mmc_send_cmd(mmc, &cmd, NULL);
        if (!err) {
            if ((cmd.response[0] & MMC_STATUS_RDY_FOR_DATA)
                    && (cmd.response[0] & MMC_STATUS_CURR_STATE) !=
                    MMC_STATE_PRG)
                break;
            else if (cmd.response[0] & MMC_STATUS_MASK) {
                debug("msc0: Status Error: 0x%08X\n", cmd.response[0]);

                return COMM_ERR;
            }
        } else if (--retries < 0) {
            return err;
        }

        udelay(1000);

    } while (timeout--);

    if (timeout <= 0) {
        debug("msc0: Timeout waiting card ready\n");

        return TIMEOUT;
    }

    return 0;
}

static int mmc_set_blocklen(struct mmc *mmc, int len)
{
    struct mmc_cmd cmd;

    cmd.cmdidx = MMC_CMD_SET_BLOCKLEN;
    cmd.resp_type = MMC_RSP_R1;
    cmd.cmdarg = len;

    return mmc_send_cmd(mmc, &cmd, NULL);
}

static int mmc_read_blocks(struct mmc *mmc, void *dst, lbaint_t start,
        lbaint_t blkcnt)
{
    struct mmc_cmd cmd;
    struct mmc_data data;

    if (blkcnt > 1)
        cmd.cmdidx = MMC_CMD_READ_MULTIPLE_BLOCK;
    else
        cmd.cmdidx = MMC_CMD_READ_SINGLE_BLOCK;

    if (mmc->high_capacity)
        cmd.cmdarg = start;
    else
        cmd.cmdarg = start * mmc->read_bl_len;

    cmd.resp_type = MMC_RSP_R1;

    data.dest = dst;
    data.blocks = blkcnt;
    data.blocksize = mmc->read_bl_len;
    data.flags = MMC_DATA_READ;

    if (mmc_send_cmd(mmc, &cmd, &data))
        return 0;

    if (blkcnt > 1) {
        cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
        cmd.cmdarg = 0;
        cmd.resp_type = MMC_RSP_R1b;
        if (mmc_send_cmd(mmc, &cmd, NULL)) {
            debug("msc0: failed to send stop cmd\n");

            return 0;
        }
    }

    return blkcnt;
}

ulong mmc_bread(lbaint_t start, lbaint_t blkcnt, void *dst)
{
    lbaint_t cur, blocks_todo = blkcnt;

    if (blkcnt == 0)
        return 0;

    struct mmc *mmc = the_device;
    if (!mmc)
        return 0;

    if (mmc_set_blocklen(mmc, mmc->read_bl_len))
        return 0;

    do {
        cur = (blocks_todo > mmc->b_max) ? mmc->b_max : blocks_todo;
        if (mmc_read_blocks(mmc, dst, start, cur) != cur)
            return 0;
        blocks_todo -= cur;
        start += cur;
        dst += cur * mmc->read_bl_len;
    } while (blocks_todo > 0);

    return blkcnt;
}

static int mmc_go_idle(struct mmc *mmc)
{
    struct mmc_cmd cmd;
    int err;

    udelay(1000);

    cmd.cmdidx = MMC_CMD_GO_IDLE_STATE;
    cmd.cmdarg = 0;
    cmd.resp_type = MMC_RSP_NONE;

    err = mmc_send_cmd(mmc, &cmd, NULL);

    if (err)
        return err;

    udelay(2000);

    return 0;
}

/* We pass in the cmd since otherwise the init seems to fail */
static int mmc_send_op_cond_iter(struct mmc *mmc, struct mmc_cmd *cmd,
        int use_arg)
{
    int err;

    cmd->cmdidx = MMC_CMD_SEND_OP_COND;
    cmd->resp_type = MMC_RSP_R3;
    cmd->cmdarg = 0;
    if (use_arg) {
        cmd->cmdarg = (mmc->voltages
                & (mmc->op_cond_response & OCR_VOLTAGE_MASK))
                | (mmc->op_cond_response & OCR_ACCESS_MODE);

        if (mmc->host_caps & MMC_MODE_HC)
            cmd->cmdarg |= OCR_HCS;
    }
    err = mmc_send_cmd(mmc, cmd, NULL);
    if (err)
        return err;
    mmc->op_cond_response = cmd->response[0];
    return 0;
}

static int mmc_send_op_cond(struct mmc *mmc)
{
    struct mmc_cmd cmd;
    int err, i;

    /* Some cards seem to need this */
    mmc_go_idle(mmc);

    /* Asking to the card its capabilities */
    mmc->op_cond_pending = 1;
    for (i = 0; i < 2; i++) {
        err = mmc_send_op_cond_iter(mmc, &cmd, i != 0);
        if (err)
            return err;

        /* exit if not busy (flag seems to be inverted) */
        if (mmc->op_cond_response & OCR_BUSY) {
            mmc->op_cond_pending = 0;

            mmc->ocr = cmd.response[0];

            mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
            mmc->rca = 0;
            return 0;
        }
    }
    return IN_PROGRESS;
}

static int mmc_complete_op_cond(struct mmc *mmc)
{
    struct mmc_cmd cmd;
    int timeout = 0;
    int err;

    mmc->op_cond_pending = 0;
    do {
        err = mmc_send_op_cond_iter(mmc, &cmd, 1);
        if (err)
            return err;

        if (timeout++ > 1000)
            return UNUSABLE_ERR;

        udelay(100);
    } while (!(mmc->op_cond_response & OCR_BUSY));

    mmc->ocr = cmd.response[0];

    mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
    mmc->rca = 0;

    return 0;
}

static int mmc_switch(struct mmc *mmc, u8 set, u8 index, u8 value)
{
    struct mmc_cmd cmd;
    int timeout = 1000;
    int ret;

    cmd.cmdidx = MMC_CMD_SWITCH;
    cmd.resp_type = MMC_RSP_R1b;
    cmd.cmdarg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) | (index << 16)
            | (value << 8);

    ret = mmc_send_cmd(mmc, &cmd, NULL);

    /* Waiting for the ready status */
    if (!ret)
        ret = mmc_send_status(mmc, timeout);

    return ret;

}

static void mmc_set_ios(struct mmc *mmc)
{
    mmc->set_ios(mmc);
}

static void mmc_set_clock(struct mmc *mmc, uint clock)
{
    if (clock > mmc->f_max)
        clock = mmc->f_max;

    if (clock < mmc->f_min)
        clock = mmc->f_min;

    mmc->clock = clock;

    mmc_set_ios(mmc);
}

static void mmc_set_bus_width(struct mmc *mmc, uint width)
{
    mmc->bus_width = width;

    mmc_set_ios(mmc);
}

static int mmc_startup(struct mmc *mmc)
{
    int err;
    struct mmc_cmd cmd;

    /* Put the Card in Identify Mode */
    cmd.cmdidx = MMC_CMD_ALL_SEND_CID; /* cmd not supported in spi */
    cmd.resp_type = MMC_RSP_R2;
    cmd.cmdarg = 0;

    err = mmc_send_cmd(mmc, &cmd, NULL);

    if (err)
        return err;

    /*
     * For MMC cards, set the Relative Address.
     * For SD cards, get the Relatvie Address.
     * This also puts the cards into Standby State
     */
    cmd.cmdidx = SD_CMD_SEND_RELATIVE_ADDR;
    cmd.cmdarg = mmc->rca << 16;
    cmd.resp_type = MMC_RSP_R6;

    err = mmc_send_cmd(mmc, &cmd, NULL);
    if (err)
        return err;

    /* Select the card, and put it into Transfer Mode */
    mmc->read_bl_len = CONFIG_EMMC_MSCO_BLOCK_SIZE;
    cmd.cmdidx = MMC_CMD_SELECT_CARD;
    cmd.resp_type = MMC_RSP_R1;
    cmd.cmdarg = mmc->rca << 16;
    err = mmc_send_cmd(mmc, &cmd, NULL);
    if (err)
        return err;

    /* Restrict card's capabilities by what the host can do */
    mmc->card_caps = mmc->host_caps;

    if (mmc->host_caps & MMC_MODE_8BIT) {
        mmc_switch(
                mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BUS_WIDTH, EXT_CSD_BUS_WIDTH_8);
        mmc_set_bus_width(mmc, 8);
    } else if (mmc->host_caps & MMC_MODE_4BIT) {
        mmc_switch(
                mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BUS_WIDTH, EXT_CSD_BUS_WIDTH_4);
        mmc_set_bus_width(mmc, 4);
    } else {
        mmc_switch(
                mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BUS_WIDTH, EXT_CSD_BUS_WIDTH_1);
        mmc_set_bus_width(mmc, 1);
    }

    if (mmc->host_caps & MMC_MODE_HS_52MHz)
        mmc->tran_speed = 52000000;
    else
        mmc->tran_speed = CONFIG_EMMC_MSC0_FREQ_MHZ * 1000 * 1000;

    mmc_set_clock(mmc, mmc->tran_speed);

    return 0;
}

static int mmc_start_init(struct mmc *mmc)
{
    int err;

    if (mmc->has_init)
        return 0;

    if (mmc->init) {
        err = mmc->init(mmc);
        if (err)
            return err;
    }

    mmc_set_bus_width(mmc, 1);

    mmc_set_clock(mmc, 1);

    /* Reset the Card */
    err = mmc_go_idle(mmc);
    if (err)
        return err;

    /* Check for an MMC card */
    err = mmc_send_op_cond(mmc);
    if (err && err != IN_PROGRESS) {
        debug("msc0: Card did not respond to voltage select!\n");

        return UNUSABLE_ERR;
    }

    if (err == IN_PROGRESS)
        mmc->init_in_progress = 1;

    return err;
}

static int mmc_complete_init(struct mmc *mmc)
{
    int err = 0;

    if (mmc->op_cond_pending)
        err = mmc_complete_op_cond(mmc);

    if (!err)
        err = mmc_startup(mmc);
    if (err)
        mmc->has_init = 0;
    else
        mmc->has_init = 1;

    mmc->init_in_progress = 0;

    return err;
}

static int mmc_init(struct mmc *mmc)
{
    the_device = mmc;

    /* Setup the universal parts of the block interface just once */
    /* mmc->block_dev.if_type = IF_TYPE_MMC; */
    if (!the_device->b_max)
        the_device->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

    int err = IN_PROGRESS;

    if (the_device->has_init)
        return 0;

    if (!the_device->init_in_progress)
        err = mmc_start_init(the_device);

    if (!err || err == IN_PROGRESS)
        err = mmc_complete_init(the_device);

    return err;
}

/*
 * ============================================================================
 * MMC host driver
 * ============================================================================
 */

#define MSC_STRPCL      0x000
#define MSC_STAT        0x004
#define MSC_CLKRT       0x008
#define MSC_CMDAT       0x00C
#define MSC_RESTO       0x010
#define MSC_RDTO        0x014
#define MSC_BLKLEN      0x018
#define MSC_NOB         0x01C
#define MSC_SNOB        0x020
#define MSC_IMASK       0x024
#define MSC_IREG        0x028
#define MSC_CMD         0x02C
#define MSC_ARG         0x030
#define MSC_RES         0x034
#define MSC_RXFIFO      0x038
#define MSC_TXFIFO      0x03C
#define MSC_LPM         0x040
#define MSC_DBG         0x0fc
#define MSC_DMAC        0x044
#define MSC_DMANDA      0x048
#define MSC_DMADA       0x04c
#define MSC_DMALEN      0x050
#define MSC_DMACMD      0x054
#define MSC_CTRL2       0x058
#define MSC_RTCNT       0x05c

/* MSC Clock and Control Register (MSC_STRPCL) */

#define MSC_STRPCL_EXIT_MULTIPLE    (1 << 7)
#define MSC_STRPCL_EXIT_TRANSFER    (1 << 6)
#define MSC_STRPCL_START_READWAIT   (1 << 5)
#define MSC_STRPCL_STOP_READWAIT    (1 << 4)
#define MSC_STRPCL_RESET        (1 << 3)
#define MSC_STRPCL_START_OP     (1 << 2)
#define MSC_STRPCL_CLOCK_CONTROL_BIT    0
#define MSC_STRPCL_CLOCK_CONTROL_MASK   (0x3 << MSC_STRPCL_CLOCK_CONTROL_BIT)
#define MSC_STRPCL_CLOCK_CONTROL_STOP   (0x1 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Stop MMC/SD clock */
#define MSC_STRPCL_CLOCK_CONTROL_START  (0x2 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Start MMC/SD clock */

/* MSC Status Register (MSC_STAT) */

#define MSC_STAT_AUTO_CMD_DONE      (1 << 31)
#define MSC_STAT_IS_RESETTING       (1 << 15)
#define MSC_STAT_SDIO_INT_ACTIVE    (1 << 14)
#define MSC_STAT_PRG_DONE       (1 << 13)
#define MSC_STAT_DATA_TRAN_DONE     (1 << 12)
#define MSC_STAT_END_CMD_RES        (1 << 11)
#define MSC_STAT_DATA_FIFO_AFULL    (1 << 10)
#define MSC_STAT_IS_READWAIT        (1 << 9)
#define MSC_STAT_CLK_EN         (1 << 8)
#define MSC_STAT_DATA_FIFO_FULL     (1 << 7)
#define MSC_STAT_DATA_FIFO_EMPTY    (1 << 6)
#define MSC_STAT_CRC_RES_ERR        (1 << 5)
#define MSC_STAT_CRC_READ_ERROR     (1 << 4)
#define MSC_STAT_CRC_WRITE_ERROR_BIT    2
#define MSC_STAT_CRC_WRITE_ERROR_MASK   (0x3 << MSC_STAT_CRC_WRITE_ERROR_BIT)
#define MSC_STAT_CRC_WRITE_ERROR_NO       (0 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No error on transmission of data */
#define MSC_STAT_CRC_WRITE_ERROR      (1 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* Card observed erroneous transmission of data */
#define MSC_STAT_CRC_WRITE_ERROR_NOSTS    (2 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No CRC status is sent back */
#define MSC_STAT_TIME_OUT_RES       (1 << 1)
#define MSC_STAT_TIME_OUT_READ      (1 << 0)

/* MSC Bus Clock Control Register (MSC_CLKRT) */

#define MSC_CLKRT_CLK_RATE_BIT      0
#define MSC_CLKRT_CLK_RATE_MASK     (0x7 << MSC_CLKRT_CLK_RATE_BIT)
#define MSC_CLKRT_CLK_RATE_DIV_1    (0x0 << MSC_CLKRT_CLK_RATE_BIT) /* CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_2    (0x1 << MSC_CLKRT_CLK_RATE_BIT) /* 1/2 of CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_4    (0x2 << MSC_CLKRT_CLK_RATE_BIT) /* 1/4 of CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_8    (0x3 << MSC_CLKRT_CLK_RATE_BIT) /* 1/8 of CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_16   (0x4 << MSC_CLKRT_CLK_RATE_BIT) /* 1/16 of CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_32   (0x5 << MSC_CLKRT_CLK_RATE_BIT) /* 1/32 of CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_64   (0x6 << MSC_CLKRT_CLK_RATE_BIT) /* 1/64 of CLK_SRC */
#define MSC_CLKRT_CLK_RATE_DIV_128      (0x7 << MSC_CLKRT_CLK_RATE_BIT) /* 1/128 of CLK_SRC */

/* MSC Command Sequence Control Register (MSC_CMDAT) */

#define MSC_CMDAT_IO_ABORT      (1 << 11)
#define MSC_CMDAT_BUS_WIDTH_BIT     9
#define MSC_CMDAT_BUS_WIDTH_MASK    (0x3 << MSC_CMDAT_BUS_WIDTH_BIT)
#define MSC_CMDAT_BUS_WIDTH_1BIT    (0x0 << MSC_CMDAT_BUS_WIDTH_BIT) /* 1-bit data bus */
#define MSC_CMDAT_BUS_WIDTH_4BIT    (0x2 << MSC_CMDAT_BUS_WIDTH_BIT) /* 4-bit data bus */
#define CMDAT_BUS_WIDTH1    (0x0 << MSC_CMDAT_BUS_WIDTH_BIT)
#define CMDAT_BUS_WIDTH4    (0x2 << MSC_CMDAT_BUS_WIDTH_BIT)
#define MSC_CMDAT_DMA_EN        (1 << 8)
#define MSC_CMDAT_INIT          (1 << 7)
#define MSC_CMDAT_BUSY          (1 << 6)
#define MSC_CMDAT_STREAM_BLOCK      (1 << 5)
#define MSC_CMDAT_WRITE         (1 << 4)
#define MSC_CMDAT_READ          (0 << 4)
#define MSC_CMDAT_DATA_EN       (1 << 3)
#define MSC_CMDAT_RESPONSE_BIT  0
#define MSC_CMDAT_RESPONSE_MASK (0x7 << MSC_CMDAT_RESPONSE_BIT)
#define MSC_CMDAT_RESPONSE_NONE  (0x0 << MSC_CMDAT_RESPONSE_BIT) /* No response */
#define MSC_CMDAT_RESPONSE_R1   (0x1 << MSC_CMDAT_RESPONSE_BIT) /* Format R1 and R1b */
#define MSC_CMDAT_RESPONSE_R2   (0x2 << MSC_CMDAT_RESPONSE_BIT) /* Format R2 */
#define MSC_CMDAT_RESPONSE_R3   (0x3 << MSC_CMDAT_RESPONSE_BIT) /* Format R3 */
#define MSC_CMDAT_RESPONSE_R4   (0x4 << MSC_CMDAT_RESPONSE_BIT) /* Format R4 */
#define MSC_CMDAT_RESPONSE_R5   (0x5 << MSC_CMDAT_RESPONSE_BIT) /* Format R5 */
#define MSC_CMDAT_RESPONSE_R6   (0x6 << MSC_CMDAT_RESPONSE_BIT) /* Format R6 */

#define CMDAT_DMA_EN    (1 << 8)
#define CMDAT_INIT  (1 << 7)
#define CMDAT_BUSY  (1 << 6)
#define CMDAT_STREAM    (1 << 5)
#define CMDAT_WRITE (1 << 4)
#define CMDAT_DATA_EN   (1 << 3)

/* MSC Interrupts Mask Register (MSC_IMASK) */

#define MSC_IMASK_TIME_OUT_RES (1 << 9)
#define MSC_IMASK_TIME_OUT_READ (1 << 8)
#define MSC_IMASK_SDIO          (1 << 7)
#define MSC_IMASK_TXFIFO_WR_REQ     (1 << 6)
#define MSC_IMASK_RXFIFO_RD_REQ     (1 << 5)
#define MSC_IMASK_END_CMD_RES       (1 << 2)
#define MSC_IMASK_PRG_DONE      (1 << 1)
#define MSC_IMASK_DATA_TRAN_DONE    (1 << 0)

/* MSC Interrupts Status Register (MSC_IREG) */

#define MSC_IREG_TIME_OUT_RES (1 << 9)
#define MSC_IREG_TIME_OUT_READ (1 << 8)
#define MSC_IREG_SDIO           (1 << 7)
#define MSC_IREG_TXFIFO_WR_REQ      (1 << 6)
#define MSC_IREG_RXFIFO_RD_REQ      (1 << 5)
#define MSC_IREG_END_CMD_RES        (1 << 2)
#define MSC_IREG_PRG_DONE       (1 << 1)
#define MSC_IREG_DATA_TRAN_DONE     (1 << 0)

#define LPM_DRV_SEL_SHF         30
#define LPM_DRV_SEL_MASK        (0x3 << LPM_DRV_SEL_SHF)
#define LPM_SMP_SEL         (1 << 29)
#define LPM_LPM             (1 << 0)

struct jz_mmc_priv {
    uintptr_t base;
    uint32_t flags;
};

/* jz_mmc_priv flags */
#define JZ_MMC_BUS_WIDTH_MASK 0x3
#define JZ_MMC_BUS_WIDTH_1    0x0
#define JZ_MMC_BUS_WIDTH_4    0x2
#define JZ_MMC_BUS_WIDTH_8    0x3
#define JZ_MMC_SENT_INIT (1 << 2)

static uint16_t jz_mmc_readw(struct jz_mmc_priv *priv, uintptr_t off)
{
    return readw(priv->base + off);
}

static uint32_t jz_mmc_readl(struct jz_mmc_priv *priv, uintptr_t off)
{
    return readl(priv->base + off);
}

static void jz_mmc_writel(uint32_t value, struct jz_mmc_priv *priv,
        uintptr_t off)
{
    writel(value, priv->base + off);
}

static int jz_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
        struct mmc_data *data)
{
    struct jz_mmc_priv *priv = mmc->priv;
    uint32_t stat, cmdat = 0;

    /* setup command */
    jz_mmc_writel(cmd->cmdidx, priv, MSC_CMD);
    jz_mmc_writel(cmd->cmdarg, priv, MSC_ARG);

    debug("msc0: command index = %u\n", cmd->cmdidx);

    if (data) {
        /* setup data */
        cmdat |= MSC_CMDAT_DATA_EN;

        jz_mmc_writel(data->blocks, priv, MSC_NOB);
        jz_mmc_writel(data->blocksize, priv, MSC_BLKLEN);
    }

    /* setup response */
    switch (cmd->resp_type) {
    case MMC_RSP_R1:
    case MMC_RSP_R1b:
        cmdat |= MSC_CMDAT_RESPONSE_R1;
        break;
    case MMC_RSP_R2:
        cmdat |= MSC_CMDAT_RESPONSE_R2;
        break;
    case MMC_RSP_R3:
        cmdat |= MSC_CMDAT_RESPONSE_R3;
        break;
    default:
        break;
    }

    if (cmd->resp_type & MMC_RSP_BUSY)
        cmdat |= MSC_CMDAT_BUSY;

    /* set init for the first command only */
    if (!(priv->flags & JZ_MMC_SENT_INIT)) {
        cmdat |= MSC_CMDAT_INIT;
        priv->flags |= JZ_MMC_SENT_INIT;
    }

    cmdat |= (priv->flags & JZ_MMC_BUS_WIDTH_MASK) << 9;

    /* write the data setup */
    jz_mmc_writel(cmdat, priv, MSC_CMDAT);

    jz_mmc_writel(0xffffffff, priv, MSC_IMASK);
    /* clear interrupts */
    jz_mmc_writel(0xffffffff, priv, MSC_IREG);

    /* start the command (& the clock) */
    jz_mmc_writel(MSC_STRPCL_START_OP, priv, MSC_STRPCL);

    /* wait for completion */
    while (!(stat = (jz_mmc_readl(priv, MSC_IREG)
            & (MSC_IREG_END_CMD_RES | MSC_IREG_TIME_OUT_RES)))) {
        udelay(100);
    }

    jz_mmc_writel(stat, priv, MSC_IREG);
    if (stat & MSC_IREG_TIME_OUT_RES) {
        debug("msc0: send command timeout!\n");
        return TIMEOUT;
    }

    if (cmd->resp_type & MMC_RSP_PRESENT) {
        /* read the response */
        if (cmd->resp_type & MMC_RSP_136) {
            uint16_t a, b, c, i;
            a = jz_mmc_readw(priv, MSC_RES);
            for (i = 0; i < 4; i++) {
                b = jz_mmc_readw(priv, MSC_RES);
                c = jz_mmc_readw(priv, MSC_RES);
                cmd->response[i] = (a << 24) | (b << 8) | (c >> 8);
                a = c;
            }
        } else {
            cmd->response[0] = jz_mmc_readw(priv, MSC_RES) << 24;
            cmd->response[0] |= jz_mmc_readw(priv, MSC_RES) << 8;
            cmd->response[0] |= jz_mmc_readw(priv, MSC_RES) & 0xff;
        }
    }

    if (cmd->resp_type == MMC_RSP_R1b) {
        while (!(jz_mmc_readl(priv, MSC_STAT) & MSC_STAT_PRG_DONE))
            continue;

        jz_mmc_writel(MSC_IREG_PRG_DONE, priv, MSC_IREG);
    }

    if (data && (data->flags & MMC_DATA_READ)) {
        /* read the data */
        int sz = data->blocks * data->blocksize;
        void *buf = data->dest;
        do {
            stat = jz_mmc_readl(priv, MSC_STAT);
            if (stat & MSC_STAT_TIME_OUT_READ)
                return TIMEOUT;
            if (stat & MSC_STAT_CRC_READ_ERROR)
                return COMM_ERR;
            if (stat & MSC_STAT_DATA_FIFO_EMPTY) {
                udelay(100);
                continue;
            }
            do {
                uint32_t val = jz_mmc_readl(priv, MSC_RXFIFO);
                if (sz == 1)
                    *(uint8_t *) buf = (uint8_t) val;
                else if (sz == 2)
                    put_unaligned_le16(val, buf);
                else if (sz >= 4)
                    put_unaligned_le32(val, buf);
                buf += 4;
                sz -= 4;
                stat = jz_mmc_readl(priv, MSC_STAT);
            } while (!(stat & MSC_STAT_DATA_FIFO_EMPTY));
        } while (!(stat & MSC_STAT_DATA_TRAN_DONE));

        while (!(jz_mmc_readl(priv, MSC_IREG) & MSC_IREG_DATA_TRAN_DONE))
            continue;

        jz_mmc_writel(MSC_IREG_DATA_TRAN_DONE, priv, MSC_IREG);
    }

    return 0;
}

static void jz_mmc_set_ios(struct mmc *mmc)
{
    struct jz_mmc_priv *priv = mmc->priv;

    uint32_t real_rate = 0;
    uint32_t lpm = LPM_LPM;
    uint8_t clk_div = 0;

    set_msc0_freq(mmc->clock);
    real_rate = get_msc0_freq();

    /* calculate clock divide */
    while ((real_rate > mmc->clock) && (clk_div < 7)) {
        real_rate >>= 1;
        clk_div++;
    }

    jz_mmc_writel(clk_div, priv, MSC_CLKRT);

    if (real_rate > 25000000)
        lpm |= (0x2 << LPM_DRV_SEL_SHF) | LPM_SMP_SEL;

    jz_mmc_writel(lpm, priv, MSC_LPM);

    /* set the bus width for the next command */
    priv->flags &= ~JZ_MMC_BUS_WIDTH_MASK;

    if (mmc->bus_width == 8)
        priv->flags |= JZ_MMC_BUS_WIDTH_8;
    else if (mmc->bus_width == 4)
        priv->flags |= JZ_MMC_BUS_WIDTH_4;
    else
        priv->flags |= JZ_MMC_BUS_WIDTH_1;

    debug("msc0: Clk want: %d, Clk set: %d, Bus width: %d\n",
            mmc->clock,
            get_msc0_freq() / (1 << clk_div),
            mmc->bus_width);
}

static int jz_mmc_core_init(struct mmc *mmc)
{
    int tmp;
    struct jz_mmc_priv *priv = mmc->priv;

    /* reset */
    jz_mmc_writel(MSC_STRPCL_RESET, priv, MSC_STRPCL);

    tmp = jz_mmc_readl(priv, MSC_STRPCL);
    tmp &= ~MSC_STRPCL_RESET;
    jz_mmc_writel(tmp, priv, MSC_STRPCL);

    /* maximum timeouts */
    jz_mmc_writel(0xffffffff, priv, MSC_RESTO);
    jz_mmc_writel(0xffffffff, priv, MSC_RDTO);

    return 0;
}

static void set_gpio_pa_as_msc0_8bit()
{
    gpio_request(GPIO_PA(18), "msc0_clk");
    gpio_request(GPIO_PA(19), "msc0_cmd");
    gpio_request(GPIO_PA(20), "msc0_d0");
    gpio_request(GPIO_PA(21), "msc0_d1");
    gpio_request(GPIO_PA(22), "msc0_d2");
    gpio_request(GPIO_PA(23), "msc0_d3");
    gpio_request(GPIO_PA(4), "msc0_d4");
    gpio_request(GPIO_PA(5), "msc0_d5");
    gpio_request(GPIO_PA(6), "msc0_d6");
    gpio_request(GPIO_PA(7), "msc0_d7");

    gpio_set_func(GPIO_PA(18), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(19), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(20), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(21), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(22), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(23), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(4), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(5), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(6), GPIO_FUNC_1);
    gpio_set_func(GPIO_PA(7), GPIO_FUNC_1);


#if (CONFIG_EMMC_MSC0_PORT_PA_ENABLE_PULL_UP == 1)
    gpio_enable_pullup(GPIO_PA(18));
    gpio_enable_pullup(GPIO_PA(19));
    gpio_enable_pullup(GPIO_PA(20));
    gpio_enable_pullup(GPIO_PA(21));
    gpio_enable_pullup(GPIO_PA(22));
    gpio_enable_pullup(GPIO_PA(23));
    gpio_enable_pullup(GPIO_PA(4));
    gpio_enable_pullup(GPIO_PA(5));
    gpio_enable_pullup(GPIO_PA(6));
    gpio_enable_pullup(GPIO_PA(7));
#else
    gpio_disable_pullup(GPIO_PA(18));
    gpio_disable_pullup(GPIO_PA(19));
    gpio_disable_pullup(GPIO_PA(20));
    gpio_disable_pullup(GPIO_PA(21));
    gpio_disable_pullup(GPIO_PA(22));
    gpio_disable_pullup(GPIO_PA(23));
    gpio_disable_pullup(GPIO_PA(4));
    gpio_disable_pullup(GPIO_PA(5));
    gpio_disable_pullup(GPIO_PA(6));
    gpio_disable_pullup(GPIO_PA(7));
#endif
}

void init_mmc_host()
{
    static struct mmc mmc_dev;
    static struct jz_mmc_priv mmc_priv;

    struct mmc *mmc = &mmc_dev;
    struct jz_mmc_priv *priv = &mmc_priv;

    /* setup priv */
    priv->base = MSC0_BASE;
    priv->flags = 0;

    /* setup mmc */
    mmc->priv = priv;
    mmc->send_cmd = jz_mmc_send_cmd;
    mmc->set_ios = jz_mmc_set_ios;
    mmc->init = jz_mmc_core_init;

    mmc->voltages = MMC_VDD_165_195
                        | MMC_VDD_27_28
                        | MMC_VDD_28_29
                        | MMC_VDD_29_30
                        | MMC_VDD_30_31
                        | MMC_VDD_31_32
                        | MMC_VDD_32_33
                        | MMC_VDD_33_34
                        | MMC_VDD_34_35
                        | MMC_VDD_35_36;

    mmc->f_max = 52000000;
    mmc->f_min = 200000;

    mmc->host_caps = MMC_MODE_HS
                        | MMC_MODE_HC;

#if (CONFIG_EMMC_MSC0_BUS_WIDTH == 1)
    mmc->host_caps |= MMC_MODE_1BIT;

#elif (CONFIG_EMMC_MSC0_BUS_WIDTH == 4)
    mmc->host_caps |= MMC_MODE_4BIT;

#elif (CONFIG_EMMC_MSC0_BUS_WIDTH == 8)
    mmc->host_caps |= MMC_MODE_8BIT;

#endif

    set_gpio_pa_as_msc0_8bit();

    if (mmc_init(mmc))
        stop(STOP_ERROR_EMMC_MSC0_INIT_FAILED);
}
