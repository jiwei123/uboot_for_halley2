#ifndef __NAND_CHIP_H__
#define __NAND_CHIP_H__

#include <ingenic_nand_mgr/nand_param.h>

/* NAND Flash Manufacturer ID Codes */
#define NAND_MFR_TOSHIBA	0x98	// Toshiba
#define NAND_MFR_SAMSUNG	0xec	// Samsung
#define NAND_MFR_FUJITSU	0x04	// Fujitsu
#define NAND_MFR_NATIONAL	0x8f	// National
#define NAND_MFR_RENESAS	0x07	// Renesas
#define NAND_MFR_STMICRO	0x20	// ST Micro
#define NAND_MFR_HYNIX		0xad	// Hynix
#define NAND_MFR_MICRON		0x2c	// Micron
#define NAND_MFR_AMD		0x01	// AMD/Spansion
#define NAND_MFR_MACRONIX	0xc2	// Macronix
#define NAND_MFR_EON		0x92	// Eon

/**
 * nand_flash->options bitmap,
 * low 16bit is support switch,
 * hight 16bit is value of support.
 **/
#define NAND_CACHE_READ         (1 << 0)	// support cache read operation
#define NAND_CACHE_PROGRAM      (1 << 1)	// support page cache program operation
#define NAND_MULTI_READ         (1 << 2)	// support multi-plane page read operation
#define NAND_MULTI_PROGRAM      (1 << 3)	// support multi-plane page program operation
#define NAND_TIMING_MODE	(1 << 4)	// support select timing mode
#define NAND_DRIVER_STRENGTH    (1 << 5)	// support select driver stength
#define NAND_RB_PULL_DOWN_STRENGTH    (1 << 6)	// support select rb pull_down strength
#define NAND_READ_RETRY	        (1 << 8)	// support read retry
#define NAND_MICRON_NORMAL      (1 << 9)	// the command of two-planes read is 00-32-00-30
#define NAND_MICRON_PARTICULAR  (1 << 10)	// the command of two-planes read is 00-00-30
#define NAND_READ_RETRY_MODE(n)	((n & 0x0f) << 16)	// read retry mode (bit(16) ~ bit(19): 0 <= (n) <= 16)
#define NAND_TIMING_MODE_V(n)	((n & 0x0f) << 20)	// timing mode (bit(20) ~ bit(23): 0 <= (n) <= 16)

enum hynix_retry_mode {
	HY_RR_F26_32G_MLC,
	HY_RR_F20_64G_MLC_A,
	HY_RR_F20_64G_MLC_B,
	HY_RR_F20_32G_MLC_C,
	HY_RR_F1Y_64G_MLC,
};

enum micron_timing_mode {
	MR_TIMING_MODE0 = 0x00,
	MR_TIMING_MODE1 = 0x01,
	MR_TIMING_MODE2 = 0x02,
	MR_TIMING_MODE3 = 0x03,
	MR_TIMING_MODE4 = 0x04,
	MR_TIMING_MODE5 = 0x05,
};

enum micron_driver_strength {
	MR_DRIVER_STRENGTH_OVER2 = 0x00,
	MR_DRIVER_STRENGTH_OVER1 = 0x01,
	MR_DRIVER_STRENGTH_NORMAL = 0x02,
	MR_DRIVER_STRENGTH_UNDER = 0x03,
};

/**
 * struct __nand_chip_id
 **/
typedef struct __nand_flash_id {
	unsigned short id;
	unsigned int extid;
} nand_flash_id;

#define is_id_null(fid) (((fid)->id == 0) && ((fid)->extid == 0))

#define copy_id(dst_fid, src_fid)			\
	do {						\
		(dst_fid)->id = (src_fid)->id;		\
		(dst_fid)->extid = (src_fid)->extid;	\
	} while (0)					\

#define cmp_id(fid1, fid2) (!(((fid1)->id == (fid2)->id) && ((fid1)->extid == (fid2)->extid)))
const nand_flash *get_nand_flash(nand_flash_id *fid);

#endif /*__NAND_CHIP_H__*/
