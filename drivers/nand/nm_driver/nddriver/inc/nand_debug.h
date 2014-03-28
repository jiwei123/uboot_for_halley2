#ifndef _NAND_DEBUG_H_
#define _NAND_DEBUG_H_

#include <nderrno.h>
#include <nddata.h>
#include <nand_api.h>

/* =================== switchs ================== */

/**
 * default debug level, if just switch NDD_WARNING
 * or NDD_INFO, this not effect DEBUG_REWRITE and
 * DEBUG_TIME_WRITE/READ
 **/
#define PRINT_LEVEL		NDD_WARNING
//#define PRINT_LEVEL		NDD_INFO

/**
 * driver init info debug switch
 **/
//#define DEBUG_INIT_INFO

/**
 * speed debug switch
 **/
//#define DEBUG_SPEED

/**
 * rewrite debug switch
 **/
//#define DEBUG_ERR_TSKMSG

/**
 * rewrite debug switch
 **/
//#define DEBUG_REWRITE

/**
 * check write success/faild switch
 **/
//#define DEBUG_WRITE_REREAD
//#define DEBUG_REREAD_WITHDATA

/**
 * ecc error debug switch
 **/
//#define DEBUG_ECCERROR

/**
 * used block check switch
 **/
//#define CHECK_USED_BLOCK

/**
 * page data full debug switch
 **/
//#define DEBUG_WRITE_PAGE_FULL

/**
 * test read full page
 * instead of read random
 **/
//#define TEST_FULL_PAGE_READ

/* =================== print tools ================== */

#define NDD_INFO		0x0
#define NDD_WARNING		0x1
#define NDD_ERROR		0x2
#define NDD_FATE_ERROR 		0x3
#define NDD_DEBUG		0x4

extern int (*ndd_printf)(const char *fmt, ...);
//extern void serial_printf(char *fmt, ...);
#define ndd_print(level, ...) do { if (level >= PRINT_LEVEL) printf(__VA_ARGS__); } while (0)

#define ndd_debug(...) ndd_print(NDD_INFO, __VA_ARGS__)

#define ERR_LABLE(x)	err_##x
#define GOTO_ERR(x)							\
	do {								\
		ndd_print(NDD_ERROR, "ERROR(nand_driver): %s(line:%d), " \
			  "goto err_"#x" !\n", __func__, __LINE__);	\
		goto err_##x;						\
	} while (0)

#define RETURN_ERR(ret, format, ...)					\
	do {								\
		ndd_print(NDD_ERROR, "ERROR(nand_driver): %s(line:%d), " \
			  #format", ret = %d !\n", __func__, __LINE__,	\
			  ##__VA_ARGS__, (int)ret);			\
		return ret;						\
	} while (0)

#define ND_MAX_ERRNO 128
#define IS_PTR_ERR(x) (unsigned int)(x) >= (unsigned int)-ND_MAX_ERRNO)

/* =================== debug interfaces ================== */
void __ndd_dump_plat_partition(plat_ptinfo *plat_ptinfo);
void __ndd_dump_ptinfo(pt_info *ptinfo);
void __ndd_dump_ppartition(PPartition *pt, unsigned int ptcount);
void __ndd_dump_chip_info(chip_info *cinfo);
void __ndd_dump_csinfo(cs_info *csinfo);
void __ndd_dump_registers(void);
void __ndd_dump_rewrite(nand_data *nddata, PPartition *ppt, PageList *pl);
void __ndd_dump_write_reread_prepare(PageList *pl);
void __ndd_dump_write_reread_complete(nand_data *nddata, PPartition *ppt, PageList *pl);
void __ndd_dump_uncorrect_err(nand_data *nddata, PPartition *ppt, PageList *pl);
int __ndd_dump_check_used_block(nand_data *nddata, PPartition *ppt, int blockid);
void __ndd_dump_taskmsg(struct task_msg *msg, int num);
int ndd_dump_status(void);

/* controled by PRINT_LEVEL */
#ifdef DEBUG_INIT_INFO
#define ndd_dump_plat_partition(plat_ptinfo)	__ndd_dump_plat_partition(plat_ptinfo)
#define ndd_dump_ptinfo(ptinfo)			__ndd_dump_ptinfo(ptinfo)
#define ndd_dump_ppartition(pt, ptcount)	__ndd_dump_ppartition(pt, ptcount)
#define ndd_dump_chip_info(cinfo)		__ndd_dump_chip_info(cinfo)
#define ndd_dump_csinfo(csinfo)			__ndd_dump_csinfo(csinfo)
#define ndd_dump_registers()			__ndd_dump_registers()
#else
#define ndd_dump_plat_partition(plat_ptinfo)
#define ndd_dump_ptinfo(ptinfo)
#define ndd_dump_ppartition(pt, ptcount)
#define ndd_dump_chip_info(cinfo)
#define ndd_dump_csinfo(csinfo)
#define ndd_dump_registers(void)
#endif

/* controled by special switch */
#ifdef DEBUG_REWRITE
#define ndd_dump_rewrite(nddata, ppt, pl) __ndd_dump_rewrite(nddata, ppt, pl)
#else
#define ndd_dump_rewrite(nddata, ppt, pl)
#endif

#ifdef DEBUG_WRITE_REREAD
#define ndd_dump_write_reread_prepare(pl) __ndd_dump_write_reread_prepare(pl)
#define ndd_dump_write_reread_complete(nddata, ppt, pl) __ndd_dump_write_reread_complete(nddata, ppt, pl)
#else
#define ndd_dump_write_reread_prepare(pl)
#define ndd_dump_write_reread_complete(nddata, ppt, pl)
#endif

#ifdef DEBUG_ECCERROR
#define ndd_dump_uncorrect_err(nddata, ppt, pl) __ndd_dump_uncorrect_err(nddata, ppt, pl)
#else
#define ndd_dump_uncorrect_err(nddata, ppt, pl)
#endif

#ifdef CHECK_USED_BLOCK
#define ndd_dump_check_used_block(nddata, ppt, blockid) __ndd_dump_check_used_block(nddata, ppt, blockid)
#else
#define ndd_dump_check_used_block(nddata, ppt, blockid)
#endif

#ifdef DEBUG_ERR_TSKMSG
#define ndd_dump_taskmsg(msg, num) __ndd_dump_taskmsg(msg, num)
#else
#define ndd_dump_taskmsg(msg, num)
#endif

#endif /* _NAND_DEBUG_H_ */
