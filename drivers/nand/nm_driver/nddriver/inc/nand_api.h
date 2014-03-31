#ifndef _NAND_API_H_
#define _NAND_API_H_

#include <mach/jznand.h>
#include <nand_chip.h>
#include <ingenic_nand_mgr/nand_param.h>

#define REDUN_PT_NUM		2 // nderr and ndvirtual
#define HW_SECTOR		512

#define CPU_COPY_MODE		1
#define DMA_COPY_MODE		2
#define DEFAULT_COPY_MODE 	CPU_COPY_MODE
#define DEFAULT_OPS_MODE 	DMA_OPS

#define DEFAULT_ECCSIZE		1024
#define SPL_ECCSIZE		256

#define REF_ZONE_SIZE 		(4 * 1024 * 1024) //4M
#define ZONE_COUNT_LIMIT	32 //min ndpartition zone count limit
#define ERR_PT_TOTALBLOCKS	4 //virtual blocks

#define BCH_USE_NEMC_RATE

#define CS_PER_NFI		4
#define  MAX_NAME_SIZE 32

#define __raw_readl(reg)     \
	*((volatile unsigned int *)(reg))
#define __raw_writel(value,reg)  \
	*((volatile unsigned int *)(reg)) = (value)

/* ####################################### */
/**
 * nemc_base: nfi features for a specified product
 **/
typedef struct __nfi_base {
	void *gate;
	unsigned long rate;
	unsigned int irq;
	void *iomem;
	void *cs_iomem[CS_PER_NFI];
	int (*readl)(int reg);
	void (*writel)(int reg, int val);
	int (*clk_enable)(void);
	void (*clk_disable)(void);
} nfi_base;

/**
 * bch_base: bch features for a specified product
 **/
typedef struct __bch_base {
	void *gate;
	void *clk;
	unsigned int irq;
	void *iomem;
	int (*readl)(int reg);
	void (*writel)(int reg, int val);
	int (*clk_enable)(void);
	void (*clk_disable)(void);
} bch_base;

/**
 * mcu_base: mcu features
 **/
typedef struct __pdma_base {
	void *iomem;
	unsigned int dma_channel;
} pdma_base;

/**
 * io_base: the collction of devices
 * features of a specified product
 *
 * @nfi: nfi_base
 * @bch: bch_base
 * @pdma: pdma_base
 **/
typedef struct __io_base {
	nfi_base nfi;
	bch_base bch;
	pdma_base pdma;
} io_base;

/* ############### rbinfo ################# */
/**
 * struct __rb_item, rb gpio or irq info used for driver
 **/
typedef struct __rb_item {
	unsigned short id;
	unsigned short gpio;
	unsigned short irq;
	void *irq_private;
} rb_item;

typedef struct __rb_info {
	unsigned short totalrbs;
	rb_item *rbinfo_table;
} rb_info;

/* ####################################### */
typedef struct __os_clib {
	void (*ndelay) (unsigned long nsecs);
	int (*div_s64_32)(long long dividend, int divisor);
	void* (*continue_alloc)(unsigned int size);
	void (*continue_free)(const void *addr);
	int (*printf)(const char *fmt, ...);
	void* (*memcpy)(void *dst, const void *src, unsigned int count);
	void* (*memset)(void *s, int c, unsigned int count);
	int (*strcmp)(const char *cs, const char *ct);
	unsigned int (*get_vaddr)(unsigned int paddr);
	void (*dma_cache_wback)(unsigned long addr, unsigned long size);
	void (*dma_cache_inv)(unsigned long addr, unsigned long size);
	unsigned long long (*get_time_nsecs)(void);
} os_clib;

/**
 * struct nand_api_osdependent:
 *
 * @rbinfo:
 * @base:
 * @gpio_wp:
 * @wp_enable:
 * @wp_disable:
 * @wait_rb_timeout:
 **/
struct nand_api_osdependent {
	io_base *base;
	rb_info *rbinfo;
	plat_ptinfo *plat_ptinfo;
	unsigned short gpio_wp;
	unsigned char drv_strength;
	unsigned char rb_pulldown;
	os_clib clib;
	void (*wp_enable) (int);
	void (*wp_disable) (int);
	int (*wait_rb_timeout) (rb_item *, int);
	int (*try_wait_rb) (rb_item *, int);
	nand_flash * (*get_nand_flash) (void);
};

int nand_api_init(struct nand_api_osdependent *private);
int nand_api_suspend(void);
int nand_api_resume(void);

#endif /*_NAND_API_H_*/
