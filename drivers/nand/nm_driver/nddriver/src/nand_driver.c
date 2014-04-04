/**
 * nand_driver.c
 **/
#include <ingenic_nand_mgr/nand_param.h>
#include "nand_api.h"
#include "xdelay.h"
#include "ndmath.h"
#include "memorydealer.h"
#include <os/clib.h>
#include "cache.h"
#include "jz4775.h"
#include "hand.h"

extern void serial_printf(char *fmt, ...);

#define NEMC_IOBASE 0xb3410000
#define BCH_IOBASE  0xb34d0000
#define PDMA_IOBASE 0xb3420000
#define NEMC_CS1_IOBASE 0xbb000000
#define NEMC_CS2_IOBASE 0xba000000

#define DRIVER_MEM_SIZE 4*1024*1024
//char driver_mem[DRIVER_MEM_SIZE];
struct nand_api_osdependent ndd_private;
nand_flash nand_flash_info;

#define NULL (void*)0

static void dump_nand_flash_info()
{
	serial_printf("\ndump nand flash info:\n");
	serial_printf("\t name =         %s\n",nand_flash_info.name);
	serial_printf("\t id =           %x\n",nand_flash_info.id);
	serial_printf("\t extid =        %x\n",nand_flash_info.extid);
	serial_printf("\t pagesize =     %d\n",nand_flash_info.pagesize);
	serial_printf("\t blocksize =    %d\n",nand_flash_info.blocksize);
	serial_printf("\t oobsize =      %d\n",nand_flash_info.oobsize);
	serial_printf("\t maxvalidblocks = %d\n",nand_flash_info.maxvalidblocks);
	serial_printf("\t minvalidblocks = %d\n",nand_flash_info.minvalidblocks);
	serial_printf("\t pagenumber = %d\n",nand_flash_info.pagenumber);
	serial_printf("\t badblockpage = %d\n",nand_flash_info.badblockpage);
	serial_printf("\t bpc = %d\n",nand_flash_info.bpc);
	serial_printf("\t eccbit =       %d\n",nand_flash_info.eccbit);
	serial_printf("\t planepdie =    %d\n",nand_flash_info.planepdie);
	serial_printf("\t diepchip =     %d\n",nand_flash_info.diepchip);
	serial_printf("\t chips =        %d\n",nand_flash_info.chips);
	serial_printf("\t buswidth =     %d\n",nand_flash_info.buswidth);
	serial_printf("\t realplanenum = %d\n",nand_flash_info.realplanenum);
	serial_printf("\t badblockpos =  %d\n",nand_flash_info.badblockpos);
	serial_printf("\t timingmod =  %d\n",nand_flash_info.timingmod);
	serial_printf("\t rowcycles =    %d\n",nand_flash_info.rowcycles);
	serial_printf("\t pageaddresscycle =    %d\n",nand_flash_info.pageaddresscycle);
	serial_printf("\t planeoffset =  %d\n",nand_flash_info.planeoffset);
	serial_printf("\t options =      %x\n",nand_flash_info.options);
	serial_printf("\t nandtype =      %d\n",nand_flash_info.nandtype);

	serial_printf("\ndump nand flash timing:\n");
	serial_printf("\t tals =     %d \n",nand_flash_info.timing.tALS);
	serial_printf("\t talh =     %d \n",nand_flash_info.timing.tALH);
	serial_printf("\t trp  =     %d \n",nand_flash_info.timing.tRP);
	serial_printf("\t twp  =     %d \n",nand_flash_info.timing.tWP);
	serial_printf("\t trhw =     %d \n",nand_flash_info.timing.tRHW);
	serial_printf("\t twhr =     %d \n",nand_flash_info.timing.tWHR);
	serial_printf("\t twhr2 =    %d \n",nand_flash_info.timing.tWHR2);
	serial_printf("\t trr =      %d \n",nand_flash_info.timing.tRR);
	serial_printf("\t twb =      %d \n",nand_flash_info.timing.tWB);
	serial_printf("\t tadl =     %d \n",nand_flash_info.timing.tADL);
	serial_printf("\t tCWAW =    %d \n",nand_flash_info.timing.tCWAW);
	serial_printf("\t tcs =      %d \n",nand_flash_info.timing.tCS);
	serial_printf("\t tCLH =     %d \n",nand_flash_info.timing.tCLH);
}

void fill_nand_flash_info(nand_flash *nand_info)
{
	memcpy(&nand_flash_info,nand_info,sizeof(nand_flash));
	nand_flash_info.options = NAND_TIMING_MODE | NAND_TIMING_MODE_V(nand_info->timingmod) | NAND_DRIVER_STRENGTH;
	//dump_nand_flash_info();
}

static void ndd_ndelay(unsigned long nsecs)
{
	unsigned int loops = (unsigned int)nsecs;
	xdelay(loops);
}


static int ndd_div_s64_32(long long dividend, int divisor)
{
	return div_s64_32(dividend, divisor);
}

static int jz_strcmp(const char *cs, const char *ct){
	unsigned char c1, c2;
	while(1) {
		c1 = *cs++;
		c2 = *ct++;
		if (c1 != c2)
			return c1 < c2 ? -1 : 1;
		if (!c1)
			break;
	}
	return 0;
}

static void* ndd_continue_alloc(unsigned int size)
{
	return Nand_ContinueAlloc(size);
}

static void ndd_continue_free(void *addr)
{
	Nand_ContinueFree(addr);
}

static unsigned int ndd_get_vaddr(unsigned int paddr)
{
	return (unsigned int)(paddr + 0x80000000);
}

static void ndd_dma_cache_wback(unsigned long addr, unsigned long size)
{
	dma_cache_wback_inv(addr, size);
}

static void ndd_dma_cache_inv(unsigned long addr, unsigned long size)
{
	//dma_cache_inv(addr, size);
}

#define TCU_IOBASE 0XB0000000

/*TCU*/
#define TDFR0 0x2040
#define TDHR0 0x2044
#define TCNT0 0x2048
#define TCSR0 0x204c
#define TER   0x2010
#define TESR  0x2014
#define TECR  0x2018
#define TFR   0x2020
#define TFSR  0x2024
#define TFCR  0x2028
#define TMR   0x2030
#define TMSR  0x2034
#define TMCR  0x2038
#define TSR   0x201c
#define TSSR  0x202c
#define TSCR  0x203c
#define TSTR  0x20f0
#define TSTSR 0x20f4
#define TSTCR 0x20f8

#define TCSR_EXT_EN_BIT  2
#define TESR_OSTST_EN_BIT 15

/*OST*/
#define OSTCSR  0x20ec
#define OSTDR   0x20e0
#define OSTCNTL 0x20e4
#define OSTCNTH 0x20e8

#define READ_CALC_SPEED  0
#define WRITE_CALC_SPEED 1

static unsigned long long ndd_get_time_nsecs(void)
{
	static int ost_inited = 0;
	union clycle_type
	{
		unsigned long long cycle64;
		unsigned int cycle32[2];
	} cycle;

	if (!ost_inited) {
		/*div=1, clk form EXTAL*/
		__raw_writel(0x1 << TCSR_EXT_EN_BIT, TCU_IOBASE + OSTCSR);
		/*OSTDR = 0xffff*/
		__raw_writel(0xffffffff, TCU_IOBASE + OSTDR);
		/*OSTCNTL = 0xffff*/
		__raw_writel(0x0, TCU_IOBASE + OSTCNTL);
		/*OSTCNTH = 0*/
		__raw_writel(0x0, TCU_IOBASE + OSTCNTH);
		/*TESR.OSTCST = 1*/
		__raw_writel(0x1 << TESR_OSTST_EN_BIT, TCU_IOBASE + TESR);

		ost_inited = 1;
	}

	do {
		cycle.cycle32[1] = __raw_readl(TCU_IOBASE + OSTCNTH);
		cycle.cycle32[0] = __raw_readl(TCU_IOBASE + OSTCNTL);
	} while (cycle.cycle32[1] != __raw_readl(TCU_IOBASE + OSTCNTH));
	//serial_printf("cycle.cycle32[0] = %x cycle.cycle32[1] = %x\n",cycle.cycle32[0],cycle.cycle32[1]);

	return cycle.cycle64 * (1000000000 / (24 * 1024 * 1024));
}

/* ############################################################################################ *\
 * callback functions used for nand_api
\* ############################################################################################ */
static void ndd_wp_enable(int gpio)
{
	__gpio_clear_pin(gpio);
}

static void ndd_wp_disable(int gpio)
{
	__gpio_set_pin(gpio);
}

#define GPIO_PXFLGC_ADDR 0xb0010058
#if 1
static int ndd_wait_rb_timeout(rb_item *rbitem, int timeout)
{
	int group = rbitem->gpio / 32;
	int rb_gpio = rbitem->gpio % 32;
	int timeout_ns = timeout * 1000 * 1000;

	while ((!(REG_GPIO_PXFLG(group) & (1 << rb_gpio))) && (timeout_ns--));
	REG_GPIO_PXFLGC(group) = 0x1 << rb_gpio;
	__raw_writel(0xffffffff,GPIO_PXFLGC_ADDR);
	if (timeout_ns > 0)
		return 0;
	else
		return -1;
}
#else
static int ndd_wait_rb_timeout(rb_item *rbitem, int timeout)
{
	int group = rbitem->gpio / 32;
	int rb_gpio = rbitem->gpio % 32;

	while ((REG_GPIO_PXPIN(group) & (0x00000001 << rb_gpio)) && timeout--);
	while (!(REG_GPIO_PXPIN(group) & (0x00000001 << rb_gpio)));
	if(timeout < 0){
		serial_printf("%s[%d] rbgpio=%d cs=%x \n",__func__,timeout,rbitem->gpio,*(volatile unsigned int*)0xb3410050);
	}

	return timeout;
}
#endif
static int ndd_try_wait_rb(rb_item *rbitem, int delay)
{
	int group = rbitem->gpio / 32;
	int rb_gpio = rbitem->gpio % 32;
	ndd_ndelay(delay * 1000 * 1000);
	if(REG_GPIO_PXFLG(group) & (0x00000001 << rb_gpio)){
		REG_GPIO_PXFLGC(group) = 0x1 << rb_gpio;
		__raw_writel(0xffffffff,GPIO_PXFLGC_ADDR);
		return 1;
	}else
		return 0;
}

static nand_flash* ndd_get_nand_flash(void)
{
	return &nand_flash_info;
}
static int nfi_readl(int reg)
{
	void *iomem = ndd_private.base->nfi.iomem;

	return __raw_readl(iomem + reg);
}

static void nfi_writel(int reg, int val)
{
	void *iomem = ndd_private.base->nfi.iomem;
	__raw_writel((val), iomem + reg);
}

static int bch_readl(int reg)
{
	void *iomem = ndd_private.base->bch.iomem;

	return __raw_readl(iomem + reg);
}

static void bch_writel(int reg, int val)
{
	void *iomem = ndd_private.base->bch.iomem;

	__raw_writel((val), iomem + reg);
}

static int nfi_clk_enable(void)
{
	return 0;
}

static void nfi_clk_disable(void)
{
}

static int bch_clk_enable(void)
{
	return 0;
}

static void bch_clk_disable(void)
{
}

/* ############################################################################################ *\
 * sub functions for driver probe
\* ############################################################################################ */
static inline int get_devices_resources(io_base *base)
{
	int i;
	nfi_base *nfi = &(base->nfi);
	bch_base *bch = &(base->bch);
	pdma_base *pdma = &(base->pdma);

	/* get resource for nfi */
	nfi->iomem = (void*)NEMC_IOBASE;
	nfi->irq = -1;

	/* get resource for bch */
	bch->iomem = (void*)BCH_IOBASE;
	bch->irq = -1;

	/* get resource for pdma */
	pdma->iomem = (void*)PDMA_IOBASE;
	pdma->dma_channel = -1;

	for (i = 0; i < CS_PER_NFI; i++)
		nfi->cs_iomem[i] = NEMC_CS1_IOBASE - i * 0x1000000;

	return 0;
}

static inline int get_devices_clk(io_base *base)
{
	nfi_base *nfi = &(base->nfi);
	nfi->rate = cpm_get_h2clk();
	return 0;
}

static inline int get_device_callbak(io_base *base)
{
	nfi_base *nfi = &(base->nfi);
	bch_base *bch = &(base->bch);

	/* nfi callbacks */
	nfi->readl = nfi_readl;
	nfi->writel = nfi_writel;
	nfi->clk_enable = nfi_clk_enable;
	nfi->clk_disable = nfi_clk_disable;

	/* bch callbacks */
	bch->readl = bch_readl;
	bch->writel = bch_writel;
	bch->clk_enable = bch_clk_enable;
	bch->clk_disable = bch_clk_disable;

	return 0;
}

static int fill_io_base(io_base *base)
{
	int ret;

	ret = get_devices_resources(base);
	if (ret) {
		serial_printf("ERROR: get devices resources error!\n");
		return ret;
	}
	ret = get_devices_clk(base);
	if (ret) {
		serial_printf("ERROR: get devices clk error");
		return ret;
	}
	get_device_callbak(base);
	return 0;
}

static rb_info* get_rbinfo_memory(void)
{
	serial_printf("%s shouldn't be called !!!!\n",__func__);
	return NULL;
}

void fill_rbinfo_table(PartitionInfo *pinfo, rb_info *rbinfo)
{
	int rb_index;
	int gpio, gpio_group, gpio_offset;

	rbinfo->totalrbs = pinfo->rbcount;
	for(rb_index = 0; rb_index < rbinfo->totalrbs; rb_index++){
		gpio = pinfo->rb_gpio[rb_index];
		gpio_group = gpio / 32;
		gpio_offset = gpio % 32;
		REG_GPIO_PXINTS(0) = 1 << gpio_offset;	// used as interrupt
		REG_GPIO_PXMASKC(0) = 1 << gpio_offset;	// set pin as interrupt source
		REG_GPIO_PXPAT1S(0) = 1 << gpio_offset;	// edge interrupt
		REG_GPIO_PXPAT0S(0) = 1 << gpio_offset; // no pull up or pull down
		serial_printf("gpio_group = %d, gpio_offset = %d\n", gpio_group, gpio_offset);
		serial_printf("INIT GPIO_PAINT = %x\n", *((volatile unsigned int *)0xb0010010));
		serial_printf("INIT GPIO_PAFLG = %x\n", *((volatile unsigned int *)0xb0010050));
		(rbinfo->rbinfo_table + rb_index)->id = rb_index;
		(rbinfo->rbinfo_table + rb_index)->gpio = gpio;
	}
}

void fill_plat_ptinfo(PartitionInfo *pinfo, plat_ptinfo * plat_info)
{
	Nandppt *uipt = pinfo->ndppt;
	int pt_index = 0, part_num;

	plat_info->ptcount = pinfo->ptcount;
	for(pt_index = 0; pt_index < plat_info->ptcount; pt_index++){
		part_num = 0;
		memcpy((plat_info->pt_table + pt_index)->name, (uipt + pt_index)->name, MAX_NAME_SIZE);
		(plat_info->pt_table + pt_index)->offset = (unsigned long long)(uipt + pt_index)->offset *
			(unsigned long long)(1024 * 1024);
		if((uipt + pt_index)->size != -1)
			(plat_info->pt_table + pt_index)->size = (unsigned long long)(uipt + pt_index)->size *
				(unsigned long long)(1024 * 1024);
		else
			(plat_info->pt_table + pt_index)->size = -1;
		if((uipt + pt_index)->managermode == ZONE_MANAGER){
			(plat_info->pt_table + pt_index)->ops_mode = DMA_OPS;
		}else{
			(plat_info->pt_table + pt_index)->ops_mode = CPU_OPS;
		}
		(plat_info->pt_table + pt_index)->nm_mode = (uipt + pt_index)->managermode;
		(plat_info->pt_table + pt_index)->flags = (uipt + pt_index)->cache;
		while((uipt + pt_index)->ui_ex_partition[part_num].size != 0){
			(plat_info->pt_table + pt_index)->ex_partition[part_num].offset =
				(unsigned long long)((uipt + pt_index)->ui_ex_partition[part_num].offset) * (unsigned long long)(1024 * 1024);
			if((uipt + pt_index)->ui_ex_partition[part_num].size != -1)
				(plat_info->pt_table + pt_index)->ex_partition[part_num].size =
					(unsigned long long)((uipt + pt_index)->ui_ex_partition[part_num].size) * (unsigned long long)(1024 * 1024);
			else
				(plat_info->pt_table + pt_index)->ex_partition[part_num].size = -1;

			memcpy((plat_info->pt_table + pt_index)->ex_partition[part_num].name,
			       (uipt + pt_index)->ui_ex_partition[part_num].name, MAX_NAME_SIZE);
			part_num++;
		}
	}
}
/* ############################################################################################ *\
 * nand driver main functions
\* ############################################################################################ */
int nand_probe(PartitionInfo *pinfo)
{
	void *heap; 
	void *h = NULL;
	int i, ret;
	rb_info *rbinfo = NULL;
	plat_ptinfo *pptinfo;
	int rbcnt = pinfo->rbcount, ptcnt = pinfo->ptcount;

	heap = (void*)malloc(DRIVER_MEM_SIZE);
	if(!heap){
		serial_printf("ERROR:alloc heap error!\n");
		goto err_alloc_base;
	}
	h = Nand_MemoryInit(heap, DRIVER_MEM_SIZE, NO_INIT);
	if (h == NULL) {
		serial_printf("ERROR:Nand memory manager init error, %s(%d)\n",
				__FUNCTION__, __LINE__);
		return 0;
	}

	/* alloc memory for io_base and fill nand base */
	ndd_private.base = Nand_VirtualAlloc(sizeof(io_base));
	if (!ndd_private.base)
		goto err_alloc_base;

	ret = fill_io_base(ndd_private.base);
	if (ret)
		goto err_fill_base;

	/**********************************************************************************/
	/* ------------------------------ WARNING --------------------------------------- */
	/*if rbinfo and plat_ptinfo is NULL, then these info will be read from errpt later*/
	/**********************************************************************************/

	/*fill rb_info*/
	rbinfo = Nand_ContinueAlloc(sizeof(rb_info) + rbcnt * sizeof(rb_item) + rbcnt * sizeof(int));
	if(!rbinfo)
		goto err_alloc_rbinfo;
	else{
		unsigned char *rb_comp_base;
		rbinfo->rbinfo_table = (rb_item *)((unsigned char *)rbinfo + sizeof(rb_info));
		rb_comp_base = (unsigned char *)rbinfo->rbinfo_table + rbcnt * sizeof(rb_item);
		for(i = 0; i < rbcnt; i++)
			(rbinfo->rbinfo_table + i)->irq_private = rb_comp_base + i * sizeof(int);
	}
	fill_rbinfo_table(pinfo, rbinfo);
	ndd_private.rbinfo = rbinfo;
	/*fill plat_ptinfo*/
	pptinfo = (plat_ptinfo*)Nand_ContinueAlloc(sizeof(plat_ptinfo) + ptcnt * sizeof(plat_ptitem));
	if(!pptinfo)
		goto err_alloc_ptinfo;
	else{
		pptinfo->ptcount = ptcnt;
		pptinfo->pt_table = (plat_ptitem*)((unsigned char *)pptinfo + sizeof(plat_ptinfo));
	}
	fill_plat_ptinfo(pinfo, pptinfo);
	ndd_private.plat_ptinfo = pptinfo;
	/* get write protect pin */
	ndd_private.gpio_wp = pinfo->gpio_wp;

	/* get drv_strength and rb pulldown strength */
	ndd_private.drv_strength = pinfo->nand_driver_strength;//DRV_STRENGTH_DEFAULT;
	ndd_private.rb_pulldown = pinfo->rb_pulldown_strength[0];//RB_PULLDOWN_STRENGTH_DEFAULT;

	/* fill fun points */
	ndd_private.wp_enable = ndd_wp_enable;
	ndd_private.wp_disable = ndd_wp_disable;
	ndd_private.wait_rb_timeout = ndd_wait_rb_timeout;
	ndd_private.try_wait_rb = ndd_try_wait_rb;
	ndd_private.get_nand_flash = ndd_get_nand_flash;
	/* clib */
	ndd_private.clib.ndelay = ndd_ndelay;
	ndd_private.clib.div_s64_32 = ndd_div_s64_32;
	ndd_private.clib.continue_alloc = ndd_continue_alloc;
	ndd_private.clib.continue_free = ndd_continue_free;
	ndd_private.clib.printf = serial_printf;
	ndd_private.clib.memcpy = memcpy;
	ndd_private.clib.memset = memset;
	ndd_private.clib.strcmp = jz_strcmp;
	ndd_private.clib.get_vaddr = ndd_get_vaddr;
	ndd_private.clib.dma_cache_wback = ndd_dma_cache_wback;
	ndd_private.clib.dma_cache_inv = ndd_dma_cache_wback;//ndd_dma_cache_inv;
	ndd_private.clib.get_time_nsecs = ndd_get_time_nsecs;
	/* nand api init */
	
	ret = nand_api_init(&ndd_private);
	if (ret)
		goto err_api_init;

	return 0;

err_api_init:
	Nand_ContinueFree(ndd_private.plat_ptinfo);
err_alloc_ptinfo:
	Nand_ContinueFree(ndd_private.rbinfo);
err_alloc_rbinfo:
err_fill_base:
	Nand_ContinueFree(ndd_private.base);
err_alloc_base:
	return -1;
}
