/**
 * nand_driver.c
 **/
#define __KERNEL__
#include <common.h>
#include <malloc.h>
#undef __KERNEL__ 

#include <ingenic_nand_mgr/nand_param.h>
#include <asm/arch/clk.h>
#include <asm/arch/gpio.h>
#include "ndmath.h"
#include "NandAlloc.h"
#include "nand_api.h"


extern int printf(const char *fmt, ...);

#define NEMC_IOBASE 0xb3410000
#define BCH_IOBASE  0xb34d0000
#define PDMA_IOBASE 0xb3420000
#define NEMC_CS1_IOBASE 0xbb000000
#define NEMC_CS2_IOBASE 0xba000000

#define DRIVER_MEM_SIZE 4*1024*1024
//char driver_mem[DRIVER_MEM_SIZE];
struct nand_api_osdependent ndd_private;
nand_sharing_params share_parms;
nand_flash nand_flash_info;


#define __raw_readl(reg)     \
	*((volatile unsigned int *)(reg))
#define __raw_writel(value,reg)  \
	*((volatile unsigned int *)(reg)) = (value)

static void dump_nand_flash_info(void)
{
	printf("\ndump nand flash info:\n");
	printf("\t name =         %s\n",nand_flash_info.name);
	printf("\t id =           %x\n",nand_flash_info.id);
	printf("\t extid =        %x\n",nand_flash_info.extid);
	printf("\t pagesize =     %d\n",nand_flash_info.pagesize);
	printf("\t blocksize =    %d\n",nand_flash_info.blocksize);
	printf("\t oobsize =      %d\n",nand_flash_info.oobsize);
	printf("\t totalblocks = %d\n",nand_flash_info.totalblocks);
	printf("\t maxvalidblocks = %d\n",nand_flash_info.maxvalidblocks);
	printf("\t eccbit =       %d\n",nand_flash_info.eccbit);
	printf("\t planepdie =    %d\n",nand_flash_info.planepdie);
	printf("\t diepchip =     %d\n",nand_flash_info.diepchip);
	printf("\t chips =        %d\n",nand_flash_info.chips);
	printf("\t buswidth =     %d\n",nand_flash_info.buswidth);
	printf("\t realplanenum = %d\n",nand_flash_info.realplanenum);
	printf("\t badblockpos =  %d\n",nand_flash_info.badblockpos);
	printf("\t rowcycles =    %d\n",nand_flash_info.rowcycles);
	printf("\t planeoffset =  %d\n",nand_flash_info.planeoffset);
	printf("\t options =      %x\n",nand_flash_info.options);

	printf("\ndump nand flash timing:\n");
	printf("\t tals =     %d \n",nand_flash_info.timing.tALS);
	printf("\t talh =     %d \n",nand_flash_info.timing.tALH);
	printf("\t trp  =     %d \n",nand_flash_info.timing.tRP);
	printf("\t twp  =     %d \n",nand_flash_info.timing.tWP);
	printf("\t trhw =     %d \n",nand_flash_info.timing.tRHW);
	printf("\t twhr =     %d \n",nand_flash_info.timing.tWHR);
	printf("\t twhr2 =    %d \n",nand_flash_info.timing.tWHR2);
	printf("\t trr =      %d \n",nand_flash_info.timing.tRR);
	printf("\t twb =      %d \n",nand_flash_info.timing.tWB);
	printf("\t tadl =     %d \n",nand_flash_info.timing.tADL);
	printf("\t tCWAW =    %d \n",nand_flash_info.timing.tCWAW);
	printf("\t tcs =      %d \n",nand_flash_info.timing.tCS);
	printf("\t tCLH =     %d \n",nand_flash_info.timing.tCLH);
}

void fill_nand_flash_info(nand_flash_param *nand_info)
{
	memcpy(nand_flash_info.name,nand_info->name,sizeof(nand_info->name));
	nand_flash_info.id = nand_info->id;
	nand_flash_info.extid = nand_info->extid;
	nand_flash_info.pagesize = nand_info->pagesize;
	nand_flash_info.blocksize = nand_info->blocksize;
	nand_flash_info.oobsize = nand_info->oobsize;
	nand_flash_info.totalblocks = nand_info->totalblocks;
	nand_flash_info.maxvalidblocks = 0;
	nand_flash_info.eccbit = nand_info->eccbit;
	nand_flash_info.planepdie = nand_info->planepdie;
	nand_flash_info.diepchip = nand_info->diepchip;
	nand_flash_info.chips = nand_info->chips;
	nand_flash_info.buswidth = nand_info->buswidth;
	nand_flash_info.realplanenum = nand_info->realplanenum;
	nand_flash_info.badblockpos = nand_info->badblockpos;
	nand_flash_info.rowcycles = nand_info->rowcycles;
	nand_flash_info.planeoffset = nand_info->planeoffset;
	if(nand_info->options != 0)
		nand_flash_info.options = NAND_TIMING_MODE | NAND_TIMING_MODE_V(nand_info->timingmod) | NAND_DRIVER_STRENGTH
			| NAND_READ_RETRY | NAND_READ_RETRY_MODE(nand_info->options);
	else
		nand_flash_info.options = NAND_TIMING_MODE | NAND_TIMING_MODE_V(nand_info->timingmod) | NAND_DRIVER_STRENGTH;

	memcpy(&(nand_flash_info.timing),&(nand_info->timing),sizeof(nand_info->timing));
	//dump_nand_flash_info();
}

static void ndd_ndelay(unsigned long nsecs)
{
	unsigned int loops = (unsigned int)nsecs / 1000;
	udelay(loops);
}


static int ndd_div_s64_32(long long dividend, int divisor)
{
	return div_s64_32(dividend, divisor);
}

#if 0
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
#endif
static void* ndd_continue_alloc(unsigned int size)
{
	return (void *)malloc(size);
}

static void ndd_continue_free(void *addr)
{
	free(addr);
}

static unsigned int ndd_get_vaddr(unsigned int paddr)
{
	return (unsigned int)(paddr + 0x80000000);
}

static void nand_dma_cache_wback(unsigned long addr, unsigned long size)
{
	flush_dcache_range(addr,addr+size);
	//	dma_cache_wback_inv(addr, size);
}
#if 0
static void ndd_dma_cache_inv(unsigned long addr, unsigned long size)
{

	flush_dcache_range(addr,addr+size);
	//dma_cache_inv(addr, size);
}
#endif

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
	//printf("cycle.cycle32[0] = %x cycle.cycle32[1] = %x\n",cycle.cycle32[0],cycle.cycle32[1]);

	return cycle.cycle64 * (1000000000 / (24 * 1024 * 1024));
}

/* ############################################################################################ *\
 * callback functions used for nand_api
\* ############################################################################################ */
static void ndd_wp_enable(int gpio)
{
	//gpio_set_value(gpio,0);
	gpio_direction_output(gpio,0);
}

static void ndd_wp_disable(int gpio)
{
	//gpio_set_value(gpio,1);
	gpio_direction_output(gpio,1);
}

#define GPIO_PXFLGC_ADDR 0xb0010058
static void ndd_clear_rb_state(rb_item *rbitem)
{
	gpio_clear_flag(rbitem->gpio);
	if(gpio_get_flag(rbitem->gpio))
	{
		printf("ERROR: clear rb error gpio = %d  reg = 0x%08x !!!!\n",rbitem->gpio,*(volatile unsigned int *)(0xb0010000));
		while(1);
	}
}
#if 0
static int ndd_wait_rb_timeout(rb_item *rbitem, int timeout)
{
	volatile int timeout_ns = timeout * 1000 * 1000;
	if(gpio_get_flag(rbitem->gpio))
	{
		printf(" --- !!! rb_gpio = %d  rb_flag = %d \n",rbitem->gpio,gpio_get_flag(rbitem->gpio));
		printf("ERROR: wait rb error !!!!\n");
		while(1);
	}
	while ((!gpio_get_flag(rbitem->gpio)) && (timeout_ns--));
	gpio_clear_flag(rbitem->gpio);
	if (timeout_ns > 0)
		return 0;
	else
		return -1;
}
#else
static int ndd_wait_rb_timeout(rb_item *rbitem, int timeout)
{
	volatile  int timeout_ns = timeout * 1000 * 1000;

	while (gpio_get_value(rbitem->gpio) && timeout_ns--);
	while ((!gpio_get_value(rbitem->gpio)) && timeout_ns--);
	if (timeout_ns > 0)
		return 0;
	else
		return -1;
}
#endif
static int ndd_try_wait_rb(rb_item *rbitem, int delay)
{
	ndd_ndelay(delay * 1000 * 1000);
	if(gpio_get_flag(rbitem->gpio)){
		gpio_clear_flag(rbitem->gpio);
		//__raw_writel(0xffffffff,GPIO_PXFLGC_ADDR);
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
static int gpio_irq_request(unsigned short gpio, unsigned short *irq, int rbcomp)
{
	return 0;
}

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
		nfi->cs_iomem[i] = (void *)(NEMC_CS1_IOBASE - i * 0x1000000);

	return 0;
}

extern unsigned int cpm_get_h2clk(void);

static inline int get_devices_clk(io_base *base)
{
	nfi_base *nfi = &(base->nfi);
	nfi->rate = clk_get_rate(H2CLK);
	clk_set_rate(BCH,nfi->rate);
	printf("---------------->>>>>>>>>>>>>>>> cpcrr =  0x%08x \n",*((volatile int *)0xb0000000));
	printf("---------------->>>>>>>>>>>>>>>> bchdiv = 0x%08x \n",*((volatile int *)0xb00000ac));
	printf("---------------->>>>>>>>>>>>>>>> nfi->rate = %ld \n",nfi->rate);
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
		printf("ERROR: get devices resources error!\n");
		return ret;
	}
	ret = get_devices_clk(base);
	if (ret) {
		printf("ERROR: get devices clk error");
		return ret;
	}
	get_device_callbak(base);
	return 0;
}

static int ndd_gpio_request(unsigned int gpio, const char *lable)
{
	return 0;
}

static rb_info* get_rbinfo_memory(void)
{
	printf("%s shouldn't be called !!!!\n",__func__);
	return NULL;
}

void fill_rbinfo_table(PartitionInfo *pinfo, rb_info *rbinfo)
{
	int rb_index;
	int gpio;
	int ret_gpio;

	rbinfo->totalrbs = pinfo->rbcount;
	for(rb_index = 0; rb_index < rbinfo->totalrbs; rb_index++){
		gpio = pinfo->rb_gpio[rb_index];

		ret_gpio = gpio_request(gpio,"wait_rb");
		gpio_as_irq_fall_edge(ret_gpio);
		gpio_disable_pull(ret_gpio);
#if 0
		REG_GPIO_PXINTS(0) = 1 << gpio_offset;	// used as interrupt
		REG_GPIO_PXMASKC(0) = 1 << gpio_offset;	// set pin as interrupt source
		REG_GPIO_PXPAT1S(0) = 1 << gpio_offset;	// edge interrupt
		REG_GPIO_PXPAT0S(0) = 1 << gpio_offset; // no pull up or pull down
#endif
		printf("INIT GPIO_PAINT = %x\n", *((volatile unsigned int *)0xb0010010));
		printf("INIT GPIO_PAFLG = %x\n", *((volatile unsigned int *)0xb0010050));
		(rbinfo->rbinfo_table + rb_index)->id = rb_index;
		(rbinfo->rbinfo_table + rb_index)->gpio = ret_gpio;
		(rbinfo->rbinfo_table + rb_index)->pulldown_strength = pinfo->rb_pulldown_strength[rb_index];
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

#define EXTID_MARK 0x00ffffff
static nand_flash_param *get_nand_info_from_ids(nand_flash_id *fid,nand_flash_param *nand_info_table,int total_nand)
{
	int index;

	for (index = 0; index < total_nand; index++) {
		if ((fid->id == nand_info_table[index].id) && ((fid->extid & EXTID_MARK) == (nand_info_table[index].extid & EXTID_MARK)))
			break;
	}
	return &nand_info_table[index];
}


#define NAND_DATAPORT   0xBB000000
#define NAND_ADDRPORT   0xBB800000
#define NAND_COMMPORT   0xBB400000
#define REG_NEMC_NFCSR  0xB3410050
#define __nand_cmd(n)       (*((volatile unsigned char *)(NAND_COMMPORT)) = (n))
#define __nand_addr(n)      (*((volatile unsigned char *)(NAND_ADDRPORT)) = (n))
#define __nand_write_data8(n)   (*((volatile unsigned char *)(NAND_DATAPORT)) = (n))
#define __nand_read_data8() (*(volatile unsigned char *)(NAND_DATAPORT))
#define __nand_enable()     (*((volatile unsigned int *)(REG_NEMC_NFCSR)) = 0x3)
#define __nand_disable()    (*((volatile unsigned int *)(REG_NEMC_NFCSR)) = 0x0)

static void nand_enable(int cs)
{

	udelay(1);
	*(volatile unsigned int *)(0xb3410050) = 0x3;
	udelay(1);

}
static void nand_reset()
{
	__nand_cmd(0xff);
}
static void nand_send_addr(int addr, unsigned int cycle, unsigned int delay)
{
	while(cycle--){
		__nand_addr((addr & 0xff));
		addr = addr >> 0x08;
	}
	udelay(delay);
}
static void nand_disable(int cs)
{
	udelay(1); // 300ns
	*(volatile unsigned int *)(0xb3410050) = 0x0;
}
static void wait_rb(int rb)
{
	int cnt = 100000;
	int gpio = rb / 32;
	int gpio_bit = rb % 32;
	int mask_rb = (1 << gpio_bit);//papin

	while((((*(volatile unsigned int *)(GPIO_BASE + gpio * 0x100)) & mask_rb) == mask_rb) && cnt--);
	if(cnt < 0){
		printf("^^^^^^^  wait rb timeout !!! \n");
	}
	while(((*(volatile unsigned int *)(GPIO_BASE + gpio * 0x100 )) & mask_rb) != mask_rb);
}

static int try_to_get_nand_id(int rb_gpio,nand_flash_id *fid)
{
	unsigned char nand_id[6];
	unsigned int i;
	unsigned int cnt = 4000000;

	__nand_enable();
//	printf("n----------->.. nfcsr = 0x%08x rb_gpio = %d \n",*(volatile unsigned int *)(0xb3410050),rb_gpio);

	nand_reset();
	wait_rb(rb_gpio);
	mdelay(1);
	while(cnt--);

	__nand_cmd(0x90);
	udelay(1);

	nand_send_addr(0x00, 1, 1 * 1000);

	for(i=0; i < 5; i++){
		nand_id[i] = __nand_read_data8();
	}
	__nand_disable();

	fid->id = ((nand_id[0] << 8) | nand_id[1]);
	fid->extid = ((nand_id[4] << 16) | (nand_id[3] << 8) | nand_id[2]);

//	printf("-------------->>>> id = 0x%x extid = 0x%x\n",fid->id,fid->extid);
}

extern int __ndd_dump_nand_id(nfi_base *base, unsigned int cs_id,nand_flash_id *fid);
extern void fill_nand_basic_info(nand_flash_param *nand_info);
extern int burn_nandmanager_init(PartitionInfo *pinfo,int eraseall);

extern void my_print_epc();

static int get_nandflash_info(nfi_base *nfi,nand_flash_param *nand_info_ids,int nand_nm)
{
	nand_flash_id fid;
	nand_flash_param *nand_info = NULL;

	try_to_get_nand_id(20,&fid);

	nand_info = get_nand_info_from_ids(&fid,nand_info_ids,nand_nm);

	fill_nand_flash_info(nand_info);
	fill_nand_basic_info(nand_info);
	
	return 0;
}

/* ############################################################################################ *\
 * nand driver main functions
\* ############################################################################################ */
int nand_probe(PartitionInfo *pinfo, nand_flash_param *nand_info_ids,int nand_nm,int eraseall)
{
	void *heap; 
	void *h = NULL;
	int i, ret;
	rb_info *rbinfo = NULL;
	int rbcnt = pinfo->rbcount, ptcnt = pinfo->ptcount;

	heap = (void*)malloc(DRIVER_MEM_SIZE);
	if(!heap){
		printf("ERROR:alloc heap error!\n");
		goto err_alloc_base;
	}
	h = Nand_MemoryInit(heap, DRIVER_MEM_SIZE, 0);
	if (h == NULL) {
		printf("ERROR:Nand memory manager init error, %s(%d)\n",
				__FUNCTION__, __LINE__);
		return 0;
	}

	/* alloc memory for io_base and fill nand base */
	ndd_private.base = malloc(sizeof(io_base));
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
	rbinfo = malloc(sizeof(rb_info) + rbcnt * sizeof(rb_item) + rbcnt * sizeof(int));
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
	ndd_private.platptinfo.pt_table = (plat_ptitem*)malloc(ptcnt * sizeof(plat_ptitem));
	if(!(ndd_private.platptinfo.pt_table))
		goto err_alloc_ptinfo;
	else{
		memset(ndd_private.platptinfo.pt_table, 0, ptcnt * sizeof(plat_ptitem));
		ndd_private.platptinfo.ptcount = ptcnt;
	}
	fill_plat_ptinfo(pinfo, &(ndd_private.platptinfo));
	//dump_partitioninfo(pinfo);
	//dump_platptinfo(&(ndd_private.platptinfo));
	/* get write protect pin */
	ndd_private.gpio_wp = gpio_request(pinfo->gpio_wp,"nand_wp");

	/* get drv_strength and rb pulldown strength */
	ndd_private.drv_strength = pinfo->nand_driver_strength;//DRV_STRENGTH_DEFAULT;

	/* fill fun points */
	ndd_private.wp_enable = ndd_wp_enable;
	ndd_private.wp_disable = ndd_wp_disable;
	ndd_private.clear_rb_state = ndd_clear_rb_state;
	ndd_private.wait_rb_timeout = ndd_wait_rb_timeout;
	ndd_private.try_wait_rb = ndd_try_wait_rb;
	ndd_private.gpio_irq_request = gpio_irq_request;
	ndd_private.ndd_gpio_request = ndd_gpio_request;
	ndd_private.get_rbinfo_memory = get_rbinfo_memory;
	ndd_private.abandon_rbinfo_memory = NULL;
	ndd_private.get_nand_flash = ndd_get_nand_flash;
	/* clib */
	ndd_private.clib.ndelay = ndd_ndelay;
	ndd_private.clib.div_s64_32 = ndd_div_s64_32;
	ndd_private.clib.continue_alloc = ndd_continue_alloc;
	ndd_private.clib.continue_free = ndd_continue_free;
	ndd_private.clib.printf = printf;
	ndd_private.clib.memcpy = memcpy;
	ndd_private.clib.memset = memset;
	ndd_private.clib.strcmp = strcmp;
	ndd_private.clib.get_vaddr = ndd_get_vaddr;
	ndd_private.clib.dma_cache_wback = nand_dma_cache_wback;
	ndd_private.clib.dma_cache_inv = nand_dma_cache_wback;//ndd_dma_cache_inv;
	ndd_private.clib.get_time_nsecs = ndd_get_time_nsecs;
	ndd_private.erasemode = eraseall;
	
	gpio_init();

	//printf("------ printf gpio \n");
	//for(i= 0;i < 32 ;i++)
	//	dump_gpio_func(i);

	get_nandflash_info(&(ndd_private.base->nfi),nand_info_ids,nand_nm);

	{
		unsigned int errpc;
		__asm__ __volatile__(
				"mfc0 %0,$30 \n\t"
				"nop"
				: "=r" (errpc)
				:
				);
		printf("errpc = %x\n",errpc);

	}

	burn_nandmanager_init(pinfo,eraseall);

	/* nand api init */
	ret = nand_api_init(&ndd_private);
	if (ret)
		goto err_api_init;
	printf("%s end!!!!\n",__func__);
	return 0;

err_api_init:
	free(ndd_private.platptinfo.pt_table);
err_alloc_ptinfo:
	free(ndd_private.rbinfo);
err_alloc_rbinfo:
err_fill_base:
	free(ndd_private.base);
err_alloc_base:
	return -1;
}
