#ifndef __JZ_NAND_H
#define __JZ_NAND_H

/* nand manager mode */
#define SPL_MANAGER  	0
#define DIRECT_MANAGER 	1
#define ZONE_MANAGER   	2
#define ONCE_MANAGER  	3

/* pt extend flags */
#define PT_USE_CACHE	0x01

/* CPU mode or DMA mode */
#define CPU_OPS		0x1
#define DMA_OPS		0x2

/* one plan or two plan ops */
#define ONE_PLANE	1
#define TWO_PLANES	2

#define MUL_PARTS 4

#define MAX_NAME_SIZE 32
/**
 * struct platform_nand_ex_partition
 * an element in platform_nand_partition
 * the member is as same as its
 */
typedef struct __plat_ex_partition {
	char name[MAX_NAME_SIZE];
	unsigned long long offset;
	unsigned long long size;
} plat_ex_partition;
/**
 * struct __plat_ndpartition:
 *
 * @name: the name of this partition
 * @offset: offset within the master MTD space
 * @size: partition size
 * @eccbit: the number of eccbit
 * @planes: operation mode, ONE_PLANE || TWO_PLANES
 * @ops_mode: DMA_OPS || CPU_OPS
 * @nm_mode: partition manager mode, SPL_MANAGER || DIRECT_MANAGER || ZONE_MANAGER
 * @flags:
 **/
typedef struct __plat_ptitem {
	char name[MAX_NAME_SIZE];
	unsigned long long offset;
	unsigned long long size;
	unsigned char ops_mode;
	unsigned char nm_mode;
	unsigned int flags;
	unsigned int *pt_badblock_info;
	plat_ex_partition ex_partition[MUL_PARTS];
} plat_ptitem;

typedef struct __plat_ptinfo {
	unsigned short ptcount;
	plat_ptitem *pt_table;
} plat_ptinfo;

typedef struct __plat_rbitem {
	unsigned short id;
	unsigned short gpio;
} plat_rbitem;

typedef struct __plat_rbinfo {
	unsigned short rbcount;
	plat_rbitem *rb_table;
} plat_rbinfo;

/**
 * struct plat_nddata: - container structure for platform-specific data
 *
 * @ptcount: platform_nand_partition array size
 * @ptarray: platform_nand_partition array
 * @rbcount:
 * @rbarray:
 * @gpio_wp: write protect gpio, we only support one write protect
 * gpio for all nand flash in one product
 **/
struct plat_nddata {
	plat_ptinfo ptinfo;
	plat_rbinfo rbinfo;
	unsigned int gpio_wp;
};

#endif /*__JZ_NAND_H*/
