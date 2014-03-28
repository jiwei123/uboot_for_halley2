#ifndef __PPAINFO_H_
#define __PPAINFO_H_

#define MAX_NAME_SIZE 32
#define MAX_RB_COUNT 4
#define MAX_PART_NUM 16
#define MUL_PARTS    4

/* driver strength, level 0 is weakest */
#define DRV_STRENGTH_DEFAULT	0
#define DRV_STRENGTH_LEVEL0	1
#define DRV_STRENGTH_LEVEL1	2
#define DRV_STRENGTH_LEVEL2	3
#define DRV_STRENGTH_LEVEL3	4

/* rb pulldown strength, level 0 is weakest */
#define RB_PULLDOWN_STRENGTH_DEFAULT	0
#define RB_PULLDOWN_STRENGTH_LEVEL0	1
#define RB_PULLDOWN_STRENGTH_LEVEL1	2
#define RB_PULLDOWN_STRENGTH_LEVEL2	3
#define RB_PULLDOWN_STRENGTH_LEVEL3	4

typedef struct _PartitionInfo PartitionInfo;
typedef struct _Nandppt Nandppt;
typedef struct __ui_plat_ex_partition ui_plat_ex_partition;

struct __ui_plat_ex_partition {
	char name[MAX_NAME_SIZE];
	int  offset;
	int  size;
};

struct _Nandppt{
	char name[MAX_NAME_SIZE];
	int offset;
	int size;
	int managermode;
	int cache;
	ui_plat_ex_partition ui_ex_partition[MUL_PARTS];
};

struct _PartitionInfo{
	Nandppt ndppt[MAX_PART_NUM];
	int ptcount;
	int rbcount;
	int rb_gpio[MAX_RB_COUNT];
	int gpio_wp;
	unsigned char  rb_pulldown_strength[MAX_RB_COUNT];
	unsigned char  nand_driver_strength;
};

#endif
