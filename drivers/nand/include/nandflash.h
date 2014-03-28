#ifndef __NANDLIB_H__
#define __NANDLIB_H__

#ifndef NULL
#define NULL	0
#endif

#define u8	unsigned char
#define u16	unsigned short
#define u32	unsigned int

#define NAND_MAX_PAGESIZE	(16 * 1024)
#define NAND_MAX_OOBSIZE	1280
#define NAND_MAX_BAD		NAND_MAX_OOBSIZE
#define NAND_MAX_BAD_PAGES	256
//#define NAND_MAX_OOB_PAR	112
//#define NAND_MAX_DATA_PAR	112

#define TYPE_COMMON	0
#define TYPE_TOGGLE	1

struct ops_bch {
	int ecc_level;
	int ecc_block_size;
	int ecc_parity_size;
};

/*  Jz4740 nandflash interface */
unsigned int nand_query_4740(u8 *);
int nand_init_4740(int bus_width, int row_cycle, int page_size, int page_per_block,
		   int,int,int,int);
int nand_fini_4740(void);
u32 nand_program_4740(void *context, int spage, int pages, int option);
//int nand_program_oob_4740(void *context, int spage, int pages, void (*notify)(int));
u32 nand_erase_4740(int blk_num, int sblk, int force);
u32 nand_read_4740(void *buf, u32 startpage, u32 pagenum,int option);
u32 nand_read_oob_4740(void *buf, u32 startpage, u32 pagenum);
u32 nand_read_raw_4740(void *buf, u32 startpage, u32 pagenum,int);
u32 nand_mark_bad_4740(int bad);
void nand_enable_4740(u32 csn);
void nand_disable_4740(u32 csn);

/*  Jz4750 nandflash interface */
unsigned int nand_query_4750(u8 *);
//int nand_init_4750(int bus_width, int row_cycle, int page_size, int page_per_block,
//		   int,int,int,int);

int nand_init_4750(int bus_width, int row_cycle, int page_size, int page_per_block,
		   int bch_bit, int ecc_pos, int bad_pos, int bad_page, int force);

int nand_fini_4750(void);
u32 nand_program_4750(void *context, int spage, int pages, int option);
//int nand_program_oob_4740(void *context, int spage, int pages, void (*notify)(int));
u32 nand_erase_4750(int blk_num, int sblk, int force);
u32 nand_read_4750(void *buf, u32 startpage, u32 pagenum,int option);
u32 nand_read_oob_4750(void *buf, u32 startpage, u32 pagenum);
u32 nand_read_raw_4750(void *buf, u32 startpage, u32 pagenum,int);
u32 nand_mark_bad_4750(int bad);

void nand_enable_4750(u32 csn);
void nand_disable_4750(u32 csn);

/*  Jz4760 nandflash interface */
unsigned int nand_query_4760(u8 *);
int nand_init_4760b(int bus_width, int row_cycle, int page_size, int page_per_block,
                   int bch_bit, int ecc_pos, int bad_pos, int bad_page, int force,
                   int oob_size, int fs_without_oob, int dpp_without_oob,
                   int fs_with_oob, int dpp_with_oob);
int nand_fini_4760(void);
u32 nand_program_4760(void *context, int spage, int pages, int option);
u32 nand_erase_4760(int blk_num, int sblk, int force);
u32 nand_read_4760(void *buf, u32 startpage, u32 pagenum,int option);
u32 nand_read_oob_4760(void *buf, u32 startpage, u32 pagenum);
u32 nand_read_raw_4760(void *buf, u32 startpage, u32 pagenum,int);
u32 nand_mark_bad_4760(int bad);
void nand_enable_4760(u32 csn);
void nand_disable_4760(u32 csn);
unsigned int nand_mark_erase_4760(void);
int hw_reset_4760(void);

/*  Jz4780 nandflash interface */
unsigned int nand_query_4780(u8 *);
//int nand_init_4780(hand_t *Hand);
u32 nand_program_4780(void *context, int spage, int pages, int option);
u32 nand_erase_4780(int blk_num, int sblk, int force);
u32 nand_read_4780(void *buf, u32 startpage, u32 pagenum,int option);
u32 nand_read_oob_4780(void *buf, u32 startpage, u32 pagenum);
u32 nand_read_raw_4780(void *buf, u32 startpage, u32 pagenum,int);
u32 nand_mark_bad_4780(int bad);
void nand_enable_4780(u32 csn);
void nand_disable_4780(u32 csn);
unsigned int nand_mark_erase_4780(void);
int hw_reset_4780(void);

// Jz4775 Nand flash interface

unsigned int nand_query_4775(u8 *);
//int nand_init_4775(hand_t *Hand);
int nand_init_4775();
u32 nand_program_4775(void *context, int spage, int pages, int option);
u32 nand_erase_4775(int blk_num, int sblk, int force);
u32 nand_read_4775(void *buf, u32 startpage, u32 pagenum,int option);
u32 nand_read_oob_4775(void *buf, u32 startpage, u32 pagenum);
u32 nand_read_raw_4775(void *buf, u32 startpage, u32 pagenum,int);
u32 nand_mark_bad_4775(int bad);
void nand_enable_4775(u32 csn);
void nand_disable_4775(u32 csn);
unsigned int nand_mark_erase_4775(void);
int hw_reset_4775(void);


#endif
