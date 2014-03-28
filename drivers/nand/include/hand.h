#ifndef __HAND_H__
#define __HAND_H__

#define NANDFLASH_NAME_SIZE  40

typedef struct {
	/* CPU ID */
	unsigned int  cpu_id;
	/* PLL args */
	unsigned char ext_clk;
	unsigned int cpu_speed;
	unsigned int ddr_speed;

	unsigned char phm_div;
	unsigned char use_uart;
	unsigned int  boudrate;

	/* SDRAM args */
	unsigned char bus_width;
	unsigned char bank_num;
	unsigned char row_addr;
	unsigned char col_addr;
	unsigned char is_mobile;
	unsigned char is_busshare;

	unsigned char cs1en;
	unsigned char cs0en;
	unsigned char cl;

	unsigned char tRAS;
	unsigned char tRTP;
	unsigned char tRP;
	unsigned char tRCD;

	unsigned char tRC;
	unsigned char tRRD;
	unsigned char tWR;
	unsigned char tWTR;

	unsigned short tRFC;
	unsigned short tMINSR;
	unsigned char tXP;
	unsigned  char tMRD;

	unsigned short tREFI;
	unsigned char clk_div;

	/* debug args */
	unsigned char debug_ops;
	unsigned char pin_num;
	unsigned int  start;
	unsigned int  size;
} fw_args_t;

typedef struct {
	/* nand flash info */
        // int nand_start;
	int pt;                 //cpu type: jz4740/jz4750 .....
	int nand_bw;
	int nand_rc;
	int nand_ps;
	int nand_fs_without_oob; //free size per page
	int nand_dpp_without_oob; //data per page
	int nand_fs_with_oob;  //free size per page
	int nand_dpp_with_oob;  //data per page
	int nand_ppb;
	int nand_force_erase;
	int nand_pn;
	int nand_os;
	int nand_eccpos;        //ECC position
	int nand_bbpage;        //bad block position
	int nand_bbpos;         //bad block position
	int nand_plane;
	int nand_bchbit;
	int nand_wppin;
	int nand_bpc;
	int nand_bchstyle;  //device os : linux or minios
	int nand_type;          //0:common nand,1:toggle nand

	char nandflash_name[NANDFLASH_NAME_SIZE];
	unsigned char nand_planepdie;
	unsigned char nand_diepchip;
	unsigned char nand_chips;
	unsigned char nand_planeoffset; //multi-plane block address offset
        unsigned int nand_id;
        unsigned int nand_extid;
	int nand_minvalidblocks;
	int nand_maxvalidblocks;
        unsigned int nand_options; //timing mode, driver strength, rb pulldonw strength

	/* nandflash timing for config NEMC/NFI*/
	int nand_tals;
	int nand_talh;
	int nand_trp;
	int nand_twp;
	int nand_trhw;
	int nand_twhr;
	int nand_twhr2;
	int nand_trr;
	int nand_twb;
	int nand_tadl;
	int nand_tcwaw;
	int nand_tcs;
	int nand_tclh;
	int nand_timingmode;

	unsigned int manager;

	fw_args_t fw_args;
} hand_t;

extern int GET_CUP_INFO_Handle(void);
extern int SET_DATA_ADDERSS_Handle(u8 *buf);
extern int SET_DATA_LENGTH_Handle(u8 *buf);
extern int PROGRAM_START1_Handle(u8 *buf);
extern int PROGRAM_START2_Handle(u8 *buf);
extern int NOR_OPS_Handle(u8 *buf);
extern int NAND_OPS_Handle(u8 *buf);
extern int CONFIGRATION_Handle(u8 *buf);
extern int SDRAM_OPS_Handle(u8 *buf);

#endif /* __HAND_H__ */
