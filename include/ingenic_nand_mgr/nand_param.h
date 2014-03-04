#ifndef __NAND_PARAM_H__
#define __NAND_PARAM_H__



/**
 *  * struct __nand_timing - NAND Flash Device timing
 *   **/
typedef struct __nand_timing {
	unsigned int tALS;  /* ... duration/width/time */
	unsigned int tALH;  /* ... duration/width/time */
	unsigned int tRP;   /* ... duration/width/time */
	unsigned int tWP;   /* ... duration/width/time */
	unsigned int tRHW;  /* ... duration/width/time */
	unsigned int tWHR;  /* ... duration/width/time */
	unsigned int tWHR2; /* ... duration/width/time */
	unsigned int tRR;   /* ... duration/width/time */
	unsigned int tWB;   /* ... duration/width/time */
	unsigned int tADL;  /* ... duration/width/time */
	unsigned int tCWAW; /* ... duration/width/time */
	unsigned int tCS;   /* ... duration/width/time */
	unsigned int tCLH;  /* ... duration/width/time */
} nand_timing;

 
typedef struct __optionalcmd {
	unsigned char multiplaneread[2];        // the sequence is [0] -ADDR- [0] -ADDR- [1] - DATA
	unsigned char multiplanewrite[2];       // the sequence is 80 -ADDR- DATA - [0] - [1] -ADDR- DATA - 10/15
	unsigned char multiplanecopyread[3];    // the sequence is [0] -ADDR- [1] -ADDR- [2]
	unsigned char multiplanecopywrite[3];   // the sequence is [0] -ADDR- [1] - [2] -ADDR- 10
	unsigned char multiplanestatus;     // the command may be 0x70/0x71/0x78/...
	unsigned char interbnk0status;      // the command may be 0xf1/0x78/...
	unsigned char interbnk1status;      // the command may be 0xf2/0x78/...
} optionalcmd;

/**
 *  * struct __nand_flash - NAND Flash Device attr Structure
 *   **/
typedef struct __nand_flash {
	char name[32];
	unsigned int id;
	unsigned int extid;
	unsigned int pagesize;
	unsigned int blocksize;
	unsigned int oobsize;
	unsigned int maxvalidblocks;
	unsigned char eccbit;
	unsigned char planepdie;
	unsigned char diepchip;
	unsigned char chips;
	unsigned char buswidth;
	unsigned char realplanenum;
	unsigned short badblockpos;
	unsigned char rowcycles;
	unsigned char planeoffset; //multi-plane block address offset
	unsigned int options;
	nand_timing timing;
	optionalcmd *optcmd;
} nand_flash;




#endif //__NAND_PARAM_H__
