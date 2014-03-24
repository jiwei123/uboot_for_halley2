#ifndef __MCP_CONFIG_H
#define __MCP_CONFIG_H

/*
 * This file contains the memory configuration parameters for the altair board.
 */
/*--------------------------------------------------------------------------------
 * MDDR-266 info
 */
/* MDDR paramters */
#define DDR_ROW    14 /* ROW : 12 to 14 row address */
#define DDR_COL    10 /* COL :  8 to 10 column address */
#define DDR_BANK8   0 /* Banks each chip: 0-4bank, 1-8bank */
#define DDR_CL      3 /* CAS latency: 1 to 7 */
/*
 * MDDR controller timing1 register
 */
#define DDR_tRAS    40 /*tRAS: ACTIVE to PRECHARGE command period to the same bank. */
#define DDR_tRTP    MAX(2, 0) /* 7.5ns READ to PRECHARGE command period. */
#define DDR_tRP     15 /* tRP: PRECHARGE command period to the same bank */
#define DDR_tRCD    15 /* ACTIVE to READ or WRITE command period to the same bank. */
#define DDR_tRC     (DDR_tRAS + DDR_tRP) /* ACTIVE to ACTIVE command period to the same bank.*/
#define DDR_tRRD    10 /* ACTIVE bank A to ACTIVE bank B command period. */
#define DDR_tWR     15 /* WRITE Recovery Time defined by register MR of DDR2 memory */
#define DDR_tWTR    MAX(2, 0) /* WRITE to READ command delay. */
/*
 * MDDR controller timing2 register
 */
#define DDR_tRFC    120 /* ns,  AUTO-REFRESH command period. */
#define DDR_tMINSR   6 /* Minimum Self-Refresh / Deep-Power-Down */
#define DDR_tXP      1 /* EXIT-POWER-DOWN to next valid command period: 1 to 8 tCK. */
#define DDR_tMRD     2 /* unit: tCK Load-Mode-Register to next valid command period: 1 to 4 tCK */

/* new add */
#define DDR_BL	     4 /* MDDR Burst length: 3 - 8 burst, 2 - 4 burst , 1 - 2 burst*/
#define DDR_tAL      0 /* Additive Latency, tCK*/
#define DDR_tRL      (DDR_tAL + DDR_CL)	/* MDDR: Read Latency = tAL + tCL */
#define DDR_tWL      1 /* MDDR: must 1 */
#define DDR_tRDLAT   (DDR_tRL - 2)	
#define DDR_tWDLAT   (DDR_tWL - 1)
#define DDR_tCCD     4 /* CAS# to CAS# command delay , tCK, MDDR no*/
#define DDR_tRTW     (DDR_tRL + DDR_tCCD + 2 - DDR_tWL)	/* Read to Write delay */
#define DDR_tFAW     50 /* Four bank activate period, ns, MDDR no */
#define DDR_tCKE     2 /* CKE minimum pulse width, tCK */
#define DDR_tXS      140	/* Exit self-refresh to next valid command delay, ns */
#define DDR_tXSRD    4 /* DDR2 only: Exit self refresh to a read command, tck */
#define DDR_tCKSRE   4 /* Valid Clock Requirement after Self Refresh Entry or Power-Down Entry */

/*
 * MDDR controller refcnt register
 */
#define DDR_tREFI    7800 /* Refresh period: 4096 refresh cycles/64ms */
#define DDR_CLK_DIV     1 /* Clock Divider. auto refresh
			   * cnt_clk = memclk/(16*(2^DDR_CLK_DIV))
			   */
#endif  /* __MCP_CONFIG_H */
