#include <stdio.h>
#include <string.h>
#include <config.h>
#include <ddr/ddr_chips.h>

unsigned int tck_ps = 0, tck_ns = 0;

static inline int DDR_GET_VALUE(int x, int y)
{
	int value;

	value = x * 1000 % y == 0 ? x * 1000 / y : x * 1000 / y + 1;

	return value;
}

static inline int MAX(int nck, int time)
{
	unsigned int value;
	value = nck * tck_ps > time ? nck * tck_ps : time;
	value = value % 1000 == 0 ? value / 1000 : value / 1000 + 1;
	return value;
}


void caculate_tck(void)
{
	tck_ps = (1000000000 / (CONFIG_SYS_MEM_FREQ / 1000));
	tck_ns = (1000000000 % CONFIG_SYS_MEM_FREQ == 0)
		? (1000000000 / CONFIG_SYS_MEM_FREQ)
		: (1000000000 / CONFIG_SYS_MEM_FREQ + 1);
}

void get_ddrc_register(void)
{
	unsigned int tmp = 0;
	/*The timing parameters are identical to the JEDEC DDR Specification */
	/* DTIMING1 has field such as tRTP,tWTR,tWR,tWL */
	/* tRTP: READ to PRECHARGE command period */
	tmp = DDR_GET_VALUE(DDR_tRTP, tck_ps);

	if (tmp < 1)
		tmp = 1;
	if (tmp > 6)
		tmp = 6;
	printf("#define DDRC_TIMING1_tRTP		0x%x\n", tmp);

	/* tWTR: WRITE to READ command delay */
	tmp = DDR_GET_VALUE(DDR_tWTR, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 6)
		tmp = 6;
	printf("#define DDRC_TIMING1_tWTR		0x%x\n",
			DDR_tWL + DDR_BL / 2 + tmp);

	/* tWR: WRITE Recovery Time defined by register MR of DDR2 DDR3 memory */
	tmp = DDR_GET_VALUE(DDR_tWR, tck_ps);
#ifdef CONFIG_SDRAM_DDR3
	if (tmp < 5)
		tmp = 5;
	if (tmp > 12)
		tmp = 12;
#else
	if (tmp < 2)
		tmp = 2;
	if (tmp > 6)
		tmp = 6;
#endif
	printf("#define	DDRC_TIMING1_tWR		0x%x\n", tmp);

	/* tWL: Write latency */
	tmp = DDR_tWL;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING1_tWL		0x%x\n", tmp);

	/* DTIMING2 has field such as tCCD,tRAS,tRCD,tRL */
	/* tCCD: CAS to CAS command delay */
	tmp = DDR_tCCD;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING2_tCCD		0x%x\n", tmp);

	/* tRAS: the ACTIVE to PRECHARGE command period to the same bank */
	tmp = DDR_GET_VALUE(DDR_tRAS, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 31)
		tmp = 31;
	printf("#define DDRC_TIMING2_tRAS		0x%x\n", tmp);

	/* tRCD: ACTIVE to READ or WRITE command period. */
	tmp = DDR_GET_VALUE(DDR_tRCD, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 11)
		tmp = 11;
	printf("#define DDRC_TIMING2_tRCD		0x%x\n", tmp);

	/* tRL: Read latency */
	tmp = DDR_tRL;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING2_tRL		0x%x\n", tmp);

	/* DTIMING3 has field such as ONUM,tCKSRE,tRP,tRRD,tRC */
	/* ONUM: Keep to 4 in this version */
	printf("#define DDRC_TIMING3_ONUM		0x%x\n", 4);

	/* tCKSRE: Valid clock after enter self-refresh */
	tmp = DDR_GET_VALUE(DDR_tCKSRE, tck_ps) / 8;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 7)
		tmp = 7;
	printf("#define DDRC_TIMING3_tCKSRE		0x%x\n", tmp);

	/* tRP: PRECHARGE command period */
	tmp = DDR_GET_VALUE(DDR_tRP, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 11)
		tmp = 11;
	printf("#define DDRC_TIMING3_tRP		0x%x\n", tmp);

	/* tRRD: ACTIVE bank A to ACTIVE bank B command period */
#if defined(CONFIG_FPGA)
	tmp = 1;
#else
	tmp = DDR_GET_VALUE(DDR_tRRD, tck_ps);
#endif
	if (tmp < 1)
		tmp = 1;
	if (tmp > 8)
		tmp = 8;
	printf("#define DDRC_TIMING3_tRRD		0x%x\n", tmp);

	/* tRC: ACTIVE to ACTIVE command period */
	tmp = DDR_GET_VALUE(DDR_tRC, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 42)
		tmp = 42;
	printf("#define DDRC_TIMING3_tRC		0x%x\n", tmp);

	/* DTIMING4 has field such as tRFC,tEXTRW,tRWCOV,tCKE,tMINSR,tXP,tMRD */
	/* tRFC: AUTO-REFRESH command period. */
	tmp = DDR_GET_VALUE(DDR_tRFC, tck_ps) - 1;
	tmp = tmp / 2;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING4_tRFC		0x%x\n", tmp);

	/* tRWCOV: keep the default value */
	tmp = 3;
	printf("#define DDRC_TIMING4_tRWCOV		0x%x\n", tmp);

	/* tCKE: minimum CKE pulse width */
	tmp = DDR_GET_VALUE(DDR_tCKE, tck_ps);
	if (tmp < 0)
		tmp = 0;
	if (tmp > 7)
		tmp = 7;
	printf("#define DDRC_TIMING4_tCKE		0x%x\n", tmp);

	/* tMINSR: Minimum Self-Refresh / Deep-Power-Down time */
	tmp = DDR_tMINSR;
	if (tmp < 9)
		tmp = 9;
	if (tmp > 129)
		tmp = 129;
	tmp = ((tmp - 1) % 8) ? ((tmp - 1) / 8) : ((tmp - 1) / 8 - 1);
	printf("#define DDRC_TIMING4_tMINSR		0x%x\n", tmp);

	/* tXP: EXIT-POWER-DOWN to next valid command period */
	tmp = DDR_tXP;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 7)
		tmp = 7;
	printf("#define DDRC_TIMING4_tXP		0x%x\n", DDR_tXP);

	/* tMRD: Load-Mode-Register to next valid command period */
	printf("#define DDRC_TIMING4_tMRD		0x%x\n", DDR_tMRD - 1);

	/* DTIMING5 has field such as tCTLUPD,tRTW,tRDLAT,tWDLAT*/
	/* tCTLUPD: Inner usage. Not need to change */
	printf("#define DDRC_TIMING5_tCTLUPD		0x%x\n", 0xff);

	/* tRTW: read to write*/
	tmp = DDR_tRTW;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING5_tRTW		0x%x\n", tmp);

	/* tRDLAT: tRL-2. (when use LPDDR2, set tRDLAT=tRL) */
	tmp = DDR_tRDLAT;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING5_tRDLAT		0x%x\n", tmp);

	/* tWDLAT: tWL-1(when use LPDDR2, set tWDLAT=tWL) */
	tmp = DDR_tWDLAT;
	if (tmp > 63)
		tmp = 63;
	printf("#define DDRC_TIMING5_tWDLAT		0x%x\n", tmp);

	/* DTIMING6 has field such as tXSRD,tFAW,tCFGW,tCFGR */
	/* tXSRD: exit self-refresh to READ delay */
	tmp = DDR_tXSRD / 4;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 63)
		tmp = 31;
	printf("#define DDRC_TIMING6_tXSRD		0x%x\n", tmp);

	/* tFAW: 4-active command window */
	tmp = DDR_GET_VALUE(DDR_tFAW, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 31)
		tmp = 31;
	printf("#define DDRC_TIMING6_tFAW		0x%x\n", tmp);

	/* tCFGW: Write PHY configure registers to other commands delay */
	printf("#define DDRC_TIMING6_tCFGW		0x%x\n", 3);

	/* tCFGR: Ready PHY configure registers to other commands delay */
	printf("#define DDRC_TIMING6_tCFGR		0x%x\n", 3);

	/* DREFCNT: DDR Auto-Refresh Counter */
	/* DREFCNT has field such as CON,CNT,CLK_DIV,REF_EN */
	/* CON: A constant value used to compare with the CNT value */
	tmp = DDR_tREFI / tck_ns;
	tmp = tmp / (16 * (1 << DDR_CLK_DIV)) - 1;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 0xff)
		tmp = 0xff;
	printf("#define DDRC_REFCNT_CON			0x%x\n", tmp);

	/* CLK_DIV : Clock Divider */
	printf("#define DDRC_REFCNT_CLK_DIV		0x%x\n",
			DDR_CLK_DIV);

	/* DCFG: Configure the external memory, static configuration only */
	/* DCFG has field such as ROW1,ROW0,BA1,IMBA,BSL,TYPE,ODTEN,MISPE,COL1,
	 * COL0,CS1EN,CS0EN,CL,BA0,DW */
	/* ROW0/1: Row Address width */
	printf("#define DDRC_CFG_ROW1			0x%x\n", DDR_ROW - 12);
	printf("#define DDRC_CFG_ROW0			0x%x\n", DDR_ROW - 12);

	/* BA0/1: Bank Address width of DDR memory */
#ifdef DDR_BANK8
	printf("#define DDRC_CFG_BA1			0x%x\n", DDR_BANK8);
	printf("#define DDRC_CFG_BA0			0x%x\n", DDR_BANK8);
#else
	printf("#define DDRC_CFG_BA1			0x%x\n", DDR_BANK4);
	printf("#define DDRC_CFG_BA0			0x%x\n", DDR_BANK4);
#endif

	/* BL: Burst length for DDR chips */
	printf("#define DDRC_CFG_BL			0x%x\n",
			(DDR_BL > 4) ? 1 : 0);

	/* COL0/1: Column Address width */
	printf("#define DDRC_CFG_COL1			0x%x\n", DDR_COL - 8);
	printf("#define DDRC_CFG_COL0			0x%x\n", DDR_COL - 8);

	/* CS1EN: DDR Chip-Select-1 Enable */
	printf("#define DDRC_CFG_CS1EN			0x%x\n", DDR_CS1EN);

	/* CS1EN: DDR Chip-Select-0 Enable */
	printf("#define DDRC_CFG_CS0EN			0x%x\n", DDR_CS0EN);

	/* CL: CAS Latency */
	tmp = DDR_CL - 1;
	if (tmp < 0)
		tmp = 0;
	if (tmp > 4)
		tmp = 4;
	printf("#define DDRC_CFG_CL			0x%x\n", \
			tmp | 0x8);

	/* DW: External DDR Memory Data Width */
#ifdef DDR_DW32
	printf("#define DDRC_CFG_DW			0x%x\n", DDR_DW32);
#else
	printf("#define DDRC_CFG_DW			0x%x\n", DDR_DW16);
#endif
}

void get_ddrp_register(void)
{
	register unsigned int tmp = 0;
	unsigned int dinit1 = 0;

	/* DTAR: Data Training Address Register */
	/* DTAR has field such as DTMPR,DTBANK,DTROW,DTCOL */
	/* DTROW: Data Training Row Address */
	printf("#define DDRP_DTAR_DTROW			0x%x\n", 0x150);
	/* DTCOL: Data Training Column Address */
	printf("#define DDRP_DTAR_DTCOL			0x%x\n", 0x0);

	/* MR0-2: Mode Register 0-2 */
	/* MR0 has field such as RSVD,PD,WR,DR,TM,CL,BT,BL */
	/* WR: the value of the write recovery in clock cycles */
	tmp = DDR_GET_VALUE(DDR_tWR, tck_ps);
	if (tmp < 5)
		tmp = 5;
	if (tmp > 12)
		tmp = 12;
	if (tmp < 8)
		tmp -= 4;
	else
		tmp = (tmp + 1) / 2;
	printf("#define DDRP_MR0_WR			0x%x\n", tmp);

	/* CL: CAS Latency */
	printf("#define DDRP_MR0_CL			0x%x\n", DDR_CL - 4);

	/* BL: Burst Length */
	printf("#define DDRP_MR0_BL			0x%x\n",
			(8 - DDR_BL) / 2);

	/* MR2 has field such as RSVD,RTTWR,SRT,ASR,CWL,PASR */
	printf("#define DDRP_MR2_tCWL			0x%x\n", DDR_tCWL - 5);

	/* PTR0-1: PHY Timing Register 0-1 */
	/* PTR0 has field such as tITMSRST,tDLLLOCK,tDLLSRST */
	/* PTR1 has filed such as tDINIT1, tDINIT0 */
	/* tDINIT1: DRAM Initialization Time 1 */
	if (((DDR_tRFC + 10) * 1000) > (5 * tck_ps))  /* ddr3 only */
		dinit1 = (DDR_tRFC + 10) * 1000;
	else
		dinit1 = 5 * tck_ps;
	tmp = DDR_GET_VALUE(dinit1 / 1000, tck_ps);
	if (tmp > 0xff)
		tmp = 0xff;
	printf("#define DDRP_PTR1_tDINIT1		0x%x\n", tmp);

	/* DTPR0-2: DRAM Timing Parameters Register 0-2 */
	/* DTPR0 has field such as tCCD,tRC,tRRD,tRAS,tRCD,tRP,tWTR,tRTP,tMRD */
	/* tCCD: Read to read and write to write command delay */
	printf("#define DDRP_DTPR0_tCCD			0x%x\n",
			(DDR_tCCD > 4) ? 1 : 0);

	/* tRC: ACTIVE to ACTIVE command period. */
	tmp = DDR_GET_VALUE(DDR_tRC, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 42)
		tmp = 42;
	printf("#define DDRP_DTPR0_tRC			0x%x\n", tmp);

	/* tRRD: ACTIVE bank A to ACTIVE bank B command period. */
#if defined(CONFIG_FPGA)
	tmp = 1;
#else
	tmp = DDR_GET_VALUE(DDR_tRRD, tck_ps);
#endif
	if (tmp < 1)
		tmp = 1;
	if (tmp > 8)
		tmp = 8;
	printf("#define DDRP_DTPR0_tRRD			0x%x\n", tmp);

	/* tRAS: ACTIVE to PRECHARGE command period */
	tmp = DDR_GET_VALUE(DDR_tRAS, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 31)
		tmp = 31;
	printf("#define DDRP_DTPR0_tRAS			0x%x\n", tmp);

	/* tRCD: ACTIVE to READ or WRITE command period. */
	tmp = DDR_GET_VALUE(DDR_tRCD, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 11)
		tmp = 11;
	printf("#define DDRP_DTPR0_tRCD			0x%x\n", tmp);

	/* tRP: PRECHARGE command period. */
	tmp = DDR_GET_VALUE(DDR_tRP, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 11)
		tmp = 11;
	printf("#define DDRP_DTPR0_tRP			0x%x\n", tmp);

	/* tWTR: Internal write to read command delay */
	tmp = DDR_GET_VALUE(DDR_tWTR, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 6)
		tmp = 6;
	printf("#define DDRP_DTPR0_tWTR			0x%x\n", tmp);

	/* tRTP: Internal read to precharge command delay */
#if defined(CONFIG_FPGA)
	tmp = 1;
#else
	tmp = DDR_GET_VALUE(DDR_tRTP, tck_ps);
#endif /* if defined(CONFIG_FPGA) */
	if (tmp < 2)
		tmp = 2;
	if (tmp > 6)
		tmp = 6;
	printf("#define DDRP_DTPR0_tRTP			0x%x\n", tmp);

	/* tMRD: Load mode cycle time */
	printf("#define DDRP_DTPR0_tMRD			0x%x\n", DDR_tMRD - 4);

	/* DTPR1 has field such as tDQSCKmax,tDQSCK,tRFC,tRTODT,tMOD,tFAW,tRTW
	 * tAOND/tAOFD
	 */
	/* tRFC: Refresh-to-Refresh */
	tmp = DDR_GET_VALUE(DDR_tRFC, tck_ps);
	if (tmp < 1)
		tmp = 1;
	if (tmp > 255)
		tmp = 255;
	printf("#define DDRP_DTPR1_tRFC			0x%x\n", tmp);

	/* tRTODT: Read to ODT delay (DDR3 only) */
	printf("#define DDRP_DTPR1_tRTODT		0x%x\n", 1);

	/* tMOD: Load mode update delay (DDR3 only) */
	tmp = DDR_GET_VALUE(DDR_tMOD, tck_ps);
	tmp -= 12;
	if (tmp < 0)
		tmp = 0;
	if (tmp > 3)
		tmp = 3;
	printf("#define DDRP_DTPR1_tMOD			0x%x\n", tmp);

	/* tFAW: 4-bank activate period */
	tmp = DDR_GET_VALUE(DDR_tFAW, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 31)
		tmp = 31;
	printf("#define DDRP_DTPR1_tFAW			0x%x\n", tmp);

	/* DTPR2 has field such as tDLLK,tCKE,tXP,tXS */
	/* tDLLK: DLL locking time */
	tmp = DDR_tDLLLOCK;
	if (tmp < 2)
		tmp = 2;
	if (tmp > 1023)
		tmp = 1023;
	printf("#define DDRP_DTPR2_tDLLK		0x%x\n", tmp);

	/* tCKE: CKE minimum pulse width */
	tmp = DDR_tCKE;
	if (tmp < 2)
		tmp = 2;
	if (tmp > 15)
		tmp = 15;
	printf("#define DDRP_DTPR2_tCKE			0x%x\n", tmp);

	/* Power down exit delay */
	tmp = (DDR_tXP > DDR_tXPDLL) ? DDR_tXP : DDR_tXPDLL;
	tmp = DDR_GET_VALUE(tmp, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 31)
		tmp = 31;
	printf("#define DDRP_DTPR2_tXP			0x%x\n", tmp);

	/* Self refresh exit delay */
	tmp = (DDR_tXS > DDR_tXSDLL) ? DDR_tXS : DDR_tXSDLL;
	tmp = DDR_GET_VALUE(tmp, tck_ps);
	if (tmp < 2)
		tmp = 2;
	if (tmp > 1023)
		tmp = 1023;
	printf("#define DDRP_DTPR2_tXS			0x%x\n", tmp);

	/* PGCR: PHY General Configuration Register */
	/*CKEN: Controls whether the CK going to the SDRAM
	 * is enabled(toggling) or disabled (static value defined by CKDV) */
	printf("#define DDRP_PGCR_CKEN			0x%x\n",
			DDR_CS0EN | DDR_CS1EN);
}

int main(int argc, char *argv[])
{
	printf("#ifndef DDR_CONFIG_H__\n");
	printf("#define DDR_CONFIG_H__\n\n");

	caculate_tck();
	printf("#define PS				%d\n", tck_ps);
	printf("#define NS				%d\n", tck_ns);
	get_ddrc_register();
	get_ddrp_register();

	printf("\n#endif\n");

	return 0;
}
