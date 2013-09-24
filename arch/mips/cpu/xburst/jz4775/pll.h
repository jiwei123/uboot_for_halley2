/*
 * The file define all the common macro for the board based on the JZ4760
 */


#ifndef __JZ4775_COMMON_H__
#define __JZ4775_COMMON_H__

#define PLL_OUT_MAX 1400		/* 1200MHz. */

//The first time boot

#define __CFG_EXTAL     (CONFIG_SYS_EXTAL / 1000000)
#define __CFG_APLL_OUT  ((CONFIG_SYS_MEM_SPEED) / 1000000)
#define __CFG_MPLL_OUT  ((CONFIG_SYS_MEM_SPEED) / 1000000)    /* Set MPLL default: 240MHz */

/*pll_0*/ 
#if (__CFG_APLL_OUT > PLL_OUT_MAX)
	#error "PLL output can NOT more than 1000MHZ"
#elif (__CFG_APLL_OUT > 600)
	#define __APLL_BS          1
	#define __APLL_OD          0
#elif (__CFG_APLL_OUT > 300)
	#define __APLL_BS          0
	#define __APLL_OD          0
#elif (__CFG_APLL_OUT > 155)
	#define __APLL_BS          0
	#define __APLL_OD          1
#elif (__CFG_APLL_OUT > 76)
	#define __APLL_BS          0
	#define __APLL_OD          2
#elif (__CFG_APLL_OUT > 47)
	#define __APLL_BS          0
	#define __APLL_OD          3
#else
	#error "APLL ouptput can NOT less than 48"
#endif

#define __APLL_NO		0
#define APLL_NR 			(__APLL_NO + 1)
#define APLL_NO 			(0x1 << __APLL_OD)
#define __APLL_MO		(((__CFG_APLL_OUT / __CFG_EXTAL) * APLL_NR * APLL_NO) - 1)
#define APLL_NF 			(__APLL_MO + 1)
#define APLL_FOUT			(__CFG_EXTAL * APLL_NF / APLL_NR / APLL_NO)

#if ((__CFG_EXTAL / APLL_NR > 50) || (__CFG_EXTAL / APLL_NR < 10))
	#error "Can NOT set the value to APLL_N"
#endif

#if ((__APLL_MO > 127) || (__APLL_MO < 1))
	#error "Can NOT set the value to APLL_M"
#endif

#if (__APLL_BS == 1)
	#if (((APLL_FOUT * APLL_NO) > PLL_OUT_MAX) || ((APLL_FOUT * APLL_NO) < 500))
		#error "FVCO check failed : APLL_BS = 1"
	#endif
#elif (__APLL_BS == 0)
	#if (((APLL_FOUT * APLL_NO) > 600) || ((APLL_FOUT * APLL_NO) < 300))
		#error "FVCO check failed : APLL_BS = 0"
	#endif
#endif

#define APLL_VALUE	((__APLL_MO << 24) | (__APLL_NO << 18) | (__APLL_OD << 16) | (__APLL_BS << 31))


/**************************************************************************************************************/

#if (__CFG_MPLL_OUT > PLL_OUT_MAX)
	#error "MPLL output can NO1T more than 1000MHZ"
#elif (__CFG_MPLL_OUT > 600)
	#define __MPLL_BS          1
	#define __MPLL_OD          0
#elif (__CFG_MPLL_OUT > 300)
	#define __MPLL_BS          0
	#define __MPLL_OD          0
#elif (__CFG_MPLL_OUT > 155)
	#define __MPLL_BS          0
	#define __MPLL_OD          1
#elif (__CFG_MPLL_OUT > 76)
	#define __MPLL_BS          0
	#define __MPLL_OD          2
#elif (__CFG_MPLL_OUT > 47)
	#define __MPLL_BS          0
	#define __MPLL_OD          3
#else
	#error "MPLL ouptput can NOT less than 48"
#endif

#define __MPLL_NO		0
#define MPLL_NR			(__MPLL_NO + 1)
#define MPLL_NO			(0x1 << __MPLL_OD)
#define __MPLL_MO		(((__CFG_MPLL_OUT / __CFG_EXTAL) * MPLL_NR * MPLL_NO) - 1)
#define MPLL_NF			(__MPLL_MO + 1)
#define MPLL_FOUT		(__CFG_EXTAL * MPLL_NF / MPLL_NR / MPLL_NO)

#if ((__CFG_EXTAL / MPLL_NR > 50) || (__CFG_EXTAL / MPLL_NR < 10))
	#error "Can NOT set the value to MPLL_N"
#endif

#if ((__MPLL_MO > 127) || (__MPLL_MO < 1))
	#error "Can NOT set the value to MPLL_M"
#endif

#if (__MPLL_BS == 1)
	#if (((MPLL_FOUT * MPLL_NO) > 1000) || ((MPLL_FOUT * MPLL_NO) < 500))
		#error "FVCO1 check failed : MPLL_BS1 = 1"
	#endif
#elif (__MPLL_BS == 0)
	#if (((MPLL_FOUT * MPLL_NO) > 600) || ((MPLL_FOUT * MPLL_NO) < 300))
		#error "FVCO1 check failed : MPLL_BS1 = 0"
	#endif
#endif

#define MPLL_VALUE	((__MPLL_MO << 24) | (__MPLL_NO << 18) | (__MPLL_OD << 16) | (__MPLL_BS << 31))


#endif /* __JZ4775_COMMON_H__ */
