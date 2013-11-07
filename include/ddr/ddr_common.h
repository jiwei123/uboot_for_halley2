#ifndef __DDR_COMMON_H__
#define __DDR_COMMON_H__

#include <asm/arch/ddr.h>
#include <ddr/ddr_chips.h>
#include <ddr/ddr_params.h>

#if defined(CONFIG_JZ4775)
#include <ddr/ddrc.h>
#include <ddr/ddrp_synopsis.h>
#elif defined(CONFIG_JZ4780)
#include <ddr/ddrc.h>
#include <ddr/ddrp_synopsis.h>
#elif defined(CONFIG_JZ4785)
/* TO DO */
#endif


#endif /* __DDR_COMMON_H__ */
