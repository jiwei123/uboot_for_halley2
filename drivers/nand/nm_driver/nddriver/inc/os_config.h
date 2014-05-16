#include <config.h>

#ifdef CONFIG_JZ4780
#define CONFIG_SOC_4780
#elif defined(CONFIG_JZ4775)
#define CONFIG_SOC_4775
#elif defined(CONFIG_JZ4785)
#define CONFIG_SOC_4785
#endif
