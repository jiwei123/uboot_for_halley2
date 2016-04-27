/*
 * Copyright (C) 2016 Ingenic Semiconductor
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * Copyright (C) 1994, 1995, 1996, 1997, 2000, 2001 by Ralf Baechle
 * Copyright (C) 2000 Silicon Graphics, Inc.
 * Modified for further R[236]000 support by Paul M. Antoine, 1996.
 * Kevin D. Kissell, kevink@mips.com and Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2000, 07 MIPS Technologies, Inc.
 * Copyright (C) 2003, 2004  Maciej W. Rozycki
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#ifndef FLOADER_M200_H
#define FLOADER_M200_H

#ifndef __ASSEMBLY__


/*
 * Sleep lib
 */
#define SLEEP_LIB_TCSM          0xb3426000
#define SLEEP_LIB_LPDDR2        0x8fff8000
#define SLEEP_LIB_SIZE          (16 * 1024)
#define SLEEP_LIB_EMMC_ADDRESS  (43 * 1024)

enum pmu_t {
    SM5007 = 0,
    RICOH_5T619
};

struct m200_sleep_lib_entry
{
    void (*enable_set_pmu_suspend_mode_voltage)(
            int enable, unsigned int core_volt_mV, unsigned int ddr_vdd2_volt_mV);
    void (*enable_sleep_poweroff_mode)(int enable);
    void (*restore_context)(void);
    int (*enter_sleep)(int state);
    void (*select_pmu)(enum pmu_t pmu);
};


/*
 * PUBL DLL bypass configuration
 */
#define CONFIG_DDR_PUBL_DLL_BYPASS_FREQ    300000000

/*
 * Types alias
 */
typedef __signed char s8;
typedef unsigned char u8;

typedef __signed short s16;
typedef unsigned short u16;

typedef __signed int s32;
typedef unsigned int u32;

typedef unsigned long ulong;

typedef __signed long long s64;
typedef unsigned long long u64;


typedef unsigned long int uintptr_t;
typedef u8 uint8_t;
typedef u16 uint16_t;
typedef u32 uint32_t;
typedef u64 uint64_t;

typedef u8 uchar;
typedef u16 ushort;
typedef u32 uint;

typedef u32 size_t;

typedef ulong lbaint_t;

#define ARCH_DMA_MINALIGN   32

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define ROUND(a,b)              (((a) + (b) - 1) & ~((b) - 1))
#define DIV_ROUND(n,d)          (((n) + ((d)/2)) / (d))
#define DIV_ROUND_UP(n,d)       (((n) + (d) - 1) / (d))
#define roundup(x, y)           ((((x) + ((y) - 1)) / (y)) * (y))

#define ALIGN(x,a)              __ALIGN_MASK((x),(typeof(x))(a)-1)
#define __ALIGN_MASK(x,mask)    (((x)+(mask))&~(mask))


#define PAD_COUNT(s, pad) ((s - 1) / pad + 1)
#define PAD_SIZE(s, pad) (PAD_COUNT(s, pad) * pad)
#define ALLOC_ALIGN_BUFFER_PAD(type, name, size, align, pad)        \
    char __##name[ROUND(PAD_SIZE(size * sizeof(type), pad), align)  \
              + (align - 1)];                   \
                                    \
    type *name = (type *) ALIGN((uintptr_t)__##name, align)
#define ALLOC_ALIGN_BUFFER(type, name, size, align)     \
    ALLOC_ALIGN_BUFFER_PAD(type, name, size, align, 1)
#define ALLOC_CACHE_ALIGN_BUFFER_PAD(type, name, size, pad)     \
    ALLOC_ALIGN_BUFFER_PAD(type, name, size, ARCH_DMA_MINALIGN, pad)
#define ALLOC_CACHE_ALIGN_BUFFER(type, name, size)          \
    ALLOC_ALIGN_BUFFER(type, name, size, ARCH_DMA_MINALIGN)

/*
 * DEFINE_CACHE_ALIGN_BUFFER() is similar to ALLOC_CACHE_ALIGN_BUFFER, but it's
 * purpose is to allow allocating aligned buffers outside of function scope.
 * Usage of this macro shall be avoided or used with extreme care!
 */
#define DEFINE_ALIGN_BUFFER(type, name, size, align)            \
    static char __##name[roundup(size * sizeof(type), align)]   \
            __aligned(align);               \
                                    \
    static type *name = (type *)__##name
#define DEFINE_CACHE_ALIGN_BUFFER(type, name, size)         \
    DEFINE_ALIGN_BUFFER(type, name, size, ARCH_DMA_MINALIGN)

/*
 * Stop errors
 */
#define STOP_ERROR_DETECT_PMU_FAILED                1
#define STOP_ERROR_DELAY_UNAVAILABLE                2
#define STOP_ERROR_WRONG_GPIO_NUM                   3
#define STOP_ERROR_PUBL_PGSR_TIMEOUT                4
#define STOP_ERROR_PUBL_PGSR_TRAINING_FAILED        5
#define STOP_ERROR_DRAM_WRONG_STATE                 6
#define STOP_ERROR_PMU_WRITE_FAILED                 7
#define STOP_ERROR_PMU_WRONG_VALUE                  8
#define STOP_ERROR_EMMC_MSC0_INIT_FAILED            9
#define STOP_ERROR_EMMC_MSC0_LOADER_FAILED          10


#define NULL ((void *) 0)
/*
 * Define the bit field macro to avoid the bit mistake
 */
#define BIT0            (1 << 0)
#define BIT1            (1 << 1)
#define BIT2            (1 << 2)
#define BIT3            (1 << 3)
#define BIT4            (1 << 4)
#define BIT5            (1 << 5)
#define BIT6            (1 << 6)
#define BIT7            (1 << 7)
#define BIT8            (1 << 8)
#define BIT9            (1 << 9)
#define BIT10           (1 << 10)
#define BIT11           (1 << 11)
#define BIT12           (1 << 12)
#define BIT13           (1 << 13)
#define BIT14           (1 << 14)
#define BIT15           (1 << 15)
#define BIT16           (1 << 16)
#define BIT17           (1 << 17)
#define BIT18           (1 << 18)
#define BIT19           (1 << 19)
#define BIT20           (1 << 20)
#define BIT21           (1 << 21)
#define BIT22           (1 << 22)
#define BIT23           (1 << 23)
#define BIT24           (1 << 24)
#define BIT25           (1 << 25)
#define BIT26           (1 << 26)
#define BIT27           (1 << 27)
#define BIT28           (1 << 28)
#define BIT29           (1 << 29)
#define BIT30           (1 << 30)
#define BIT31           (1 << 31)

/*
 * GPIO helpers
 */
#define GPIO_PA(n)  (0*32 + n)
#define GPIO_PB(n)  (1*32 + n)
#define GPIO_PC(n)  (2*32 + n)
#define GPIO_PD(n)  (3*32 + n)
#define GPIO_PE(n)  (4*32 + n)
#define GPIO_PF(n)  (5*32 + n)
#define GPIO_PG(n)  (6*32 + n)

enum gpio_function {
    GPIO_FUNC_0,
    GPIO_FUNC_1,
    GPIO_FUNC_2,
    GPIO_FUNC_3,
};

/*
 * IO helpers
 */
#define readb(addr) (*(volatile unsigned char *)(addr))
#define readw(addr) ((*(volatile unsigned short *)(addr)))
#define readl(addr) ((*(volatile unsigned int *)(addr)))

#define writeb(b, addr) ((*(volatile unsigned char *)(addr)) = (b))
#define writew(b, addr) ((*(volatile unsigned short *)(addr)) = (unsigned short)(b))
#define writel(b, addr) ((*(volatile unsigned int *)(addr)) = (unsigned int)(b))


/*
 * Unaligned access helpers
 */
static inline u16 __get_unaligned_le16(const u8 *p)
{
    return p[0] | p[1] << 8;
}

static inline u32 __get_unaligned_le32(const u8 *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline u64 __get_unaligned_le64(const u8 *p)
{
    return (u64)__get_unaligned_le32(p + 4) << 32 |
           __get_unaligned_le32(p);
}

static inline void __put_unaligned_le16(u16 val, u8 *p)
{
    *p++ = val;
    *p++ = val >> 8;
}

static inline void __put_unaligned_le32(u32 val, u8 *p)
{
    __put_unaligned_le16(val >> 16, p + 2);
    __put_unaligned_le16(val, p);
}

static inline void __put_unaligned_le64(u64 val, u8 *p)
{
    __put_unaligned_le32(val >> 32, p + 4);
    __put_unaligned_le32(val, p);
}

static inline u16 get_unaligned_le16(const void *p)
{
    return __get_unaligned_le16((const u8 *)p);
}

static inline u32 get_unaligned_le32(const void *p)
{
    return __get_unaligned_le32((const u8 *)p);
}

static inline u64 get_unaligned_le64(const void *p)
{
    return __get_unaligned_le64((const u8 *)p);
}

static inline void put_unaligned_le16(u16 val, void *p)
{
    __put_unaligned_le16(val, p);
}

static inline void put_unaligned_le32(u32 val, void *p)
{
    __put_unaligned_le32(val, p);
}

static inline void put_unaligned_le64(u64 val, void *p)
{
    __put_unaligned_le64(val, p);
}

struct i2c {
    unsigned int scl;
    unsigned int sda;
    char* name;
};

int i2c_write(struct i2c *i2c,unsigned char chip,
        unsigned int addr, int alen, unsigned char *buffer, int len);
int i2c_read(struct i2c *i2c,unsigned char chip,
        unsigned int addr, int alen, int repeat_start,
        unsigned char *buffer, int len);

int i2c_probe(struct i2c *i2c, unsigned char addr);

void init_mmc_host();
ulong mmc_bread(lbaint_t start, lbaint_t blkcnt, void *dst);

void pmu_sm5007_set_core_voltage(int mV);
void pmu_sm5007_set_core_sleep_voltage(int mV);
int probe_sm5007();

void pmu_5t619_set_core_voltage(int mV);
void pmu_5t619_set_core_sleep_voltage(int mV);
void pmu_5t619_set_ddr_vdd2_voltage(int mV);
void pmu_5t619_set_ddr_vdd2_sleep_voltage(int mV);
void pmu_5t619_set_buck5_for_vibrate(int mV);
int probe_5t619();

int is_boot_from_powerup();

void init_uart();
void init_clk();
void reset_clk_tree();

unsigned int get_ddr_freq();
void set_msc0_freq(unsigned int freq);
unsigned int get_msc0_freq();

void boot_from_ddr();
void boot_from_emmc();
void init_sleep_lib(enum pmu_t pmu);
void enter_sleep_with_powerkey_wake();

void init_lpddr2(int is_from_powerup);
void lpddr2_enter_self_refresh();
void lpddr2_exit_self_refresh(
                        unsigned int training_pir,
                        unsigned int done_mask,
                        unsigned int training_timeout);
void prepare_lpddr2_for_sleep();
void restore_lpddr2();
int has_two_lpddr2_chips();

void enable_clock_uart3();

#ifdef DEBUG
    #define debug(formats, args...) printf(formats, ##args)
#else
    #define debug(formats, args...)
#endif

void putc(const char c);
void puts(const char *s);
int printf(const char *fmt, ...);
int sprintf(char *buf, const char *fmt, ...);

void udelay(unsigned long usec);
void mdelay(unsigned long msec);

char *strcpy(char *dest,const char *src);

void *memcpy(void *dest, const void *src, size_t count);
void *memset(void *s,int c, size_t count);
int memcmp(const void *cs,const void *ct,size_t count);

void dump_mem_u8(const void *src, unsigned int size_by_byte);
void dump_mem_u32(const void *src, unsigned int size_by_byte);

void flush_icache_all(void);
void flush_dcache_all(void);
void flush_cache_all(void);
void init_cache(void);
void flush_scache_all(void);

void local_flush_tlb_all(void);

int gpio_request(int gpio, const char *label);
int gpio_free(int gpio);
int gpio_set_value(int gpio, int value);
int gpio_get_value(int gpio);
int gpio_direction_input(int gpio);
int gpio_direction_output(int gpio, int value);
int gpio_enable_pullup(int gpio);
int gpio_disable_pullup(int gpio);
int gpio_set_func(int gpio, enum gpio_function func);
void gpio_enable_pa30_as_fall_edge_irq();
void gpio_disable_ack_pa30_irq();

void stop(int stop_reason);

void select_pmu(enum pmu_t pmu);
enum pmu_t get_pmu();
/*
 * Define the module base addresses
 */

/* AHB0 BUS Devices Base */
#define HARB0_BASE      0xb3000000
#define DDRC_BASE       0xb34f0000
#define DDRC_APB_BASE   0xb3012000
#define DDR_PHY_OFFSET  (-0x4e0000 + 0x1000)
#define X2D_BASE    0xb3030000
#define GPU_BASE    0xb3040000
#define LCDC0_BASE  0xb3050000
#define CIM_BASE    0xb3060000
#define COMPRESS_BASE   0xb3070000
#define IPU0_BASE   0xb3080000
#define GPVLC_BASE  0xb3090000
/*#define LCDC1_BASE    0xb30a0000*/
#define IPU1_BASE   0xb30b0000
#define MONITOR_BASE    0xb30f0000

/* AHB1 BUS Devices Base */
#define SCH_BASE    0xb3200000
#define VDMA_BASE   0xb3210000
#define EFE_BASE    0xb3240000
#define MCE_BASE    0xb3250000
#define DBLK_BASE   0xb3270000
#define VMAU_BASE   0xb3280000
#define SDE_BASE    0xb3290000
#define AUX_BASE    0xb32a0000
#define TCSM_BASE   0xb32c0000
#define JPGC_BASE   0xb32e0000
#define SRAM_BASE   0xb32f0000

/* AHB2 BUS Devices Base */
#define HARB2_BASE  0xb3400000
#define NFI_BASE    0xB3410000
#define NEMC_BASE   0xb3410000
#define PDMA_BASE   0xb3420000
#define MSC0_BASE   0xb3450000
#define MSC1_BASE   0xb3460000
#define MSC2_BASE   0xb3470000
#define GPS_BASE    0xb3480000
#define EHCI_BASE   0xb3490000
#define OHCI_BASE   0xb34a0000
#define ETHC_BASE   0xb34b0000
#define BCH_BASE    0xb34d0000
#define TSSI0_BASE  0xb34e0000
#define TSSI1_BASE  0xb34f0000
#define OTG_BASE    0xb3500000
#define EFUSE_BASE  0xb3540000

#define OST_BASE    0xb2000000
#define HDMI_BASE   0xb0180000

/* APB BUS Devices Base */
#define CPM_BASE    0xb0000000
#define INTC_BASE   0xb0001000
#define TCU_BASE    0xb0002000
#define RTC_BASE    0xb0003000
#define GPIO_BASE   0xb0010000
#define AIC0_BASE   0xb0020000
#define AIC1_BASE   0xb0021000
#define UART0_BASE  0xb0030000
#define UART1_BASE  0xb0031000
#define UART2_BASE  0xb0032000
#define UART3_BASE  0xb0033000
#define UART4_BASE  0xb0034000
#define SSC_BASE    0xb0040000
#define SSI0_BASE   0xb0043000
#define SSI1_BASE   0xb0044000
#define I2C0_BASE   0xb0050000
#define I2C1_BASE   0xb0051000
#define I2C2_BASE   0xb0052000
#define I2C4_BASE   0xb0054000
#define KMC_BASE    0xb0060000
#define DES_BASE    0xb0061000
#define SADC_BASE   0xb0070000
#define PCM0_BASE   0xb0071000
#define OWI_BASE    0xb0072000
#define PCM1_BASE   0xb0074000
#define WDT_BASE    0xb0002000

/* NAND CHIP Base Address*/
#define NEMC_CS1_BASE 0xbb000000
#define NEMC_CS2_BASE 0xba000000
#define NEMC_CS3_BASE 0xb9000000
#define NEMC_CS4_BASE 0xb8000000
#define NEMC_CS5_BASE 0xb7000000
#define NEMC_CS6_BASE 0xb6000000

#define AUX_BASE    0xb32a0000

/*
 * Clock management segment
 */

enum clk_id {
    DDR,
    VPU,
    OTG,
    I2S,
    LCD,
    MSC,
    MSC0 = MSC,
    MSC1,
    MSC2,
    UHC,
    SSI,
    CIM,
    PCM,
    GPU,
    ISP,
    BCH,
    CGU_CNT,
    CPU = CGU_CNT,
    H2CLK,
    APLL,
    MPLL,
    EXCLK,
};

#define ISR_OFF     (0x00)
#define IMR_OFF     (0x04)
#define IMSR_OFF    (0x08)
#define IMCR_OFF    (0x0c)
#define IPR_OFF     (0x10)

#define CPM_CPCCR   (0x00)
#define CPM_CPCSR   (0xd4)

#define CPM_DDRCDR  (0x2c)
#define CPM_VPUCDR  (0x30)
#define CPM_CPSPR   (0x34)
#define CPM_CPSPPR  (0x38)
#define CPM_USBPCR  (0x3c)
#define CPM_USBRDT  (0x40)
#define CPM_USBVBFIL (0x44)
#define CPM_USBPCR1 (0x48)
#define CPM_USBCDR  (0x50)
#define CPM_I2SCDR  (0x60)
#define CPM_LPCDR   (0x64)
#define CPM_MSC0CDR (0x68)
#define CPM_MSC1CDR (0xa4)
#define CPM_MSC2CDR (0xa8)
#define CPM_USBCDR  (0x50)
#define CPM_UHCCDR  (0x6c)
#define CPM_SSICDR  (0x74)
#define CPM_CIMCDR  (0x7c)
#define CPM_ISPCDR  (0x80)
#define CPM_PCMCDR  (0x84)
#define CPM_GPUCDR  (0x88)
#define CPM_BCHCDR  (0xac)

#define CPM_DRCG    (0xd0)

#define CPM_CPPCR   (0x0c)
#define CPM_CPAPCR  (0x10)
#define CPM_CPMPCR  (0x14)
#define CPM_CPAPACR (0X18)
#define CPM_CPMPACR (0X1C)

#define CPM_INTR    (0xb0)
#define CPM_INTRE   (0xb4)
#define CPM_SPCR0   (0xb8)

#define CPM_USBPCR  (0x3c)
#define CPM_USBRDT  (0x40)
#define CPM_USBVBFIL    (0x44)
#define CPM_USBPCR1 (0x48)

#define CPM_LCR     (0x04)
#define CPM_PGR     (0xe4)
#define CPM_PSWC0ST     (0x90)
#define CPM_PSWC1ST     (0x94)
#define CPM_PSWC2ST     (0x98)
#define CPM_PSWC3ST     (0x9c)
#define CPM_CLKGR   (0x20)
#define CPM_CLKGR1  (0x28)
#define CPM_SRBC    (0xc4)
#define CPM_SLBC    (0xc8)
#define CPM_SLPC    (0xcc)
#define CPM_OPCR    (0x24)

#define CPM_RSR     (0x08)

#define LCR_LPM_MASK        (0x3)
#define LCR_LPM_SLEEP       (0x1)

#define CPM_LCR_PD_P0       (0x1<<31)
#define CPM_LCR_PD_P1       (0x1<<30)
#define CPM_LCR_PD_VPU      (0x1<<29)
#define CPM_LCR_PD_GPU      (0x1<<28)
#define CPM_LCR_PD_ISP      (0x1<<27)
#define CPM_LCR_PD_H2D      (0x1<<26)
#define CPM_LCR_P0S     (0x1<<25)
#define CPM_LCR_P1S     (0x1<<24)
#define CPM_LCR_VPUS        (0x1<<23)
#define CPM_LCR_GPUS        (0x1<<22)
#define CPM_LCR_ISPS        (0x1<<21)
#define CPM_LCR_H2DS        (0x1<<20)
#define CPM_LCR_PD_DMIC     (0x1<<7)
#define CPM_LCR_DMIC_S      (0x1<<6)
#define CPM_LCR_IDLE_DS     (0x1<<3)
#define CPM_LCR_SLEEP_DS    (0x1<<2)

#define CPM_CLKGR_DDR       (1 << 31)
#define CPM_CLKGR_TCU       (1 << 30)
#define CPM_CLKGR_RTC       (1 << 29)
#define CPM_CLKGR_DES       (1 << 28)
#define CPM_CLKGR_PCM       (1 << 27)
#define CPM_CLKGR_DSI       (1 << 26)
#define CPM_CLKGR_CSI       (1 << 25)
#define CPM_CLKGR_LCD       (1 << 24)
#define CPM_CLKGR_ISP       (1 << 23)
#define CPM_CLKGR_UHC       (1 << 22)
#define CPM_CLKGR_PDMA      (1 << 21)
#define CPM_CLKGR_SSI2      (1 << 20)
#define CPM_CLKGR_SSI1      (1 << 19)
#define CPM_CLKGR_UART4     (1 << 18)
#define CPM_CLKGR_UART3     (1 << 17)
#define CPM_CLKGR_UART2     (1 << 16)
#define CPM_CLKGR_UART1     (1 << 15)
#define CPM_CLKGR_UART0     (1 << 14)
#define CPM_CLKGR_SADC      (1 << 13)
#define CPM_CLKGR_MSC2      (1 << 12)
#define CPM_CLKGR_AIC       (1 << 11)
#define CPM_CLKGR_I2C3      (1 << 10)
#define CPM_CLKGR_I2C2      (1 << 9)
#define CPM_CLKGR_I2C1      (1 << 8)
#define CPM_CLKGR_I2C0      (1 << 7)
#define CPM_CLKGR_SSI0      (1 << 6)
#define CPM_CLKGR_MSC1      (1 << 5)
#define CPM_CLKGR_MSC0      (1 << 4)
#define CPM_CLKGR_OTG       (1 << 3)
#define CPM_CLKGR_BCH       (1 << 2)
#define CPM_CLKGR_NEMC      (1 << 1)
#define CPM_CLKGR_NFI       (1 << 0)

#define CPM_CLKGR_CPU       (1 << 15)
#define CPM_CLKGR_APB0      (1 << 14)
#define CPM_CLKGR_DLINE     (1 << 13)
#define CPM_CLKGR_TCU_EXCLK (1 << 12)
#define CPM_CLKGR_SYS_OST   (1 << 11)
#define CPM_CLKGR_AHB0      (1 << 10)
#define CPM_CLKGR_P0        (1 << 9)
#define CPM_CLKGR_P1        (1 << 8)
#define CPM_CLKGR_DMIC      (1 << 7)
#define CPM_CLKGR_HASH      (1 << 6)
#define CPM_CLKGR_AES       (1 << 5)
#define CPM_CLKGR_EPD       (1 << 4)
#define CPM_CLKGR_AHB_MON   (1 << 3)
#define CPM_CLKGR_IPU       (1 << 2)
#define CPM_CLKGR_GPU       (1 << 1)
#define CPM_CLKGR_VPU       (1 << 0)

#define CPM_RSR_HR      (1 << 3)
#define CPM_RSR_P0R     (1 << 2)
#define CPM_RSR_WR      (1 << 1)
#define CPM_RSR_PR      (1 << 0)

#define OPCR_ERCS       (0x1<<2)
#define OPCR_PD         (0x1<<3)
#define OPCR_IDLE       (0x1<<31)

#define CLKGR_VPU       (0x1<<19)

#define cpm_inl(off)        readl(CPM_BASE + (off))
#define cpm_outl(val,off)   writel(val, CPM_BASE + (off))
#define cpm_readl(offset)       readl(CPM_BASE + (offset))
#define cpm_writel(b, offset)   writel(b, CPM_BASE + (offset))
#define cpm_test_bit(bit,off)   (cpm_inl(off) & 0x1<<(bit))
#define cpm_set_bit(bit,off)    (cpm_outl((cpm_inl(off) | 0x1<<(bit)),off))
#define cpm_clear_bit(bit,off)  (cpm_outl(cpm_inl(off) & ~(0x1 << bit), off))

/*USBCDR*/
#define USBCDR_UCS_PLL      (1 << 31)
#define USBCDR_UPCS_MPLL    (1 << 30)
#define USBCDR_CE_USB       (1 << 29)
#define USBCDR_USB_BUSY     (1 << 28)
#define USBCDR_USB_STOP     (1 << 27)
#define USBCDR_USB_DIS      (1 << 26)
#define USBCDR_MIPI_CS      (1 << 25)
#define USBCDR_USBCDR_MSK   (0xff)

/*USBPCR*/
#define USBPCR_USB_MODE_ORG (1 << 31)
#define USBPCR_VBUSVLDEXT   (1 << 24)
#define USBPCR_VBUSVLDEXTSEL    (1 << 23)
#define USBPCR_POR      (1 << 22)
#define USBPCR_OTG_DISABLE  (1 << 20)

/*USBPCR1*/
#define USBPCR1_REFCLKSEL_BIT   (26)
#define USBPCR1_REFCLKSEL_MSK   (0x3 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKSEL_CORE  (0x2 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKSEL_EXT   (0x1 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKSEL_CSL   (0x0 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKDIV_BIT   (24)
#define USBPCR1_REFCLKDIV_MSK   (0X3 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_19_2M (0x3 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_48M   (0x2 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_24M   (0x1 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_12M   (0x0 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_WORD_IF0_16_30  (1 << 19)

/*OPCR*/
#define OPCR_SPENDN0        (1 << 7)

/* CPM scratch pad protected register(CPSPPR) */
#define CPSPPR_CPSPR_WRITABLE   (0x00005a5a)
#define RECOVERY_SIGNATURE      (0x1a1a)        /* means "RECY" */
#define RECOVERY_SIGNATURE_SEC  0x800           /* means "RECY" */
#define FASTBOOT_SIGNATURE      (0x0666)        /* means "FASTBOOT" */

#define cpm_get_scrpad()        readl(CPM_BASE + CPM_CPSPR)
#define cpm_set_scrpad(data)                    \
do {                                            \
    volatile int i = 0x3fff;                \
    writel(0x5a5a,CPM_BASE + CPM_CPSPPR);       \
    while(i--);             \
    writel(data,CPM_BASE + CPM_CPSPR);          \
    writel(0xa5a5,CPM_BASE + CPM_CPSPPR);       \
} while (0)

/*************************************************************************
 * DDRC (DDR Controller)
 *************************************************************************/
struct sleep_context {
    unsigned int gpr_s[8];
    unsigned int gpr_gp;
    unsigned int gpr_sp;
    unsigned int gpr_fp;
    unsigned int gpr_ra;

    unsigned int cp0_status;
    unsigned int cp0_ebase;
    unsigned int cp0_random;
    unsigned int cp0_pagemask;
    unsigned int cp0_wired;
    unsigned int cp0_entryhi;
    unsigned int cp0_config;
    unsigned int cp0_config7;
    unsigned int cp0_watchlo;
    unsigned int cp0_watchhi;
    unsigned int cp0_desave;
    unsigned int pmon_csr;
    unsigned int pmon_high;
    unsigned int pmon_lc;
    unsigned int pmon_rc;
    unsigned int gpr_hi;
    unsigned int gpr_lo;

    unsigned int core_ctrl;         /* $12, sel 2 */
    unsigned int core_reim;         /* $12, sel 4 */
    unsigned int cp0_ecc;
    unsigned int core_reim_low;     /* $12, sel 7 */

    unsigned int cpm_lcr;
    unsigned int cpm_pgr;
    unsigned int cpm_opcr;
    unsigned int cpm_spcr0;
    unsigned int cpm_clkgr;
    unsigned int cpm_clkgr1;
    unsigned int cpm_usbpcr;
};

typedef union ddrc_timing1 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tWL:6;
        unsigned reserved6_7:2;
        unsigned tWR:6;
        unsigned reserved14_15:2;
        unsigned tWTR:6;
        unsigned reserved22_23:2;
        unsigned tRTP:6;
        unsigned reserved30_31:2;
    } b;
} ddrc_timing1_t;

typedef union ddrc_timing2 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tRL:6;
        unsigned reserved6_7:2;
        unsigned tRCD:6;
        unsigned reserved14_15:2;
        unsigned tRAS:6;
        unsigned reserved22_23:2;
        unsigned tCCD:6;
        unsigned reserved30_31:2;
    } b;

} ddrc_timing2_t;

typedef union ddrc_timing3 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tRC:6;
        unsigned reserved6_7:2;
        unsigned tRRD:6;
        unsigned reserved14_15:2;
        unsigned tRP:6;
        unsigned reserved22_23:2;
        unsigned tCKSRE:3;
        unsigned ONUM:4;
        unsigned reserved31:1;
    } b;
} ddrc_timing3_t;

typedef union ddrc_timing4 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tMRD:2;
        unsigned reserved2_3:2;
        unsigned tXP:3;
        unsigned reserved7:1;
        unsigned tMINSR:4;
        unsigned reserved12_15:4;
        unsigned tCKE:3;
        unsigned tRWCOV:2;
        unsigned tEXTRW:3;
        unsigned tRFC:6;
        unsigned reserved30_31:2;
    } b;
} ddrc_timing4_t;

typedef union ddrc_timing5 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tWDLAT:6;
        unsigned reserved6_7:2;
        unsigned tRDLAT:6;
        unsigned reserved14_15:2;
        unsigned tRTW:6;
        unsigned reserved22_23:2;
        unsigned tCTLUPD:8;
    } b;
} ddrc_timing5_t;

typedef union ddrc_timing6 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tCFGR:6;
        unsigned reserved6_7:2;
        unsigned tCFGW:6;
        unsigned reserved14_15:2;
        unsigned tFAW:6;
        unsigned reserved22_23:2;
        unsigned tXSRD:8;
    } b;
} ddrc_timing6_t;

typedef union ddrc_cfg {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned DW:1;
        unsigned BA0:1;
        unsigned CL:4;
        unsigned CS0EN:1;
        unsigned CS1EN:1;
        unsigned COL0:3;
        unsigned ROW0:3;
        unsigned reserved14:1;
        unsigned MISPE:1;
        unsigned ODTEN:1;
        unsigned TYPE:3;
        unsigned reserved20:1;
        unsigned BSL:1;
        unsigned IMBA:1;
        unsigned BA1:1;
        unsigned COL1:3;
        unsigned ROW1:3;
        unsigned reserved30_31:2;
    } b;
} ddrc_cfg_t;

struct ddrc_reg {
    ddrc_cfg_t cfg;
    unsigned int ctrl;
    unsigned int refcnt;
    unsigned int mmap[2];
    unsigned int remap[5];
    ddrc_timing1_t timing1;
    ddrc_timing2_t timing2;
    ddrc_timing3_t timing3;
    ddrc_timing4_t timing4;
    ddrc_timing5_t timing5;
    ddrc_timing6_t timing6;
};

typedef union ddrp_mr0 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned BL:2;
        unsigned CL_2:1;
        unsigned BT:1;
        unsigned CL_4_6:3;
        unsigned TM:1;
        unsigned DR:1;
        unsigned WR:3;
        unsigned PD:1;
        unsigned RSVD:3;
        unsigned reserved16_31:16;
    } ddr3; /* MR0 */
    struct {
        unsigned BL:3;
        unsigned BT:1;
        unsigned CL:3;
        unsigned TM:1;
        unsigned RSVD8_11:4;
        unsigned RSVD12_15:4;
        unsigned reserved16_31:16;
    } lpddr; /* MR */
    struct {
        unsigned unsupported:31;
    } ddr2; /* MR */
    struct {
        unsigned unsupported:31;
    } ddr; /* MR */
} ddrp_mr0_t;

typedef union ddrp_mr1 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned DE:1;
        unsigned DIC1:1;
        unsigned RTT2:1;
        unsigned AL:2;
        unsigned DIC5:1;
        unsigned RTT6:1;
        unsigned LEVEL:1;
        unsigned RSVD8:1;
        unsigned RTT9:1;
        unsigned RSVD10:1;
        unsigned TDQS:1;
        unsigned QOFF:1;
        unsigned RSVD13_15:3;
        unsigned reserved16_31:16;
    } ddr3; /* MR1 */
    struct {
        unsigned BL:3;
        unsigned BT:1;
        unsigned WC:1;
        unsigned nWR:3;
        unsigned reserved8_31:24;
    } lpddr2; /* MR1 */
    struct {
        unsigned unsupported:31;
    } ddr2; /* EMR */
    struct {
        unsigned unsupported:31;
    } ddr; /* EMR */
} ddrp_mr1_t;

typedef union ddrp_mr2 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned PASR:3;
        unsigned CWL:3;
        unsigned ASR:1;
        unsigned SRT:1;
        unsigned RSVD8:1;
        unsigned RTTWR:2;
        unsigned RSVD11_15:5;
        unsigned reserved16_31:16;
    } ddr3; /* MR2 */
    struct {
        unsigned PASR:3;
        unsigned TCSR:2;
        unsigned DS:3;
        unsigned RSVD8_15:8;
        unsigned reserved16_31:16;
    } lpddr; /* EMR */
    struct {
        unsigned RL_WL:4;
        unsigned RSVD4_7:4;
        unsigned reserved8_31:24;
    } lpddr2; /* MR2 */
    struct {
        unsigned unsupported:31;
    } ddr2; /* EMR2 */
} ddrp_mr2_t;

typedef union ddrp_mr3 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned MPRLOC:2;
        unsigned MPR:1;
        unsigned RSVD3_15:13;
        unsigned reserved16_31:16;
    } ddr3; /* MR3 */
    struct {
        unsigned DS:4;
        unsigned RSVD4_7:4;
        unsigned reserved8_31:24;
    } lpddr2; /* MR2 */
    struct {
        unsigned unsupported:31;
    } ddr2; /* EMR3 */
} ddrp_mr3_t;

typedef union ddrp_ptr0 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tDLLSRST:6;
        unsigned tDLLLOCK:12;
        unsigned tITMSRST:4;
        unsigned reserved22_31:10;
    } b;
} ddrp_ptr0_t;

typedef union ddrp_ptr1 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tDINIT0:19;
        unsigned tDINIT1:8;
        unsigned reserved27_31:5;
    } b;
} ddrp_ptr1_t;

typedef union ddrp_ptr2 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tDINIT2:17;
        unsigned tDINIT3:10;
        unsigned reserved27_31:5;
    } b;
} ddrp_ptr2_t;

typedef union ddrp_dtpr0 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tMRD:2;
        unsigned tRTP:3;
        unsigned tWTR:3;
        unsigned tRP:4;
        unsigned tRCD:4;
        unsigned tRAS:5;
        unsigned tRRD:4;
        unsigned tRC:6;
        unsigned tCCD:1;
    } b;
} ddrp_dtpr0_t;

typedef union ddrp_dtpr1 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tAOND_tAOFD:2;
        unsigned tRTW:1;
        unsigned tFAW:6;
        unsigned tMOD:2;
        unsigned tRTODT:1;
        unsigned reserved12_15:4;
        unsigned tRFC:8;
        unsigned tDQSCK:3;
        unsigned tDQSCKmax:3;
        unsigned reserved30_31:2;
    } b;
} ddrp_dtpr1_t;

typedef union ddrp_dtpr2 {
    /** raw register data */
    unsigned int d32;
    /** register bits */
    struct {
        unsigned tXS:10;
        unsigned tXP:5;
        unsigned tCKE:4;
        unsigned tDLLK:10;
        unsigned reserved29_31:3;
    } b;
} ddrp_dtpr2_t;

struct ddrp_reg {
    unsigned int dcr;
    ddrp_mr0_t mr0;
    ddrp_mr1_t mr1;
    ddrp_mr2_t mr2;
    ddrp_mr3_t mr3;
    unsigned int odtcr;
    unsigned int pgcr;
    ddrp_ptr0_t ptr0;
    ddrp_ptr1_t ptr1;
    ddrp_ptr2_t ptr2;
    ddrp_dtpr0_t dtpr0;
    ddrp_dtpr1_t dtpr1;
    ddrp_dtpr2_t dtpr2;
};

enum {
    DDR3,
    LPDDR,
    LPDDR2,
    DDR2,
    VARIABLE,
};

struct tck {
    unsigned int ps;
    unsigned int ns;
};

struct RL_LPDDR2 {
    unsigned int memclk;
    unsigned int RL;
};

struct WL_LPDDR2 {
    unsigned int memclk;
    unsigned int WL;
};

struct size {
    unsigned int chip0;
    unsigned int chip1;
};

struct lpddr2_params {
    unsigned int tRAS;
    unsigned int tRP;
    unsigned int tRCD;
    unsigned int tRC;
    unsigned int tWR;
    unsigned int tRRD;
    unsigned int tRTP;
    unsigned int tWTR;
    unsigned int tRFC;
    unsigned int tMINSR;
    unsigned int tXP;
    unsigned int tMRD;
    unsigned int tCCD;
    unsigned int tFAW;
    unsigned int tCKE;
    unsigned int tRL;
    unsigned int tWL;
    unsigned int tRDLAT;
    unsigned int tWDLAT;
    unsigned int tRTW;
    unsigned int tCKSRE;
    unsigned int tDLLLOCK;
    unsigned int tXSDLL;
    unsigned int tMOD;
    unsigned int tXPDLL;
    unsigned int tXS;
    unsigned int tXSRD;
    unsigned int tREFI;
    unsigned int tDLLSRST;
    unsigned int tDQSCK;
    unsigned int tDQSCKmax;
};

struct ddr_params {
    unsigned int type;
    unsigned int freq;
    unsigned int div;
    unsigned int cs0;
    unsigned int cs1;
    unsigned int dw32;
    unsigned int cl;
    unsigned int bl;
    unsigned int col;
    unsigned int row;
    unsigned int col1;
    unsigned int row1;
    unsigned int bank8;
    struct tck tck;
    struct size size;
    struct lpddr2_params lpddr2_params;
};

struct global_info {
    uint32_t extal;
    uint32_t cpufreq;
    uint32_t ddrfreq;
    uint32_t ddr_div;
    uint32_t uart_idx;
    uint32_t baud_rate;
    struct ddr_params ddr_params;
};

#define DDR_MEM_PHY_BASE        0x20000000

#define DDRC_STATUS         0x0
#define DDRC_CFG            0x4
#define DDRC_CTRL           0x8
#define DDRC_LMR            0xc
#define DDRC_REFCNT         0x18
#define DDRC_MMAP0          0x24
#define DDRC_MMAP1          0x28
#define DDRC_DLP            0xbc
#define DDRC_STRB           0x34
#define DDRC_AUTOSR_CNT     0x308
#define DDRC_AUTOSR_EN      0x304


/*
 * DDRC APB register offset
 */
#define DDRC_APB_CLKSTP_CFG 0x68
#define DDRC_APB_CKC_CFG0   0x84
#define DDRC_APB_CKC_CFG1   0x88

#define DDRC_TIMING(n)          (0x60 + 4 * (n - 1))
#define DDRC_REMAP(n)           (0x9c + 4 * (n - 1))

#define DDRP_PIR    (DDR_PHY_OFFSET + 0x4) /* PHY Initialization Register */
#define DDRP_PGCR   (DDR_PHY_OFFSET + 0x8) /* PHY General Configuration Register*/
#define DDRP_PGSR   (DDR_PHY_OFFSET + 0xc) /* PHY General Status Register*/
#define DDRP_DLLGCR (DDR_PHY_OFFSET + 0x10) /* DLL General Control Register*/
#define DDRP_ACDLLCR    (DDR_PHY_OFFSET + 0x14) /* AC DLL Control Register*/

#define DDRP_PTR0   (DDR_PHY_OFFSET + 0x18) /* PHY Timing Register 0 */
#define DDRP_PTR1   (DDR_PHY_OFFSET + 0x1c) /* PHY Timing Register 1 */
#define DDRP_PTR2   (DDR_PHY_OFFSET + 0x20) /* PHY Timing Register 2 */

#define DDRP_ACIOCR (DDR_PHY_OFFSET + 0x24) /* AC I/O Configuration Register */
#define DDRP_DXCCR  (DDR_PHY_OFFSET + 0x28) /* DATX8 Common Configuration Register */
#define DDRP_DSGCR  (DDR_PHY_OFFSET + 0x2c) /* DDR System General Configuration Register */
#define DDRP_DCR    (DDR_PHY_OFFSET + 0x30) /* DRAM Configuration Register*/

#define DDRP_DTPR0  (DDR_PHY_OFFSET + 0x34) /* DRAM Timing Parameters Register 0 */
#define DDRP_DTPR1  (DDR_PHY_OFFSET + 0x38) /* DRAM Timing Parameters Register 1 */
#define DDRP_DTPR2  (DDR_PHY_OFFSET + 0x3c) /* DRAM Timing Parameters Register 2 */
#define DDRP_MR0    (DDR_PHY_OFFSET + 0x40) /* Mode Register 0 */
#define DDRP_MR1    (DDR_PHY_OFFSET + 0x44) /* Mode Register 1 */
#define DDRP_MR2    (DDR_PHY_OFFSET + 0x48) /* Mode Register 2 */
#define DDRP_MR3    (DDR_PHY_OFFSET + 0x4c) /* Mode Register 3 */

#define DDRP_ODTCR  (DDR_PHY_OFFSET + 0x50) /* ODT Configure Register */
#define DDRP_DTAR   (DDR_PHY_OFFSET + 0x54) /* Data Training Address Register */
#define DDRP_DTDR0  (DDR_PHY_OFFSET + 0x58) /* Data Training Data Register 0 */
#define DDRP_DTDR1  (DDR_PHY_OFFSET + 0x5c) /* Data Training Data Register 1 */

#define DDRP_DCUAR  (DDR_PHY_OFFSET + 0xc0) /* DCU Address Register */
#define DDRP_DCUDR  (DDR_PHY_OFFSET + 0xc4) /* DCU Data Register */
#define DDRP_DCURR  (DDR_PHY_OFFSET + 0xc8) /* DCU Run Register */
#define DDRP_DCULR  (DDR_PHY_OFFSET + 0xcc) /* DCU Loop Register */
#define DDRP_DCUGCR (DDR_PHY_OFFSET + 0xd0) /* DCU Gerneral Configuration Register */
#define DDRP_DCUTPR (DDR_PHY_OFFSET + 0xd4) /* DCU Timing Parameters Register */
#define DDRP_DCUSR0 (DDR_PHY_OFFSET + 0xd8) /* DCU Status Register 0 */
#define DDRP_DCUSR1 (DDR_PHY_OFFSET + 0xdc) /* DCU Status Register 1 */

#define DDRP_DXGCR(n)   (DDR_PHY_OFFSET + 0x1c0 + n * 0x40) /* DATX8 n General Configuration Register */
#define DDRP_DXGSR0(n)  (DDR_PHY_OFFSET + 0x1c4 + n * 0x40) /* DATX8 n General Status Register */
#define DDRP_DXGSR1(n)  (DDR_PHY_OFFSET + 0x1c8 + n * 0x40) /* DATX8 n General Status Register */
#define DDRP_DXDLLCR(n) (DDR_PHY_OFFSET + 0x1cc + n * 0x40) /* DATX8 n General Status Register */
#define DDRP_DXDQSTR(n) (DDR_PHY_OFFSET + 0x1d4 + n * 0x40) /* DATX8 n DQS Timing Register */
#define DDRP_ZQXCR0(n)  (DDR_PHY_OFFSET + 0x180 + n * 0x10) /* ZQ impedance Control Register 0 */
#define DDRP_ZQXCR1(n)  (DDR_PHY_OFFSET + 0x184 + n * 0x10) /* ZQ impedance Control Register 1 */
#define DDRP_ZQXSR0(n)  (DDR_PHY_OFFSET + 0x188 + n * 0x10) /* ZQ impedance Status Register 0 */
#define DDRP_ZQXSR1(n)  (DDR_PHY_OFFSET + 0x18c + n * 0x10) /* ZQ impedance Status Register 1 */

/* DDRC Status Register */
#define DDRC_ST_ENDIAN  (1 << 7) /* 0 Little data endian
                        1 Big data endian */
#define DDRC_DSTATUS_MISS   (1 << 6)
#define DDRC_ST_DPDN        (1 << 5) /* 0 DDR memory is NOT in deep-power-down state
                        1 DDR memory is in deep-power-down state */
#define DDRC_ST_PDN     (1 << 4) /* 0 DDR memory is NOT in power-down state
                        1 DDR memory is in power-down state */
#define DDRC_ST_AREF        (1 << 3) /* 0 DDR memory is NOT in auto-refresh state
                        1 DDR memory is in auto-refresh state */
#define DDRC_ST_SREF        (1 << 2) /* 0 DDR memory is NOT in self-refresh state
                        1 DDR memory is in self-refresh state */
#define DDRC_ST_CKE1        (1 << 1) /* 0 CKE1 Pin is low
                        1 CKE1 Pin is high */
#define DDRC_ST_CKE0        (1 << 0) /* 0 CKE0 Pin is low
                        1 CKE0 Pin is high */

/* DDRC Configure Register */
#define DDRC_CFG_ROW1_BIT   27 /* Row Address width. */
#define DDRC_CFG_ROW1_MASK  (0x7 << DDRC_CFG_ROW1_BIT)
#define DDRC_CFG_COL1_BIT   24 /* Row Address width. */
#define DDRC_CFG_COL1_MASK  (0x7 << DDRC_CFG_COL1_BIT)
#define DDRC_CFG_BA1_BIT    23 /* Bank Address width of DDR memory */
#define DDRC_CFG_IMBA       (1 << 22)
#define DDRC_CFG_BL_BIT     21 /* Burst length for DDR chips */
#define DDRC_CFG_TYPE_BIT   17
//#define DDRC_CFG_TYPE_MASK    (0x7 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_DDR1  (2 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_MDDR  (3 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_DDR2  (4 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_LPDDR2    (5 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_DDR3  (6 << DDRC_CFG_TYPE_BIT)
//#define DDRC_CFG_ODT_EN       (1 << 16)  /* ODT EN */
#define DDRC_CFG_ODT_EN     1  /* ODT EN */
#define DDRC_CFG_ODT_EN_BIT 16  /* ODT EN */
#define DDRC_CFG_MISPE      (1 << 15)  /* mem protect */
#define DDRC_CFG_ROW0_BIT   11 /* Row Address width. */
#define DDRC_CFG_ROW0_MASK  (0x7 << DDRC_CFG_ROW0_BIT)
//#define DDRC_CFG_ROW_12   (0 << DDRC_CFG_ROW_BIT) /* 12-bit row address is used */
//#define DDRC_CFG_ROW_13   (1 << DDRC_CFG_ROW_BIT) /* 13-bit row address is used */
//#define DDRC_CFG_ROW_14   (2 << DDRC_CFG_ROW_BIT) /* 14-bit row address is used */
#define DDRC_CFG_COL0_BIT   8 /* Column Address width.
                     Specify the Column address width of external DDR. */
#define DDRC_CFG_COL0_MASK  (0x7 << DDRC_CFG_COL0_BIT)
//#define DDRC_CFG_COL_8    (0 << DDRC_CFG_COL_BIT) /* 8-bit Column address is used */
//#define DDRC_CFG_COL_9    (1 << DDRC_CFG_COL_BIT) /* 9-bit Column address is used */
//#define DDRC_CFG_COL_10   (2 << DDRC_CFG_COL_BIT) /* 10-bit Column address is used */
//#define DDRC_CFG_COL_11   (3 << DDRC_CFG_COL_BIT) /* 11-bit Column address is used */
#define DDRC_CFG_CS1EN_BIT  7 /* DDR Chip-Select-1 Enable */
#define DDRC_CFG_CS0EN_BIT  6 /* DDR Chip-Select-0 Enable */
//#define DDRC_CFG_CS1EN        (1 << 7) /* 0 DDR Pin CS1 un-used
//                      1 There're DDR memory connected to CS1 */
//#define DDRC_CFG_CS0EN        (1 << 6) /* 0 DDR Pin CS0 un-used
//                      1 There're DDR memory connected to CS0 */
#define DDRC_CFG_CL_BIT     2 /* CAS Latency */
//#define DDRC_CFG_CL_MASK  (0xf << DDRC_CFG_CL_BIT)
//#define DDRC_CFG_CL_3     (0 << DDRC_CFG_CL_BIT) /* CL = 3 tCK */
//#define DDRC_CFG_CL_4     (1 << DDRC_CFG_CL_BIT) /* CL = 4 tCK */
//#define DDRC_CFG_CL_5     (2 << DDRC_CFG_CL_BIT) /* CL = 5 tCK */
//#define DDRC_CFG_CL_6     (3 << DDRC_CFG_CL_BIT) /* CL = 6 tCK */
#define DDRC_CFG_BA0_BIT    1 /* Bank Address width of DDR memory */
//#define DDRC_CFG_BA0      (1 << 1) /* 0 4 bank device, Pin ba[1:0] valid, ba[2] un-used
//                      1 8 bank device, Pin ba[2:0] valid*/
#define DDRC_CFG_DW_BIT     0 /* External DDR Memory Data Width */
//#define DDRC_CFG_DW       (1 << 0) /*0 External memory data width is 16-bit
//                     1 External memory data width is 32-bit */

/* DDRC Control Register */
#define DDRC_CTRL_DFI_RST   (1 << 23)
#define DDRC_CTRL_DLL_RST   (1 << 22)
#define DDRC_CTRL_CTL_RST   (1 << 21)
#define DDRC_CTRL_CFG_RST   (1 << 20)
#define DDRC_CTRL_ACTPD     (1 << 15) /* 0 Precharge all banks before entering power-down
                         1 Do not precharge banks before entering power-down */
#define DDRC_CTRL_PDT_BIT   12 /* Power-Down Timer */
#define DDRC_CTRL_PDT_MASK  (0x7 << DDRC_CTRL_PDT_BIT)
#define DDRC_CTRL_PDT_DIS   (0 << DDRC_CTRL_PDT_BIT) /* power-down disabled */
#define DDRC_CTRL_PDT_8 (1 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 8 tCK idle */
#define DDRC_CTRL_PDT_16    (2 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 16 tCK idle */
#define DDRC_CTRL_PDT_32    (3 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 32 tCK idle */
#define DDRC_CTRL_PDT_64    (4 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 64 tCK idle */
#define DDRC_CTRL_PDT_128   (5 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 128 tCK idle */
#define DDRC_CTRL_ACTSTP    (1 << 11)
#define DDRC_CTRL_PRET_BIT  8 /* Precharge Timer */
#define DDRC_CTRL_PRET_MASK (0x7 << DDRC_CTRL_PRET_BIT) /*  */
#define DDRC_CTRL_PRET_DIS  (0 << DDRC_CTRL_PRET_BIT) /* PRET function Disabled */
#define DDRC_CTRL_PRET_8    (1 << DDRC_CTRL_PRET_BIT) /* Precharge active bank after 8 tCK idle */
#define DDRC_CTRL_PRET_16   (2 << DDRC_CTRL_PRET_BIT) /* Precharge active bank after 16 tCK idle */
#define DDRC_CTRL_PRET_32   (3 << DDRC_CTRL_PRET_BIT) /* Precharge active bank after 32 tCK idle */
#define DDRC_CTRL_PRET_64   (4 << DDRC_CTRL_PRET_BIT) /* Precharge active bank after 64 tCK idle */
#define DDRC_CTRL_PRET_128  (5 << DDRC_CTRL_PRET_BIT) /* Precharge active bank after 128 tCK idle */

#define DDRC_CTRL_KEEPSR    (1 << 17)
#define DDRC_CTRL_DPD       (1 << 6) /* 1 Drive external MDDR device entering Deep-Power-Down mode */

#define DDRC_CTRL_SR        (1 << 5) /* 1 Drive external DDR device entering self-refresh mode
                        0 Drive external DDR device exiting self-refresh mode */
#define DDRC_CTRL_UNALIGN   (1 << 4) /* 0 Disable unaligned transfer on AXI BUS
                        1 Enable unaligned transfer on AXI BUS */
#define DDRC_CTRL_ALH       (1 << 3) /* Advanced Latency Hiding:
                        0 Disable ALH
                        1 Enable ALH */
#define DDRC_CTRL_RDC       (1 << 2) /* 0 dclk clock frequency is lower than 60MHz
                        1 dclk clock frequency is higher than 60MHz */
#define DDRC_CTRL_CKE       (1 << 1) /* 0 Not set CKE Pin High
                        1 Set CKE Pin HIGH */
#define DDRC_CTRL_RESET (1 << 0) /* 0 End resetting ddrc_controller
                        1 Resetting ddrc_controller */


/* DDRC Load-Mode-Register */
#define DDRC_LMR_DDR_ADDR_BIT   16 /* When performing a DDR command, DDRC_ADDR[13:0]
                          corresponding to external DDR address Pin A[13:0] */
#define DDRC_LMR_DDR_ADDR_MASK  (0x3fff << DDRC_LMR_DDR_ADDR_BIT)

#define DDRC_LMR_BA_BIT     8 /* When performing a DDR command, BA[2:0]
                     corresponding to external DDR address Pin BA[2:0]. */
#define DDRC_LMR_BA_MASK    (0x7 << DDRC_LMR_BA_BIT)
/* For DDR2 */
#define DDRC_LMR_BA_MRS (0 << DDRC_LMR_BA_BIT) /* Mode Register set */
#define DDRC_LMR_BA_EMRS1   (1 << DDRC_LMR_BA_BIT) /* Extended Mode Register1 set */
#define DDRC_LMR_BA_EMRS2   (2 << DDRC_LMR_BA_BIT) /* Extended Mode Register2 set */
#define DDRC_LMR_BA_EMRS3   (3 << DDRC_LMR_BA_BIT) /* Extended Mode Register3 set */
/* For mobile DDR */
#define DDRC_LMR_BA_M_MRS   (0 << DDRC_LMR_BA_BIT) /* Mode Register set */
#define DDRC_LMR_BA_M_EMRS  (2 << DDRC_LMR_BA_BIT) /* Extended Mode Register set */
#define DDRC_LMR_BA_M_SR    (1 << DDRC_LMR_BA_BIT) /* Status Register set */
/* For Normal DDR1 */
#define DDRC_LMR_BA_N_MRS   (0 << DDRC_LMR_BA_BIT) /* Mode Register set */
#define DDRC_LMR_BA_N_EMRS  (1 << DDRC_LMR_BA_BIT) /* Extended Mode Register set */

#define DDRC_LMR_CMD_BIT    4
#define DDRC_LMR_CMD_MASK   (0x3 << DDRC_LMR_CMD_BIT)
#define DDRC_LMR_CMD_PREC   (0 << DDRC_LMR_CMD_BIT)/* Precharge one bank/All banks */
#define DDRC_LMR_CMD_AUREF  (1 << DDRC_LMR_CMD_BIT)/* Auto-Refresh */
#define DDRC_LMR_CMD_LMR    (2 << DDRC_LMR_CMD_BIT)/* Load Mode Register */

#define DDRC_LMR_START      (1 << 0) /* 0 No command is performed
                            1 On the posedge of START, perform a command
                            defined by CMD field */

/* DDRC Timing Config Register 1 */
#define DDRC_TIMING1_tRTP_BIT   24 /* READ to PRECHARGE command period. */
#define DDRC_TIMING1_tRTP_MASK  (0x3f << DDRC_TIMING1_tRTP_BIT)
#define DDRC_TIMING1_tWTR_BIT   16 /* WRITE to READ command delay. */
#define DDRC_TIMING1_tWTR_MASK  (0x3f << DDRC_TIMING1_tWTR_BIT)
#define DDRC_TIMING1_tWTR_1     (0 << DDRC_TIMING1_tWTR_BIT)
#define DDRC_TIMING1_tWTR_2     (1 << DDRC_TIMING1_tWTR_BIT)
#define DDRC_TIMING1_tWTR_3     (2 << DDRC_TIMING1_tWTR_BIT)
#define DDRC_TIMING1_tWTR_4     (3 << DDRC_TIMING1_tWTR_BIT)
#define DDRC_TIMING1_tWR_BIT    8 /* WRITE Recovery Time defined by register MR of DDR2 DDR3 memory */
#define DDRC_TIMING1_tWR_MASK   (0x3f << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWR_1      (0 << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWR_2      (1 << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWR_3      (2 << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWR_4      (3 << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWR_5      (4 << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWR_6      (5 << DDRC_TIMING1_tWR_BIT)
#define DDRC_TIMING1_tWL_BIT    0 /* Write latency = RL - 1 */
#define DDRC_TIMING1_tWL_MASK   (0x3f << DDRC_TIMING1_tWL_BIT)

/* DDRC Timing Config Register 2 */
#define DDRC_TIMING2_tCCD_BIT   24 /* CAS# to CAS# command delay */
#define DDRC_TIMING2_tCCD_MASK  (0x3f << DDRC_TIMING2_tCCD_BIT)
#define DDRC_TIMING2_tRAS_BIT   16 /* ACTIVE to PRECHARGE command period (2 * tRAS + 1) */
#define DDRC_TIMING2_tRAS_MASK  (0x3f << DDRC_TIMING2_tRAS_BIT)
#define DDRC_TIMING2_tRCD_BIT   8  /* ACTIVE to READ or WRITE command period. */
#define DDRC_TIMING2_tRCD_MASK  (0x3f << DDRC_TIMING2_tRCD_BIT)
#define DDRC_TIMING2_tRL_BIT    0  /* Read latency = AL + CL*/
#define DDRC_TIMING2_tRL_MASK   (0x3f << DDRC_TIMING2_tRL_BIT)

/* DDRC Timing Config Register 3 */
#define DDRC_TIMING3_ONUM_BIT  27
#define DDRC_TIMING3_tCKSRE_BIT     24 /* Valid clock after enter self-refresh. */
#define DDRC_TIMING3_tCKSRE_MASK    (0x3f << DDRC_TIMING3_tCKSRE_BIT)
#define DDRC_TIMING3_tRP_BIT    16 /* PRECHARGE command period. */
#define DDRC_TIMING3_tRP_MASK   (0x3f << DDRC_TIMING3_tRP_BIT)
#define DDRC_TIMING3_tRRD_BIT   8 /* ACTIVE bank A to ACTIVE bank B command period. */
#define DDRC_TIMING3_tRRD_MASK  (0x3f << DDRC_TIMING3_tRRD_BIT)
#define DDRC_TIMING3_tRRD_DISABLE   (0 << DDRC_TIMING3_tRRD_BIT)
#define DDRC_TIMING3_tRRD_2     (1 << DDRC_TIMING3_tRRD_BIT)
#define DDRC_TIMING3_tRRD_3     (2 << DDRC_TIMING3_tRRD_BIT)
#define DDRC_TIMING3_tRRD_4     (3 << DDRC_TIMING3_tRRD_BIT)
#define DDRC_TIMING3_tRC_BIT    0 /* ACTIVE to ACTIVE command period. */
#define DDRC_TIMING3_tRC_MASK   (0x3f << DDRC_TIMING3_tRC_BIT)

/* DDRC Timing Config Register 4 */
#define DDRC_TIMING4_tRFC_BIT         24 /* AUTO-REFRESH command period. */
#define DDRC_TIMING4_tRFC_MASK        (0x3f << DDRC_TIMING4_tRFC_BIT)
#define DDRC_TIMING4_tEXTRW_BIT       21 /* ??? */
#define DDRC_TIMING4_tEXTRW_MASK      (0x7 << DDRC_TIMING4_tEXTRW_BIT)
#define DDRC_TIMING4_tRWCOV_BIT       19 /* ??? */
#define DDRC_TIMING4_tRWCOV_MASK      (0x3 << DDRC_TIMING4_tRWCOV_BIT)
#define DDRC_TIMING4_tCKE_BIT         16 /* ??? */
#define DDRC_TIMING4_tCKE_MASK        (0x7 << DDRC_TIMING4_tCKE_BIT)
#define DDRC_TIMING4_tMINSR_BIT       8  /* Minimum Self-Refresh / Deep-Power-Down time */
#define DDRC_TIMING4_tMINSR_MASK      (0xf << DDRC_TIMING4_tMINSR_BIT)
#define DDRC_TIMING4_tXP_BIT          4  /* EXIT-POWER-DOWN to next valid command period. */
#define DDRC_TIMING4_tXP_MASK         (0x7 << DDRC_TIMING4_tXP_BIT)
#define DDRC_TIMING4_tMRD_BIT         0  /* Load-Mode-Register to next valid command period. */
#define DDRC_TIMING4_tMRD_MASK        (0x3 << DDRC_TIMING4_tMRD_BIT)

/* DDRC Timing Config Register 5 */
#define DDRC_TIMING5_tCTLUPD_BIT    24 /* ??? */
#define DDRC_TIMING4_tCTLUPD_MASK   (0x3f << DDRC_TIMING5_tCTLUDP_BIT)
#define DDRC_TIMING5_tRTW_BIT       16 /* ??? */
#define DDRC_TIMING5_tRTW_MASK      (0x3f << DDRC_TIMING5_tRTW_BIT)
#define DDRC_TIMING5_tRDLAT_BIT     8 /* RL - 2 */
#define DDRC_TIMING5_tRDLAT_MASK    (0x3f << DDRC_TIMING5_tRDLAT_BIT)
#define DDRC_TIMING5_tWDLAT_BIT     0 /* WL - 1 */
#define DDRC_TIMING5_tWDLAT_MASK    (0x3f << DDRC_TIMING5_tWDLAT_BIT)

/* DDRC Timing Config Register 6 */
#define DDRC_TIMING6_tXSRD_BIT      24 /* exit power-down to READ delay */
#define DDRC_TIMING6_tXSRD_MASK     (0x3f << DDRC_TIMING6_tXSRD_BIT)
#define DDRC_TIMING6_tFAW_BIT       16 /* 4-active command window */
#define DDRC_TIMING6_tFAW_MASK      (0x3f << DDRC_TIMING6_tFAW_BIT)
#define DDRC_TIMING6_tCFGW_BIT      8 /* Write PHY configure registers to other commands delay */
#define DDRC_TIMING6_tCFGW_MASK     (0x3f << DDRC_TIMING6_tCFGW_BIT)
#define DDRC_TIMING6_tCFGR_BIT      0 /* Ready PHY configure registers to other commands delay */
#define DDRC_TIMING6_tCFGR_MASK     (0x3f << DDRC_TIMING6_tCFGR_BIT)

/* DDRC  Auto-Refresh Counter */
#define DDRC_REFCNT_CON_BIT           16 /* Constant value used to compare with CNT value. */
#define DDRC_REFCNT_CON_MASK          (0xff << DDRC_REFCNT_CON_BIT)
#define DDRC_REFCNT_CNT_BIT           8  /* 8-bit counter */
#define DDRC_REFCNT_CNT_MASK          (0xff << DDRC_REFCNT_CNT_BIT)
#define DDRC_REFCNT_CLK_DIV_BIT        1  /* Clock Divider for auto-refresh counter. */
#define DDRC_REFCNT_CLK_DIV_MASK       (0x7 << DDRC_REFCNT_CLKDIV_BIT)
#define DDRC_REFCNT_REF_EN            (1 << 0) /* Enable Refresh Counter */

/* DDRC DQS Delay Control Register */
#define DDRC_DQS_ERROR                (1 << 29) /* ahb_clk Delay Detect ERROR, read-only. */
#define DDRC_DQS_READY                (1 << 28) /* ahb_clk Delay Detect READY, read-only. */
#define DDRC_DQS_AUTO                 (1 << 23) /* Hardware auto-detect & set delay line */
#define DDRC_DQS_DET                  (1 << 24) /* Start delay detecting. */
#define DDRC_DQS_SRDET                (1 << 25) /* Hardware auto-redetect & set delay line. */
#define DDRC_DQS_CLKD_BIT             16 /* CLKD is reference value for setting WDQS and RDQS.*/
#define DDRC_DQS_CLKD_MASK            (0x3f << DDRC_DQS_CLKD_BIT)
#define DDRC_DQS_WDQS_BIT             8  /* Set delay element number to write DQS delay-line. */
#define DDRC_DQS_WDQS_MASK            (0x3f << DDRC_DQS_WDQS_BIT)
#define DDRC_DQS_RDQS_BIT             0  /* Set delay element number to read DQS delay-line. */
#define DDRC_DQS_RDQS_MASK            (0x3f << DDRC_DQS_RDQS_BIT)

/* DDRC DQS Delay Adjust Register */
#define DDRC_DQS_ADJWDQS_BIT          8 /* The adjust value for WRITE DQS delay */
#define DDRC_DQS_ADJWDQS_MASK         (0x1f << DDRC_DQS_ADJWDQS_BIT)
#define DDRC_DQS_ADJRDQS_BIT          0 /* The adjust value for READ DQS delay */
#define DDRC_DQS_ADJRDQS_MASK         (0x1f << DDRC_DQS_ADJRDQS_BIT)

/* DDRC Memory Map Config Register */
#define DDRC_MMAP_BASE_BIT            8 /* base address */
#define DDRC_MMAP_BASE_MASK           (0xff << DDRC_MMAP_BASE_BIT)
#define DDRC_MMAP_MASK_BIT            0 /* address mask */
#define DDRC_MMAP_MASK_MASK           (0xff << DDRC_MMAP_MASK_BIT)

#define DDRC_MMAP0_BASE          (0x20 << DDRC_MMAP_BASE_BIT)
#define DDRC_MMAP1_BASE_64M (0x24 << DDRC_MMAP_BASE_BIT) /*when bank0 is 128M*/
#define DDRC_MMAP1_BASE_128M    (0x28 << DDRC_MMAP_BASE_BIT) /*when bank0 is 128M*/
#define DDRC_MMAP1_BASE_256M    (0x30 << DDRC_MMAP_BASE_BIT) /*when bank0 is 128M*/

#define DDRC_MMAP_MASK_64_64    (0xfc << DDRC_MMAP_MASK_BIT)  /*mask for two 128M SDRAM*/
#define DDRC_MMAP_MASK_128_128  (0xf8 << DDRC_MMAP_MASK_BIT)  /*mask for two 128M SDRAM*/
#define DDRC_MMAP_MASK_256_256  (0xf0 << DDRC_MMAP_MASK_BIT)  /*mask for two 128M SDRAM*/

/* DDRP PHY Initialization Register */
#define DDRP_PIR_INIT       (1 << 0)
#define DDRP_PIR_DLLSRST    (1 << 1)
#define DDRP_PIR_DLLLOCK    (1 << 2)
#define DDRP_PIR_ZCAL       (1 << 3)
#define DDRP_PIR_ITMSRST    (1 << 4)
#define DDRP_PIR_DRAMRST    (1 << 5)
#define DDRP_PIR_DRAMINT    (1 << 6)
#define DDRP_PIR_QSTRN      (1 << 7)
#define DDRP_PIR_RVTRN      (1 << 8)
#define DDRP_PIR_DLLBYP     (1 << 17)
/* DDRP PHY General Configurate Register */
#define DDRP_PGCR_ITMDMD    (1 << 0)
#define DDRP_PGCR_DQSCFG    (1 << 1)
#define DDRP_PGCR_DFTCMP    (1 << 2)
#define DDRP_PGCR_DFTLMT_BIT    3
#define DDRP_PGCR_DTOSEL_BIT    5
#define DDRP_PGCR_CKEN_BIT  9
#define DDRP_PGCR_CKDV_BIT  12
#define DDRP_PGCR_CKINV     (1 << 14)
#define DDRP_PGCR_RANKEN_BIT    18
#define DDRP_PGCR_ZCKSEL_32 (2 << 22)
#define DDRP_PGCR_PDDISDX   (1 << 24)
/* DDRP PHY General Status Register */
#define DDRP_PGSR_IDONE     (1 << 0)
#define DDRP_PGSR_DLDONE    (1 << 1)
#define DDRP_PGSR_ZCDONE    (1 << 2)
#define DDRP_PGSR_DIDONE    (1 << 3)
#define DDRP_PGSR_DTDONE    (1 << 4)
#define DDRP_PGSR_DTERR     (1 << 5)
#define DDRP_PGSR_DTIERR    (1 << 6)
#define DDRP_PGSR_DFTEERR   (1 << 7)
/* DDRP DRAM Configuration Register */
#define DDRP_DCR_TYPE_BIT   0
#define DDRP_DCR_TYPE_MASK  (0x7 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_MDDR  (0 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_DDR   (1 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_DDR2  (2 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_DDR3  (3 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_LPDDR2    (4 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_DDR8BNK_BIT    3
#define DDRP_DCR_DDR8BNK_MASK   (1 << DDRP_DCR_DDR8BNK_BIT)
#define DDRP_DCR_DDR8BNK    (1 << DDRP_DCR_DDR8BNK_BIT)
#define DDRP_DCR_DDR8BNK_DIS    (0 << DDRP_DCR_DDR8BNK_BIT)
/* DDRP PHY Timing Register 0 */
/* DDRP PHY Timing Register 1 */
/* DDRP PHY Timing Register 2 */
#define DDRP_PTR0_tDLLSRST      50      // 50ns
#define DDRP_PTR0_tDLLSRST_BIT  0
#define DDRP_PTR0_tDLLLOCK  5120        // 5.12us
#define DDRP_PTR0_tDLLLOCK_BIT  6
#define DDRP_PTR0_tITMSRST_8    8       // 8tck
#define DDRP_PTR0_tITMSRST_BIT  18
#define DDRP_PTR1_tDINIT1_BIT   19
#define DDRP_PTR1_tDINIT0_DDR3  500 * 1000  // 500us
#define DDRP_PTR1_tDINIT0_BIT   0
#define DDRP_PTR2_tDINIT2_BIT   0
#define DDRP_PTR2_tDINIT2_DDR3  200 * 1000  // 200us
#define DDRP_PTR2_tDINIT3_BIT   17
#define DDRP_PTR2_tDINIT3_DDR3  512         // 512 tck

/* DDRP DRAM Timing Parameters Register 0 */
#define DDRP_DTPR0_tCCD_BIT 31
#define DDRP_DTPR0_tRC_BIT  25
#define DDRP_DTPR0_tRRD_BIT 21
#define DDRP_DTPR0_tRAS_BIT 16
#define DDRP_DTPR0_tRCD_BIT 12
#define DDRP_DTPR0_tRP_BIT  8
#define DDRP_DTPR0_tWTR_BIT 5
#define DDRP_DTPR0_tRTP_BIT 2
#define DDRP_DTPR0_tMRD_BIT 0
/* DDRP DRAM Timing Parameters Register 1 */
#define DRP_DTRP1_RTODT  (1 << 11)    /* ODT may not be turned on until one clock after the read post-amble */
#define DDRP_DTPR1_tRFC_BIT 16
#define DDRP_DTPR1_tRTODT_BIT   11
#define DDRP_DTPR1_tMOD_BIT 9
#define DDRP_DTPR1_tFAW_BIT 3
/* DDRP DRAM Timing Parameters Register 2 */
#define DDRP_DTPR2_tDLLK_BIT    19
#define DDRP_DTPR2_tCKE_BIT 15
#define DDRP_DTPR2_tXP_BIT  10
#define DDRP_DTPR2_tXS_BIT  0
/* DDRP DATX8 n Gen#define eral Configuration Register */
#define DDRP_DXGCR_DXEN  (1 << 0)    /* Data Byte Enable */

#define DDRP_ZQXCR_ZDEN_BIT     28
#define DDRP_ZQXCR_ZDEN         (1 << DDRP_ZQXCR_ZDEN_BIT)
#define DDRP_ZQXCR_PULLUP_IMPED_BIT 5
#define DDRP_ZQXCR_PULLDOWN_IMPED_BIT   0

/* DDR Mode Register Set*/
#define DDR1_MRS_OM_BIT     7        /* Operating Mode */
#define DDR1_MRS_OM_MASK    (0x3f << DDR1_MRS_OM_BIT)
#define DDR1_MRS_OM_NORMAL  (0 << DDR1_MRS_OM_BIT)
#define DDR1_MRS_OM_TEST    (1 << DDR1_MRS_OM_BIT)
#define DDR1_MRS_OM_DLLRST  (2 << DDR1_MRS_OM_BIT)
#define DDR_MRS_CAS_BIT     4        /* CAS Latency */
#define DDR_MRS_CAS_MASK    (7 << DDR_MRS_CAS_BIT)
#define DDR_MRS_BT_BIT      3        /* Burst Type */
#define DDR_MRS_BT_MASK     (1 << DDR_MRS_BT_BIT)
#define DDR_MRS_BT_SEQ  (0 << DDR_MRS_BT_BIT) /* Sequential */
#define DDR_MRS_BT_INT  (1 << DDR_MRS_BT_BIT) /* Interleave */
#define DDR_MRS_BL_BIT      0        /* Burst Length */
#define DDR_MRS_BL_MASK     (7 << DDR_MRS_BL_BIT)
#define DDR_MRS_BL_4        (2 << DDR_MRS_BL_BIT)
#define DDR_MRS_BL_8        (3 << DDR_MRS_BL_BIT)
/* DDR1 Extended Mode Register */
#define DDR1_EMRS_OM_BIT    2   /* Partial Array Self Refresh */
#define DDR1_EMRS_OM_MASK   (0x3ff << DDR1_EMRS_OM_BIT)
#define DDR1_EMRS_OM_NORMAL (0 << DDR1_EMRS_OM_BIT) /*All Banks*/

#define DDR1_EMRS_DS_BIT    1   /* Driver strength */
#define DDR1_EMRS_DS_MASK   (1 << DDR1_EMRS_DS_BIT)
#define DDR1_EMRS_DS_FULL   (0 << DDR1_EMRS_DS_BIT) /*Full*/
#define DDR1_EMRS_DS_HALF   (1 << DDR1_EMRS_DS_BIT) /*1/2 Strength*/

#define DDR1_EMRS_DLL_BIT   0   /* Driver strength */
#define DDR1_EMRS_DLL_MASK  (1 << DDR1_EMRS_DLL_BIT)
#define DDR1_EMRS_DLL_EN    (0 << DDR1_EMRS_DLL_BIT)    /*Full*/
#define DDR1_EMRS_DLL_DIS   (1 << DDR1_EMRS_DLL_BIT)    /*1/2 Strength*/

/* MDDR Mode Register Set*/
/* Mobile SDRAM Extended Mode Register */
#define DDR_EMRS_DS_BIT     5   /* Driver strength */
#define DDR_EMRS_DS_MASK    (3 << DDR_EMRS_DS_BIT)
#define DDR_EMRS_DS_FULL    (0 << DDR_EMRS_DS_BIT)  /*Full*/
#define DDR_EMRS_DS_HALF    (1 << DDR_EMRS_DS_BIT)  /*1/2 Strength*/
#define DDR_EMRS_DS_QUTR    (2 << DDR_EMRS_DS_BIT)  /*1/4 Strength*/

#define DDR_EMRS_PRSR_BIT   0   /* Partial Array Self Refresh */
#define DDR_EMRS_PRSR_MASK  (7 << DDR_EMRS_PRSR_BIT)
#define DDR_EMRS_PRSR_ALL   (0 << DDR_EMRS_PRSR_BIT) /*All Banks*/
#define DDR_EMRS_PRSR_HALF_TL   (1 << DDR_EMRS_PRSR_BIT) /*Half of Total Bank*/
#define DDR_EMRS_PRSR_QUTR_TL   (2 << DDR_EMRS_PRSR_BIT) /*Quarter of Total Bank*/
#define DDR_EMRS_PRSR_HALF_B0   (5 << DDR_EMRS_PRSR_BIT) /*Half of Bank0*/
#define DDR_EMRS_PRSR_QUTR_B0   (6 << DDR_EMRS_PRSR_BIT) /*Quarter of Bank0*/

/* DDR2 Mode Register Set*/
#define DDR2_MRS_PD_BIT     10 /* Active power down exit time */
#define DDR2_MRS_PD_MASK    (1 << DDR_MRS_PD_BIT)
#define DDR2_MRS_PD_FAST_EXIT   (0 << 10)
#define DDR2_MRS_PD_SLOW_EXIT   (1 << 10)
#define DDR2_MRS_WR_BIT     9 /* Write Recovery for autoprecharge */
#define DDR2_MRS_WR_MASK    (7 << DDR_MRS_WR_BIT)
#define DDR2_MRS_DLL_RST    (1 << 8) /* DLL Reset */
#define DDR2_MRS_TM_BIT     7        /* Operating Mode */
#define DDR2_MRS_TM_MASK    (1 << DDR_MRS_TM_BIT)
#define DDR2_MRS_TM_NORMAL  (0 << DDR_MRS_TM_BIT)
#define DDR2_MRS_TM_TEST    (1 << DDR_MRS_TM_BIT)
/* DDR2 Extended Mode Register1 Set */
#define DDR_EMRS1_QOFF      (1<<12) /* 0 Output buffer enabled
                       1 Output buffer disabled */
#define DDR_EMRS1_RDQS_EN   (1<<11) /* 0 Disable
                       1 Enable */
#define DDR_EMRS1_DQS_DIS   (1<<10) /* 0 Enable
                       1 Disable */
#define DDR_EMRS1_OCD_BIT   7 /* Additive Latency 0 -> 6 */
#define DDR_EMRS1_OCD_MASK  (0x7 << DDR_EMRS1_OCD_BIT)
#define DDR_EMRS1_OCD_EXIT      (0 << DDR_EMRS1_OCD_BIT)
#define DDR_EMRS1_OCD_D0        (1 << DDR_EMRS1_OCD_BIT)
#define DDR_EMRS1_OCD_D1        (2 << DDR_EMRS1_OCD_BIT)
#define DDR_EMRS1_OCD_ADJ       (4 << DDR_EMRS1_OCD_BIT)
#define DDR_EMRS1_OCD_DFLT      (7 << DDR_EMRS1_OCD_BIT)
#define DDR_EMRS1_AL_BIT    3 /* Additive Latency 0 -> 6 */
#define DDR_EMRS1_AL_MASK   (7 << DDR_EMRS1_AL_BIT)
#define DDR_EMRS1_RTT_BIT   2 /* On Die Termination */
#define DDR_EMRS1_RTT_MASK  (0x11 << DDR_EMRS1_RTT_BIT) /* Bit 6 and  Bit 2 */
#define DDR_EMRS1_RTT_DIS   (0x00 << DDR_EMRS1_RTT_BIT)
#define DDR_EMRS1_RTT_75    (0x01 << DDR_EMRS1_RTT_BIT)
#define DDR_EMRS1_RTT_150   (0x10 << DDR_EMRS1_RTT_BIT)
#define DDR_EMRS1_RTT_50    (0x11 << DDR_EMRS1_RTT_BIT)
#define DDR_EMRS1_DIC_BIT   1        /* Output Driver Impedence Control */
#define DDR_EMRS1_DIC_MASK  (1 << DDR_EMRS1_DIC_BIT) /* 100% */
#define DDR_EMRS1_DIC_NORMAL    (0 << DDR_EMRS1_DIC_BIT) /* 60% */
#define DDR_EMRS1_DIC_HALF  (1 << DDR_EMRS1_DIC_BIT)
#define DDR_EMRS1_DLL_BIT   0        /* DLL Enable  */
#define DDR_EMRS1_DLL_MASK  (1 << DDR_EMRS1_DLL_BIT)
#define DDR_EMRS1_DLL_EN    (0 << DDR_EMRS1_DLL_BIT)
#define DDR_EMRS1_DLL_DIS   (1 << DDR_EMRS1_DLL_BIT)

/* LPDDR2 Mode2 Register Set*/
#define LPDDR2_MRS2_BL_BIT      0        /* Burst Length */
#define LPDDR2_MRS2_BL_MASK     (7 << DDR_MRS_BL_BIT)
#define LPDDR2_MRS2_BL_4        (2 << DDR_MRS_BL_BIT)
#define LPDDR2_MRS2_BL_8        (3 << DDR_MRS_BL_BIT)
#define LPDDR2_MRS2_BL_16       (4 << DDR_MRS_BL_BIT)

/* DDR3 Mode Register Set*/
#define DDR3_MR0_BL_BIT     0
#define DDR3_MR0_BL_MASK    (3 << DDR3_MR0_BL_BIT)
#define DDR3_MR0_BL_8       (0 << DDR3_MR0_BL_BIT)
#define DDR3_MR0_BL_fly (1 << DDR3_MR0_BL_BIT)
#define DDR3_MR0_BL_4       (2 << DDR3_MR0_BL_BIT)
#define DDR3_MR0_BT_BIT     3
#define DDR3_MR0_BT_MASK    (1 << DDR3_MR0_BT_BIT)
#define DDR3_MR0_BT_SEQ     (0 << DDR3_MR0_BT_BIT)
#define DDR3_MR0_BT_INTER   (1 << DDR3_MR0_BT_BIT)
#define DDR3_MR0_CL2_BIT    2
#define DDR3_MR0_CL4_BIT    4
#define DDR3_MR0_WR_BIT     9

#define DDR3_MR1_DLL_DISABLE    1
#define DDR3_MR1_DIC_6      (0 << 5 | 0 << 1)
#define DDR3_MR1_DIC_7      (0 << 5 | 1 << 1)
#define DDR3_MR1_RTT_DIS    (0 << 9 | 0 << 6 | 0 << 2)
#define DDR3_MR1_RTT_4      (0 << 9 | 0 << 6 | 1 << 2)
#define DDR3_MR1_RTT_2      (0 << 9 | 1 << 6 | 0 << 2)
#define DDR3_MR1_RTT_6      (0 << 9 | 1 << 6 | 1 << 2)
#define DDR3_MR1_RTT_12     (1 << 9 | 0 << 6 | 0 << 2)
#define DDR3_MR1_RTT_8      (1 << 9 | 0 << 6 | 1 << 2)

#define DDR3_MR2_CWL_BIT    3


#define DDRC_MDELAY_MAUTO_BIT (6)
#define DDRC_MDELAY_MAUTO  (1 << DDRC_MDELAY_MAUTO_BIT)

#define ddr_writel(value, reg)  writel((value), DDRC_BASE + reg)
#define ddr_readl(reg)      readl(DDRC_BASE + reg)

#define ddr_apb_writel(value, reg)  writel((value), DDRC_APB_BASE + reg)
#define ddr_apb_readl(reg)      readl(DDRC_APB_BASE + reg)

#ifdef MAX
#undef MAX
#endif
#define MAX(tck, time)                              \
({                                      \
    unsigned long value;                            \
    value = (tck * tck_g.ps > time) ? (tck * tck_g.ps) : time;      \
    value = (value % 1000 == 0) ? (value / 1000) : (value / 1000 + 1);  \
    value;                                  \
})

#define DDR_GET_VALUE(x, y)                 \
({                              \
    unsigned long value, temp;                      \
     temp = x * 1000;                   \
     value = (temp % y == 0) ? (temp / y) : (temp / y + 1); \
     value;                         \
 })

/*only for lpddr2 tRL and tWL*/
#define MATCH(clk,type)                             \
({                                      \
    unsigned long value,i;                          \
    if(type == 0){    \
        for(i = 0; i < ARRAY_SIZE(rl_LPDDR2);i++){      \
            if(rl_LPDDR2[i].memclk > clk){              \
                value = rl_LPDDR2[i].RL;              \
                break;                                  \
            }                           \
        }                                   \
    }else{  \
        for(i = 0; i < ARRAY_SIZE(wl_LPDDR2);i++){      \
            if(wl_LPDDR2[i].memclk > clk){              \
                value = wl_LPDDR2[i].WL;  \
                break;              \
            }               \
        }           \
    }           \
    value;              \
})


#endif  /* !__ASSEMBLY__ */


/*
 * The following macros are especially useful for __asm__
 * inline assembler.
 */
#ifndef __STR
#define __STR(x) #x
#endif
#ifndef STR
#define STR(x) __STR(x)
#endif

/*
 *  Configure language
 */
#ifdef __ASSEMBLY__
#define _ULCAST_
#else
#define _ULCAST_ (unsigned long)
#endif


#define     zero    $0      /* wired zero           */
#define     AT      $1      /* assembler temp  - uppercase because of ".set at" */
#define     v0      $2      /* return value         */
#define     v1      $3
#define     a0      $4      /* argument registers   */
#define     a1      $5
#define     a2      $6
#define     a3      $7
#define     t0      $8      /* caller saved         */
#define     t1      $9
#define     t2      $10
#define     t3      $11
#define     t4      $12
#define     t5      $13
#define     t6      $14
#define     t7      $15
#define     s0      $16     /* callee saved         */
#define     s1      $17
#define     s2      $18
#define     s3      $19
#define     s4      $20
#define     s5      $21
#define     s6      $22
#define     s7      $23
#define     t8      $24     /* caller saved         */
#define     t9      $25
#define     jp      $25     /* PIC jump register    */
#define     k0      $26     /* kernel scratch       */
#define     k1      $27
#define     gp      $28     /* global pointer       */
#define     sp      $29     /* stack pointer        */
#define     fp      $30     /* frame pointer        */
#define     s8      $30     /* same like fp!        */
#define     ra      $31     /* return address       */

/*
 * Cache Operations available on all MIPS processors with R4000-style caches
 */
#define     INDEX_INVALIDATE_I      0x00
#define     INDEX_WRITEBACK_INV_D   0x01
#define     INDEX_LOAD_TAG_I        0x04
#define     INDEX_LOAD_TAG_D        0x05
#define     INDEX_STORE_TAG_I       0x08
#define     INDEX_STORE_TAG_D       0x09
#define     HIT_INVALIDATE_I        0x10
#define     HIT_INVALIDATE_D        0x11
#define     HIT_WRITEBACK_INV_D     0x15

/*
 * R4000-specific cacheops
 */
#define     CREATE_DIRTY_EXCL_D     0x0d
#define     FILL                    0x14
#define     HIT_WRITEBACK_I         0x18
#define     HIT_WRITEBACK_D         0x19
#define     Index_Prefetch_I        0x1c

/*
 * R4000SC and R4400SC-specific cacheops
 */
#define     INDEX_INVALIDATE_SI     0x02
#define     INDEX_WRITEBACK_INV_SD  0x03
#define     INDEX_LOAD_TAG_SI       0x06
#define     INDEX_LOAD_TAG_SD       0x07
#define     INDEX_STORE_TAG_SI      0x0A
#define     INDEX_STORE_TAG_SD      0x0B
#define     CREATE_DIRTY_EXCL_SD    0x0f
#define     HIT_INVALIDATE_SI       0x12
#define     HIT_INVALIDATE_SD       0x13
#define     HIT_WRITEBACK_INV_SD    0x17
#define     HIT_WRITEBACK_SD        0x1b
#define     HIT_SET_VIRTUAL_SI      0x1e
#define     HIT_SET_VIRTUAL_SD      0x1f

/*
 * R5000-specific cacheops
 */
#define     R5K_PAGE_INVALIDATE_S   0x17

/*
 * RM7000-specific cacheops
 */
#define     PAGE_INVALIDATE_T       0x16

/*
 * R10000-specific cacheops
 *
 * Cacheops 0x02, 0x06, 0x0a, 0x0c-0x0e, 0x16, 0x1a and 0x1e are unused.
 * Most of the _S cacheops are identical to the R4000SC _SD cacheops.
 */
#define     INDEX_WRITEBACK_INV_S   0x03
#define     INDEX_LOAD_TAG_S        0x07
#define     INDEX_STORE_TAG_S       0x0B
#define     HIT_INVALIDATE_S        0x13
#define     CACHE_BARRIER           0x14
#define     HIT_WRITEBACK_INV_S     0x17
#define     INDEX_LOAD_DATA_I       0x18
#define     INDEX_LOAD_DATA_D       0x19
#define     INDEX_LOAD_DATA_S       0x1b
#define     INDEX_STORE_DATA_I      0x1c
#define     INDEX_STORE_DATA_D      0x1d
#define     INDEX_STORE_DATA_S      0x1f

/*
 * Coprocessor 0 register names
 */
#define CP0_INDEX           $0
#define CP0_RANDOM          $1
#define CP0_ENTRYLO0        $2
#define CP0_ENTRYLO1        $3
#define CP0_CONF            $3
#define CP0_CONTEXT         $4
#define CP0_PAGEMASK        $5
#define CP0_WIRED           $6
#define CP0_INFO            $7
#define CP0_BADVADDR        $8
#define CP0_COUNT           $9
#define CP0_ENTRYHI         $10
#define CP0_COMPARE         $11
#define CP0_STATUS          $12
#define CP0_CAUSE           $13
#define CP0_EPC             $14
#define CP0_PRID            $15
#define CP0_EBASE           $15
#define CP0_CONFIG          $16
#define CP0_CONFIG1         $16
#define CP0_CONFIG7         $16
#define CP0_LLADDR          $17
#define CP0_WATCHLO         $18
#define CP0_WATCHHI         $19
#define CP0_XCONTEXT        $20
#define CP0_FRAMEMASK       $21
#define CP0_DIAGNOSTIC      $22
#define CP0_DEBUG           $23
#define CP0_DEPC            $24
#define CP0_PERFORMANCE     $25
#define CP0_ECC             $26
#define CP0_CACHEERR        $27
#define CP0_TAGLO           $28
#define CP0_TAGHI           $29
#define CP0_ERROREPC        $30
#define CP0_DESAVE          $31

#define PMON_CSR    $17, 7
#define PMON_HIGH   $17, 4
#define PMON_LC     $17, 5
#define PMON_RC     $17, 6

#define CORE_CTRL       $12, 2
#define CORE_STATUS     $12, 3
#define CORE_REIM       $12, 4
#define CORE_REIM_LOW   $12, 7

/*
 * R4640/R4650 cp0 register names.  These registers are listed
 * here only for completeness; without MMU these CPUs are not useable
 * by Linux.  A future ELKS port might take make Linux run on them
 * though ...
 */
#define CP0_IBASE $0
#define CP0_IBOUND $1
#define CP0_DBASE $2
#define CP0_DBOUND $3
#define CP0_CALG $17
#define CP0_IWATCH $18
#define CP0_DWATCH $19

/*
 * Coprocessor 0 Set 1 register names
 */
#define CP0_S1_DERRADDR0  $26
#define CP0_S1_DERRADDR1  $27
#define CP0_S1_INTCONTROL $20

/*
 * Coprocessor 0 Set 2 register names
 */
#define CP0_S2_SRSCTL   $12 /* MIPSR2 */

/*
 * Coprocessor 0 Set 3 register names
 */
#define CP0_S3_SRSMAP   $12 /* MIPSR2 */

/*
 *  TX39 Series
 */
#define CP0_TX39_CACHE  $7

/*
 * Coprocessor 1 (FPU) register names
 */
#define CP1_REVISION    $0
#define CP1_STATUS  $31

/*
 * FPU Status Register Values
 */
/*
 * Status Register Values
 */

#define FPU_CSR_FLUSH   0x01000000  /* flush denormalised results to 0 */
#define FPU_CSR_COND    0x00800000  /* $fcc0 */
#define FPU_CSR_COND0   0x00800000  /* $fcc0 */
#define FPU_CSR_COND1   0x02000000  /* $fcc1 */
#define FPU_CSR_COND2   0x04000000  /* $fcc2 */
#define FPU_CSR_COND3   0x08000000  /* $fcc3 */
#define FPU_CSR_COND4   0x10000000  /* $fcc4 */
#define FPU_CSR_COND5   0x20000000  /* $fcc5 */
#define FPU_CSR_COND6   0x40000000  /* $fcc6 */
#define FPU_CSR_COND7   0x80000000  /* $fcc7 */

/*
 * X the exception cause indicator
 * E the exception enable
 * S the sticky/flag bit
 */
#define FPU_CSR_ALL_X   0x0003f000
#define FPU_CSR_UNI_X   0x00020000
#define FPU_CSR_INV_X   0x00010000
#define FPU_CSR_DIV_X   0x00008000
#define FPU_CSR_OVF_X   0x00004000
#define FPU_CSR_UDF_X   0x00002000
#define FPU_CSR_INE_X   0x00001000

#define FPU_CSR_ALL_E   0x00000f80
#define FPU_CSR_INV_E   0x00000800
#define FPU_CSR_DIV_E   0x00000400
#define FPU_CSR_OVF_E   0x00000200
#define FPU_CSR_UDF_E   0x00000100
#define FPU_CSR_INE_E   0x00000080

#define FPU_CSR_ALL_S   0x0000007c
#define FPU_CSR_INV_S   0x00000040
#define FPU_CSR_DIV_S   0x00000020
#define FPU_CSR_OVF_S   0x00000010
#define FPU_CSR_UDF_S   0x00000008
#define FPU_CSR_INE_S   0x00000004

/* rounding mode */
#define FPU_CSR_RN  0x0 /* nearest */
#define FPU_CSR_RZ  0x1 /* towards zero */
#define FPU_CSR_RU  0x2 /* towards +Infinity */
#define FPU_CSR_RD  0x3 /* towards -Infinity */

/*
 * Values for PageMask register
 */
#ifdef CONFIG_CPU_VR41XX

/* Why doesn't stupidity hurt ... */

#define PM_1K       0x00000000
#define PM_4K       0x00001800
#define PM_16K      0x00007800
#define PM_64K      0x0001f800
#define PM_256K     0x0007f800

#else

#define PM_4K       0x00000000
#define PM_16K      0x00006000
#define PM_64K      0x0001e000
#define PM_256K     0x0007e000
#define PM_1M       0x001fe000
#define PM_4M       0x007fe000
#define PM_16M      0x01ffe000
#define PM_64M      0x07ffe000
#define PM_256M     0x1fffe000

#endif

/*
 * Values used for computation of new tlb entries
 */
#define PL_4K       12
#define PL_16K      14
#define PL_64K      16
#define PL_256K     18
#define PL_1M       20
#define PL_4M       22
#define PL_16M      24
#define PL_64M      26
#define PL_256M     28

/*
 * R4x00 interrupt enable / cause bits
 */
#define IE_SW0      (_ULCAST_(1) <<  8)
#define IE_SW1      (_ULCAST_(1) <<  9)
#define IE_IRQ0     (_ULCAST_(1) << 10)
#define IE_IRQ1     (_ULCAST_(1) << 11)
#define IE_IRQ2     (_ULCAST_(1) << 12)
#define IE_IRQ3     (_ULCAST_(1) << 13)
#define IE_IRQ4     (_ULCAST_(1) << 14)
#define IE_IRQ5     (_ULCAST_(1) << 15)

/*
 * R4x00 interrupt cause bits
 */
#define C_SW0       (_ULCAST_(1) <<  8)
#define C_SW1       (_ULCAST_(1) <<  9)
#define C_IRQ0      (_ULCAST_(1) << 10)
#define C_IRQ1      (_ULCAST_(1) << 11)
#define C_IRQ2      (_ULCAST_(1) << 12)
#define C_IRQ3      (_ULCAST_(1) << 13)
#define C_IRQ4      (_ULCAST_(1) << 14)
#define C_IRQ5      (_ULCAST_(1) << 15)

/*
 * Bitfields in the R4xx0 cp0 status register
 */
#define ST0_IE          0x00000001
#define ST0_EXL         0x00000002
#define ST0_ERL         0x00000004
#define ST0_KSU         0x00000018
#  define KSU_USER      0x00000010
#  define KSU_SUPERVISOR    0x00000008
#  define KSU_KERNEL        0x00000000
#define ST0_UX          0x00000020
#define ST0_SX          0x00000040
#define ST0_KX          0x00000080
#define ST0_DE          0x00010000
#define ST0_CE          0x00020000

/*
 * Setting c0_status.co enables Hit_Writeback and Hit_Writeback_Invalidate
 * cacheops in userspace.  This bit exists only on RM7000 and RM9000
 * processors.
 */
#define ST0_CO          0x08000000

/*
 * Bitfields in the R[23]000 cp0 status register.
 */
#define ST0_IEC         0x00000001
#define ST0_KUC         0x00000002
#define ST0_IEP         0x00000004
#define ST0_KUP         0x00000008
#define ST0_IEO         0x00000010
#define ST0_KUO         0x00000020
/* bits 6 & 7 are reserved on R[23]000 */
#define ST0_ISC         0x00010000
#define ST0_SWC         0x00020000
#define ST0_CM          0x00080000

/*
 * Bits specific to the R4640/R4650
 */
#define ST0_UM          (_ULCAST_(1) <<  4)
#define ST0_IL          (_ULCAST_(1) << 23)
#define ST0_DL          (_ULCAST_(1) << 24)

/*
 * Enable the MIPS MDMX and DSP ASEs
 */
#define ST0_MX          0x01000000

/*
 * Bitfields in the TX39 family CP0 Configuration Register 3
 */
#define TX39_CONF_ICS_SHIFT 19
#define TX39_CONF_ICS_MASK  0x00380000
#define TX39_CONF_ICS_1KB   0x00000000
#define TX39_CONF_ICS_2KB   0x00080000
#define TX39_CONF_ICS_4KB   0x00100000
#define TX39_CONF_ICS_8KB   0x00180000
#define TX39_CONF_ICS_16KB  0x00200000

#define TX39_CONF_DCS_SHIFT 16
#define TX39_CONF_DCS_MASK  0x00070000
#define TX39_CONF_DCS_1KB   0x00000000
#define TX39_CONF_DCS_2KB   0x00010000
#define TX39_CONF_DCS_4KB   0x00020000
#define TX39_CONF_DCS_8KB   0x00030000
#define TX39_CONF_DCS_16KB  0x00040000

#define TX39_CONF_CWFON     0x00004000
#define TX39_CONF_WBON      0x00002000
#define TX39_CONF_RF_SHIFT  10
#define TX39_CONF_RF_MASK   0x00000c00
#define TX39_CONF_DOZE      0x00000200
#define TX39_CONF_HALT      0x00000100
#define TX39_CONF_LOCK      0x00000080
#define TX39_CONF_ICE       0x00000020
#define TX39_CONF_DCE       0x00000010
#define TX39_CONF_IRSIZE_SHIFT  2
#define TX39_CONF_IRSIZE_MASK   0x0000000c
#define TX39_CONF_DRSIZE_SHIFT  0
#define TX39_CONF_DRSIZE_MASK   0x00000003

/*
 * Status register bits available in all MIPS CPUs.
 */
#define ST0_IM          0x0000ff00
#define  STATUSB_IP0        8
#define  STATUSF_IP0        (_ULCAST_(1) <<  8)
#define  STATUSB_IP1        9
#define  STATUSF_IP1        (_ULCAST_(1) <<  9)
#define  STATUSB_IP2        10
#define  STATUSF_IP2        (_ULCAST_(1) << 10)
#define  STATUSB_IP3        11
#define  STATUSF_IP3        (_ULCAST_(1) << 11)
#define  STATUSB_IP4        12
#define  STATUSF_IP4        (_ULCAST_(1) << 12)
#define  STATUSB_IP5        13
#define  STATUSF_IP5        (_ULCAST_(1) << 13)
#define  STATUSB_IP6        14
#define  STATUSF_IP6        (_ULCAST_(1) << 14)
#define  STATUSB_IP7        15
#define  STATUSF_IP7        (_ULCAST_(1) << 15)
#define  STATUSB_IP8        0
#define  STATUSF_IP8        (_ULCAST_(1) <<  0)
#define  STATUSB_IP9        1
#define  STATUSF_IP9        (_ULCAST_(1) <<  1)
#define  STATUSB_IP10       2
#define  STATUSF_IP10       (_ULCAST_(1) <<  2)
#define  STATUSB_IP11       3
#define  STATUSF_IP11       (_ULCAST_(1) <<  3)
#define  STATUSB_IP12       4
#define  STATUSF_IP12       (_ULCAST_(1) <<  4)
#define  STATUSB_IP13       5
#define  STATUSF_IP13       (_ULCAST_(1) <<  5)
#define  STATUSB_IP14       6
#define  STATUSF_IP14       (_ULCAST_(1) <<  6)
#define  STATUSB_IP15       7
#define  STATUSF_IP15       (_ULCAST_(1) <<  7)
#define ST0_CH          0x00040000
#define ST0_SR          0x00100000
#define ST0_TS          0x00200000
#define ST0_BEV         0x00400000
#define ST0_RE          0x02000000
#define ST0_FR          0x04000000
#define ST0_CU          0xf0000000
#define ST0_CU0         0x10000000
#define ST0_CU1         0x20000000
#define ST0_CU2         0x40000000
#define ST0_CU3         0x80000000
#define ST0_XX          0x80000000  /* MIPS IV naming */

/*
 * Bitfields and bit numbers in the coprocessor 0 cause register.
 *
 * Refer to your MIPS R4xx0 manual, chapter 5 for explanation.
 */
#define  CAUSEB_EXCCODE     2
#define  CAUSEF_EXCCODE     (_ULCAST_(31)  <<  2)
#define  CAUSEB_IP      8
#define  CAUSEF_IP      (_ULCAST_(255) <<  8)
#define  CAUSEB_IP0     8
#define  CAUSEF_IP0     (_ULCAST_(1)   <<  8)
#define  CAUSEB_IP1     9
#define  CAUSEF_IP1     (_ULCAST_(1)   <<  9)
#define  CAUSEB_IP2     10
#define  CAUSEF_IP2     (_ULCAST_(1)   << 10)
#define  CAUSEB_IP3     11
#define  CAUSEF_IP3     (_ULCAST_(1)   << 11)
#define  CAUSEB_IP4     12
#define  CAUSEF_IP4     (_ULCAST_(1)   << 12)
#define  CAUSEB_IP5     13
#define  CAUSEF_IP5     (_ULCAST_(1)   << 13)
#define  CAUSEB_IP6     14
#define  CAUSEF_IP6     (_ULCAST_(1)   << 14)
#define  CAUSEB_IP7     15
#define  CAUSEF_IP7     (_ULCAST_(1)   << 15)
#define  CAUSEB_IV      23
#define  CAUSEF_IV      (_ULCAST_(1)   << 23)
#define  CAUSEB_CE      28
#define  CAUSEF_CE      (_ULCAST_(3)   << 28)
#define  CAUSEB_BD      31
#define  CAUSEF_BD      (_ULCAST_(1)   << 31)

/*
 * Bits in the coprocessor 0 config register.
 */
/* Generic bits.  */
#define CONF_CM_CACHABLE_NO_WA      0
#define CONF_CM_CACHABLE_WA     1
#define CONF_CM_UNCACHED        2
#define CONF_CM_CACHABLE_NONCOHERENT    3
#define CONF_CM_CACHABLE_CE     4
#define CONF_CM_CACHABLE_COW        5
#define CONF_CM_CACHABLE_CUW        6
#define CONF_CM_CACHABLE_ACCELERATED    7
#define CONF_CM_CMASK           7
#define CONF_BE         (_ULCAST_(1) << 15)

/* Bits common to various processors.  */
#define CONF_CU         (_ULCAST_(1) <<  3)
#define CONF_DB         (_ULCAST_(1) <<  4)
#define CONF_IB         (_ULCAST_(1) <<  5)
#define CONF_DC         (_ULCAST_(7) <<  6)
#define CONF_IC         (_ULCAST_(7) <<  9)
#define CONF_EB         (_ULCAST_(1) << 13)
#define CONF_EM         (_ULCAST_(1) << 14)
#define CONF_SM         (_ULCAST_(1) << 16)
#define CONF_SC         (_ULCAST_(1) << 17)
#define CONF_EW         (_ULCAST_(3) << 18)
#define CONF_EP         (_ULCAST_(15)<< 24)
#define CONF_EC         (_ULCAST_(7) << 28)
#define CONF_CM         (_ULCAST_(1) << 31)

/* Bits specific to the R4xx0.  */
#define R4K_CONF_SW     (_ULCAST_(1) << 20)
#define R4K_CONF_SS     (_ULCAST_(1) << 21)
#define R4K_CONF_SB     (_ULCAST_(3) << 22)

/* Bits specific to the R5000.  */
#define R5K_CONF_SE     (_ULCAST_(1) << 12)
#define R5K_CONF_SS     (_ULCAST_(3) << 20)

/* Bits specific to the RM7000.  */
#define RM7K_CONF_SE        (_ULCAST_(1) <<  3)
#define RM7K_CONF_TE        (_ULCAST_(1) << 12)
#define RM7K_CONF_CLK       (_ULCAST_(1) << 16)
#define RM7K_CONF_TC        (_ULCAST_(1) << 17)
#define RM7K_CONF_SI        (_ULCAST_(3) << 20)
#define RM7K_CONF_SC        (_ULCAST_(1) << 31)

/* Bits specific to the R10000.  */
#define R10K_CONF_DN        (_ULCAST_(3) <<  3)
#define R10K_CONF_CT        (_ULCAST_(1) <<  5)
#define R10K_CONF_PE        (_ULCAST_(1) <<  6)
#define R10K_CONF_PM        (_ULCAST_(3) <<  7)
#define R10K_CONF_EC        (_ULCAST_(15)<<  9)
#define R10K_CONF_SB        (_ULCAST_(1) << 13)
#define R10K_CONF_SK        (_ULCAST_(1) << 14)
#define R10K_CONF_SS        (_ULCAST_(7) << 16)
#define R10K_CONF_SC        (_ULCAST_(7) << 19)
#define R10K_CONF_DC        (_ULCAST_(7) << 26)
#define R10K_CONF_IC        (_ULCAST_(7) << 29)

/* Bits specific to the VR41xx.  */
#define VR41_CONF_CS        (_ULCAST_(1) << 12)
#define VR41_CONF_P4K       (_ULCAST_(1) << 13)
#define VR41_CONF_BP        (_ULCAST_(1) << 16)
#define VR41_CONF_M16       (_ULCAST_(1) << 20)
#define VR41_CONF_AD        (_ULCAST_(1) << 23)

/* Bits specific to the R30xx.  */
#define R30XX_CONF_FDM      (_ULCAST_(1) << 19)
#define R30XX_CONF_REV      (_ULCAST_(1) << 22)
#define R30XX_CONF_AC       (_ULCAST_(1) << 23)
#define R30XX_CONF_RF       (_ULCAST_(1) << 24)
#define R30XX_CONF_HALT     (_ULCAST_(1) << 25)
#define R30XX_CONF_FPINT    (_ULCAST_(7) << 26)
#define R30XX_CONF_DBR      (_ULCAST_(1) << 29)
#define R30XX_CONF_SB       (_ULCAST_(1) << 30)
#define R30XX_CONF_LOCK     (_ULCAST_(1) << 31)

/* Bits specific to the TX49.  */
#define TX49_CONF_DC        (_ULCAST_(1) << 16)
#define TX49_CONF_IC        (_ULCAST_(1) << 17)  /* conflict with CONF_SC */
#define TX49_CONF_HALT      (_ULCAST_(1) << 18)
#define TX49_CONF_CWFON     (_ULCAST_(1) << 27)

/* Bits specific to the MIPS32/64 PRA.  */
#define MIPS_CONF_MT        (_ULCAST_(7) <<  7)
#define MIPS_CONF_AR        (_ULCAST_(7) << 10)
#define MIPS_CONF_AT        (_ULCAST_(3) << 13)
#define MIPS_CONF_M     (_ULCAST_(1) << 31)

/*
 * Bits in the MIPS32/64 PRA coprocessor 0 config registers 1 and above.
 */
#define MIPS_CONF1_FP       (_ULCAST_(1) <<  0)
#define MIPS_CONF1_EP       (_ULCAST_(1) <<  1)
#define MIPS_CONF1_CA       (_ULCAST_(1) <<  2)
#define MIPS_CONF1_WR       (_ULCAST_(1) <<  3)
#define MIPS_CONF1_PC       (_ULCAST_(1) <<  4)
#define MIPS_CONF1_MD       (_ULCAST_(1) <<  5)
#define MIPS_CONF1_C2       (_ULCAST_(1) <<  6)
#define MIPS_CONF1_DA       (_ULCAST_(7) <<  7)
#define MIPS_CONF1_DL       (_ULCAST_(7) << 10)
#define MIPS_CONF1_DS       (_ULCAST_(7) << 13)
#define MIPS_CONF1_IA       (_ULCAST_(7) << 16)
#define MIPS_CONF1_IL       (_ULCAST_(7) << 19)
#define MIPS_CONF1_IS       (_ULCAST_(7) << 22)
#define MIPS_CONF1_TLBS     (_ULCAST_(63)<< 25)

#define MIPS_CONF2_SA       (_ULCAST_(15)<<  0)
#define MIPS_CONF2_SL       (_ULCAST_(15)<<  4)
#define MIPS_CONF2_SS       (_ULCAST_(15)<<  8)
#define MIPS_CONF2_SU       (_ULCAST_(15)<< 12)
#define MIPS_CONF2_TA       (_ULCAST_(15)<< 16)
#define MIPS_CONF2_TL       (_ULCAST_(15)<< 20)
#define MIPS_CONF2_TS       (_ULCAST_(15)<< 24)
#define MIPS_CONF2_TU       (_ULCAST_(7) << 28)

#define MIPS_CONF3_TL       (_ULCAST_(1) <<  0)
#define MIPS_CONF3_SM       (_ULCAST_(1) <<  1)
#define MIPS_CONF3_MT       (_ULCAST_(1) <<  2)
#define MIPS_CONF3_SP       (_ULCAST_(1) <<  4)
#define MIPS_CONF3_VINT     (_ULCAST_(1) <<  5)
#define MIPS_CONF3_VEIC     (_ULCAST_(1) <<  6)
#define MIPS_CONF3_LPA      (_ULCAST_(1) <<  7)
#define MIPS_CONF3_DSP      (_ULCAST_(1) << 10)
#define MIPS_CONF3_ULRI     (_ULCAST_(1) << 13)

#define MIPS_CONF7_WII      (_ULCAST_(1) << 31)

#define MIPS_CONF7_RPS      (_ULCAST_(1) << 2)

/*
 * Bits in the MIPS32/64 coprocessor 1 (FPU) revision register.
 */
#define MIPS_FPIR_S     (_ULCAST_(1) << 16)
#define MIPS_FPIR_D     (_ULCAST_(1) << 17)
#define MIPS_FPIR_PS        (_ULCAST_(1) << 18)
#define MIPS_FPIR_3D        (_ULCAST_(1) << 19)
#define MIPS_FPIR_W     (_ULCAST_(1) << 20)
#define MIPS_FPIR_L     (_ULCAST_(1) << 21)
#define MIPS_FPIR_F64       (_ULCAST_(1) << 22)

#ifndef __ASSEMBLY__

#define cache_op(op, addr)      \
    __asm__ __volatile__(       \
        ".set   push\n"     \
        ".set   noreorder\n"    \
        ".set   mips3\n"    \
        "cache  %0, %1\n"   \
        ".set   pop\n"      \
        :           \
        : "i" (op), "R" (*(unsigned char *)(addr)))

#define CONFIG_SYS_DCACHE_SIZE      32768
#define CONFIG_SYS_ICACHE_SIZE      32768
#define CONFIG_SYS_CACHELINE_SIZE   32

/*
 * Memory segments (32bit kernel mode addresses)
 * These are the traditional names used in the 32-bit universe.
 */
#define KUSEG           0x00000000
#define KSEG0           0x80000000
#define KSEG1           0xa0000000
#define KSEG2           0xc0000000
#define KSEG3           0xe0000000

#define CKUSEG          0x00000000
#define CKSEG0          0x80000000
#define CKSEG1          0xa0000000
#define CKSEG2          0xc0000000
#define CKSEG3          0xe0000000

#define ASMMACRO(name, code...)                     \
__asm__(".macro " #name "; " #code "; .endm");              \
                                    \
static inline void name(void)                       \
{                                   \
    __asm__ __volatile__ (#name);                   \
}

ASMMACRO(_ssnop,
     sll    $0, $0, 1
    )

ASMMACRO(_ehb,
     sll    $0, $0, 3
    )


ASMMACRO(mtc0_tlbw_hazard,
    _ssnop; _ssnop; _ehb
    )
ASMMACRO(tlbw_use_hazard,
    _ssnop; _ssnop; _ssnop; _ehb
    )
ASMMACRO(tlb_probe_hazard,
     _ssnop; _ssnop; _ssnop; _ehb
    )
ASMMACRO(irq_enable_hazard,
     _ssnop; _ssnop; _ssnop; _ehb
    )
ASMMACRO(irq_disable_hazard,
    _ssnop; _ssnop; _ssnop; _ehb
    )
ASMMACRO(back_to_back_c0_hazard,
     _ssnop; _ssnop; _ssnop; _ehb
    )

/*
 * Functions to access the R10000 performance counters.  These are basically
 * mfc0 and mtc0 instructions from and to coprocessor register with a 5-bit
 * performance counter number encoded into bits 1 ... 5 of the instruction.
 * Only performance counters 0 to 1 actually exist, so for a non-R10000 aware
 * disassembler these will look like an access to sel 0 or 1.
 */
#define read_r10k_perf_cntr(counter)                \
({                              \
    unsigned int __res;                 \
    __asm__ __volatile__(                   \
    "mfpc\t%0, %1"                      \
    : "=r" (__res)                      \
    : "i" (counter));                   \
                                \
    __res;                          \
})

#define write_r10k_perf_cntr(counter,val)           \
do {                                \
    __asm__ __volatile__(                   \
    "mtpc\t%0, %1"                      \
    :                           \
    : "r" (val), "i" (counter));                \
} while (0)

#define read_r10k_perf_event(counter)               \
({                              \
    unsigned int __res;                 \
    __asm__ __volatile__(                   \
    "mfps\t%0, %1"                      \
    : "=r" (__res)                      \
    : "i" (counter));                   \
                                \
    __res;                          \
})

#define write_r10k_perf_cntl(counter,val)           \
do {                                \
    __asm__ __volatile__(                   \
    "mtps\t%0, %1"                      \
    :                           \
    : "r" (val), "i" (counter));                \
} while (0)

/*
 * Macros to access the system control coprocessor
 */

#define __read_32bit_c0_register(source, sel)               \
({ int __res;                               \
    if (sel == 0)                           \
        __asm__ __volatile__(                   \
            "mfc0\t%0, " #source "\n\t"         \
            : "=r" (__res));                \
    else                                \
        __asm__ __volatile__(                   \
            ".set\tmips32\n\t"              \
            "mfc0\t%0, " #source ", " #sel "\n\t"       \
            ".set\tmips0\n\t"               \
            : "=r" (__res));                \
    __res;                              \
})

#define __read_64bit_c0_register(source, sel)               \
({ unsigned long long __res;                        \
    if (sizeof(unsigned long) == 4)                 \
        __res = __read_64bit_c0_split(source, sel);     \
    else if (sel == 0)                      \
        __asm__ __volatile__(                   \
            ".set\tmips3\n\t"               \
            "dmfc0\t%0, " #source "\n\t"            \
            ".set\tmips0"                   \
            : "=r" (__res));                \
    else                                \
        __asm__ __volatile__(                   \
            ".set\tmips64\n\t"              \
            "dmfc0\t%0, " #source ", " #sel "\n\t"      \
            ".set\tmips0"                   \
            : "=r" (__res));                \
    __res;                              \
})

#define __write_32bit_c0_register(register, sel, value)         \
do {                                    \
    if (sel == 0)                           \
        __asm__ __volatile__(                   \
            "mtc0\t%z0, " #register "\n\t"          \
            : : "Jr" ((unsigned int)(value)));      \
    else                                \
        __asm__ __volatile__(                   \
            ".set\tmips32\n\t"              \
            "mtc0\t%z0, " #register ", " #sel "\n\t"    \
            ".set\tmips0"                   \
            : : "Jr" ((unsigned int)(value)));      \
} while (0)

#define __write_64bit_c0_register(register, sel, value)         \
do {                                    \
    if (sizeof(unsigned long) == 4)                 \
        __write_64bit_c0_split(register, sel, value);       \
    else if (sel == 0)                      \
        __asm__ __volatile__(                   \
            ".set\tmips3\n\t"               \
            "dmtc0\t%z0, " #register "\n\t"         \
            ".set\tmips0"                   \
            : : "Jr" (value));              \
    else                                \
        __asm__ __volatile__(                   \
            ".set\tmips64\n\t"              \
            "dmtc0\t%z0, " #register ", " #sel "\n\t"   \
            ".set\tmips0"                   \
            : : "Jr" (value));              \
} while (0)

#define __read_ulong_c0_register(reg, sel)              \
    ((sizeof(unsigned long) == 4) ?                 \
    (unsigned long) __read_32bit_c0_register(reg, sel) :        \
    (unsigned long) __read_64bit_c0_register(reg, sel))

#define __write_ulong_c0_register(reg, sel, val)            \
do {                                    \
    if (sizeof(unsigned long) == 4)                 \
        __write_32bit_c0_register(reg, sel, val);       \
    else                                \
        __write_64bit_c0_register(reg, sel, val);       \
} while (0)

/*
 * On RM7000/RM9000 these are uses to access cop0 set 1 registers
 */
#define __read_32bit_c0_ctrl_register(source)               \
({ int __res;                               \
    __asm__ __volatile__(                       \
        "cfc0\t%0, " #source "\n\t"             \
        : "=r" (__res));                    \
    __res;                              \
})

#define __write_32bit_c0_ctrl_register(register, value)         \
do {                                    \
    __asm__ __volatile__(                       \
        "ctc0\t%z0, " #register "\n\t"              \
        : : "Jr" ((unsigned int)(value)));          \
} while (0)

/*
 * These versions are only needed for systems with more than 38 bits of
 * physical address space running the 32-bit kernel.  That's none atm :-)
 */
#define __read_64bit_c0_split(source, sel)              \
({                                  \
    unsigned long long __val;                   \
    if (sel == 0)                           \
        __asm__ __volatile__(                   \
            ".set\tmips64\n\t"              \
            "dmfc0\t%M0, " #source "\n\t"           \
            "dsll\t%L0, %M0, 32\n\t"            \
            "dsrl\t%M0, %M0, 32\n\t"            \
            "dsrl\t%L0, %L0, 32\n\t"            \
            ".set\tmips0"                   \
            : "=r" (__val));                \
    else                                \
        __asm__ __volatile__(                   \
            ".set\tmips64\n\t"              \
            "dmfc0\t%M0, " #source ", " #sel "\n\t"     \
            "dsll\t%L0, %M0, 32\n\t"            \
            "dsrl\t%M0, %M0, 32\n\t"            \
            "dsrl\t%L0, %L0, 32\n\t"            \
            ".set\tmips0"                   \
            : "=r" (__val));                \
                                    \
    __val;                              \
})

#define __write_64bit_c0_split(source, sel, val)            \
do {                                    \
    if (sel == 0)                           \
        __asm__ __volatile__(                   \
            ".set\tmips64\n\t"              \
            "dsll\t%L0, %L0, 32\n\t"            \
            "dsrl\t%L0, %L0, 32\n\t"            \
            "dsll\t%M0, %M0, 32\n\t"            \
            "or\t%L0, %L0, %M0\n\t"             \
            "dmtc0\t%L0, " #source "\n\t"           \
            ".set\tmips0"                   \
            : : "r" (val));                 \
    else                                \
        __asm__ __volatile__(                   \
            ".set\tmips64\n\t"              \
            "dsll\t%L0, %L0, 32\n\t"            \
            "dsrl\t%L0, %L0, 32\n\t"            \
            "dsll\t%M0, %M0, 32\n\t"            \
            "or\t%L0, %L0, %M0\n\t"             \
            "dmtc0\t%L0, " #source ", " #sel "\n\t"     \
            ".set\tmips0"                   \
            : : "r" (val));                 \
} while (0)

#define read_c0_index()     __read_32bit_c0_register($0, 0)
#define write_c0_index(val) __write_32bit_c0_register($0, 0, val)

#define read_c0_entrylo0()  __read_ulong_c0_register($2, 0)
#define write_c0_entrylo0(val)  __write_ulong_c0_register($2, 0, val)

#define read_c0_entrylo1()  __read_ulong_c0_register($3, 0)
#define write_c0_entrylo1(val)  __write_ulong_c0_register($3, 0, val)

#define read_c0_conf()      __read_32bit_c0_register($3, 0)
#define write_c0_conf(val)  __write_32bit_c0_register($3, 0, val)

#define read_c0_context()   __read_ulong_c0_register($4, 0)
#define write_c0_context(val)   __write_ulong_c0_register($4, 0, val)

#define read_c0_userlocal() __read_ulong_c0_register($4, 2)
#define write_c0_userlocal(val) __write_ulong_c0_register($4, 2, val)

#define read_c0_pagemask()  __read_32bit_c0_register($5, 0)
#define write_c0_pagemask(val)  __write_32bit_c0_register($5, 0, val)

#define read_c0_wired()     __read_32bit_c0_register($6, 0)
#define write_c0_wired(val) __write_32bit_c0_register($6, 0, val)

#define read_c0_info()      __read_32bit_c0_register($7, 0)

#define read_c0_cache()     __read_32bit_c0_register($7, 0) /* TX39xx */
#define write_c0_cache(val) __write_32bit_c0_register($7, 0, val)

#define read_c0_badvaddr()  __read_ulong_c0_register($8, 0)
#define write_c0_badvaddr(val)  __write_ulong_c0_register($8, 0, val)

#define read_c0_count()     __read_32bit_c0_register($9, 0)
#define write_c0_count(val) __write_32bit_c0_register($9, 0, val)

#define read_c0_count2()    __read_32bit_c0_register($9, 6) /* pnx8550 */
#define write_c0_count2(val)    __write_32bit_c0_register($9, 6, val)

#define read_c0_count3()    __read_32bit_c0_register($9, 7) /* pnx8550 */
#define write_c0_count3(val)    __write_32bit_c0_register($9, 7, val)

#define read_c0_entryhi()   __read_ulong_c0_register($10, 0)
#define write_c0_entryhi(val)   __write_ulong_c0_register($10, 0, val)

#define read_c0_compare()   __read_32bit_c0_register($11, 0)
#define write_c0_compare(val)   __write_32bit_c0_register($11, 0, val)

#define read_c0_compare2()  __read_32bit_c0_register($11, 6) /* pnx8550 */
#define write_c0_compare2(val)  __write_32bit_c0_register($11, 6, val)

#define read_c0_compare3()  __read_32bit_c0_register($11, 7) /* pnx8550 */
#define write_c0_compare3(val)  __write_32bit_c0_register($11, 7, val)

#define read_c0_status()    __read_32bit_c0_register($12, 0)
#ifdef CONFIG_MIPS_MT_SMTC
#define write_c0_status(val)                        \
do {                                    \
    __write_32bit_c0_register($12, 0, val);             \
    __ehb();                            \
} while (0)
#else
/*
 * Legacy non-SMTC code, which may be hazardous
 * but which might not support EHB
 */
#define write_c0_status(val)    __write_32bit_c0_register($12, 0, val)
#endif /* CONFIG_MIPS_MT_SMTC */

#define read_c0_cause()     __read_32bit_c0_register($13, 0)
#define write_c0_cause(val) __write_32bit_c0_register($13, 0, val)

#define read_c0_epc()       __read_ulong_c0_register($14, 0)
#define write_c0_epc(val)   __write_ulong_c0_register($14, 0, val)

#define read_c0_prid()      __read_32bit_c0_register($15, 0)

#define read_c0_config()    __read_32bit_c0_register($16, 0)
#define read_c0_config1()   __read_32bit_c0_register($16, 1)
#define read_c0_config2()   __read_32bit_c0_register($16, 2)
#define read_c0_config3()   __read_32bit_c0_register($16, 3)
#define read_c0_config4()   __read_32bit_c0_register($16, 4)
#define read_c0_config5()   __read_32bit_c0_register($16, 5)
#define read_c0_config6()   __read_32bit_c0_register($16, 6)
#define read_c0_config7()   __read_32bit_c0_register($16, 7)
#define write_c0_config(val)    __write_32bit_c0_register($16, 0, val)
#define write_c0_config1(val)   __write_32bit_c0_register($16, 1, val)
#define write_c0_config2(val)   __write_32bit_c0_register($16, 2, val)
#define write_c0_config3(val)   __write_32bit_c0_register($16, 3, val)
#define write_c0_config4(val)   __write_32bit_c0_register($16, 4, val)
#define write_c0_config5(val)   __write_32bit_c0_register($16, 5, val)
#define write_c0_config6(val)   __write_32bit_c0_register($16, 6, val)
#define write_c0_config7(val)   __write_32bit_c0_register($16, 7, val)

/*
 * The WatchLo register.  There may be upto 8 of them.
 */
#define read_c0_watchlo0()  __read_ulong_c0_register($18, 0)
#define read_c0_watchlo1()  __read_ulong_c0_register($18, 1)
#define read_c0_watchlo2()  __read_ulong_c0_register($18, 2)
#define read_c0_watchlo3()  __read_ulong_c0_register($18, 3)
#define read_c0_watchlo4()  __read_ulong_c0_register($18, 4)
#define read_c0_watchlo5()  __read_ulong_c0_register($18, 5)
#define read_c0_watchlo6()  __read_ulong_c0_register($18, 6)
#define read_c0_watchlo7()  __read_ulong_c0_register($18, 7)
#define write_c0_watchlo0(val)  __write_ulong_c0_register($18, 0, val)
#define write_c0_watchlo1(val)  __write_ulong_c0_register($18, 1, val)
#define write_c0_watchlo2(val)  __write_ulong_c0_register($18, 2, val)
#define write_c0_watchlo3(val)  __write_ulong_c0_register($18, 3, val)
#define write_c0_watchlo4(val)  __write_ulong_c0_register($18, 4, val)
#define write_c0_watchlo5(val)  __write_ulong_c0_register($18, 5, val)
#define write_c0_watchlo6(val)  __write_ulong_c0_register($18, 6, val)
#define write_c0_watchlo7(val)  __write_ulong_c0_register($18, 7, val)

/*
 * The WatchHi register.  There may be upto 8 of them.
 */
#define read_c0_watchhi0()  __read_32bit_c0_register($19, 0)
#define read_c0_watchhi1()  __read_32bit_c0_register($19, 1)
#define read_c0_watchhi2()  __read_32bit_c0_register($19, 2)
#define read_c0_watchhi3()  __read_32bit_c0_register($19, 3)
#define read_c0_watchhi4()  __read_32bit_c0_register($19, 4)
#define read_c0_watchhi5()  __read_32bit_c0_register($19, 5)
#define read_c0_watchhi6()  __read_32bit_c0_register($19, 6)
#define read_c0_watchhi7()  __read_32bit_c0_register($19, 7)

#define write_c0_watchhi0(val)  __write_32bit_c0_register($19, 0, val)
#define write_c0_watchhi1(val)  __write_32bit_c0_register($19, 1, val)
#define write_c0_watchhi2(val)  __write_32bit_c0_register($19, 2, val)
#define write_c0_watchhi3(val)  __write_32bit_c0_register($19, 3, val)
#define write_c0_watchhi4(val)  __write_32bit_c0_register($19, 4, val)
#define write_c0_watchhi5(val)  __write_32bit_c0_register($19, 5, val)
#define write_c0_watchhi6(val)  __write_32bit_c0_register($19, 6, val)
#define write_c0_watchhi7(val)  __write_32bit_c0_register($19, 7, val)

#define read_c0_xcontext()  __read_ulong_c0_register($20, 0)
#define write_c0_xcontext(val)  __write_ulong_c0_register($20, 0, val)

#define read_c0_intcontrol()    __read_32bit_c0_ctrl_register($20)
#define write_c0_intcontrol(val) __write_32bit_c0_ctrl_register($20, val)

#define read_c0_framemask() __read_32bit_c0_register($21, 0)
#define write_c0_framemask(val) __write_32bit_c0_register($21, 0, val)

/* RM9000 PerfControl performance counter control register */
#define read_c0_perfcontrol()   __read_32bit_c0_register($22, 0)
#define write_c0_perfcontrol(val) __write_32bit_c0_register($22, 0, val)

#define read_c0_diag()      __read_32bit_c0_register($22, 0)
#define write_c0_diag(val)  __write_32bit_c0_register($22, 0, val)

#define read_c0_diag1()     __read_32bit_c0_register($22, 1)
#define write_c0_diag1(val) __write_32bit_c0_register($22, 1, val)

#define read_c0_diag2()     __read_32bit_c0_register($22, 2)
#define write_c0_diag2(val) __write_32bit_c0_register($22, 2, val)

#define read_c0_diag3()     __read_32bit_c0_register($22, 3)
#define write_c0_diag3(val) __write_32bit_c0_register($22, 3, val)

#define read_c0_diag4()     __read_32bit_c0_register($22, 4)
#define write_c0_diag4(val) __write_32bit_c0_register($22, 4, val)

#define read_c0_diag5()     __read_32bit_c0_register($22, 5)
#define write_c0_diag5(val) __write_32bit_c0_register($22, 5, val)

#define read_c0_debug()     __read_32bit_c0_register($23, 0)
#define write_c0_debug(val) __write_32bit_c0_register($23, 0, val)

#define read_c0_depc()      __read_ulong_c0_register($24, 0)
#define write_c0_depc(val)  __write_ulong_c0_register($24, 0, val)

/*
 * MIPS32 / MIPS64 performance counters
 */
#define read_c0_perfctrl0() __read_32bit_c0_register($25, 0)
#define write_c0_perfctrl0(val) __write_32bit_c0_register($25, 0, val)
#define read_c0_perfcntr0() __read_32bit_c0_register($25, 1)
#define write_c0_perfcntr0(val) __write_32bit_c0_register($25, 1, val)
#define read_c0_perfctrl1() __read_32bit_c0_register($25, 2)
#define write_c0_perfctrl1(val) __write_32bit_c0_register($25, 2, val)
#define read_c0_perfcntr1() __read_32bit_c0_register($25, 3)
#define write_c0_perfcntr1(val) __write_32bit_c0_register($25, 3, val)
#define read_c0_perfctrl2() __read_32bit_c0_register($25, 4)
#define write_c0_perfctrl2(val) __write_32bit_c0_register($25, 4, val)
#define read_c0_perfcntr2() __read_32bit_c0_register($25, 5)
#define write_c0_perfcntr2(val) __write_32bit_c0_register($25, 5, val)
#define read_c0_perfctrl3() __read_32bit_c0_register($25, 6)
#define write_c0_perfctrl3(val) __write_32bit_c0_register($25, 6, val)
#define read_c0_perfcntr3() __read_32bit_c0_register($25, 7)
#define write_c0_perfcntr3(val) __write_32bit_c0_register($25, 7, val)

/* RM9000 PerfCount performance counter register */
#define read_c0_perfcount() __read_64bit_c0_register($25, 0)
#define write_c0_perfcount(val) __write_64bit_c0_register($25, 0, val)

#define read_c0_ecc()       __read_32bit_c0_register($26, 0)
#define write_c0_ecc(val)   __write_32bit_c0_register($26, 0, val)

#define read_c0_derraddr0() __read_ulong_c0_register($26, 1)
#define write_c0_derraddr0(val) __write_ulong_c0_register($26, 1, val)

#define read_c0_cacheerr()  __read_32bit_c0_register($27, 0)

#define read_c0_derraddr1() __read_ulong_c0_register($27, 1)
#define write_c0_derraddr1(val) __write_ulong_c0_register($27, 1, val)

#define read_c0_taglo()     __read_32bit_c0_register($28, 0)
#define write_c0_taglo(val) __write_32bit_c0_register($28, 0, val)

#define read_c0_dtaglo()    __read_32bit_c0_register($28, 2)
#define write_c0_dtaglo(val)    __write_32bit_c0_register($28, 2, val)

#define read_c0_taghi()     __read_32bit_c0_register($29, 0)
#define write_c0_taghi(val) __write_32bit_c0_register($29, 0, val)

#define read_c0_errorepc()  __read_ulong_c0_register($30, 0)
#define write_c0_errorepc(val)  __write_ulong_c0_register($30, 0, val)

/* MIPSR2 */
#define read_c0_hwrena()    __read_32bit_c0_register($7, 0)
#define write_c0_hwrena(val)    __write_32bit_c0_register($7, 0, val)

#define read_c0_intctl()    __read_32bit_c0_register($12, 1)
#define write_c0_intctl(val)    __write_32bit_c0_register($12, 1, val)

#define read_c0_srsctl()    __read_32bit_c0_register($12, 2)
#define write_c0_srsctl(val)    __write_32bit_c0_register($12, 2, val)

#define read_c0_srsmap()    __read_32bit_c0_register($12, 3)
#define write_c0_srsmap(val)    __write_32bit_c0_register($12, 3, val)

#define read_c0_ebase()     __read_32bit_c0_register($15, 1)
#define write_c0_ebase(val) __write_32bit_c0_register($15, 1, val)

/*
 * Macros to access the floating point coprocessor control registers
 */
#define read_32bit_cp1_register(source)             \
({ int __res;                           \
    __asm__ __volatile__(                   \
    ".set\tpush\n\t"                    \
    ".set\treorder\n\t"                 \
    "cfc1\t%0,"STR(source)"\n\t"                \
    ".set\tpop"                     \
    : "=r" (__res));                    \
    __res;})

#define rddsp(mask)                         \
({                                  \
    unsigned int __res;                     \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push                \n"     \
    "   .set    noat                \n"     \
    "   # rddsp $1, %x1             \n"     \
    "   .word   0x7c000cb8 | (%x1 << 16)    \n"     \
    "   move    %0, $1              \n"     \
    "   .set    pop             \n"     \
    : "=r" (__res)                          \
    : "i" (mask));                          \
    __res;                              \
})

#define wrdsp(val, mask)                        \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # wrdsp $1, %x1                 \n" \
    "   .word   0x7c2004f8 | (%x1 << 11)        \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (val), "i" (mask));                   \
} while (0)

#define mfhi0()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mfhi  %0, $ac0        \n"         \
    "   .word   0x00000810      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mfhi1()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mfhi  %0, $ac1        \n"         \
    "   .word   0x00200810      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mfhi2()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mfhi  %0, $ac2        \n"         \
    "   .word   0x00400810      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mfhi3()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mfhi  %0, $ac3        \n"         \
    "   .word   0x00600810      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mflo0()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mflo  %0, $ac0        \n"         \
    "   .word   0x00000812      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mflo1()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mflo  %0, $ac1        \n"         \
    "   .word   0x00200812      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mflo2()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mflo  %0, $ac2        \n"         \
    "   .word   0x00400812      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mflo3()                             \
({                                  \
    unsigned long __treg;                       \
                                    \
    __asm__ __volatile__(                       \
    "   .set    push            \n"         \
    "   .set    noat            \n"         \
    "   # mflo  %0, $ac3        \n"         \
    "   .word   0x00600812      \n"         \
    "   move    %0, $1          \n"         \
    "   .set    pop         \n"         \
    : "=r" (__treg));                       \
    __treg;                             \
})

#define mthi0(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mthi  $1, $ac0                \n" \
    "   .word   0x00200011              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mthi1(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mthi  $1, $ac1                \n" \
    "   .word   0x00200811              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mthi2(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mthi  $1, $ac2                \n" \
    "   .word   0x00201011              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mthi3(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mthi  $1, $ac3                \n" \
    "   .word   0x00201811              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mtlo0(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mtlo  $1, $ac0                \n" \
    "   .word   0x00200013              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mtlo1(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mtlo  $1, $ac1                \n" \
    "   .word   0x00200813              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mtlo2(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mtlo  $1, $ac2                \n" \
    "   .word   0x00201013              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

#define mtlo3(x)                            \
do {                                    \
    __asm__ __volatile__(                       \
    "   .set    push                    \n" \
    "   .set    noat                    \n" \
    "   move    $1, %0                  \n" \
    "   # mtlo  $1, $ac3                \n" \
    "   .word   0x00201813              \n" \
    "   .set    pop                 \n" \
    :                               \
    : "r" (x));                         \
} while (0)

/*
 * TLB operations.
 *
 * It is responsibility of the caller to take care of any TLB hazards.
 */
static inline void tlb_probe(void)
{
    __asm__ __volatile__(
        ".set noreorder\n\t"
        "tlbp\n\t"
        ".set reorder");
}

static inline void tlb_read(void)
{
#if MIPS34K_MISSED_ITLB_WAR
    int res = 0;

    __asm__ __volatile__(
    "   .set    push                    \n"
    "   .set    noreorder               \n"
    "   .set    noat                    \n"
    "   .set    mips32r2                \n"
    "   .word   0x41610001      # dvpe $1   \n"
    "   move    %0, $1                  \n"
    "   ehb                     \n"
    "   .set    pop                 \n"
    : "=r" (res));

    instruction_hazard();
#endif

    __asm__ __volatile__(
        ".set noreorder\n\t"
        "tlbr\n\t"
        ".set reorder");

#if MIPS34K_MISSED_ITLB_WAR
    if ((res & _ULCAST_(1)))
        __asm__ __volatile__(
        "   .set    push                \n"
        "   .set    noreorder           \n"
        "   .set    noat                \n"
        "   .set    mips32r2            \n"
        "   .word   0x41600021  # evpe      \n"
        "   ehb                 \n"
        "   .set    pop             \n");
#endif
}

static inline void tlb_write_indexed(void)
{
    __asm__ __volatile__(
        ".set noreorder\n\t"
        "tlbwi\n\t"
        ".set reorder");
}

static inline void tlb_write_random(void)
{
    __asm__ __volatile__(
        ".set noreorder\n\t"
        "tlbwr\n\t"
        ".set reorder");
}

/*
 * Manipulate bits in a c0 register.
 */
#define __BUILD_SET_C0(name)                    \
static inline unsigned int                  \
set_c0_##name(unsigned int set)                 \
{                               \
    unsigned int res;                   \
                                \
    res = read_c0_##name();                 \
    res |= set;                     \
    write_c0_##name(res);                   \
                                \
    return res;                     \
}                               \
                                \
static inline unsigned int                  \
clear_c0_##name(unsigned int clear)             \
{                               \
    unsigned int res;                   \
                                \
    res = read_c0_##name();                 \
    res &= ~clear;                      \
    write_c0_##name(res);                   \
                                \
    return res;                     \
}                               \
                                \
static inline unsigned int                  \
change_c0_##name(unsigned int change, unsigned int new)     \
{                               \
    unsigned int res;                   \
                                \
    res = read_c0_##name();                 \
    res &= ~change;                     \
    res |= (new & change);                  \
    write_c0_##name(res);                   \
                                \
    return res;                     \
}

__BUILD_SET_C0(status)
__BUILD_SET_C0(cause)
__BUILD_SET_C0(config)
__BUILD_SET_C0(intcontrol)
__BUILD_SET_C0(intctl)
__BUILD_SET_C0(srsmap)


#define cache_prefetch_label(label, size)                  \
do{                                 \
    unsigned long addr,end;                     \
    /* Prefetch codes from label */                 \
    addr = (unsigned long)(&&label) & ~(32 - 1);            \
    end = (unsigned long)(&&label + (size)) & ~(32 - 1);      \
    end += 32;                          \
    for (; addr < end; addr += 32) {                \
        __asm__ volatile (                  \
                ".set mips32\n\t"           \
                " cache %0, 0(%1)\n\t"          \
                ".set mips32\n\t"           \
                :                   \
                : "I" (Index_Prefetch_I), "r"(addr));   \
    }                               \
}                                   \
while(0)

#define cache_prefetch_addr(address, size)                  \
do{                                 \
    unsigned long addr,end;                     \
    /* Prefetch codes from label */                 \
    addr = (unsigned long)(address) & ~(32 - 1);            \
    end = (unsigned long)(address + (size)) & ~(32 - 1);      \
    end += 32;                          \
    for (; addr < end; addr += 32) {                \
        __asm__ volatile (                  \
                ".set mips32\n\t"           \
                " cache %0, 0(%1)\n\t"          \
                ".set mips32\n\t"           \
                :                   \
                : "I" (Index_Prefetch_I), "r"(addr));   \
    }                               \
}                                   \
while(0)

ASMMACRO(enable_fpu_hazard,
     nop; nop; nop; nop
)

ASMMACRO(disable_fpu_hazard,
     _ehb
)

#define __enable_fpu()                          \
do {                                    \
    set_c0_status(ST0_CU1);                     \
    enable_fpu_hazard();                        \
} while (0)

#define __disable_fpu()                         \
do {                                    \
    clear_c0_status(ST0_CU1);                   \
    disable_fpu_hazard();                       \
} while (0)


#define disable_fpu()                           \
do {                                    \
    __disable_fpu();                    \
} while (0)

/*
 * read_barrier_depends - Flush all pending reads that subsequents reads
 * depend on.
 *
 * No data-dependent reads from memory-like regions are ever reordered
 * over this barrier.  All reads preceding this primitive are guaranteed
 * to access memory (but not necessarily other CPUs' caches) before any
 * reads following this primitive that depend on the data return by
 * any of the preceding reads.  This primitive is much lighter weight than
 * rmb() on most CPUs, and is never heavier weight than is
 * rmb().
 *
 * These ordering constraints are respected by both the local CPU
 * and the compiler.
 *
 * Ordering is not guaranteed by anything other than these primitives,
 * not even by data dependencies.  See the documentation for
 * memory_barrier() for examples and URLs to more information.
 *
 * For example, the following code would force ordering (the initial
 * value of "a" is zero, "b" is one, and "p" is "&a"):
 *
 * <programlisting>
 *  CPU 0               CPU 1
 *
 *  b = 2;
 *  memory_barrier();
 *  p = &b;             q = p;
 *                  read_barrier_depends();
 *                  d = *q;
 * </programlisting>
 *
 * because the read of "*q" depends on the read of "p" and these
 * two reads are separated by a read_barrier_depends().  However,
 * the following code, with the same initial values for "a" and "b":
 *
 * <programlisting>
 *  CPU 0               CPU 1
 *
 *  a = 2;
 *  memory_barrier();
 *  b = 3;              y = b;
 *                  read_barrier_depends();
 *                  x = a;
 * </programlisting>
 *
 * does not enforce ordering, since there is no data dependency between
 * the read of "a" and the read of "b".  Therefore, on some CPUs, such
 * as Alpha, "y" could be set to 3 and "x" to 0.  Use rmb()
 * in cases like this where there are no data dependencies.
 */

#define CONFIG_CPU_HAS_SYNC

#define read_barrier_depends()      do { } while(0)
#define smp_read_barrier_depends()  do { } while(0)

#ifdef CONFIG_CPU_HAS_SYNC
#define __sync()                \
    __asm__ __volatile__(           \
        ".set   push\n\t"       \
        ".set   noreorder\n\t"      \
        ".set   mips2\n\t"      \
        "sync\n\t"          \
        ".set   pop"            \
        : /* no output */       \
        : /* no input */        \
        : "memory")
#else
#define __sync()    do { } while(0)
#endif

#define __fast_iob()                \
    __asm__ __volatile__(           \
        ".set   push\n\t"       \
        ".set   noreorder\n\t"      \
        "lw $0,%0\n\t"      \
        "nop\n\t"           \
        ".set   pop"            \
        : /* no output */       \
        : "m" (*(int *)CKSEG1)      \
        : "memory")
#ifdef CONFIG_CPU_CAVIUM_OCTEON
# define OCTEON_SYNCW_STR   ".set push\n.set arch=octeon\nsyncw\nsyncw\n.set pop\n"
# define __syncw()  __asm__ __volatile__(OCTEON_SYNCW_STR : : : "memory")

# define fast_wmb() __syncw()
# define fast_rmb() barrier()
# define fast_mb()  __sync()
# define fast_iob() do { } while (0)
#else /* ! CONFIG_CPU_CAVIUM_OCTEON */
# define fast_wmb() __sync()
# define fast_rmb() __sync()
# define fast_mb()  __sync()
# ifdef CONFIG_SGI_IP28
#  define fast_iob()                \
    __asm__ __volatile__(           \
        ".set   push\n\t"       \
        ".set   noreorder\n\t"      \
        "lw $0,%0\n\t"      \
        "sync\n\t"          \
        "lw $0,%0\n\t"      \
        ".set   pop"            \
        : /* no output */       \
        : "m" (*(int *)CKSEG1ADDR(0x1fa00004)) \
        : "memory")
# else
#  define fast_iob()                \
    do {                    \
        __sync();           \
        __fast_iob();           \
    } while (0)
# endif
#endif /* CONFIG_CPU_CAVIUM_OCTEON */

#ifdef CONFIG_CPU_HAS_WB

#include <asm/wbflush.h>

#define wmb()       fast_wmb()
#define rmb()       fast_rmb()
#define mb()        wbflush()
#define iob()       wbflush()

#else /* !CONFIG_CPU_HAS_WB */

#define wmb()       fast_wmb()
#define rmb()       fast_rmb()
#define mb()        fast_mb()
#define iob()       fast_iob()

#endif /* !CONFIG_CPU_HAS_WB */

#if defined(CONFIG_WEAK_ORDERING) && defined(CONFIG_SMP)
# ifdef CONFIG_CPU_CAVIUM_OCTEON
#  define smp_mb()  __sync()
#  define smp_rmb() barrier()
#  define smp_wmb() __syncw()
# else
#  define smp_mb()  __asm__ __volatile__("sync" : : :"memory")
#  define smp_rmb() __asm__ __volatile__("sync" : : :"memory")
#  define smp_wmb() __asm__ __volatile__("sync" : : :"memory")
# endif
#else
#define smp_mb()    barrier()
#define smp_rmb()   barrier()
#define smp_wmb()   barrier()
#endif

#if defined(CONFIG_WEAK_REORDERING_BEYOND_LLSC) && defined(CONFIG_SMP)
#define __WEAK_LLSC_MB      "       sync    \n"
#else
#define __WEAK_LLSC_MB      "       \n"
#endif

#define set_mb(var, value) \
    do { var = value; smp_mb(); } while (0)

#define smp_llsc_mb()   __asm__ __volatile__(__WEAK_LLSC_MB : : :"memory")

#ifdef CONFIG_CPU_CAVIUM_OCTEON
#define smp_mb__before_llsc() smp_wmb()
/* Cause previous writes to become visible on all CPUs as soon as possible */
#define nudge_writes() __asm__ __volatile__(".set push\n\t"     \
                        ".set arch=octeon\n\t"  \
                        "syncw\n\t"         \
                        ".set pop" : : : "memory")
#else
#define smp_mb__before_llsc() smp_llsc_mb()
#define nudge_writes() mb()
#endif


#define get_smp_ctrl()      __read_32bit_c0_register($12, 2)
#define set_smp_ctrl(val)   __write_32bit_c0_register($12, 2, val)
#define get_smp_status()    __read_32bit_c0_register($12, 3)
#define set_smp_status(val) __write_32bit_c0_register($12, 3, val)
#define get_smp_reim()      __read_32bit_c0_register($12, 4)
#define set_smp_reim(val)   __write_32bit_c0_register($12, 4, val)
#define get_smp_lock()      __read_32bit_c0_register($12, 5)
#define set_smp_lock(val)   __write_32bit_c0_register($12, 5, val)
#define get_smp_val()       __read_32bit_c0_register($12, 6)
#define set_smp_val(val)    __write_32bit_c0_register($12, 6, val)

#define get_smp_mbox0()     __read_32bit_c0_register($20, 0)
#define set_smp_mbox0(val)  __write_32bit_c0_register($20, 0, val)
#define get_smp_mbox1()     __read_32bit_c0_register($20, 1)
#define set_smp_mbox1(val)  __write_32bit_c0_register($20, 1, val)
#define get_smp_mbox2()     __read_32bit_c0_register($20, 2)
#define set_smp_mbox2(val)  __write_32bit_c0_register($20, 2, val)
#define get_smp_mbox3()     __read_32bit_c0_register($20, 3)
#define set_smp_mbox3(val)  __write_32bit_c0_register($20, 3, val)


#define smp_ipi_unmask(mask) do {       \
        unsigned int reim;      \
        reim = get_smp_reim();      \
        reim |= (mask) & 0xff;      \
        set_smp_reim(reim);     \
    } while(0)
#define smp_ipi_mask(mask) do {         \
        unsigned int reim;      \
        reim = get_smp_reim();      \
        reim &= ~((mask) & 0xff);   \
        set_smp_reim(reim);     \
    } while(0)

#define smp_clr_pending(mask) do {          \
        unsigned int stat;          \
        stat = get_smp_status();        \
        stat &= ~((mask) & 0xff);       \
        set_smp_status(stat);           \
    } while(0)


#endif /* !__ASSEMBLY__ */

#endif

