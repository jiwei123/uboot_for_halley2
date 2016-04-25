/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * Copyright 2007, The Android Open Source Project
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#include "./floader_m200.h"


#if (CONFIG_FLOADER_FUNCTION == 0)

    #define BOOT_MAGIC "ANDROID!"
    #define BOOT_MAGIC_SIZE 8
    #define BOOT_NAME_SIZE 16
    #define BOOT_ARGS_SIZE 512

    static struct boot_img_hdr
    {
        unsigned char magic[BOOT_MAGIC_SIZE];

        unsigned int kernel_size;  /* size in bytes */
        unsigned int kernel_addr;  /* physical load addr */

        unsigned int ramdisk_size; /* size in bytes */
        unsigned int ramdisk_addr; /* physical load addr */

        unsigned int second_size;  /* size in bytes */
        unsigned int second_addr;  /* physical load addr */

        unsigned int tags_addr;    /* physical addr for kernel tags */
        unsigned int page_size;    /* flash page size we assume */
        unsigned int unused[2];    /* future expansion: should be 0 */

        unsigned char name[BOOT_NAME_SIZE]; /* asciiz product name */

        unsigned char cmdline[BOOT_ARGS_SIZE];

        unsigned int id[8]; /* timestamp / checksum / sha1 / etc */
    } the_hdr;

    /*
    ** +-----------------+
    ** | boot header     | 1 page
    ** +-----------------+
    ** | kernel          | n pages
    ** +-----------------+
    ** | ramdisk         | m pages
    ** +-----------------+
    ** | second stage    | o pages
    ** +-----------------+
    **
    ** n = (kernel_size + page_size - 1) / page_size
    ** m = (ramdisk_size + page_size - 1) / page_size
    ** o = (second_size + page_size - 1) / page_size
    **
    ** 0. all entities are page_size aligned in flash
    ** 1. kernel and ramdisk are required (size != 0)
    ** 2. second is optional (second_size == 0 -> no second)
    ** 3. load each element (kernel, ramdisk, second) at
    **    the specified physical address (kernel_addr, etc)
    ** 4. prepare tags at tag_addr.  kernel_args[] is
    **    appended to the kernel commandline in the tags.
    ** 5. r0 = 0, r1 = MACHINE_TYPE, r2 = tags_addr
    ** 6. if second_size != 0: jump to second_addr
    **    else: jump to kernel_addr
    */


    static void boot_android_image()
    {
        /*
         * Step 1. Load image hdr and check
         */
        void *dst = (void *)CONFIG_BOOT_IMG_LOAD_ADDRESS_FROM_LPDDR2;

        lbaint_t sblk =
                CONFIG_BOOT_IMG_LOAD_ADDRESS_FROM_EMMC / CONFIG_EMMC_MSCO_BLOCK_SIZE;

        lbaint_t nblks = DIV_ROUND_UP(sizeof(the_hdr), CONFIG_EMMC_MSCO_BLOCK_SIZE);

        debug("sblk = %d, nblks = %d, dst = 0x%x\n", sblk, nblks, dst);

        if (mmc_bread(sblk, nblks, dst) != nblks) {
            debug("Loader: read hdr failed.\n");
            stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
        }

        memcpy((void *)&the_hdr, (void *)dst, sizeof(the_hdr));

        if (memcmp(the_hdr.magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
            debug("Loader: not a android boot image.\n");
            stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
        }

        debug("kernel_addr = 0x%x\n", the_hdr.kernel_addr);
        debug("ramdisk_addr = 0x%x\n", the_hdr.ramdisk_addr);

        /*
         * Step 2. Load kernel and ramdisk
         */

            /*
             * Load kernel
             */
        unsigned int page_size = the_hdr.page_size;
        unsigned int kernel_pages = DIV_ROUND_UP(the_hdr.kernel_size, page_size);

        sblk += page_size / CONFIG_EMMC_MSCO_BLOCK_SIZE;
        nblks = DIV_ROUND_UP(kernel_pages * page_size, CONFIG_EMMC_MSCO_BLOCK_SIZE);

        debug("sblk = %d, nblks = %d, dst = 0x%x\n", sblk, nblks, dst);

        if (mmc_bread(sblk, nblks, dst) != nblks) {
            debug("Loader: load kernel failed.\n");
            stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
        }

            /*
             * Load ramdisk
             */
        unsigned int ramdisk_pages = DIV_ROUND_UP(the_hdr.ramdisk_size, page_size);

        sblk += nblks;
        nblks = DIV_ROUND_UP(ramdisk_pages * page_size, CONFIG_EMMC_MSCO_BLOCK_SIZE);
        dst += the_hdr.ramdisk_addr - the_hdr.kernel_addr;

        debug("sblk = %d, nblks = %d, dst = 0x%x\n", sblk, nblks, dst);

        if (mmc_bread(sblk, nblks, dst) != nblks) {
            debug("Loader: load ramdisk failed.\n");
            stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
        }

        /*
         * Step 3. Prepare booting params
         */
        sprintf((char *)&the_hdr.cmdline, "%s,%dn8 %s %s %s %s%x %s%x",
                CONFIG_KERNEL_ARG_CONSOLE,
                CONFIG_KERNEL_ARG_CONSOLE_BAUD_RATE,
                CONFIG_KERNEL_ARG_MEM,
                CONFIG_KERNEL_ARG_ROOTFS,
                CONFIG_KERNEL_MISC,
                "rd_start=0x", (u32)dst,
                "rd_size=0x", the_hdr.ramdisk_size);

        debug("%s\n", (char *)&the_hdr.cmdline);

        u32 *params = (u32 *)&the_hdr;
        params[0] = 0;
        params[1] = (u32)&the_hdr.cmdline;

        /*
         * Step 4. Go kernel
         */
        flush_cache_all();

        void (*kernel)(int, char **) =
                (void (*)(int, char **)) CONFIG_BOOT_IMG_ENTRY_ADDRESS_FROM_LPDDR2;
        (*kernel)(2, (char **)params);
    }

#elif (CONFIG_FLOADER_FUNCTION == 1)

    static void boot_uboot_image()
    {
        void *dst = (void *)CONFIG_UBOOT_IMG_LOAD_ADDRESS_FROM_LPDDR2;
        lbaint_t sblk =
                CONFIG_UBOOT_IMG_LOAD_ADDRESS_FROM_EMMC / CONFIG_EMMC_MSCO_BLOCK_SIZE;

        lbaint_t nblks = DIV_ROUND_UP(CONFIG_UBOOT_IMG_SIZE, CONFIG_EMMC_MSCO_BLOCK_SIZE);

        if (mmc_bread(sblk, nblks, dst) != nblks) {
            debug("Loader: load uboot failed.\n");
            stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
        }

        flush_cache_all();

        void (*uboot)(void) =
                (void (*)(void)) CONFIG_UBOOT_IMG_ENTRY_ADDRESS_FROM_LPDDR2;
        uboot();
    }
#endif /* CONFIG_FLOADER_FUNCTION */

static void test_emmc()
{
#if 0
    u8 *dst  = (u8 *)0x88100000;
    u8 *dst1 = (u8 *)0x80100000;

    volatile u8 *p = dst;
    volatile u8 *p1 = dst1;

    ulong blocks;

    blocks = mmc_bread(0, 20480, dst);
    for (int i = 0; i < 512; i++)
        debug("0x%x\n", p[i]);

    debug("received blocks: %u\n", blocks);

    ulong times = 0;
    while (1) {
        blocks = mmc_bread(0, 20480, dst1);

        debug("received blocks: %u\n", blocks);

        for (int i = 0; i < 512 * 20480; i++) {
            if (p[i] != p1[i]) {
                debug("error !!!!!\n");

                while (1)
                    continue;
            }
        }

        debug("%u\n", times++);
    }
#endif
}


void boot_from_ddr()
{
    struct m200_sleep_lib_entry *entry =
                        (struct m200_sleep_lib_entry *)SLEEP_LIB_LPDDR2;
    entry->restore_context();
}

void boot_from_emmc()
{
    test_emmc();

#if (CONFIG_FLOADER_FUNCTION == 0)
    boot_android_image();

#elif (CONFIG_FLOADER_FUNCTION == 1)
    boot_uboot_image();

#endif
}

void init_sleep_lib(enum pmu_t pmu)
{
    u32 sblk = SLEEP_LIB_EMMC_ADDRESS / CONFIG_EMMC_MSCO_BLOCK_SIZE;
    u32 nblks = SLEEP_LIB_SIZE / CONFIG_EMMC_MSCO_BLOCK_SIZE;

    /*
     * First section
     */
    if (mmc_bread(sblk, nblks, (void *)SLEEP_LIB_TCSM) != nblks) {
        debug("Loader: read sleep lib of TCSM failed.\n");
        stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
    }

    /*
     * Second section
     */
    if (mmc_bread(sblk + nblks, nblks, (void *)SLEEP_LIB_LPDDR2) != nblks) {
        debug("Loader: read sleep lib of LPDDR2 failed.\n");
        stop(STOP_ERROR_EMMC_MSC0_LOADER_FAILED);
    }

    flush_cache_all();

    struct m200_sleep_lib_entry *entry =
                        (struct m200_sleep_lib_entry *)SLEEP_LIB_TCSM;

    entry->select_pmu(pmu);
    entry->enable_set_pmu_suspend_mode_voltage(
                CONFIG_SLEEP_VOLTAGE_ADJUST,
                CONFIG_CORE_SLEEP_VOLTAGE_MV, CONFIG_DDR_VDD2_SLEEP_VOLTAGE_MV);
}

