HOW-TO Build U-boot
==================

1.Build Floader-m200
    a) cd floader-m200
    b) Change Makefile Line12
    c) create your board config file in floader-m200/boards/
        * Your board config file name is floader-m200/boards/your-boards.h for example
        * See demo file floader-m200/boards/aw808_board.h for m200 based board
        * See demo file floader-m200/boards/solar_board.h for m200s based board
    d) Build floader-m200
        * See floader-m200/README
    e) Copy build out to up-level directory
        * cp -rf floader_m200_with_mbr_gpt_and_sleeplib.bin ../

2. Build Uboot
    * Return U-boot root directory
    * Build U-boot as common ways 

3. Make final U-boot
    * cd ..
    * ./mk_floader_m200_with_mbr_gpt_sleeplib_and_uboot.sh

4. Final burning image is
    floader_m200_with_mbr_gpt_sleeplib_and_uboot.bin

5. Question
    孙文中 <wenzhong.sun@ingenic.com>

Thank You
