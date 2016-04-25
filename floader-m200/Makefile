#
# Copyright (C) 2016 Ingenic Semiconductor
#
# SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
#
# For project-s5
#
# Release under GPLv2
#
#

CROSS ?= mipsel-unknown-linux-gnu-

CFLAGS := -Os -march=mips32r2 -include "./board_base.h" -include $(CONFIG_BOARD) -fno-pic -fno-builtin -mno-abicalls -nostdlib -EL -g -Wall -msoft-float -Bstatic -std=gnu11

LDFLAGS := -nostdlib -T floader_m200.lds -EL

# TCSM Bank4
SLEEP_LIB_IN_TCSM_ADDRESS := 0xb3426000
# Last 32KB of the first 256MB of LPDDR2
SLEEP_LIB_IN_DDR_ADDRESS := 0x8fff8000

OBJS := start.o         \
		gpio.o          \
		clk.o           \
		i2c.o           \
		uart.o          \
		mmc.o           \
		lpddr2.o        \
		utils.o         \
		loader.o        \
		sm5007.o        \
		ricoh-5t619.o   \
		sensor_hub.o    \
		pmu_sel.o       \
		main.o

SLEEP_LIB_OBJS := sleep_lib_entry.o    \
					utils.o            \
					gpio.o             \
					i2c.o              \
					sm5007.o           \
					ricoh-5t619.o      \
					sensor_hub.o       \
					sleep_lib_uart.o   \
					sleep_lib.o        \
					sleep_lib_lpddr2.o \
					sleep_lib_clk.o    \
					pmu_sel.o

all: 		clean							\
			create_mbr_gpt					\
			calc_uart_bund_rate				\
			create_lpddr2_params			\
			spl_params_fixer				\
			sleep_lib.elf floader_m200.elf
	$(CROSS)objdump -D sleep_lib_tcsm.elf > sleep_lib_tcsm.elf.dump
	$(CROSS)objcopy --gap-fill=0x0 -O binary sleep_lib_tcsm.elf sleep_lib_tcsm.bin
	$(CROSS)objcopy --gap-fill=0x0 --pad-to=16384 -I binary -O binary sleep_lib_tcsm.bin sleep_lib_tcsm_pad.bin
	$(CROSS)objdump -D sleep_lib_ddr.elf > sleep_lib_ddr.elf.dump
	$(CROSS)objcopy --gap-fill=0x0 -O binary sleep_lib_ddr.elf sleep_lib_ddr.bin
	$(CROSS)objcopy --gap-fill=0x0 --pad-to=16384 -I binary -O binary sleep_lib_ddr.bin sleep_lib_ddr_pad.bin
	$(CROSS)objdump -D floader_m200.elf > floader_m200.elf.dump
	$(CROSS)objcopy --gap-fill=0x0 -O binary floader_m200.elf floader_m200.bin
	@echo ""
	@echo ""
	@echo ""
	@echo "======================================================================"
	@echo `ls -l sleep_lib_tcsm.bin`
	@echo `ls -l sleep_lib_ddr.bin`
	@echo `ls -l floader_m200.bin`
	@echo "======================================================================"
	@echo ""
	@echo ""
	@echo ""
	$(CROSS)objcopy --gap-fill=0x0 --pad-to=26624 -I binary -O binary floader_m200.bin floader_m200_pad.bin
	cat mbr_gpt.bin floader_m200_pad.bin sleep_lib_tcsm_pad.bin sleep_lib_ddr_pad.bin > floader_m200_with_mbr_gpt_and_sleeplib.bin
	dd if=./spl_params.bin of=./floader_m200_with_mbr_gpt_and_sleeplib.bin conv=notrunc
	mv floader_m200_with_mbr_gpt_and_sleeplib.bin .floader_m200_with_mbr_gpt_and_sleeplib.bin
	dd if=./floader_m200.bin of=./spl_lpddr2.bin skip=2048 bs=1
	mv spl_lpddr2.bin .spl_lpddr2.bin
	rm *.bin
	mv .floader_m200_with_mbr_gpt_and_sleeplib.bin floader_m200_with_mbr_gpt_and_sleeplib.bin
	mv .spl_lpddr2.bin spl_lpddr2.bin
	rm -rf *.o create_lpddr2_params calc_uart_bund_rate spl_params_fixer
	chmod 777 floader_m200_with_mbr_gpt_and_sleeplib.bin	\
		spl_lpddr2.bin									\
		sleep_lib_tcsm.elf.dump							\
		sleep_lib_ddr.elf.dump							\
		floader_m200.elf.dump
	@echo ""
	@echo ""
	@echo ""
	@echo "======================================================================"
	@echo -e "|| \033[32mfloader_m200_with_mbr_gpt_and_sleeplib.bin\033[0m is ready for launch . ||"
	@echo "======================================================================"
	@echo ""
	@echo ""
	@echo ""

floader_m200.elf: $(OBJS) 
	$(CROSS)ld $(LDFLAGS) $(OBJS) -o $@

sleep_lib.elf: $(SLEEP_LIB_OBJS) 
	$(CROSS)ld $(LDFLAGS) -T sleep_lib.lds -Ttext $(SLEEP_LIB_IN_TCSM_ADDRESS) $(SLEEP_LIB_OBJS) -o sleep_lib_tcsm.elf
	$(CROSS)ld $(LDFLAGS) -T sleep_lib.lds -Ttext $(SLEEP_LIB_IN_DDR_ADDRESS) $(SLEEP_LIB_OBJS) -o sleep_lib_ddr.elf

create_mbr_gpt:
	./emcp-lpddr2/gpt_creator $(CONFIG_PART_TAB) mbr.bin gpt.bin
	cat ./mbr.bin ./gpt.bin > mbr_gpt.bin
	dd if=/dev/zero of=gpt_pad.bin bs=512 count=33
	cat mbr_gpt.bin gpt_pad.bin > mbr_gpt_pad.bin
	dd if=mbr_gpt_pad.bin of=mbr_gpt.bin bs=512 count=34

calc_uart_bund_rate:
	gcc -include "./board_base.h" -include $(CONFIG_BOARD) calc_uart_bund_rate.c -o calc_uart_bund_rate
	./calc_uart_bund_rate > uart_baud_rate.h
	./calc_uart_bund_rate

create_lpddr2_params:
	gcc -include "./board_base.h" -include $(CONFIG_BOARD) -fno-builtin emcp-lpddr2/create_lpddr2_params.c -o create_lpddr2_params
	./create_lpddr2_params > lpddr2_params.h
	./create_lpddr2_params

spl_params_fixer:
	gcc -include "./board_base.h" -include $(CONFIG_BOARD) -fno-builtin spl_params_fixer.c -o spl_params_fixer
	./spl_params_fixer
	chmod 777 spl_params.bin

.c.o:
	$(CROSS)gcc $(CFLAGS) $(EXTRA_CFLGAS) -c $< -o $@

.S.o:
	$(CROSS)gcc -D__ASSEMBLY__ $(CFLAGS) $(EXTRA_CFLGAS) -c $< -o $@


clean:
	rm -rf *.o *.elf *.bin *.dump *.map lpddr2_params.h uart_baud_rate.h
	rm -rf create_lpddr2_params calc_uart_bund_rate spl_params_fixer
