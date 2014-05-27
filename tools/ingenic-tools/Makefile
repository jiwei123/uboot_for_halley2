#
# Ingenic tools makefile
#
# Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
# Author: Zoro <ykli@ingenic.cn>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

ifndef CONFIG_SPL_BUILD
HOSTCPPFLAGS = -I$(TOPDIR)/include -DUSE_HOSTCC

DDR-$(CONFIG_DDR_PARAMS_CREATOR)	+= ddr_params_creator ddr_reg_values.h
MBR-$(CONFIG_MBR_CREATOR)		+= mbr_creator mbr.bin
GPT-$(CONFIG_GPT_CREATOR)		+= mbr-gpt.bin
SPI-$(CONFIG_SPI_SPL_CHECK)		+= spi_checksum
SPL-$(CONFIG_SPL_PARAMS_FIXER)		+= spl_params_fixer
#IMG2RLE-$(CONFIG_RLE_LCD_LOGO)          += img2rle
#BIN2ARRAY-$(CONFIG_RLE_LCD_LOGO)        += bin2array
LOGO-$(CONFIG_LCD_LOGO) += $(RLE_LOGO_C)
CHARGE_LOGO-$(CONFIG_CMD_BATTERYDET) += $(RLE_CHARGE_LOGO_H)
RLE_LOGO_C = $(OBJTREE)/common/rle_logo.c
RLE_CHARGE_LOGO_H = $(OBJTREE)/include/rle_charge_logo.h

#### Use jpg to make logo, added by ykliu ####
ifeq ($(LOGO_JPG),)
ifneq ($(wildcard $(TOPDIR)/board/$(BOARDDIR)/logo/*.jpg),)
LOGO_JPG= $(TOPDIR)/board/$(BOARDDIR)/logos/ingenic.jpg
# Use board logo and fallback to vendor
ifneq ($(wildcard $(TOPDIR)/board/$(BOARDDIR)/logo/$(BOARD).jpg),)
LOGO_JPG= $(TOPDIR)/board/$(BOARDDIR)/logo/$(BOARD).jpg
else
ifneq ($(wildcard $(TOPDIR)/board/$(BOARDDIR)/logo/(VENDOR).jpg),)
LOGO_JPG= $(TOPDIR)/board/$(BOARDDIR)/logo/$(VENDOR).jpg
endif
endif
else #use the default logo
LOGO_JPG= $(TOPDIR)/tools/logos/ingenic.jpg
# Use board logo and fallback to vendor
ifneq ($(wildcard $(TOPDIR)/tools/logos/$(BOARD).jpg),)
LOGO_JPG= $(TOPDIR)/tools/logos/$(BOARD).jpg
else
ifneq ($(wildcard $(TOPDIR)/tools/logos/(VENDOR).jpg),)
LOGO_JPG= $(TOPDIR)/tools/logos/$(VENDOR).jpg
endif
endif
endif
LOGO_RLE= $(LOGO_JPG).rle
endif


ifeq ($(CHARGE_LOGO_JPG),)
ifneq ($(wildcard $(TOPDIR)/board/$(BOARDDIR)/charge_logo/*.jpg),)
CHARGE_LOGO_JPG1= $(TOPDIR)/board/$(BOARDDIR)/charge_logo/charge_1.jpg
CHARGE_LOGO_JPG2= $(TOPDIR)/board/$(BOARDDIR)/charge_logo/charge_2.jpg
CHARGE_LOGO_JPG3= $(TOPDIR)/board/$(BOARDDIR)/charge_logo/charge_3.jpg
CHARGE_LOGO_JPG4= $(TOPDIR)/board/$(BOARDDIR)/charge_logo/charge_4.jpg
CHARGE_LOGO_JPG5= $(TOPDIR)/board/$(BOARDDIR)/charge_logo/charge_5.jpg
CHARGE_LOGO_JPG6= $(TOPDIR)/board/$(BOARDDIR)/charge_logo/charge_6.jpg
else ## use the default logo
CHARGE_LOGO_JPG1= $(TOPDIR)/tools/charge_logo/charge_1.jpg
CHARGE_LOGO_JPG2= $(TOPDIR)/tools/charge_logo/charge_2.jpg
CHARGE_LOGO_JPG3= $(TOPDIR)/tools/charge_logo/charge_3.jpg
CHARGE_LOGO_JPG4= $(TOPDIR)/tools/charge_logo/charge_4.jpg
CHARGE_LOGO_JPG5= $(TOPDIR)/tools/charge_logo/charge_5.jpg
CHARGE_LOGO_JPG6= $(TOPDIR)/tools/charge_logo/charge_6.jpg
endif

CHARGE_LOGO_RLE1= $(CHARGE_LOGO_JPG1).rle
CHARGE_LOGO_RLE2= $(CHARGE_LOGO_JPG2).rle
CHARGE_LOGO_RLE3= $(CHARGE_LOGO_JPG3).rle
CHARGE_LOGO_RLE4= $(CHARGE_LOGO_JPG4).rle
CHARGE_LOGO_RLE5= $(CHARGE_LOGO_JPG5).rle
CHARGE_LOGO_RLE6= $(CHARGE_LOGO_JPG6).rle

CHARGE_LOGO_RLE_ALL= $(CHARGE_LOGO_RLE1) \
		$(CHARGE_LOGO_RLE2) \
		$(CHARGE_LOGO_RLE3) \
		$(CHARGE_LOGO_RLE4) \
		$(CHARGE_LOGO_RLE5) \
		$(CHARGE_LOGO_RLE6)
endif

TOOLS	= $(DDR-y) $(MBR-y) $(GPT-y) $(SPI-y) $(SPL-y) $(LOGO-y) $(CHARGE_LOGO-y) #$(IMG2RLE-y) $(BIN2ARRAY-y)
TOOLS	:= $(addprefix $(obj),$(TOOLS))

all:	$(obj).depend $(TOOLS)

$(obj)ddr_params_creator:	ddr_params_creator.c
	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^
	$(HOSTSTRIP) $@

$(obj)ddr_reg_values.h:	$(obj)ddr_params_creator
	$(OBJTREE)/tools/ingenic-tools/ddr_params_creator > $(OBJTREE)/include/generated/$@

$(obj)mbr_creator:	mbr_creator.c
	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^
	$(HOSTSTRIP) $@

$(obj)mbr.bin:	$(obj)mbr_creator
	$(OBJTREE)/tools/ingenic-tools/mbr_creator	\
		p0off=$(CONFIG_MBR_P0_OFF),p0end=$(CONFIG_MBR_P0_END),p0type=$(CONFIG_MBR_P0_TYPE) \
		p1off=$(CONFIG_MBR_P1_OFF),p1end=$(CONFIG_MBR_P1_END),p1type=$(CONFIG_MBR_P1_TYPE) \
		p2off=$(CONFIG_MBR_P2_OFF),p2end=$(CONFIG_MBR_P2_END),p2type=$(CONFIG_MBR_P2_TYPE) \
		p3off=$(CONFIG_MBR_P3_OFF),p3end=$(CONFIG_MBR_P3_END),p3type=$(CONFIG_MBR_P3_TYPE) \
		-o $@ > /dev/zero

$(obj)gpt_creator:	$(SRCTREE)/lib/crc32.c gpt_creator.c
	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^
	$(HOSTSTRIP) $@

ifdef CONFIG_GPT_TABLE_PATH
$(obj)mbr-gpt.bin:	$(obj)gpt_creator
	$(OBJTREE)/tools/ingenic-tools/gpt_creator		\
	$(CONFIG_GPT_TABLE_PATH)/partitions.tab mbr-of-gpt.bin gpt.bin
	cat mbr-of-gpt.bin gpt.bin > $@
	dd if=/dev/zero of=file.bin bs=512 count=33
	cat $@ file.bin > file2.bin
	dd if=file2.bin of=$@ bs=512 count=34
	rm file* -rf
else
$(obj)mbr-gpt.bin:	$(obj)mbr.bin
	dd if=/dev/zero of=gpt.bin bs=512 count=33
	cat mbr.bin gpt.bin > $@
endif

$(obj)spi_checksum:	spi_checksum.c
	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^
	$(HOSTSTRIP) $@

$(obj)spl_params_fixer:	spl_params_fixer.c
	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^
	$(HOSTSTRIP) $@

#$(obj)img2rle:	img2rle.c
#	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) $^ -ljpeg -lpng12 -o $@
#	$(HOSTSTRIP) $@

#$(obj)bin2array: bin2array.c
#	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^

#-------  added by ykliu -----#
$(LOGO_RLE):  $(obj)img2rle $(LOGO_JPG)
	$(obj)./img2rle  $(LOGO_JPG) $@

$(RLE_LOGO_C):	$(obj)bin2array $(LOGO_RLE)
	$(obj)./bin2array --one $(LOGO_RLE) $@



$(CHARGE_LOGO_RLE1):  $(obj)img2rle $(CHARGE_LOGO_JPG1)
	$(obj)./img2rle  $(CHARGE_LOGO_JPG1) $@
$(CHARGE_LOGO_RLE2):  $(obj)img2rle $(CHARGE_LOGO_JPG2)
	$(obj)./img2rle  $(CHARGE_LOGO_JPG2) $@
$(CHARGE_LOGO_RLE3):  $(obj)img2rle $(CHARGE_LOGO_JPG3)
	$(obj)./img2rle  $(CHARGE_LOGO_JPG3) $@
$(CHARGE_LOGO_RLE4):  $(obj)img2rle $(CHARGE_LOGO_JPG4)
	$(obj)./img2rle  $(CHARGE_LOGO_JPG4) $@
$(CHARGE_LOGO_RLE5):  $(obj)img2rle $(CHARGE_LOGO_JPG5)
	$(obj)./img2rle  $(CHARGE_LOGO_JPG5) $@
$(CHARGE_LOGO_RLE6):  $(obj)img2rle $(CHARGE_LOGO_JPG6)
	$(obj)./img2rle  $(CHARGE_LOGO_JPG6) $@

$(RLE_CHARGE_LOGO_H):	$(obj)bin2array $(CHARGE_LOGO_RLE_ALL)
	$(obj)./bin2array --mult $(CHARGE_LOGO_RLE_ALL)  $@

clean:
	rm -f $(TOOLS) $(obj).depend  \
		$(TOPDIR)/tools/logos/*.rle \
		$(TOPDIR)/tools/charge_logo/*.rle \
		$(TOPDIR)/board/$(BOARDDIR)/logo/*.rle \
		$(TOPDIR)/board/$(BOARDDIR)/charge_logo/*.rle

distclean:	clean

else
LIB	= $(obj)lib_ddr_params_creator.o

COBJS	:= ddr_params_creator.o

SRCS	:= $(SOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(SOBJS) $(COBJS))

$(LIB):	$(obj).depend $(OBJS) $(SOBJS)
	$(call cmd_link_o_target, $(OBJS))
endif

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
