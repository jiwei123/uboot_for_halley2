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
HOSTCPPFLAGS = -I$(TOPDIR)/include

DDR-$(CONFIG_DDR_PARAMS_CREATOR)	+= ddr_params_creator ddr_reg_values.h
MBR-$(CONFIG_MBR_CREATOR)		+= mbr_creator mbr.bin
MBR-$(CONFIG_GPT_CREATOR)		+= mbr-gpt.bin #gpt_creator mbr-gpt.bin

TOOLS	= $(DDR-y) $(MBR-y)
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

#$(obj)gpt_creator:	gpt_creator.c
#	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^
#	$(HOSTSTRIP) $@
#
# GPT creator will come back soon, now gpt-table is filled by 0.

$(obj)mbr-gpt.bin:	#$(obj)gpt_creator
	dd if=/dev/zero of=$@ bs=512 count=34

clean:
	rm -f $(TOOLS) $(obj).depend

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
