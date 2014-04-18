#
#
# Copyright (C) 2006 Ingenic Semiconductor Inc.
# By Lucifer
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#

# -----------------------------

include $(TOPDIR)/config.mk

LIB	:= $(obj)libnand.o


NAND_DRIVER_DIR := .

LOCAL_CFLAGS  += -I$(TOPDIR)/include/ingenic_nand_mgr -I$(TOPDIR)/drivers/nand/include -I$(TOPDIR)/drivers/nand/nm_driver/nddriver/inc -I$(TOPDIR)/drivers/nand/nm_driver/ndmanager/inc -I$(TOPDIR)/drivers/nand/nm_driver/ndmanager/inc_os -DCONFIG_SOC_4775	-U__KERNEL__  #-Werror

CFLAGS  += $(LOCAL_CFLAGS)
CPPFLAGS  += $(LOCAL_CFLAGS) 
HOSTCFLAGS  += $(LOCAL_CFLAGS)


COBJS-y += do_nand_request.o
COBJS-y += jzdma/jzdma.o
COBJS-y += nm_driver/ndmanager/os_src/clib.o
COBJS-y += nm_driver/ndmanager/os_src/nanddebug.o
COBJS-y += nm_driver/ndmanager/os_src/NandSemaphore.o
COBJS-y += nm_driver/ndmanager/os_src/NandThread.o
COBJS-y += nm_driver/nddriver/nand_api.o
COBJS-y += nm_driver/nddriver/nand_driver.o
COBJS-y += nm_driver/nddriver/utils/nand_debug.o
COBJS-y += nm_driver/nddriver/utils/os_clib.o
COBJS-y += nm_driver/nddriver/utils/speed_dug.o
COBJS-y += nm_driver/nddriver/utils/rebuild_nand_spl.o
COBJS-y += nm_driver/nddriver/ops/nand_ops.o
COBJS-y += nm_driver/nddriver/ops/handler/bad_block.o
COBJS-y += nm_driver/nddriver/ops/handler/cpu_msg_handler.o
COBJS-y += nm_driver/nddriver/ops/handler/dma_msg_handler.o
COBJS-y += nm_driver/nddriver/ops/handler/spl_rw.o
COBJS-y += nm_driver/nddriver/ops/handler/spl_rw.o
COBJS-y += nm_driver/nddriver/hw/nand_bch.o
COBJS-y += nm_driver/nddriver/hw/nand_io.o
COBJS-y += nm_driver/nddriver/hw/transadaptor/cpu_trans.o
COBJS-y += nm_driver/nddriver/hw/transadaptor/dma_trans.o
COBJS-y += nm_driver/nddriver/chips/nand_chip.o
COBJS-y += nm_driver/nddriver/chips/nand_info.o
COBJS-y += nm_driver/nddriver/chips/nand_errpt.o

COBJS	:= $(sort $(COBJS-y))
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))
OBJS	+= libnm.lib  libops.lib
all:    $(LIB)

$(LIB): $(obj).depend $(OBJS)
	    $(call cmd_link_o_target, $(OBJS))


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
