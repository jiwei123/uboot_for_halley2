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

LOCAL_CFLAGS  += -I$(TOPDIR)/include/ingenic_nand_mgr -I$(TOPDIR)/drivers/nand/inc -I$(TOPDIR)/drivers/nand/driver/inc -I$(TOPDIR)/drivers/nand/manager/inc -I$(TOPDIR)/drivers/nand/manager/inc/os -U__KERNEL__  #-Werror

CFLAGS  += $(LOCAL_CFLAGS)
CPPFLAGS  += $(LOCAL_CFLAGS)
HOSTCFLAGS  += $(LOCAL_CFLAGS)

ifndef CONFIG_SPL_BUILD

COBJS-y += do_nand_request.o
COBJS-y += jzdma/jzdma.o
COBJS-y += manager/os/clib.o
COBJS-y += manager/os/nanddebug.o
COBJS-y += manager/os/NandSemaphore.o
COBJS-y += manager/os/NandThread.o
COBJS-y += driver/nand_api.o
COBJS-y += driver/nand_driver.o
COBJS-y += driver/utils/nand_debug.o
COBJS-y += driver/utils/os_clib.o
COBJS-y += driver/utils/speed_dug.o
COBJS-y += driver/utils/rebuild_nand_spl.o
COBJS-y += driver/ops/nand_ops.o
COBJS-y += driver/ops/handler/bad_block.o
COBJS-y += driver/ops/handler/cpu_msg_handler.o
COBJS-y += driver/ops/handler/dma_msg_handler.o
COBJS-y += driver/ops/handler/spl_rw.o
COBJS-y += driver/ops/handler/spl_rw.o
COBJS-y += driver/hw/nand_bch.o
COBJS-y += driver/hw/nand_io_emc.o
COBJS-y += driver/hw/transadaptor/cpu_trans.o
COBJS-y += driver/hw/transadaptor/dma_trans.o
COBJS-y += driver/chips/nand_chip.o
COBJS-y += driver/chips/nand_info.o
COBJS-y += driver/chips/nand_errpt.o

else
COBJS-y += spl/nand_spl.o
COBJS-y += driver/utils/os_clib.o
COBJS-y += driver/hw/nand_io_emc.o
COBJS-y += driver/hw/nand_bch.o
COBJS-y += driver/hw/transadaptor/cpu_trans.o
endif
COBJS	:= $(sort $(COBJS-y))
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))
OBJS	+= libnm.hex  libops.hex
all:    $(LIB)

$(LIB): $(obj).depend $(OBJS)
	    $(call cmd_link_o_target, $(OBJS))


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
