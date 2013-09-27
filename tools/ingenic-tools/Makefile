#
# Copyright (C) 2012 Marek Vasut <marex@denx.de>
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
all:	$(obj)ddr_params_creator $(obj)ddr_params.h

HOSTCPPFLAGS = -I$(TOPDIR)/include

$(obj)ddr_params_creator:	ddr_params_creator.c
	$(HOSTCC) $(HOSTCFLAGS) $(HOSTLDFLAGS) -o $@ $^
	$(HOSTSTRIP) $@

$(obj)ddr_params.h:	$(obj)ddr_params_creator
	$(OBJTREE)/tools/ingenic-tools/ddr_params_creator > $(OBJTREE)/include/generated/$@

clean:
	rm -f $(obj)ddr_params_creator

distclean:	clean

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
