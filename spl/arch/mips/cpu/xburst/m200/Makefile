LIB	= lib$(SOC).o

START	= start.o

sinclude $(TOPDIR)/include/autoconf.mk

$(LIB): $(TOPDIR)/include/autoconf.mk
ifeq ($(CONFIG_DDR_CS1), y)
	@cp libm200_cs1.bk $@
else
	@cp libm200.bk $@
endif

$(START):
	@cp start.bk $@
