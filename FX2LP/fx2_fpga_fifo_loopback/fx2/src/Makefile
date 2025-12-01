FX2LIBDIR=../
BASENAME = bulkloop
SOURCES=bulkloop.c
A51_SOURCES=dscr.a51
PID=0x1004
BUILDDIR=build

include $(FX2LIBDIR)lib/fx2.mk

# Default target: build .hex file
all: $(BASENAME).hex

# Copy .ihx to .hex
$(BASENAME).hex: $(BUILDDIR)/$(BASENAME).ihx
	cp $(BUILDDIR)/$(BASENAME).ihx $(BUILDDIR)/$(BASENAME).hex


