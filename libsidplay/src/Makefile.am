
SUBDIRS    = c64 mos6510 mos6526 mos656x xsid sid6526 sidtune
EXTRA_DIST = psiddrv.a65 psiddrv.bin


# The library.

lib_LTLIBRARIES = libsidplay2.la

libsidplay2_la_SOURCES = config.cpp event.cpp \
player.cpp player.h \
psiddrv.cpp mixer.cpp \
reloc65.c sidplay2.cpp

libsidplay2_la_LIBADD  = ./c64/libc64.la \
./mos6510/libmos6510.la ./mos6526/libmos6526.la \
./mos656x/libmos656x.la ./xsid/libxsid.la \
./sidtune/libsidtune.la ./sid6526/libsid6526.la

# We substitute the libtool-specific library version in configure.in.
libsidplay2_la_LDFLAGS = -version-info $(LIBVERSION)
