
SUBDIRS    = mos6510 mos6526 mos656x mos6581 sidtune xsid
EXTRA_DIST = player.bin


# The library.

lib_LTLIBRARIES = libsidplay2.la

libsidplay2_la_SOURCES = player.cpp player.h mixer.cpp sidplay2.cpp

libsidplay2_la_LIBADD  = ./mos6510/libmos6510.la \
./mos6526/libmos6526.la ./mos656x/libmos656x.la \
./xsid/libxsid.la ./sidtune/libsidtune.la \
$(RESID_LDADD) -lresid -lstdc++

# We substitute the libtool-specific library version in configure.in.
libsidplay2_la_LDFLAGS = -version-info $(LIBVERSION)
