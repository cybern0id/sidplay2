
SUBDIRS = mos6510 mos6581 sidtune xsid

INCLUDES = -Isidtune $(RESID_INCLUDES)


# The library.

lib_LTLIBRARIES = libsidplay2.la

libsidplay2_la_SOURCES = fake6526.cpp fake6526.h \
player.cpp player.h mixer.cpp sidplay2.cpp

libsidplay2_la_LIBADD  = ./mos6510/libmos6510.la ./xsid/libxsid.la \
./sidtune/libsidtune.la $(RESID_LDADD) -lresid -lstdc++

# We substitute the libtool-specific library version in configure.in.
libsidplay2_la_LDFLAGS = -version-info $(LIBVERSION)
