
SUBDIRS = audio docs mos6510 mos6581 sidtune xsid

INCLUDES = -Isidtune $(RESID_INCLUDES)

# The console user interface.

bin_PROGRAMS = sidplay2

sidplay2_SOURCES = keyboard.cpp main.cpp
sidplay2_LDADD = ./audio/oss/liboss.a ./audio/sunos/libsunos.a \
./audio/wav/libwav.a ./.libs/libsidplay2.so.0


# The library.

lib_LTLIBRARIES = libsidplay2.la

libsidplay2_la_SOURCES = fake6526.cpp fake6526.h keyboard.h \
player.cpp player.h mixer.cpp sidplay2.cpp

libsidplay2_la_LIBADD   = ./mos6510/libmos6510.la ./xsid/libxsid.la \
./sidtune/libsidtune.la $(RESID_LDADD) -lresid

## We substitute the libtool-specific library version in configure.in.
libsidplay2_la_LDFLAGS = -i -version-info $(LIBVERSION)
