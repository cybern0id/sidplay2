
SUBDIRS = audio docs mos6510 mos6581 sidtune xsid

INCLUDES = -Isidtune $(RESID_INCLUDES)

# The console user interface.

bin_PROGRAMS = sidplay2

sidplay2_SOURCES = main.cpp
sidplay2_LDADD = ./audio/oss/liboss.a ./audio/wav/libwav.a \
./audio/sunos/libsunos.a \
-L.libs -lsidplay \
$(RESID_LDADD) -lresid

# The library.

sidplayincludedir = $(includedir)/sidplay2

sidplayinclude_HEADERS = sidplayer.h

lib_LTLIBRARIES = libsidplay.la

libsidplay_la_SOURCES = fake6526.cpp fake6526.h \
sidmixer.cpp \
sidplayer.cpp sidplayer.h sidplayer_pr.h

libsidplay_la_LIBADD   = ./mos6510/libmos6510.la ./xsid/libxsid.la \
./sidtune/libsidtune.la

## We substitute the libtool-specific library version in configure.in.
#libsidplay_la_LDFLAGS = -version-info $(LIBVERSION)
## We substitute the library release version in configure.in.
libsidplay_la_LDFLAGS = -release $(LIBVERSION)

