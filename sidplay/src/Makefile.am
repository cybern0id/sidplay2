
SUBDIRS = audio docs

# The console user interface.

bin_PROGRAMS = sidplay2

sidplay2_SOURCES = IniConfig.cpp IniConfig.h keyboard.cpp keyboard.h \
main.cpp

sidplay2_LDADD = ./audio/null/libnull.a ./audio/wav/libwav.a \
$(AUDIO_DRV) $(AUDIO_LDADD) \
$(LIBSIDPLAY2_LDADD) -lsidplay2 \
$(LIBSIDUTILS_LDADD) -lsidutils
