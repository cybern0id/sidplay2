
SUBDIRS = audio docs

# The console user interface.

bin_PROGRAMS = sidplay2

sidplay2_SOURCES = keyboard.cpp main.cpp
sidplay2_LDADD = ./audio/null/libnull.a ./audio/wav/libwav.a \
$(AUDIO_DRV) $(AUDIO_LIB) -lsidplay2
