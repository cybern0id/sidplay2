####### kdevelop will overwrite this part!!! (begin)##########
bin_PROGRAMS = libsidplay
libsidplay_SOURCES = sidmixer.cpp sidplayer.cpp fake6526.cpp main.cpp 
libsidplay_LDADD   = ./mos6510/libmos6510.a ./xsid/libxsid.a ./sidtune/libsidtune.a ./audio/oss/liboss.a ./audio/wav/libwav.a ./audio/sunos/libsunos.a $(RESID_LDADD) -lresid

SUBDIRS = mos6510 xsid sidtune audio mos6581 docs 

EXTRA_DIST = main.cpp fake6526.cpp fake6526.h sidplayer.cpp sidplayer.h sidmixer.cpp sidplayer_pr.h 

install-data-local:
	$(mkinstalldirs) /usr/local/include/sidplay2/
	$(INSTALL_DATA) sidplayer.h /usr/local/include/sidplay2/sidplayer.h

uninstall-local:
	-rm -f /usr/local/include/sidplay2/sidplayer.h

####### kdevelop will overwrite this part!!! (end)############

INCLUDES = $(RESID_INCLUDES)
