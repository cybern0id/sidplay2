####### kdevelop will overwrite this part!!! (begin)##########
noinst_LIBRARIES = libsidtune.a

libsidtune_a_SOURCES = SidTuneTools.cpp SidTune.cpp PSID.cpp PP20.cpp MUS.cpp InfoFile.cpp IconInfo.cpp 


EXTRA_DIST = Buffer.h IconInfo.cpp InfoFile.cpp MUS.cpp PP20.cpp PP20.h PP20_Defs.h PSID.cpp SidTune.cpp SidTune.h SidTuneCfg.h SidTuneEndian.h SidTuneTools.cpp SidTuneTools.h SidTuneTypes.h SmartPtr.h 

install-data-local:
	$(mkinstalldirs) /usr/local/include/sidplay2/
	$(INSTALL_DATA) SidTune.h /usr/local/include/sidplay2/SidTune.h
	$(mkinstalldirs) /usr/local/include/sidplay2/
	$(INSTALL_DATA) SidTuneCfg.h /usr/local/include/sidplay2/SidTuneCfg.h
	$(mkinstalldirs) /usr/local/include/sidplay2/
	$(INSTALL_DATA) SidTuneTypes.h /usr/local/include/sidplay2/SidTuneTypes.h

uninstall-local:
	-rm -f /usr/local/include/sidplay2/SidTune.h
	-rm -f /usr/local/include/sidplay2/SidTuneCfg.h
	-rm -f /usr/local/include/sidplay2/SidTuneTypes.h

####### kdevelop will overwrite this part!!! (end)############
