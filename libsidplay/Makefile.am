####### kdevelop will overwrite this part!!! (begin)##########

SUBDIRS = src include 

EXTRA_DIST = libsidplay.kdevprj AUTHORS COPYING ChangeLog INSTALL README TODO libsidplay.lsm sidconfig.h.in config.guess config.sub Makefile.dist config.status config.h 

install-data-local:
	$(mkinstalldirs) /usr/local/include/sidplay2/
	$(INSTALL_DATA) sidconfig.h /usr/local/include/sidplay2/sidconfig.h

uninstall-local:
	-rm -f /usr/local/include/sidplay2/sidconfig.h

####### kdevelop will overwrite this part!!! (end)############
AUTOMAKE_OPTIONS = foreign
