
AUTOMAKE_OPTIONS = foreign

SUBDIRS = src include 

EXTRA_DIST = scripts win libsidplay.lsm Makefile.dist sidconfig.h.in

sidplayincludedir = $(includedir)/sidplay2

sidplayinclude_HEADERS = sidconfig.h


# Remove CVS directories from any subdirs that get
# included completely (e.g. win, directx,...).

dist-hook:
		rm -rf `find $(distdir) -path \*/CVS`

