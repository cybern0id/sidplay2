
AUTOMAKE_OPTIONS = foreign

SUBDIRS = src 

EXTRA_DIST = include unix win Makefile.dist acconfig.h


# Remove CVS directories from any subdirs that get
# included completely (e.g. win, directx,...).

dist-hook:
		rm -rf `find $(distdir) -path \*/CVS`
