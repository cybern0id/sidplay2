
AUTOMAKE_OPTIONS = foreign

SUBDIRS = include src

EXTRA_DIST = unix win Makefile.dist

sidplayincludedir = $(includedir)/sidplay


# Remove CVS directories from any subdirs that get
# included completely (e.g. win, directx,...).

dist-hook:
		rm -rf `find $(distdir) -path \*/CVS`
