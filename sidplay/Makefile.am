
AUTOMAKE_OPTIONS = foreign

SUBDIRS = doc src unix

EXTRA_DIST = include win acconfig.h


# Remove CVS directories from any subdirs that get
# included completely (e.g. win, directx,...).

dist-hook:
		rm -rf `find $(distdir) -path \*/CVS`
