
AUTOMAKE_OPTIONS = foreign

SUBDIRS = unix src 

EXTRA_DIST = win Makefile.dist


# Remove CVS directories from any subdirs that get
# included completely (e.g. win, directx,...).

dist-hook:
		rm -rf `find $(distdir) -path \*/CVS`
