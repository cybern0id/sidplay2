EXTRA_DIST =

pkglibdir = $(xmmsdir)/Input

pkglib_LTLIBRARIES = libsid2.la

libsid2_la_LDFLAGS = -module -avoid-version

libsid2_la_SOURCES = sid2_plugin.cpp

libsid2_la_LIBADD = -lsidplay2

LIBS := @LIBS@ @GTK_LIBS@
CFLAGS := @CFLAGS@ @GTK_CFLAGS@
CXXFLAGS := @CXXFLAGS@ @GTK_CFLAGS@
