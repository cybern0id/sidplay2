sidplayincludedir = $(includedir)/sidplay/builders
sidplayinclude_HEADERS = resid.h

# The library.

libdir = $(LIBSIDPLAY2_DIR)
lib_LTLIBRARIES = libresid-builder.la

libresid_builder_la_SOURCES = resid.cpp resid-builder.cpp
libresid_builder_la_LIBADD  = $(RESID_LIBRARIES)

# We substitute the libtool-specific library version in configure.in.
libresid_builder_la_LDFLAGS = -version-info $(LIBVERSION)
