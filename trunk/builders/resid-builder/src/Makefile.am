# The library.

lib_LTLIBRARIES = libresid-builder.la

libresid_builder_la_SOURCES = resid.cpp resid-builder.cpp resid-builder.h resid-emu.h
libresid_builder_la_LIBADD  = $(LIBRESID_LDFLAGS)

# We substitute the libtool-specific library version in configure.in.
libresid_builder_la_LDFLAGS = -module -avoid-version

# Remove bad default includes
DEFAULT_INCLUDES=
