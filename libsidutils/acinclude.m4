dnl -------------------------------------------------------------------------

AC_DEFUN(SID_SUBST_DEF,
[
    eval "$1=\"#define $1\""
    AC_SUBST($1)
])

AC_DEFUN(SID_SUBST_UNDEF,
[
    eval "$1=\"#undef $1\""
    AC_SUBST($1)
])

AC_DEFUN(SID_SUBST,
[
    eval "$1=$2"
    AC_SUBST($1)
])


dnl -------------------------------------------------------------------------
dnl Check whether C++ environment provides the "nothrow allocator".
dnl Will substitute @HAVE_EXCEPTIONS@ if test code compiles.
dnl -------------------------------------------------------------------------

AC_DEFUN(CHECK_EXCEPTIONS,
[
    AC_MSG_CHECKING([whether nothrow allocator is available])
    AC_CACHE_VAL(test_cv_have_exceptions,
    [
        AC_TRY_COMPILE(
            [#include <new>],
            [char* buf = new(nothrow) char[1024];],
            [test_cv_have_exceptions=yes],
            [test_cv_have_exceptions=no]
        )
    ])
    AC_MSG_RESULT($test_cv_have_exceptions)
    if test "$test_cv_have_exceptions" = yes; then
        AC_DEFINE(HAVE_EXCEPTIONS)
    fi
])


dnl -------------------------------------------------------------------------
dnl Find libsidplay2.  Don't bother to check if it works as we only require
dnl the header files.
dnl -------------------------------------------------------------------------
AC_DEFUN(LIBSIDPLAY2_TRY_COMPILE,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX

    CXXFLAGS="$CXXFLAGS $LIBSIDPLAY2_CXXFLAGS"
    LDFLAGS="$LDFLAGS $LIBSIDPLAY2_LDFLAGS"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

    AC_TRY_LINK(
        [#include <sidplay/sidplay2.h>],
        [sidplay2 *myEngine;],
        [LIBSIDPLAY2_WORKS=yes],
        [LIBSIDPLAY2_WORKS=no]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    CXX="$sid_cxx_save"
])


dnl -------------------------------------------------------------------------
dnl Pass C++ compiler options to libtool which supports C only.
dnl -------------------------------------------------------------------------

AC_DEFUN(CONFIG_LIBTOOL,
[
    save_cc=$CC
    save_cflags=$CFLAGS
    CC=$CXX
    CFLAGS=$CXXFLAGS
    AM_PROG_LIBTOOL
    CC=$save_cc
    CFLAGS=$save_cflags
])
