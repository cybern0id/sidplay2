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
dnl Try to find a file (or one of more files in a list of dirs).
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_FIND_FILE,
    [
    $3=NO
    for i in $2;
    do
        for j in $1;
        do
	        if test -r "$i/$j"; then
		        $3=$i
                break 2
            fi
        done
    done
    ]
)


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
dnl Find libsidplay2 on the system.
dnl -------------------------------------------------------------------------
AC_DEFUN(LIBSIDPLAY2_FIND,
[
    AC_MSG_CHECKING([for working SIDPlay2 library and headers])

    dnl Be pessimistic.
    LIBSIDPLAY2_LIBDIR=NO
    LIBSIDPLAY2_INCLUDEDIR=NO

    AC_ARG_WITH(sidplay2,
        [  --with-sidplay2=DIR
            where the libsidplay2 is located],
        [LIBSIDPLAY2_INCLUDEDIR="$withval"
         LIBSIDPLAY2_LIBDIR="$withval"]
    )

    AC_ARG_WITH(sidplay2-includes,
        [  --with-sidplay2-includes=DIR
            where the libsidplay2 includes are located],
        [LIBSIDPLAY2_INCLUDEDIR="$withval"]
    )

    AC_ARG_WITH(sidplay2-library,
        [  --with-sidplay2-library=DIR
            where the libsidplay2 library is installed],
        [LIBSIDPLAY2_LIBDIR="$withval"]
    )

    dnl If user didn't provide paths see if pkg-config knows them
    if test "$LIBSIDPLAY2_INCLUDEDIR" = NO || test "$LIBSIDPLAY2_LIBDIR" = NO; then
        if $PKG_CONFIG --atleast-version $LIBSIDPLAY2_REQUIRED_VERSION libsidplay2; then
            :
        else
            AC_MSG_ERROR([
libsidplay $LIBSIDPLAY2_REQUIRED_VERSION library and/or headers not found.
Please check your installation!
            ]);
        fi
    fi

    dnl Find headers
    if test "$LIBSIDPLAY2_INCLUDEDIR" = NO; then
        LIBSIDPLAY2_INCLUDEDIR=`$PKG_CONFIG --variable=includedir libsidplay2`
        LIBSIDPLAY2_CXXFLAGS=`$PKG_CONFIG --cflags libsidplay2`
    else
        LIBSIDPLAY2_DIRS="$LIBSIDPLAY2_INCLUDEDIR $LIBSIDPLAY2_INCLUDEDIR/include"
        SID_FIND_FILE(sidplay/sidplay2.h,$LIBSIDPLAY2_DIRS,LIBSIDPLAY2_INCLUDEDIR)
        LIBSIDPLAY2_CXXFLAGS="-I$LIBSIDPLAY2_INCLUDEDIR"
    fi

    dnl find libs
    if test "$LIBSIDPLAY2_LIBDIR" = NO; then
        LIBSIDPLAY2_LIBDIR=`$PKG_CONFIG --variable=libdir libsidplay2`
        LIBSIDPLAY2_LDFLAGS=`$PKG_CONFIG --libs libsidplay2`
    else
        LIBSIDPLAY2_DIRS="$LIBSIDPLAY2_LIBDIR $LIBSIDPLAY2_LIBDIR/lib \
                          $LIBSIDPLAY2_LIBDIR/src"
        SID_FIND_FILE(libsidplay2.la,$LIBSIDPLAY2_DIRS,LIBSIDPLAY2_LIBDIR)
        LIBSIDPLAY2_LDFLAGS="-L$LIBSIDPLAY2_LIBDIR -lsidplay2"
    fi

    AC_MSG_RESULT([$LIBSIDPLAY2_LIBDIR, $LIBSIDPLAY2_INCLUDEDIR])
    LIBSIDPLAY2_TRY_COMPILE
    if test "$LIBSIDPLAY2_WORKS" = NO; then
        AC_MSG_ERROR([
libsidplay2 build test failed with found library and header files.
Please check your installation!
        ])
    fi

    AC_SUBST(LIBSIDPLAY2_LDFLAGS)
    AC_SUBST(LIBSIDPLAY2_CXXFLAGS)
])


dnl -------------------------------------------------------------------------
dnl Make sure libsidplay2 works.
dnl -------------------------------------------------------------------------
AC_DEFUN(LIBSIDPLAY2_TRY_COMPILE,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX

    CXXFLAGS="$CXXFLAGS $LIBSIDPLAY2_CXXFLAGS -DHAVE_UNIX"
    LDFLAGS="$LDFLAGS $LIBSIDPLAY2_LDFLAGS"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

    AC_TRY_LINK(
        [#include <sidplay/sidplay2.h>],
        [sidplay2 *myEngine;],
        [LIBSIDPLAY2_WORKS=YES],
        [LIBSIDPLAY2_WORKS=NO]
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
