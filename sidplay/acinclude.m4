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
dnl Check whether compiler has a working ``bool'' type.
dnl Will substitute @HAVE_BOOL@ with either a def or undef line.
dnl -------------------------------------------------------------------------

AC_DEFUN(CHECK_BOOL,
[
    AC_MSG_CHECKING([for bool])
    AC_CACHE_VAL(test_cv_have_bool,
    [
        AC_TRY_COMPILE(
            [],
            [bool aBool = true;],
            [test_cv_have_bool=yes],
            [test_cv_have_bool=no]
        )
    ])
    if test "$test_cv_have_bool" = yes; then
        test_cv_have_bool=yes
        AC_DEFINE(HAVE_BOOL)
    fi
    AC_MSG_RESULT($test_cv_have_bool)
])

dnl -------------------------------------------------------------------------
dnl Check whether C++ library has member ios::bin instead of ios::binary.
dnl Will substitute @HAVE_IOS_BIN@ with either a def or undef line.
dnl -------------------------------------------------------------------------

AC_DEFUN(CHECK_IOS_BIN,
[
    AC_MSG_CHECKING([whether standard member ios::binary is available])
    AC_CACHE_VAL(test_cv_have_ios_binary,
    [
        AC_TRY_COMPILE(
            [#include <fstream.h>],
		    [ifstream myTest(ios::in|ios::binary);],
		    [test_cv_have_ios_binary=yes],
		    [test_cv_have_ios_binary=no]
	    )
    ])
    AC_MSG_RESULT($test_cv_have_ios_binary)
    if test "$test_cv_have_ios_binary" = yes; then
        AC_DEFINE(HAVE_IOS_BIN)
    fi
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
        LIBSIDPLAY2_BUILDERS=`$PKG_CONFIG --variable=builders libsidplay2`
    else
        LIBSIDPLAY2_DIRS="$LIBSIDPLAY2_LIBDIR $LIBSIDPLAY2_LIBDIR/lib \
                          $LIBSIDPLAY2_LIBDIR/.libs"
        SID_FIND_FILE(libsidplay2.la,$LIBSIDPLAY2_DIRS,LIBSIDPLAY2_LIBDIR)
        LIBSIDPLAY2_LDFLAGS="-L$LIBSIDPLAY2_LIBDIR -lsidplay2"
        LIBSIDPLAY2_BUILDERS="$LIBSIDPLAY2_LIBDIR/sidplay/builders"
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

    CXXFLAGS="$CXXFLAGS $LIBSIDPLAY2_CXXFLAGS"
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
dnl Find libsidutils on the system.
dnl -------------------------------------------------------------------------
AC_DEFUN(LIBSIDUTILS_FIND,
[
    AC_MSG_CHECKING([for working SIDUtils library and headers])

    dnl Be pessimistic.
    LIBSIDUTILS_LIBDIR=NO
    LIBSIDUTILS_INCLUDEDIR=NO

    AC_ARG_WITH(sidplay2,
        [  --with-sidplay2=DIR
            where the libsidutils is located],
        [LIBSIDUTILS_INCLUDEDIR="$withval"
         LIBSIDUTILS_LIBDIR="$withval"]
    )

    AC_ARG_WITH(sidplay2-includes,
        [  --with-sidplay2-includes=DIR
            where the libsidutils includes are located],
        [LIBSIDUTILS_INCLUDEDIR="$withval"]
    )

    AC_ARG_WITH(sidplay2-library,
        [  --with-sidplay2-library=DIR
            where the libsidutils library is installed],
        [LIBSIDUTILS_LIBDIR="$withval"]
    )

    dnl If user didn't provide paths see if pkg-config knows them
    if test "$LIBSIDUTILS_INCLUDEDIR" = NO || test "$LIBSIDUTILS_LIBDIR" = NO; then
        if $PKG_CONFIG --atleast-version $LIBSIDUTILS_REQUIRED_VERSION libsidutils; then
            :
        else
            AC_MSG_ERROR([
libsidplay $LIBSIDUTILS_REQUIRED_VERSION library and/or headers not found.
Please check your installation!
            ]);
        fi
    fi

    dnl Find headers
    if test "$LIBSIDUTILS_INCLUDEDIR" = NO; then
        LIBSIDUTILS_INCLUDEDIR=`$PKG_CONFIG --variable=includedir libsidutils`
        LIBSIDUTILS_CXXFLAGS=`$PKG_CONFIG --cflags libsidutils`
    else
        LIBSIDUTILS_DIRS="$LIBSIDUTILS_INCLUDEDIR $LIBSIDUTILS_INCLUDEDIR/include"
        SID_FIND_FILE(sidplay/sidplay2.h,$LIBSIDUTILS_DIRS,LIBSIDUTILS_INCLUDEDIR)
        LIBSIDUTILS_CXXFLAGS="-I$LIBSIDUTILS_INCLUDEDIR"
    fi

    dnl find libs
    if test "$LIBSIDUTILS_LIBDIR" = NO; then
        LIBSIDUTILS_LIBDIR=`$PKG_CONFIG --variable=libdir libsidutils`
        LIBSIDUTILS_LDFLAGS=`$PKG_CONFIG --libs libsidutils`
    else
        LIBSIDUTILS_DIRS="$LIBSIDUTILS_LIBDIR $LIBSIDUTILS_LIBDIR/lib \
                          $LIBSIDUTILS_LIBDIR/.libs"
        SID_FIND_FILE(libsidutils.la,$LIBSIDUTILS_DIRS,LIBSIDUTILS_LIBDIR)
        LIBSIDUTILS_LDFLAGS="-L$LIBSIDUTILS_LIBDIR -lsidutils"
    fi

    AC_MSG_RESULT([$LIBSIDUTILS_LIBDIR, $LIBSIDUTILS_INCLUDEDIR])
    LIBSIDUTILS_TRY_COMPILE
    if test "$LIBSIDUTILS_WORKS" = NO; then
        AC_MSG_ERROR([
libsidutils build test failed with found library and header files.
Please check your installation!
        ])
    fi

    AC_SUBST(LIBSIDUTILS_LDFLAGS)
    AC_SUBST(LIBSIDUTILS_CXXFLAGS)
])


dnl -------------------------------------------------------------------------
dnl Make sure libsidutils works.
dnl -------------------------------------------------------------------------
AC_DEFUN(LIBSIDUTILS_TRY_COMPILE,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX

    CXXFLAGS="$CXXFLAGS $LIBSIDUTILS_CXXFLAGS"
    LDFLAGS="$LDFLAGS $LIBSIDUTILS_LDFLAGS"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

    AC_TRY_LINK(
        [#include <sidplay/utils/SidDatabase.h>],
        [SidDatabase *d;],
        [LIBSIDUTILS_WORKS=YES],
        [LIBSIDUTILS_WORKS=NO]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    CXX="$sid_cxx_save"
])

dnl -------------------------------------------------------------------------
dnl Find builders dir
dnl -------------------------------------------------------------------------
AC_DEFUN(BUILDERS_FIND,
[
    AC_MSG_CHECKING([for sidbuilders install directory])

    dnl Be pessimistic.
    BUILDERS_LIBDIR=NO
    BUILDERS_INCLUDEDIR=NO

    AC_ARG_WITH(sidbuilders,
        [  --with-sidbuilders=DIR
            where the sid builder modules are instealled],
        [BUILDERS_INCLUDEDIR="$withval/include"
         BUILDERS_LIBDIR="$withval/sidplay/builders"]
    )

    AC_ARG_WITH(sidbuilder-includes,
        [  --with-sidbuilder-includes=DIR
            where the sid builder includes are located],
        [BUILDERS_INCLUDEDIR="$withval"]
    )

    AC_ARG_WITH(sidbuilder-library,
        [  --with-sidbuilder-library=DIR
            where the sid builder libraries is installed],
        [BUILDERS_LIBDIR="$withval"]
    )

    if test "$BUILDERS_INCLUDEDIR" = NO; then
        BUILDERS_INCLUDEDIR="$LIBSIDPLAY2_INCLUDEDIR"
    fi

    if test "$BUILDERS_LIBDIR" = NO; then
        BUILDERS_LIBDIR="$LIBSIDPLAY2_BUILDERS"
    fi

    BUILDERS_CXXFLAGS="-I$BUILDERS_INCLUDEDIR"
    BUILDERS_LDFLAGS="-L$BUILDERS_LIBDIR"
    AC_SUBST(BUILDERS_CXXFLAGS)
    AC_SUBST(BUILDERS_LDFLAGS)
    AC_MSG_RESULT([$BUILDERS_LIBDIR, $BUILDERS_INCLUDEDIR])
])

dnl -------------------------------------------------------------------------
dnl Test for working resid
dnl -------------------------------------------------------------------------
AC_DEFUN(BUILDERS_FIND_RESID,
[
    AC_MSG_CHECKING([for resid builder module])
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX
    CXXFLAGS="$CXXFLAGS $BUILDERS_CXXFLAGS"
    LDFLAGS="$LDFLAGS $BUILDERS_LDFLAGS -lresid-builder"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

    AC_TRY_LINK(
        [#include <sidplay/builders/resid.h>],
        [ReSID *sid;],
        [BUILDERS_WORK=YES],
        [BUILDERS_WORK=NO]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    CXX="$sid_cxx_save"

    if test "$BUILDERS_WORK" = YES; then
        BUILDERS_AVAILABLE=YES
        AC_DEFINE(HAVE_RESID_BUILDER)
        RESID_LDFLAGS="-lresid-builder"
        AC_SUBST(RESID_LDFLAGS)
    fi
    AC_MSG_RESULT($BUILDERS_WORK)
])

dnl -------------------------------------------------------------------------
dnl Test for working hardsid
dnl -------------------------------------------------------------------------
AC_DEFUN(BUILDERS_FIND_HARDSID,
[
    AC_MSG_CHECKING([for hardsid builder module])
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX

    CXXFLAGS="$CXXFLAGS $BUILDERS_CXXFLAGS"
    LDFLAGS="$LDFLAGS $BUILDERS_LDFLAGS -lhardsid-builder"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

    AC_TRY_LINK(
        [#include <sidplay/builders/hardsid.h>],
        [HardSID *sid;],
        [BUILDERS_WORK=YES],
        [BUILDERS_WORK=NO]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    CXX="$sid_cxx_save"

    if test "$BUILDERS_WORK" = YES; then
        BUILDERS_AVAILABLE=YES
        AC_DEFINE(HAVE_HARDSID_BUILDER)
        HARDSID_LDFLAGS="-lhardsid-builder"
        AC_SUBST(HARDSID_LDFLAGS)
    fi
    AC_MSG_RESULT($BUILDERS_WORK)
])
