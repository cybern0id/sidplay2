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
dnl Check whether C++ environment provides the "nothrow allocator".
dnl Will substitute @HAVE_EXCEPTIONS@ if test code compiles.
dnl -------------------------------------------------------------------------

AC_DEFUN(CHECK_EXCEPTIONS,
[
    AC_MSG_CHECKING([whether nothrow allocator is available])
    AC_CACHE_VAL(test_cv_have_exceptions,
    [
        AC_TRY_COMPILE(
            [#include <new.h>],
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
dnl Try to find reSID includes and library.
dnl $sid_have_resid will be "yes" or "no"
dnl @RESID_LDADD@ will be substituted with -L$resid_libdir
dnl @RESID_INCLUDES@ will be substituted with -I$resid_incdir
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_PATH_LIBRESID,
[
    AC_MSG_CHECKING([for working reSID library and headers])
    
    dnl Be pessimistic.
    sid_resid_library=NO
    sid_resid_includes=NO
    sid_resid_install=system

    AC_ARG_WITH(resid,
        [  --with-resid=DIR
            where the resid is located],
        [sid_resid_includes="$withval"
         sid_resid_library="$withval"
        ]
    )

    AC_ARG_WITH(resid-includes,
        [  --with-resid-includes=DIR
            where the resid includes are located],
        [sid_resid_includes="$withval"]
    )

    AC_ARG_WITH(resid-library,
        [  --with-resid-library=DIR
            where the resid library is installed],
        [sid_resid_library="$withval"]
    )

    # Test compilation with library and headers in standard path.
    sid_resid_libadd=""
    sid_resid_incadd=""

    # Use library path given by user (if any).
    if test "$sid_resid_library" != NO; then
        # Help to try and better locate library just from --with-resid option
        resid_libdirs="$sid_resid_library $sid_resid_library/lib"
        SID_FIND_FILE(libresid.la,$resid_libdirs,resid_foundlibdir)
        sid_resid_library=$resid_foundlibdir
    fi

    if test "$sid_resid_library" != NO; then
        sid_resid_libadd="-L$sid_resid_library"
    fi

    # Use include path given by user (if any).
    if test "$sid_resid_includes" != NO; then
        resid_incdirs="$sid_resid_includes $sid_resid_includes/include"
        SID_FIND_FILE(sid.h resid/sid.h,$resid_incdirs,resid_foundincdir)
        sid_resid_includes=$resid_foundincdir
    fi

    if test "$sid_resid_includes" != NO; then
        sid_resid_install=user
        sid_resid_incadd="-I$sid_resid_includes"
    fi

    # Run test compilation.
    if test "$sid_resid_install" = user; then
        SID_TRY_USER_LIBRESID
        if test "$sid_libresid_works" = no; then
            sid_resid_install=local
        fi
    fi
    
    if test "$sid_resid_install" != user; then
        SID_TRY_LIBRESID
    fi

    if test "$sid_libresid_works" = no; then
        sid_resid_install=local
        # Test compilation failed.
        # Need to search for library and headers
        # Search common locations where header files might be stored.
        resid_incdirs="$includedir $prefix/include src/mos6581 src/mos6581/resid/include \
                       /usr/include /usr/local/include /usr/lib/resid/include \
                       /usr/local/lib/resid/include"
        SID_FIND_FILE(resid/sid.h,$resid_incdirs,resid_foundincdir)
        sid_resid_includes=$resid_foundincdir

        # Search common locations where library might be stored.
        resid_libdirs="$libdir $prefix/lib src/mos6581/resid src/mos6581/resid/lib \
                       /usr/lib /usr/local/lib /usr/lib/resid/lib /usr/local/lib/resid/lib"
        SID_FIND_FILE(libresid.la,$resid_libdirs,resid_foundlibdir)
        sid_resid_library=$resid_foundlibdir

        if test "$sid_resid_includes" = NO || test "$sid_resid_library" = NO; then
            sid_have_resid=no
        else
            sid_have_resid=yes
        fi
        
        if test "$sid_have_resid" = yes; then
            sid_resid_libadd="-L$sid_resid_library"
            sid_resid_incadd="-I$sid_resid_includes"
            
            # Test compilation with found paths.
            SID_TRY_LIBRESID

            sid_have_resid=$sid_libresid_works
            if test "$sid_have_resid" = no; then
                # Found library does not link without errors.
                sid_have_resid=no
                AC_MSG_RESULT([$sid_have_resid]);
            else
                AC_MSG_RESULT([library $sid_resid_library, headers $sid_resid_includes])
            fi
        else
            # Either library or headers not found.
            AC_MSG_RESULT([$sid_have_resid]);
        fi

        if test "$sid_have_resid" = no; then
            AC_MSG_ERROR(
[
reSID library and/or header files not found.
Please check your installation!
]);
        fi
    else
        # Simply print 'yes' without printing the standard path.
        sid_have_resid=yes
        AC_MSG_RESULT([$sid_have_resid]);
    fi

    RESID_LDFLAGS="$sid_resid_libadd -lresid"
    RESID_CXXFLAGS="$sid_resid_incadd"

    AC_SUBST(RESID_LDFLAGS)
    AC_SUBST(RESID_CXXFLAGS)

    if test "$sid_resid_install" != system; then
        if test "$sid_resid_install" = local; then
            AC_DEFINE(HAVE_LOCAL_RESID)
        else
            AC_DEFINE(HAVE_USER_RESID)
        fi
    fi
])

dnl Function used by SID_PATH_LIBRESID.
dnl This is for local or expected named paths

AC_DEFUN(SID_TRY_LIBRESID,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX

    CXXFLAGS="$CXXFLAGS $sid_resid_incadd"
    LDFLAGS="$LDFLAGS $sid_resid_libadd -lresid"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"
    

    AC_TRY_LINK(
        [#include "resid/sid.h"],
        [SID *mySID;],
        [sid_libresid_works=yes],
        [sid_libresid_works=no]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    CXX="$sid_cxx_save"
])

dnl Function used by SID_PATH_LIBRESID.
dnl This is for custom explict specified paths

AC_DEFUN(SID_TRY_USER_LIBRESID,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_cxx_save=$CXX

    CXXFLAGS="$CXXFLAGS $sid_resid_incadd"
    LDFLAGS="$LDFLAGS $sid_resid_libadd -lresid"
    CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

    AC_TRY_LINK(
        [#include "sid.h"],
        [SID *mySID;],
        [sid_libresid_works=yes],
        [sid_libresid_works=no]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    CXX="$sid_cxx_save"
])


dnl -------------------------------------------------------------------------
dnl Check for extended resid features
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_EXTENDED_LIBRESID,
[
    if test "$enable_resid" != no; then
        AC_MSG_CHECKING([for extended reSID features])
        sid_cxxflags_save=$CXXFLAGS
        sid_ldflags_save=$LDFLAGS
        sid_cxx_save=$CXX

        if test "$sid_resid_install" = user; then
            sid_libresid_header="#include \"sid.h\""
        else
            sid_libresid_header="#include \"resid/sid.h\""
        fi

        CXXFLAGS="$CXXFLAGS $sid_resid_incadd"
        LDFLAGS="$LDFLAGS $sid_resid_libadd -lresid"
        CXX="${SHELL-/bin/sh} ${srcdir}/libtool $CXX"

        AC_TRY_LINK(
            [$sid_libresid_header],
            [SID mySID;
             mySID.mute(0,true);
            ],
            [sid_libresid_extended=yes],
            [sid_libresid_extended=no]
        )

        CXXFLAGS="$sid_cxxflags_save"
        LDFLAGS="$sid_ldflags_save"
        CXX="$sid_cxx_save"

        if test "$sid_libresid_extended" = no; then
            # Found library does not link without errors.
            AC_MSG_ERROR(
[
reSID requires patching to function with libsidplay2.
Patches are available from http://sidplay2.sourceforge.net
]);
            AC_MSG_RESULT([$sid_have_resid]);
        else
            AC_MSG_RESULT([yes])
        fi
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
                          $LIBSIDPLAY2_LIBDIR/src"
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

    dnl setup builder dir
    if test "${libdir}" == '${exec_prefix}/lib'; then
        libdir="$LIBSIDPLAY2_BUILDERS"
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
