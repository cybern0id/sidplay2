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
dnl Try to find SIDPlay2 includes and library.
dnl $sid_have_libsidplay2 will be "yes" or "no"
dnl @LIBSIDPLAY2_LDADD@ will be substituted with -L$libsidplay2_libdir
dnl @LIBSIDPLAY2_INCLUDES@ will be substituted with -I$libsidplay2_incdir
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_PATH_LIBSIDPLAY2,
[
    AC_MSG_CHECKING([for working SIDPlay2 library and headers])
    
    dnl Be pessimistic.
    sid_libsidplay2_library=NO
    sid_libsidplay2_includes=NO

    AC_ARG_WITH(sidplay2,
        [  --with-libsidplay2=DIR
            where the sidplay2 library is located],
        [sid_libsidplay2_includes="$withval"
         sid_libsidplay2_library="$withval"
        ]
    )

    AC_ARG_WITH(libsidplay2-includes,
        [  --with-libsidplay2-includes=DIR
            where the libsidplay2 includes are located],
        [sid_libsidplay2_includes="$withval"]
    )

    AC_ARG_WITH(libsidplay2-library,
        [  --with-libsidplay2-library=DIR
            where the libsidplay2 library is installed],
        [sid_libsidplay2_library="$withval"]
    )

    # Test compilation with library and headers in standard path.
    sid_libsidplay2_libadd=""
    sid_libsidplay2_incadd=""

    # Use library path given by user (if any).
    if test "$sid_libsidplay2_library" != NO; then
        # Help to try and better locate library just from --with-libsidplay2 option
        libsidplay2_libdirs="$sid_libsidplay2_library $sid_libsidplay2_library/lib $sid_libsidplay2_library/.libs"
        SID_FIND_FILE(libsidplay2.a libsidplay2.so,$libsidplay2_libdirs,libsidplay2_foundlibdir)
        sid_libsidplay2_library=$libsidplay2_foundlibdir
    fi

    if test "$sid_libsidplay2_library" != NO; then
        sid_libsidplay2_libadd="-L$sid_libsidplay2_library"
    fi

    # Use include path given by user (if any).
    if test "$sid_libsidplay2_includes" != NO; then
        libsidplay2_incdirs="$sid_libsidplay2_includes $sid_libsidplay2_includes/include"
        SID_FIND_FILE(sidplay2.h sidplay/sidplay2.h,$libsidplay2_incdirs,libsidplay2_foundincdir)
        sid_libsidplay2_includes=$libsidplay2_foundincdir
    fi

    if test "$sid_libsidplay2_includes" != NO; then
        sid_libsidplay2_incadd="-I$sid_libsidplay2_includes"
    fi

    # Run test compilation.
    SID_TRY_LIBSIDPLAY2

    if test "$sid_libsidplay2_works" = no; then
        # Test compilation failed.
        # Need to search for library and headers
        # Search common locations where header files might be stored.
        libsidplay2_incdirs="/usr/include /usr/local/include /usr/lib/sidplay2/include /usr/local/lib/sidplay2/include"
        SID_FIND_FILE(sidplay/sidplay2.h,$libsidplay2_incdirs,libsidplay2_foundincdir)
        sid_libsidplay2_includes=$libsidplay2_foundincdir

        # Search common locations where library might be stored.
        libsidplay2_libdirs="/usr/lib /usr/local/lib /usr/lib/sidplay2/lib /usr/local/lib/sidplay2/lib"
        SID_FIND_FILE(libsidplay2.a libsidplay2.so,$libsidplay2_libdirs,libsidplay2_foundlibdir)
        sid_libsidplay2_library=$libsidplay2_foundlibdir

        if test "$sid_libsidplay2_includes" = NO || test "$sid_libsidplay2_library" = NO; then
            sid_have_libsidplay2=no
        else
            sid_have_libsidplay2=yes
        fi
        
        if test "$sid_have_libsidplay2" = yes; then
            sid_libsidplay2_libadd="-L$sid_libsidplay2_library"
            sid_libsidplay2_incadd="-I$sid_libsidplay2_includes"
            
            # Test compilation with found paths.
            SID_TRY_LIBSIDPLAY2

            sid_have_libsidplay2=$sid_libsidplay2_works
            if test "$sid_have_libsidplay2" = no; then
                # Found library does not link without errors.
                sid_have_libsidplay2=no
                AC_MSG_RESULT([$sid_have_libsidplay2]);
            else
                AC_MSG_RESULT([library $sid_libsidplay2_library, headers $sid_libsidplay2_includes])
            fi
        else
            # Either library or headers not found.
            AC_MSG_RESULT([$sid_have_libsidplay2]);
        fi

        if test "$sid_have_libsidplay2" = no; then
            AC_MSG_ERROR(
[
libsidplay2 library and/or header files not found.
Please check your installation!
]);
        fi
    else
        # Simply print 'yes' without printing the standard path.
        sid_have_libsidplay2=yes
        AC_MSG_RESULT([$sid_have_libsidplay2]);
    fi

    LIBSIDPLAY2_LDADD="$sid_libsidplay2_libadd"
    LIBSIDPLAY2_INCLUDES="$sid_libsidplay2_incadd"

    AC_SUBST(LIBSIDPLAY2_LDADD)
    AC_SUBST(LIBSIDPLAY2_INCLUDES)
])


dnl Function used by SID_PATH_LIBSIDPLAY2.

AC_DEFUN(SID_TRY_LIBSIDPLAY2,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_libs_save=$LIBS

    CXXFLAGS="$CXXFLAGS $sid_libsidplay2_incadd"
    LDFLAGS="$LDFLAGS $sid_libsidplay2_libadd"
    LIBS="-lsidplay2"

    AC_TRY_LINK(
        [#include <sidplay/sidplay2.h>],
        [sidplay2 *player;],
        [sid_libsidplay2_works=yes],
        [sid_libsidplay2_works=no]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    LIBS="$sid_libs_save"
])


dnl -------------------------------------------------------------------------
dnl Try to find Hardsid.  If so add support for it.
dnl $sid_have_hardsid will be "yes" or "no"
dnl -------------------------------------------------------------------------

AC_DEFUN(CHECK_HARDSID,
[
    AC_MSG_CHECKING([for hardsid soundcard])
    AC_TRY_RUN(
        [#include <sys/types.h>
         #include <sys/stat.h>
         #include <fcntl.h>
         #include <unistd.h>
         int main () {
             int fd = open ("/dev/sid0", O_RDWR);
             if (fd < 0) return -1;
             close (fd);
             return 0;
         }
        ],
        [sid_have_hardsid=yes],
        [sid_have_hardsid=no]
    )
    AC_MSG_RESULT($sid_have_hardsid)
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
