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
dnl Check whether compiler has a working ``bool'' type.
dnl Will substitute @SID_HAVE_BOOL@ with either a def or undef line.
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_CHECK_BOOL,
[
    AC_MSG_CHECKING([for bool])
    AC_CACHE_VAL(sid_cv_have_bool,
    [
        AC_TRY_COMPILE(
            [],
            [bool aBool = true;],
            [sid_cv_have_bool=yes],
            [sid_cv_have_bool=no]
        )
    ])
    AC_MSG_RESULT($sid_cv_have_bool)
    if test "$sid_cv_have_bool" = yes; then
        SID_SUBST_DEF(SID_HAVE_BOOL)
    else
        SID_SUBST_UNDEF(SID_HAVE_BOOL)
    fi
])

dnl -------------------------------------------------------------------------
dnl Check whether C++ library has member ios::bin instead of ios::binary.
dnl Will substitute @SID_HAVE_IOS_BIN@ with either a def or undef line.
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_CHECK_IOS_BIN,
[
    AC_MSG_CHECKING(whether standard member ios::binary is available)
    AC_CACHE_VAL(sid_cv_have_ios_binary,
    [
        AC_TRY_COMPILE(
            [#include <fstream.h>],
		    [ifstream myTest(ios::in|ios::binary);],
		    [sid_cv_have_ios_binary=yes],
		    [sid_cv_have_ios_binary=no]
	    )
    ])
    AC_MSG_RESULT($sid_cv_have_ios_binary)
    if test "$sid_cv_have_ios_binary" = no; then
        SID_SUBST_DEF(SID_HAVE_IOS_BIN)
    else
        SID_SUBST_UNDEF(SID_HAVE_IOS_BIN)
    fi
])

dnl -------------------------------------------------------------------------
dnl Check whether C++ environment provides the "nothrow allocator".
dnl Will substitute @SID_HAVE_EXCEPTIONS@ if test code compiles.
dnl -------------------------------------------------------------------------

AC_DEFUN(SID_CHECK_EXCEPTIONS,
[
    AC_MSG_CHECKING([whether nothrow allocator is available])
    AC_CACHE_VAL(sid_cv_have_exceptions,
    [
        AC_TRY_COMPILE(
            [#include <new>],
		    [char* buf = new(nothrow) char[1024];],
		    [sid_cv_have_exceptions=yes],
		    [sid_cv_have_exceptions=no]
	    )
    ])
    AC_MSG_RESULT($sid_cv_have_exceptions)
    if test "$sid_cv_have_exceptions" = yes; then
        SID_SUBST_DEF(SID_HAVE_EXCEPTIONS)
    else
        SID_SUBST_UNDEF(SID_HAVE_EXCEPTIONS)
    fi
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
    sid_resid_installed=yes

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
        sid_resid_libadd="-L$sid_resid_library"
   fi

    # Use include path given by user (if any).
    if test "$sid_resid_includes" != NO; then
        sid_resid_incadd="-I$sid_resid_includes"
        sid_resid_installed=no
        sid_resid_local=yes
    fi

    # Run test compilation.
    SID_TRY_LIBRESID
    if test "$sid_libresid_works" = no; then
        sid_resid_local=no
        SID_TRY_USER_LIBRESID
    fi

    if test "$sid_libresid_works" = no; then
        # Test compilation failed.
        # Need to search for library and headers
        # Search common locations where header files might be stored.
        sid_resid_installed=no
        sid_resid_local=yes
        resid_incdirs="src/mos6581 /usr/include /usr/local/include /usr/lib/resid/include /usr/local/lib/resid/include"
        SID_FIND_FILE(resid/sid.h,$resid_incdirs,resid_foundincdir)
        sid_resid_includes=$resid_foundincdir

        # Search common locations where library might be stored.
        resid_libdirs="src/mos6581 /usr/lib /usr/local/lib /usr/lib/resid /usr/local/lib/resid"
        SID_FIND_FILE(libresid.a libresid.so,$resid_libdirs,resid_foundlibdir)
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
reSID library and/or headers found not found.
Please check your installation!
]);
        fi
    else
        # Simply print 'yes' without printing the standard path.
        sid_have_resid=yes
        AC_MSG_RESULT([$sid_have_resid]);
    fi

    RESID_LDADD="$sid_resid_libadd"
    RESID_INCLUDES="$sid_resid_incadd"

    AC_SUBST(RESID_LDADD)
    AC_SUBST(RESID_INCLUDES)

    if test "$sid_resid_installed" = no; then
        if test "$sid_resid_local" = yes; then
            SID_SUBST_DEF(SID_HAVE_LOCAL_RESID)
            SID_SUBST_UNDEF(SID_HAVE_USER_RESID)
        else
            SID_SUBST_UNDEF(SID_HAVE_LOCAL_RESID)
            SID_SUBST_DEF(SID_HAVE_USER_RESID)
        fi
    else
        SID_SUBST_UNDEF(SID_HAVE_LOCAL_RESID)
        SID_SUBST_UNDEF(SID_HAVE_USER_RESID)
    fi
])

dnl Function used by SID_PATH_LIBRESID.
dnl This is for local or expected named paths

AC_DEFUN(SID_TRY_LIBRESID,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_libs_save=$LIBS

    CXXFLAGS="$CXXFLAGS $sid_resid_incadd"
    LDFLAGS="$LDFLAGS $sid_resid_libadd"
    LIBS="-lresid"

    AC_TRY_LINK(
        [#include "resid/sid.h"],
        [SID *mySID;],
        [sid_libresid_works=yes],
        [sid_libresid_works=no]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    LIBS="$sid_libs_save"
])

dnl Function used by SID_PATH_LIBRESID.
dnl This is for custom explict specified paths

AC_DEFUN(SID_TRY_USER_LIBRESID,
[
    sid_cxxflags_save=$CXXFLAGS
    sid_ldflags_save=$LDFLAGS
    sid_libs_save=$LIBS

    CXXFLAGS="$CXXFLAGS $sid_resid_incadd"
    LDFLAGS="$LDFLAGS $sid_resid_libadd"
    LIBS="-lresid"

    AC_TRY_LINK(
        [#include "sid.h"],
        [SID *mySID;],
        [sid_libresid_works=yes],
        [sid_libresid_works=no]
    )

    CXXFLAGS="$sid_cxxflags_save"
    LDFLAGS="$sid_ldflags_save"
    LIBS="$sid_libs_save"
])
