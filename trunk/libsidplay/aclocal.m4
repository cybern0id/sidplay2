dnl aclocal.m4 generated automatically by aclocal 1.4

dnl Copyright (C) 1994, 1995-8, 1999 Free Software Foundation, Inc.
dnl This file is free software; the Free Software Foundation
dnl gives unlimited permission to copy and/or distribute it,
dnl with or without modifications, as long as this notice is preserved.

dnl This program is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY, to the extent permitted by law; without
dnl even the implied warranty of MERCHANTABILITY or FITNESS FOR A
dnl PARTICULAR PURPOSE.

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

# Like AC_CONFIG_HEADER, but automatically create stamp file.

AC_DEFUN(AM_CONFIG_HEADER,
[AC_PREREQ([2.12])
AC_CONFIG_HEADER([$1])
dnl When config.status generates a header, we must update the stamp-h file.
dnl This file resides in the same directory as the config header
dnl that is generated.  We must strip everything past the first ":",
dnl and everything past the last "/".
AC_OUTPUT_COMMANDS(changequote(<<,>>)dnl
ifelse(patsubst(<<$1>>, <<[^ ]>>, <<>>), <<>>,
<<test -z "<<$>>CONFIG_HEADERS" || echo timestamp > patsubst(<<$1>>, <<^\([^:]*/\)?.*>>, <<\1>>)stamp-h<<>>dnl>>,
<<am_indx=1
for am_file in <<$1>>; do
  case " <<$>>CONFIG_HEADERS " in
  *" <<$>>am_file "*<<)>>
    echo timestamp > `echo <<$>>am_file | sed -e 's%:.*%%' -e 's%[^/]*$%%'`stamp-h$am_indx
    ;;
  esac
  am_indx=`expr "<<$>>am_indx" + 1`
done<<>>dnl>>)
changequote([,]))])

# Do all the work for Automake.  This macro actually does too much --
# some checks are only needed if your package does certain things.
# But this isn't really a big deal.

# serial 1

dnl Usage:
dnl AM_INIT_AUTOMAKE(package,version, [no-define])

AC_DEFUN(AM_INIT_AUTOMAKE,
[AC_REQUIRE([AC_PROG_INSTALL])
PACKAGE=[$1]
AC_SUBST(PACKAGE)
VERSION=[$2]
AC_SUBST(VERSION)
dnl test to see if srcdir already configured
if test "`cd $srcdir && pwd`" != "`pwd`" && test -f $srcdir/config.status; then
  AC_MSG_ERROR([source directory already configured; run "make distclean" there first])
fi
ifelse([$3],,
AC_DEFINE_UNQUOTED(PACKAGE, "$PACKAGE", [Name of package])
AC_DEFINE_UNQUOTED(VERSION, "$VERSION", [Version number of package]))
AC_REQUIRE([AM_SANITY_CHECK])
AC_REQUIRE([AC_ARG_PROGRAM])
dnl FIXME This is truly gross.
missing_dir=`cd $ac_aux_dir && pwd`
AM_MISSING_PROG(ACLOCAL, aclocal, $missing_dir)
AM_MISSING_PROG(AUTOCONF, autoconf, $missing_dir)
AM_MISSING_PROG(AUTOMAKE, automake, $missing_dir)
AM_MISSING_PROG(AUTOHEADER, autoheader, $missing_dir)
AM_MISSING_PROG(MAKEINFO, makeinfo, $missing_dir)
AC_REQUIRE([AC_PROG_MAKE_SET])])

#
# Check to make sure that the build environment is sane.
#

AC_DEFUN(AM_SANITY_CHECK,
[AC_MSG_CHECKING([whether build environment is sane])
# Just in case
sleep 1
echo timestamp > conftestfile
# Do `set' in a subshell so we don't clobber the current shell's
# arguments.  Must try -L first in case configure is actually a
# symlink; some systems play weird games with the mod time of symlinks
# (eg FreeBSD returns the mod time of the symlink's containing
# directory).
if (
   set X `ls -Lt $srcdir/configure conftestfile 2> /dev/null`
   if test "[$]*" = "X"; then
      # -L didn't work.
      set X `ls -t $srcdir/configure conftestfile`
   fi
   if test "[$]*" != "X $srcdir/configure conftestfile" \
      && test "[$]*" != "X conftestfile $srcdir/configure"; then

      # If neither matched, then we have a broken ls.  This can happen
      # if, for instance, CONFIG_SHELL is bash and it inherits a
      # broken ls alias from the environment.  This has actually
      # happened.  Such a system could not be considered "sane".
      AC_MSG_ERROR([ls -t appears to fail.  Make sure there is not a broken
alias in your environment])
   fi

   test "[$]2" = conftestfile
   )
then
   # Ok.
   :
else
   AC_MSG_ERROR([newly created file is older than distributed files!
Check your system clock])
fi
rm -f conftest*
AC_MSG_RESULT(yes)])

dnl AM_MISSING_PROG(NAME, PROGRAM, DIRECTORY)
dnl The program must properly implement --version.
AC_DEFUN(AM_MISSING_PROG,
[AC_MSG_CHECKING(for working $2)
# Run test in a subshell; some versions of sh will print an error if
# an executable is not found, even if stderr is redirected.
# Redirect stdin to placate older versions of autoconf.  Sigh.
if ($2 --version) < /dev/null > /dev/null 2>&1; then
   $1=$2
   AC_MSG_RESULT(found)
else
   $1="$3/missing $2"
   AC_MSG_RESULT(missing)
fi
AC_SUBST($1)])

