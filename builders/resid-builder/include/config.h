/* include/config.h.  Generated automatically by configure.  */
/* include/config.h.in.  Generated automatically from configure.in by autoheader.  */
/* config.h (template) */
#ifndef _config_h_
#define _config_h_

/* @FOO@ : Define or undefine value FOO as appropriate. */

/* Define if your C++ compiler implements exception-handling.  */
#define HAVE_EXCEPTIONS 1

/* Defines to indicate that resid is in an non standard location.
   HAVE_LOCAL_RESID indicates that an include path has been used
   to where the resid directory is located but does not include
   the resid directory name.
   HAVE_USER_RESID indicates that a fully qualified include path
   has been provided that includes the resid directory name and therefore
   resids include files are available by calling them directly.
*/
/* #undef HAVE_LOCAL_RESID */
/* #undef HAVE_USER_RESID */



/* Define if the C++ compiler supports BOOL */
/* #undef HAVE_BOOL */


/* Define if you have the <dlfcn.h> header file. */
#define HAVE_DLFCN_H 1

/* Name of package */
#define PACKAGE "resid-builder"

/* Version number of package */
#define VERSION "1.0.0"
#endif /* _config_h_ */
