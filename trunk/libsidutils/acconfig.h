/* config.h (template) */
#ifndef _config_h_
#define _config_h_

/* @FOO@ : Define or undefine value FOO as appropriate. */

/* Operating System */
#define HAVE_UNIX

/* Define if your C++ compiler implements exception-handling.  */
#undef HAVE_EXCEPTIONS
@TOP@

/* Define if the C++ compiler supports BOOL */
#undef HAVE_BOOL

#undef VERSION

#undef PACKAGE

/* Define if you need the GNU extensions to compile */
#undef _GNU_SOURCE

@BOTTOM@
#endif /* _config_h_ */
