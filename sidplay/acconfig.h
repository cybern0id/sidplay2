/* config.h (template) */
#ifndef _config_h_
#define _config_h_

/* @FOO@ : Define or undefine value FOO as appropriate. */

/* Operating System */
#define HAVE_UNIX

/* Define if your C++ compiler implements exception-handling.  */
#undef HAVE_EXCEPTIONS

/* Define if standard member ``ios::binary'' is called ``ios::bin''. */
#undef HAVE_IOS_BIN
@TOP@

/* Define if the C++ compiler supports BOOL */
#undef HAVE_BOOL

#undef VERSION

#undef PACKAGE

/* Define if you need the GNU extensions to compile */
#undef _GNU_SOURCE

/* Defines Supported Unix Audio Drivers */
#undef HAVE_HARDSID
#undef HAVE_ALSA
#undef HAVE_IRIX
#undef HAVE_HPUX
#undef HAVE_MMSYSTEM
#undef HAVE_OSS
#undef HAVE_SGI
#undef HAVE_SUNOS
#undef HAVE_WAV_ONLY

@BOTTOM@
#endif /* _config_h_ */
