/* config.h (template) */
#ifndef _config_h_
#define _config_h_

/* @FOO@ : Define or undefine value FOO as appropriate. */

/* Define the sidbuilder modules at appropriate */
#undef HAVE_RESID_BUILDER
#undef HAVE_HARDSID_BUILDER

/* Define if your C++ compiler implements exception-handling.  */
#undef HAVE_EXCEPTIONS

/* Define if standard member ``ios::binary'' is called ``ios::bin''. */
#undef HAVE_IOS_BIN

/* Define if ``ios::openmode'' is supported. */
#undef HAVE_IOS_OPENMODE
@TOP@
/* Define if the C++ compiler supports BOOL */
#undef HAVE_BOOL

/* Define if you need the GNU extensions to compile */
#undef _GNU_SOURCE

/* Define supported audio driver */
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
