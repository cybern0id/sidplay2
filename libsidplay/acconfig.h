/* config.h (template) */
#ifndef _config_h_
#define _config_h_

/* @FOO@ : Define or undefine value FOO as appropriate. */

/* Operating System */
#define HAVE_UNIX

/* Define if your C++ compiler implements exception-handling.  */
#undef HAVE_EXCEPTIONS

/* Define if you have the <strstrea.h> header file.  */
#undef HAVE_STRSTREA_H

/* Define if you have the strncasecmp function.  */
#undef HAVE_STRNCASECMP

/* Define if you have the strcasecmp function.  */
#undef HAVE_STRCASECMP

/* Define if standard member ``ios::binary'' is called ``ios::bin''. */
#undef HAVE_IOS_BIN

/* Define if standard member function ``fstream::is_open()'' is not available.  */
#undef DONT_HAVE_IS_OPEN

/* Define whether istream member function ``seekg(streamoff,seekdir).offset()''
   should be used instead of standard ``seekg(streamoff,seekdir); tellg()''.
*/
#undef HAVE_SEEKG_OFFSET

/* Defines to indicate that resid is in an non standard location.
   HAVE_LOCAL_RESID indicates that an include path has been used
   to where the resid directory is located but does not include
   the resid directory name.
   HAVE_USER_RESID indicates that a fully qualified include path
   has been provided that includes the resid directory name and therefore
   resids include files are available by calling them directly.
*/
#undef HAVE_LOCAL_RESID
#undef HAVE_USER_RESID

/* Define if your processor stores words with the least significant
   byte first (like Intel and VAX).  */
#undef WORDS_LITTLEENDIAN
@TOP@

/* Define if the C++ compiler supports BOOL */
#undef HAVE_BOOL

#undef VERSION

#undef PACKAGE

/* Define if you need the GNU extensions to compile */
#undef _GNU_SOURCE

/* Defines Supported Unix Audio Drivers */
#undef HAVE_OSS
#undef HAVE_HPUX
#undef HAVE_SGI
#undef HAVE_SUNOS
#undef HAVE_MMSYSTEM
#undef HAVE_WAV_ONLY

@BOTTOM@
#endif /* _config_h_ */
