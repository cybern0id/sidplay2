/* Setup for Microsoft Visual C++ Version 5 */
#ifndef _config_h_
#define _config_h_

#define PACKAGE "libsidplay"
#define VERSION "2.0.6"

/* Operating System */
#define HAVE_MSWINDOWS

/* Define sound driver */
//#define HAVE_DIRECTX
#define HAVE_MMSYSTEM

/* Define if your processor stores words with the most significant
   byte first (like Motorola and SPARC, unlike Intel and VAX).  */
/* SID_WORDS_BIGENDIAN, SID_WORDS_LITTLEENDIAN  */
#define WORDS_LITTLEENDIAN

/* Define if your C++ compiler implements exception-handling.  */
/* #define HAVE_EXCEPTIONS */

/* Define if you support file names longer than 14 characters.  */
#define HAVE_LONG_FILE_NAMES

/* Define if you have the <strstrea.h> header file.  */
#define HAVE_STRSTREA_H

/* Defines to indicate that resid is in an non standard location.
   SID_HAVE_LOCAL_RESID indicates that an include path has been used
   to where the resid directory is located but does not include
   the resid directory name.
   SID_HAVE_USER_RESID indicates that a fully qualified include path
   has been provided that includes the resid directory name and therefore
   resids include files are available by calling them directly.
*/
#define HAVE_LOCAL_RESID

#endif // _config_h_
