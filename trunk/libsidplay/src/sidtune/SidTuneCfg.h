/* SidTuneCfg.h (template) */

#ifndef SIDTUNECFG_H
#define SIDTUNECFG_H

#include "config.h"

/* --------------------------------------------------------------------------
 * Don't touch these!
 * --------------------------------------------------------------------------
 */
#undef SIDTUNE_NO_STDIN_LOADER
#undef SIDTUNE_REJECT_UNKNOWN_FIELDS


/* Define the file/path separator(s) that your filesystem uses:
   SID_FS_IS_COLON_AND_BACKSLASH, SID_FS_IS_COLON_AND_SLASH,
   SID_FS_IS_BACKSLASH, SID_FS_IS_COLON, SID_FS_IS_SLASH  */
#if defined(HAVE_MSWINDOWS) || defined(HAVE_MSDOS) || defined(HAVE_OS2)
  #define SID_FS_IS_COLON_AND_BACKSLASH_AND_SLASH
#elif defined(HAVE_MACOS)
  #define SID_FS_IS_COLON
#elif defined(HAVE_AMIGAOS)
  #define SID_FS_IS_COLON_AND_SLASH
#else
  #define SID_FS_IS_SLASH
#endif	  

#endif  /* SIDTUNECFG_H */
