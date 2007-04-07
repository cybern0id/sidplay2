/* Define your operating system. Leave undefined if none applies.  */
#undef HAVE_LINUX
#undef HAVE_UNIX
#undef HAVE_MSWINDOWS
#undef HAVE_MACOS
#undef HAVE_OS2
#undef HAVE_AMIGAOS
#undef HAVE_MSDOS
#undef HAVE_BEOS

@TOP@

/* Define whether the compiler supports a built-in bool type.  */
#undef HAVE_BOOL

/* Define whether Standard C++ I/O library has openmode ios::bin
   instead of ios::binary.  */
#undef HAVE_IOS_BIN

@BOTTOM@


/* Define the file/path separator(s) that your filesystem uses:
   FS_IS_COLON_AND_BACKSLASH, FS_IS_COLON_AND_SLASH, FS_IS_BACKSLASH,
   FS_IS_COLON, FS_IS_SLASH  */
#if defined(HAVE_MSWINDOWS) || defined(HAVE_MSDOS) || defined(HAVE_OS2)
  #define FS_IS_COLON_AND_BACKSLASH
#elif defined(HAVE_MACOS)
  #define FS_IS_COLON
#elif defined(HAVE_AMIGAOS)
  #define FS_IS_COLON_AND_SLASH
#elif defined(HAVE_LINUX) || defined(HAVE_UNIX) || defined(HAVE_BEOS)
  #define FS_IS_SLASH
#else
  #error Please select an operating system in ``config.h''.
#endif	  

/* Define if standard member function ``fstream::is_open()'' is not available.  */
#undef DONT_HAVE_IS_OPEN

/* Define whether istream member function ``seekg(streamoff,seekdir).offset()''
   should be used instead of standard ``seekg(streamoff,seekdir); tellg()''.  */
#undef HAVE_SEEKG_OFFSET

/* --------------------------------------------------------------------------
 * Hardware-specific speed optimizations.
 * Check here for things you can configure manually only.
 * --------------------------------------------------------------------------
 *
 * Caution: This may not work on every hardware and therefore can result in
 * trouble. Some hardware-specific speed optimizations use a union to access
 * integer fixpoint operands in memory. An assumption is made about the
 * hardware and software architecture and therefore is considered a hack!
 * But try it in need for speed. You will notice if it doesn't work ;)
 *
 * This option enables direct byte-access of little/big endian values in
 * memory structures or arrays, disregarding alignment.
 * On SPARC CPUs don't define these. Else you get a ``bus error'' due
 * to 32-bit alignment of the CPU.  */
#undef OPTIMIZE_ENDIAN_ACCESS
	
/* This option is highly used and tested. A failing little endian system
 * has not been reported so far.  */
#undef DIRECT_FIXPOINT

/* --------------------------------------------------------------------------
 * Don't touch these!
 * --------------------------------------------------------------------------
 */
#define NO_STDIN_LOADER 1

