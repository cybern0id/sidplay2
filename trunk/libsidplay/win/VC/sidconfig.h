/* Setup for Microsoft Visual C++ Version 5 */
#ifndef _sidconfig_h_
#define _sidconfig_h_

/* Define the sizeof of types in bytes */
#define SID_SIZEOF_CHAR      1
#define SID_SIZEOF_SHORT_INT 2
#define SID_SIZEOF_INT       4
#define SID_SIZEOF_LONG_INT  4

/* Define if your compiler supports type "bool".
   If not, a user-defined signed integral type will be used.  */
#define SID_HAVE_BOOL

/* Define if your compiler supports AC99 header "stdint.h" */
//#define SID_HAVE_STDINT_H

/* Define if your compiler supports AC99 header "stdbool.h" */
//#define SID_HAVE_STDBOOL_H

#ifdef _MSC_VER
    /* Microsoft specific compiler problem */
#   define SID_HAVE_BAD_COMPILER
#endif

#ifndef SID2_LIB_API
#   define SID2_LIB_API __declspec(dllimport)
#endif

#endif // _sidconfig_h_
