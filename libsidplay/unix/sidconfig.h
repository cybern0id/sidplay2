/* sidconfig.h (template) */
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
#define SID_HAVE_STDINT_H

/* Define if your compiler supports AC99 header "stdbool.h" */
#define SID_HAVE_STDBOOL_H

/* Platform specific declaration to export interfaces for
   shared/dynamic libraries */
#define SID2_LIB_API

#endif /* _sidconfig_h_ */
