//
// /home/ms/source/sidplay/libsidplay/fformat/RCS/pp_.h,v
//

#ifndef PP__H
#define PP__H


#include "config.h"

#if defined(HAVE_FSTREAM)
  #include <fstream>
  using std::ifstream;
#else
  #include <fstream.h>
#endif
#if defined(HAVE_IOSTREAM)
  #include <iostream>
  using std::ios;
#else
  #include <iostream.h>
#endif
#include <string.h>
#include <limits.h>
#include "mytypes.h"
#include "myendian.h"


#endif
