//
// /home/ms/source/sidplay/libsidplay/fformat/RCS/psid_.h,v
//

#ifndef PSID__H
#define PSID__H

#include <config.h>

#if defined(HAVE_FSTREAM)
  #include <fstream>
  using std::ofstream;
#else
  #include <fstream.h>
#endif
#include <string.h>

#include "mytypes.h"
#include "myendian.h"
#include "sidtune.h"
#include "hvscver.h"

extern HVSCVER HVSCversion_found;

// Header has been extended for 'RSID' format
// The following changes are present:
//     id = 'RSID'
//     version = 2 only
//     play, load and speed reserved 0
//     psidspecific flag reserved 0
//     init cannot be under ROMS/IO
//     load cannot be less than 0x0801 (start of basic)

struct psidHeader
{
    //
    // All values in big-endian order.
    //
    char id[4];          // 'PSID'
    ubyte version[2];    // 0x0001 or 0x0002
    ubyte data[2];       // 16-bit offset to binary data in file
    ubyte load[2];       // 16-bit C64 address to load file to
    ubyte init[2];       // 16-bit C64 address of init subroutine
    ubyte play[2];       // 16-bit C64 address of play subroutine
    ubyte songs[2];      // number of songs
    ubyte start[2];      // start song (1-256 !)
    ubyte speed[4];      // 32-bit speed info
                         // bit: 0=50 Hz, 1=CIA 1 Timer A (default: 60 Hz)
    char name[32];       // ASCII strings, 31 characters long and
    char author[32];     // terminated by a trailing zero
    char copyright[32];  //
    ubyte flags[2];             // only version 0x0002
    ubyte relocStartPage[1];    // only version 0x0002
    ubyte relocPages[1];        // only version 0x0002
    ubyte reserved[2];          // only version 0x0002
};

enum
{
    PSID_MUS       = 1 << 0,
    PSID_SPECIFIC  = 1 << 1,
    PSID_BASIC     = 1 << 1,
    PSID_CLOCK     = 3 << 2,
    PSID_SIDMODEL  = 3 << 4
};

enum
{
    PSID_CLOCK_UNKNOWN = 0,
    PSID_CLOCK_PAL     = 1 << 2,
    PSID_CLOCK_NTSC    = 1 << 3,
    PSID_CLOCK_ANY     = PSID_CLOCK_PAL | PSID_CLOCK_NTSC
};

enum
{
    PSID_SIDMODEL_UNKNOWN = 0,
    PSID_SIDMODEL_6581    = 1 << 4,
    PSID_SIDMODEL_8580    = 1 << 5,
    PSID_SIDMODEL_ANY     = PSID_SIDMODEL_6581 | PSID_SIDMODEL_8580
};

#endif
