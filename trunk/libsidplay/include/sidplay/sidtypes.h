/***************************************************************************
                          sidtypes.h  -  type definition file
                             -------------------
    begin                : Mon Jul 3 2000
    copyright            : (C) 2000 by Simon White
    email                : s_a_white@email.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _sidtypes_h_
#define _sidtypes_h_

#include "sidconfig.h"

#if SID_SIZEOF_CHAR == 1
#   if (SID_SIZEOF_SHORT_INT == 2) || (SID_SIZEOF_INT == 2)
#       if (SID_SIZEOF_INT == 4) || (SID_SIZEOF_LONG_INT == 4)
#           define SID_OPTIMISE_MEMORY_ACCESS
#       endif
#   endif
#endif

#if SID_SIZEOF_CHAR != 1
#   error Code cannot work correctly on this platform as no real 8 bit data type supported!
#endif

#ifndef SID_HAVE_BOOL
#   ifdef SID_HAVE_STDBOOL_H
#       include <stdbool.h>
#   else
        typedef int   bool;
#       define  true  1
#       define  false 0
#   endif /* SID_HAVE_STDBOOL_H */
#endif /* HAVE_BOOL */

#ifdef SID_HAVE_STDINT_H
#   include <stdint.h>
#else

    /* Wanted: Exactly 8-bit unsigned/signed (1 byte). */
    typedef signed char        int8_t;
    typedef unsigned char      uint8_t;

    /* Small types.  */
    /* Wanted: Atleast 8-bit unsigned/signed (1 byte). */
    typedef signed char        int_least8_t;
    typedef unsigned char      uint_least8_t;

    /* Wanted: Atleast 16-bit unsigned/signed (2 bytes). */
    #if SID_SIZEOF_SHORT_INT >= 2
    typedef short int          int_least16_t;
    typedef unsigned short int uint_least16_t;
    #else
    typedef int                int_least16_t;
    typedef unsigned int       uint_least16_t;
    #endif /* SID_SIZEOF_SHORT_INT */

    /* Wanted: Atleast 32-bit unsigned/signed (4 bytes). */
    #if SID_SIZEOF_INT >= 4
    typedef int                int_least32_t;
    typedef unsigned int       uint_least32_t;
    #else
    typedef long int           uint_least32_t;
    typedef unsigned long int  uint_least32_t;
    #endif /* SID_SIZEOF_INT */

#endif /* SID_HAVE_STDINT_H */

/* Custom types */
typedef unsigned int uint;
typedef float    float32_t;
typedef double   float64_t;

typedef int sid_fc_t[2];

#define FOREVER for(;;)
#define SWAP(x,y) ((x)^=(y)^=(x)^=(y))

typedef enum {sid2_left  = 0, sid2_mono,  sid2_stereo, sid2_right} sid2_playback_t;
typedef enum {sid2_envPS = 0, sid2_envTP, sid2_envBS,  sid2_envR } sid2_env_t;
typedef enum {SID2_MODEL_CORRECT, SID2_MOS6581, SID2_MOS8580}      sid2_model_t;
typedef enum {SID2_CLOCK_CORRECT, SID2_CLOCK_PAL, SID2_CLOCK_NTSC} sid2_clock_t;

/* Environment Modes
sid2_envPS = Playsid
sid2_envTP = Sidplay  - Transparent Rom
sid2_envBS = Sidplay  - Bankswitching
sid2_envR  = Sidplay2 - Real C64 Environment
*/

#endif /* _sidtypes_h_ */
