/***************************************************************************
                          sidtypes.h  -  description
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

#ifndef SID_HAVE_BOOL
    typedef int   bool;
#   define  true  1
#   define  false 0
#endif /* SID_HAVE_BOOL */

/* Wanted: 8-bit unsigned/signed (1 bytes). */
typedef unsigned char ubyte_sidt;
typedef signed   char sbyte_sidt;

/* Wanted: 16-bit unsigned/signed (2 bytes). */
#if SID_SIZEOF_SHORT_INT >= 2
    typedef unsigned short uword_sidt;
    typedef signed   short sword_sidt;
#else
    typedef unsigned int   uword_sidt;
    typedef signed   int   sword_sidt;
#endif /* SID_SIZEOF_SHORT_INT */

/* Wanted: 32-bit unsigned/signed (4 bytes). */
#if SID_SIZEOF_INT >= 4
    typedef unsigned int   udword_sidt;
    typedef signed   int   sdword_sidt;
#else
    typedef unsigned long  udword_sidt;
    typedef signed   long  sdword_sidt;
#endif /* SID_SIZEOF_INT */

#define FOREVER for(;;)

#endif /* _sidtypes_h_ */
