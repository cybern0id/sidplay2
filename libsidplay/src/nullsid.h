/***************************************************************************
                          nullsid.h  -  Null SID Emulation
                             -------------------
    begin                : Thurs Sep 20 2001
    copyright            : (C) 2001 by Simon White
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

#ifndef _nullsid_h_
#define _nullsid_h_

#include "sidbuilder.h"

class NullSID: public sidemu
{
public:
    NullSID () : sidemu (NULL) {;}

    // Standard component functions
    void    reset (void) { ; }
    uint8_t read  (const uint_least8_t) { return 0; }
    void    write (const uint_least8_t, const uint8_t) { ; }
    const   char *credits (void) { return ""; }
    const   char *error   (void) { return ""; }

    // Standard SID functions
    int_least32_t output (const uint_least8_t) { return 0; }
    void          voice  (const uint_least8_t, const uint_least8_t,
                          const bool) { ; }
    void          gain   (const uint_least8_t) { ; }
};

#endif // _nullsid_h_
