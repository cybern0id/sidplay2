/***************************************************************************
                          sid6510c.h  -  Special MOS6510 to be fully
                                         compatible with sidplay
                             -------------------
    begin                : Thu May 11 2000
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 ***************************************************************************/

#ifndef _sid6510c_h_
#define _sid6510c_h_

#include "mos6510c.h"

class SID6510: public MOS6510
{
public:
    // Sidplay Specials
    bool SPWrapped;
    bool PCWrapped;

    SID6510 ();

    // Standard Functions
    void        reset (void);
    void        reset (uint8_t a, uint8_t x, uint8_t y);
    inline void clock (void);

private:
    inline void sid_FetchEffAddrDataByte (void);
    inline void sid_suppressError        (void);

    inline void sid_brk (void);
    inline void sid_jmp (void);
    inline void sid_rts (void);
};

#endif // _sid6510c_h_
