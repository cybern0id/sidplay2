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
 *  Revision 1.3  2001/02/13 21:02:25  s_a_white
 *  Small tidy up and possibly a small performace increase.
 *
 *  Revision 1.2  2000/12/11 19:04:32  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

#ifndef _sid6510c_h_
#define _sid6510c_h_

#include "mos6510c.h"


class SID6510: public MOS6510
{
private:
    // Sidplay Specials
    bool status;

public:
    SID6510 ();

    // Standard Functions
    void reset (void);
    void reset (uint8_t a, uint8_t x, uint8_t y);
    void clock (void);

    operator bool() { return status; }

private:
    inline void sid_FetchEffAddrDataByte (void);
    inline void sid_suppressError        (void);

    inline void sid_brk (void);
    inline void sid_jmp (void);
    inline void sid_rts (void);
};


inline void SID6510::clock (void)
{   // Call inherited emulate
    MOS6510::clock ();

    // Sidplay requires that we check to see if
    // the stack has overflowed.  This then returns
    // control back to sidplay so music can be played
    status &= (endian_16hi8  (Register_StackPointer)   == SP_PAGE);
    status &= (endian_32hi16 (Register_ProgramCounter) == 0);
}

#endif // _sid6510c_h_
