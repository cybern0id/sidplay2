/***************************************************************************
                          fake6526.h  -  fake timer to produce interrupts
                             -------------------
    begin                : Wed Jun 7 2000
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
 *  Revision 1.5  2001/02/13 21:00:01  s_a_white
 *  Support for real interrupts.
 *
 *  Revision 1.3  2000/12/11 18:52:12  s_a_white
 *  Conversion to AC99
 *
 ***************************************************************************/

#ifndef _fake6526_h_
#define _fake6526_h_

#include "config.h"
#include "sidtypes.h"
#include "sidendian.h"
#include "sidenv.h"

class fake6526: public C64Environment
{
private:
    uint8_t        regs[0x10];
    bool           idr;
    uint8_t        cra;  // Timer A Control Register
    uint_least16_t ta_latch;
    uint_least16_t ta;   // Timer A Count (reduces to zero)
                         // value of -1 means off.
public:
    bool locked; // Prevent code changing CIA.

public:
    fake6526  ();
    ~fake6526 ();

    //Common:
    void clock (void);
    void reset (void);

    // Specific:
    void    reset (uint_least16_t count);
    uint8_t read  (uint_least8_t  addr);
    void    write (uint_least8_t  addr, uint8_t data);
};

inline void fake6526::clock (void)
{   // Make sure count is running
    if (!(cra & 0x01)) return;
    if (!ta--)
    {
        ta = ta_latch;
        if (cra & 0x08)
        {   // one shot, stop timer A
            cra &= (~0x01);
        }
        if (!idr)
        {
            idr = true;
            envTriggerIRQ ();
        }
    }
}

#endif // _fake6526_h_
