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

#ifndef _fake6526_h_
#define _fake6526_h_

#include "config.h"
#include "sidtypes.h"
#include "sidenv.h"

class fake6526: public C64Environment
{
public:
    bool locked; // Prevent code changing CIA.

public:
    fake6526  ();

    //Common:
    void clock (void);
    void reset (void);

    // Specific:
    void       reset (uword_sidt _count);
    ubyte_sidt read  (ubyte_sidt addr);
    void       write (ubyte_sidt addr, ubyte_sidt data);

private:
    ubyte_sidt regs[0x10];
    ubyte_sidt cra;      // Timer A Control Register
    uword_sidt defCount; // On a reset setCount will always go to defCount
                         // value of -1 means off.
    uword_sidt setCount; // The last set count either by reset or programming CIA
    uword_sidt count;    // Current count (reduces to zero)
};

inline void fake6526::clock (void)
{   // Make sure count is running
    if (!(cra & 0x01)) return;
    if (!--count)
    {
        count = setCount;
        envTriggerIRQ ();
    }
}

#endif // _fake6526_h_
