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
 ***************************************************************************/

#ifndef _fake6526_h_
#define _fake6526_h_

#include "config.h"
#include "sidtypes.h"
#include "sidenv.h"

class fake6526: public C64Environment
{
private:
    uint8_t regs[0x10];
    uint8_t cra;             // Timer A Control Register
    uint_least16_t defCount; // On a reset setCount will always go to defCount
                             // value of -1 means off.
    uint_least16_t setCount; // The last set count either by reset or programming CIA
    uint_least16_t _count;   // Current count (reduces to zero)

public:
    bool locked; // Prevent code changing CIA.

public:
    fake6526  ();

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
    if (!--_count)
    {
        _count = setCount;
        envTriggerIRQ ();
    }
}

#endif // _fake6526_h_
