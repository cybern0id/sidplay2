/***************************************************************************
                          mos6526.h  -  CIA timer to produce interrupts
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
 *  Revision 1.2  2001/03/22 22:41:45  s_a_white
 *  Replaced tab characters
 *
 *  Revision 1.1  2001/03/21 22:41:45  s_a_white
 *  Non faked CIA emulation with NMI support.  Removal of Hacked VIC support
 *  off CIA timer.
 *
 *  Revision 1.7  2001/03/09 23:44:30  s_a_white
 *  Integrated more 6526 features.  All timer modes and interrupts correctly
 *  supported.
 *
 *  Revision 1.6  2001/02/21 22:07:10  s_a_white
 *  Prevent re-triggering of interrupt if it's already active.
 *
 *  Revision 1.5  2001/02/13 21:00:01  s_a_white
 *  Support for real interrupts.
 *
 *  Revision 1.3  2000/12/11 18:52:12  s_a_white
 *  Conversion to AC99
 *
 ***************************************************************************/

#ifndef _mos6526_h_
#define _mos6526_h_

#include "config.h"
#include "sidtypes.h"
#include "sidendian.h"
#include "sidenv.h"

class MOS6526: public C64Environment
{
protected:
    uint8_t regs[0x10];

    // Timer A
    uint8_t cra;
    uint_least16_t ta, ta_latch;

    // Timer B
    uint8_t crb;
    uint_least16_t tb, tb_latch;

    uint8_t icr, idr; // Interrupt Control Register
    const bool nmi;

protected:
    bool ta_clock (void);
    void tb_clock (bool ta_underflow);
    void trigger  (int interrupt);

public:
    MOS6526  (const bool isnmi);
    ~MOS6526 ();

    //Common:
    void    clock (void);
    void    reset (void);
    uint8_t read  (uint_least8_t  addr);
    void    write (uint_least8_t  addr, uint8_t data);
};

inline void MOS6526::clock (void)
{
    bool ta_underflow = false;
    if ((cra & 0x21) == 0x01)
        (void) ta_clock ();

    if (crb & 0x01)
    {
        if ((crb & 0x61) != 0x21)
            tb_clock (ta_underflow);
    }
}

#endif // _mos6526_h_
