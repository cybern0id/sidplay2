/***************************************************************************
                          sid6526.cpp  -  description
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
 *  Revision 1.5  2002/03/03 22:03:49  s_a_white
 *  Tidy.
 *
 *  Revision 1.4  2001/10/02 18:01:36  s_a_white
 *  Support for cleaned c64env.
 *
 *  Revision 1.3  2001/09/18 02:22:37  jpaana
 *  Fixed include filename to lowercase
 *
 *  Revision 1.2  2001/09/03 22:24:09  s_a_white
 *  New counts for timer A are correctly formed.
 *
 *  Revision 1.1  2001/09/01 11:11:19  s_a_white
 *  This is the old fake6526 code required for sidplay1 environment modes.
 *
 ***************************************************************************/

#include <time.h>
#include "sidendian.h"
#include "sid6526.h"

const char * const SID6526::credit =
{   // Optional information
    "*SID6526 (SIDPlay1 Fake CIA) Emulation:\0"
    "\tCopyright (C) 2001 Simon White <sidplay2@email.com>\0"
};

SID6526::SID6526 (c64env *env)
:m_env(*env),
 m_eventContext(m_env.context ()),
 rnd(0),
 m_taEvent(*this)
{
    clock (0xffff);
    reset ();
}

void SID6526::reset (void)
{
    locked = false;
    ta   = ta_latch = m_count;
    cra  = 0;
    rnd += time(NULL) & 0xff;
    m_accessClk = 0;
}

uint8_t SID6526::read (uint_least8_t addr)
{
    if (addr > 0x0f)
        return 0;

    switch (addr)
    {
    case 0x04:
    case 0x05:
    case 0x11:
    case 0x12:
        rnd = rnd * 13 + 1;
        return (uint8_t) (rnd >> 3);
    break;
    default:
        return regs[addr];
    }
}

void SID6526::write (uint_least8_t addr, uint8_t data)
{
    if (addr > 0x0f)
        return;

    regs[addr] = data;

    if (locked)
        return; // Stop program changing time interval

    {   // Sync up timer
        event_clock_t cycles;
        cycles       = m_eventContext.getTime (m_accessClk);
        m_accessClk += cycles;
        ta          -= cycles;
    }

    switch (addr)
    {
    case 0x4: endian_16lo8 (ta_latch, data); break;
    case 0x5:
        endian_16hi8 (ta_latch, data);
        if (!(cra & 0x01)) // Reload timer if stopped
            ta = ta_latch;
    break;
    case 0x0e:
        cra = data;
        if (data & 0x10)
        {
            cra &= (~0x10);
            ta   = ta_latch;
        }
        m_eventContext.schedule (&m_taEvent, (event_clock_t) ta + 1);
    break;
    default:
    break;
    }
}

void SID6526::event (void)
{   // Timer Modes
    m_accessClk = m_eventContext.getTime ();
    ta = ta_latch;
    m_eventContext.schedule (&m_taEvent, (event_clock_t) ta + 1);
    m_env.interruptIRQ (true);
}
