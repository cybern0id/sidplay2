/***************************************************************************
                          fake6526.cpp  -  description
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
 *  Revision 1.4  2000/12/11 18:52:12  s_a_white
 *  Conversion to AC99
 *
 ***************************************************************************/

#include "fake6526.h"

fake6526::fake6526 ()
{
    reset ();
}

void fake6526::reset (void)
{
    reset (0);
}

void fake6526::reset (uint_least16_t count)
{
    locked = false;
    ta     = ta_latch = count;
    cra    = 0;
	idr    = 0;
}

uint8_t fake6526::read (uint_least8_t addr)
{
    if (addr > 0x0f) return 0;

    switch (addr)
    {
    case 0x4: return endian_16lo8 (ta_latch);
    case 0x5: return endian_16hi8 (ta_latch);
    case 0xd:
        if (idr)
        {
            idr = false;
            envClearIRQ ();
        }
        // Deliberate run on
    default:  return regs[addr];
    }
}

void fake6526::write (uint_least8_t addr, uint8_t data)
{
    if (addr > 0x0f) return;

    regs[addr] = data;
    if (locked) return; // Stop programming changing time interval
    switch (addr)
    {
    case 0x4:
        endian_16lo8 (ta_latch, data);
    break;
    case 0x5:
        endian_16hi8 (ta_latch, data);
        if (!(cra & 0x01))
            ta = ta_latch;
    break;
    case 0x0e:
        // Check for forced load
        if (data & 0x10)
            ta = ta_latch;
        cra = data & 0xef;  // (ms) mask strobe flag
   break;
   default:
   break;
   }
}

