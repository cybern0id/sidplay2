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
 ***************************************************************************/

#include "fake6526.h"

fake6526::fake6526 ()
{
    defCount = 0xFFFF;
    reset ();
}

void fake6526::reset (void)
{
    locked = false;
    _count = (setCount = defCount);
    cra    = 0;
}

void fake6526::reset (uint_least16_t count)
{
    defCount = count;
    reset ();
}

uint8_t fake6526::read (uint_least8_t addr)
{
   if (addr > 0x0f) return 0;

   switch (addr)
   {
   case 0x04: return (uint8_t) _count;
   case 0x05: return (uint8_t) (_count >> 8);
   default:   return regs[addr];
   }
}

void fake6526::write (uint_least8_t addr, uint8_t data)
{
   static uint_least16_t latch = 0;
   if (addr > 0x0f) return;

   regs[addr] = data;
   if (locked) return; // Stop programming changing time interval
   switch (addr)
   {
   case 0x04:
       latch = (uint_least16_t) data;
   break;
   case 0x05:
       latch   |= (uint_least16_t) data << 8;
       setCount = latch;
   break;
   case 0x0e:
       cra = data & 0xef;  // (ms) mask strobe flag
       if (cra & 0x01)
           _count = setCount;
   break;
   default:
   break;
   }
}

