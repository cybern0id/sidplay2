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

#define  _COMPONENT_
#include "fake6526.h"

fake6526::fake6526 ()
{
    defCount = 0xFFFF;
    reset ();
}

void fake6526::reset (void)
{
    locked = false;
    count  = (setCount = defCount);
    cra    = 0;
}

void fake6526::reset (uword_sidt _count)
{
    defCount = _count;
    reset ();
}

ubyte_sidt fake6526::read (ubyte_sidt addr)
{
   if (addr > 0x0f) return 0;

   switch (addr)
   {
   case 0x04: return (ubyte_sidt) count;
   case 0x05: return (ubyte_sidt) (count >> 8);
   default:   return regs[addr];
   }
}

void fake6526::write (ubyte_sidt addr, ubyte_sidt data)
{
   static uword_sidt latch = 0;
   if (addr > 0x0f) return;

   regs[addr] = data;
   if (locked) return; // Stop programming changing time interval
   switch (addr)
   {
   case 0x04:
       latch = (uword_sidt) data;
   break;
   case 0x05:
       latch   |= (uword_sidt) (data << 8);
       setCount = latch;
   break;
   case 0x0e:
       cra = data & 0xef;  // (ms) mask strobe flag
       if (cra & 0x01)
           count = setCount;
   break;
   default:
   break;
   }
}

