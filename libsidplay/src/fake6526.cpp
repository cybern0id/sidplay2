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
 *  Revision 1.7  2001/02/21 22:07:10  s_a_white
 *  Prevent re-triggering of interrupt if it's already active.
 *
 *  Revision 1.6  2001/02/13 21:00:01  s_a_white
 *  Support for real interrupts.
 *
 *  Revision 1.4  2000/12/11 18:52:12  s_a_white
 *  Conversion to AC99
 *
 ***************************************************************************/

#include "fake6526.h"

enum
{
	INTERRUPT_TA      = 1 << 0,
	INTERRUPT_TB      = 1 << 1,
	INTERRUPT_ALARM   = 1 << 2,
	INTERRUPT_SP      = 1 << 3,
	INTERRUPT_FLAG    = 1 << 4,
	INTERRUPT_REQUEST = 1 << 7
};


fake6526::fake6526 ()
: idr(0)
{
    reset ();
}

fake6526::~fake6526 ()
{
	reset ();
}

void fake6526::reset (void)
{
    reset (0xffff);
}

void fake6526::reset (uint_least16_t count)
{
    locked = false;
    ta     = ta_latch = count;
    cra    = 0;
    // Clear off any IRQs
	trigger (0);
//	icr = (idr = 0);
    // Temporary bodge
	idr = 0;
	icr = 0x1f;
}

uint8_t fake6526::read (uint_least8_t addr)
{
    if (addr > 0x0f) return 0;

    switch (addr)
    {
    case 0x4: return endian_16lo8 (ta_latch);
    case 0x5: return endian_16hi8 (ta_latch);

	case 0xd:
	{   // Clear IRQs, and return interrupt
		// data register
		uint8_t ret = idr;
		trigger (0);
		return ret;
	}

	case 0x0e: return cra;
	case 0x0f: return crb;
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

	case 0xd:
		if (data & 0x80)
			icr |= data & 0x1f;
		else
			icr &= ~data;
		trigger (idr);
	break;

    case 0x0e:
        // Check for forced load
        if (data & 0x10)
            ta = ta_latch;
        cra = data & 0xef;  // (ms) mask strobe flag
    break;

    case 0x0f:
        // Check for forced load
        if (data & 0x10)
            tb = tb_latch;
        crb = data & 0xef;
    break;

    default:
    break;
    }
}

void fake6526::trigger (int interrupt)
{
	if (!interrupt)
	{   // Clear any requested IRQs
		if (idr & INTERRUPT_REQUEST)
		{
			idr = 0;
			envClearIRQ ();
		}
		return;
	}

	idr |= interrupt;
	if (icr & idr)
	{
		if (!(idr & INTERRUPT_REQUEST))
		{
			idr |= INTERRUPT_REQUEST;
			envTriggerIRQ ();
		}
	}
}

bool fake6526::ta_clock (void)
{
	if (!ta--)
	{   // Underflow 
		ta = ta_latch;
		if (cra & 0x08)
		{   // one shot, stop timer A
			cra &= (~0x01);
		}
		trigger (INTERRUPT_TA);
		return true;
	}
	return false;
}

void fake6526::tb_clock (bool ta_underflow)
{
	// All timer
	switch (crb & 0x61)
	{
	case 0x01: goto MOS6526_tb_clock;
	case 0x21:
//		if (cnt_edge)
//			goto MOS6526_tb_clock;
	break;
	case 0x41:
		if (ta_underflow)
			goto MOS6526_tb_clock;
	break;
	case 0x61:
//		if (ta_underflow && cnt_high)
//			goto MOS6526_tb_clock;
	break;
	default:
		break;
	}
return;

MOS6526_tb_clock:
	if (!tb--)
	{   // Underflow 
		tb = tb_latch;
		if (crb & 0x08)
		{   // one shot, stop timer
			crb &= (~0x01);
		}
		trigger (INTERRUPT_TB);
	}
}
