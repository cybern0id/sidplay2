/***************************************************************************
                          mos656x.cpp  -  Minimal VIC emulation
                             -------------------
    begin                : Wed May 21 2001
    copyright            : (C) 2001 by Simon White
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

#include "mos656x.h"


MOS656X::MOS656X ()
{
    chip  (MOS6569);
    reset ();
}

void MOS656X::reset ()
{
    icr        = idr = 0;
    raster_y   = rasters;
    raster_irq = 0;
    cycle      = cycles;
}

void MOS656X::chip (mos656x_model_t model)
{
    switch (model)
    {
    // Seems to be an older NTSC chip
    case MOS6567R56A:
        rasters = 262;
        cycles  = 64;
    break;

    // NTSC Chip
    case MOS6567R8:
        rasters = 263;
        cycles  = 65;
    break;

    // PAL Chip
    case MOS6569:
        rasters = 312;
        cycles  = 63;
    break;
    }
}

uint8_t MOS656X::read (uint_least8_t addr)
{
    if (addr > 0x3f) return 0;
    if (addr > 0x2e) return 0xff;

    switch (addr)
    {
    case 0x11:    // Control register 1 
        return (raster_y & 0x100) >> 1;
    case 0x12:    // Raster counter
        return raster_y & 0xFF; 
    case 0x19:    // IRQ flags 
        return idr; 
    case 0x1a:    // IRQ mask 
        return icr | 0xf0; 
    default: return regs[addr];
    }
}

void MOS656X::write (uint_least8_t addr, uint8_t data)
{
    if (addr > 0x3f) return;

    regs[addr] = data;
    switch (addr)
    {
    case 0x11: // Control register 1
    {
        uint_least16_t raster_new = (raster_irq & 0xff) |
                                    ((uint_least16_t) (data & 0x80) << 1); 
        if ((raster_irq != raster_new) && (raster_y == raster_new))
            trigger (MOS656X_INTERRUPT_RST);
        raster_irq = raster_new;
        break;
    }

    case 0x12: // Raster counter
    {
        uint_least16_t raster_new = (raster_irq & 0xff00) | data;
        if ((raster_irq != raster_new) && (raster_y == raster_new))
            trigger (MOS656X_INTERRUPT_RST);
        raster_irq = raster_new;
        break;
    }

    case 0x19: // IRQ flags
        idr &= ((~data & 0x0f) | 0x80);
        if (idr == 0x80)
            trigger (0);
    break;

    case 0x1a: // IRQ mask 
        icr = data & 0x0f; 
        trigger (icr & idr); 
    break;
    }
}


void MOS656X::trigger (int interrupt)
{
    if (!interrupt)
    {   // Clear any requested IRQs
        if (idr & MOS656X_INTERRUPT_REQUEST)
        {
            idr = 0;
            envClearIRQ ();
        }
        return;
    }

    idr |= interrupt;
    if (icr & idr)
    {
        if (!(idr & MOS656X_INTERRUPT_REQUEST))
        {
            idr |= MOS656X_INTERRUPT_REQUEST;
            envTriggerIRQ ();
        }
    }
}
