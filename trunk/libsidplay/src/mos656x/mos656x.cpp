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

#include "sidendian.h"
#include "mos656x.h"

const char *MOS656X::credit =
{   // Optional information
    "*MOS656X (VICII) Emulation:\0"
    "\tCopyright (C) 2001 Simon White <sidplay2@email.com>\0"
};


MOS656X::MOS656X (EventContext *context)
:event_context(*context),
 event_raster(this)
{
    chip  (MOS6569);
    reset ();
}

void MOS656X::reset ()
{
    icr          = idr = 0;
    raster_cycle = raster_cycles - 1;
    raster_irq   = 0;
    event_context.schedule (&event_raster, 1);
    m_accessClk  = 0;
}

void MOS656X::chip (mos656x_model_t model)
{
    switch (model)
    {
    // Seems to be an older NTSC chip
    case MOS6567R56A:
        yrasters = 262;
        xrasters = 64;
    break;

    // NTSC Chip
    case MOS6567R8:
        yrasters = 263;
        xrasters = 65;
    break;

    // PAL Chip
    case MOS6569:
        yrasters = 312;
        xrasters = 63;
    break;
    }

    raster_cycles = xrasters * yrasters;
}

uint8_t MOS656X::read (uint_least8_t addr)
{
    uint_least16_t raster_y;
    event_clock_t  cycles;
    if (addr > 0x3f) return 0;
    if (addr > 0x2e) return 0xff;
 
    // Sync up
    cycles        = event_context.getTime (m_accessClk);
    m_accessClk  += cycles;
    raster_cycle += cycles;
    raster_cycle %= raster_cycles;
    raster_y      = raster_cycle / xrasters;

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
    event_clock_t  cycles;
    if (addr > 0x3f) return;

    regs[addr] = data;

    // Sync up
    cycles        = event_context.getTime (m_accessClk);
    m_accessClk  += cycles;
    raster_cycle += cycles;
    raster_cycle %= raster_cycles;
    cycles = 0;

    switch (addr)
    {
    case 0x11: // Control register 1
        endian_16hi8 (raster_irq, data >> 7);
        goto MOS656X_write_generateEvent;

    case 0x12: // Raster counter
        endian_16lo8 (raster_irq, data);

MOS656X_write_generateEvent:
        if (raster_irq >= yrasters)
        {   // Oops, out of range
            event_context.cancel (&event_raster);
            break;
        }

        {   // Calculate cycles to event
            event_clock_t cyclesToEvent = raster_irq * xrasters;
            if (cyclesToEvent < raster_cycle)
                cyclesToEvent += (raster_cycles - raster_cycle);
            else
                cyclesToEvent -= raster_cycle;
            event_context.schedule (&event_raster, cyclesToEvent);
        }
    break;

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


void MOS656X::trigger (int irq)
{
    if (!irq)
    {   // Clear any requested IRQs
        if (idr & MOS656X_INTERRUPT_REQUEST)
            interrupt (false);
        idr = 0;
        return;
    }

    idr |= irq;
    if (icr & idr)
    {
        if (!(idr & MOS656X_INTERRUPT_REQUEST))
        {
            idr |= MOS656X_INTERRUPT_REQUEST;
            interrupt (true);
        }
    }
}

void MOS656X::rasterEvent (void)
{
    m_accessClk  = event_context.getTime ();
    raster_cycle = raster_irq * xrasters;
    trigger (MOS656X_INTERRUPT_RST);
    event_context.schedule (&event_raster, raster_cycles);
}
