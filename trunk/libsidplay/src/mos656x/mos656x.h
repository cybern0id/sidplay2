/***************************************************************************
                          mos656x.h  -  Minimal VIC emulation
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

#ifndef _mos656x_h_
#define _mos656x_h_

#include "component.h"
#include "event.h"

typedef enum
{
    MOS6567R56A, /* OLD NTSC CHIP */
    MOS6567R8,   /* NTSC */
    MOS6569      /* PAL */
} mos656x_model_t;


class MOS656X: public component
{
private:
    static const char *credit;

protected:
    uint8_t        regs[0x40];
    uint8_t        icr, idr;
    uint_least16_t yrasters, xrasters, raster_irq;
    event_clock_t  raster_cycle, raster_cycles;

    event_clock_t m_accessClk;
    EventContext &event_context;

    class EventRaster: public Event
    {
    private:
        MOS656X &m_vic;
        void event (void) {m_vic.rasterEvent ();}

    public:
        EventRaster (MOS656X *vic)
            :Event("VIC Raster"),
             m_vic(*vic) {}
    } event_raster;

    friend class EventRaster;

protected:
    MOS656X (EventContext *context);
    void    rasterEvent (void);
    void    trigger     (int irq);

    // Environment Interface
    virtual void interrupt (const bool state) = 0;

public:
    void    chip  (mos656x_model_t model);

    // Component Standard Calls
    void    reset (void);
    uint8_t read  (const uint_least8_t addr);
    void    write (const uint_least8_t addr, const uint8_t data);
    const   char *credits (void) {return credit;}
};


/***************************************************************************
 * Inline functions
 **************************************************************************/

enum
{
    MOS656X_INTERRUPT_RST     = 1 << 0,
    MOS656X_INTERRUPT_REQUEST = 1 << 7
};

#endif // _mos656x_h_

