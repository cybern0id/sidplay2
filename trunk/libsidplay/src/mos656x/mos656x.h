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

#include "sidtypes.h"
#include "sidenv.h"

typedef enum
{
    MOS6567R56A, /* OLD NTSC CHIP */
    MOS6567R8,   /* NTSC */
    MOS6569      /* PAL */
} mos656x_model_t;


class MOS656X: public C64Environment
{
protected:
    uint8_t        regs[0x40];
    uint8_t        icr, idr;
    uint_least16_t raster_y, raster_irq;
    uint_least16_t rasters, cycles, cycle;

protected:
    void    rasterClock (void);
    void    trigger     (int interrupt);

public:
    MOS656X ();

    void    chip  (mos656x_model_t model);
    void    clock (void);
    uint8_t read  (uint_least8_t addr);
    void    reset (void);
    void    write (uint_least8_t addr, uint8_t data);
};


/***************************************************************************
 * Inline functions
 **************************************************************************/

enum
{
    MOS656X_INTERRUPT_RST     = 1 << 0,
    MOS656X_INTERRUPT_REQUEST = 1 << 7
};

inline void MOS656X::clock ()
{
    if (!--cycle)
        rasterClock ();
}

inline void MOS656X::rasterClock ()
{
    raster_y++;
    raster_y %= rasters;
    cycle = cycles;
    if (raster_y == raster_irq)
        trigger (MOS656X_INTERRUPT_RST);
}

#endif // _mos656x_h_
