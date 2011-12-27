/***************************************************************************
                          acid64-cmd.h
                             -------------------
    begin                : Sat Dec 24 2011
    copyright            : (C) 2011 by Simon White
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

#ifndef _acid64_cmd_h_
#define _acid64_cmd_h_

#include <sidplay/event.h>

class Acid64Cmd
{
public:
    virtual ~Acid64Cmd () { ; }
    virtual void    delay (event_clock_t cycles) = 0;
    virtual void    write (event_clock_t cycles, uint_least8_t addr, uint8_t data) = 0;
    virtual uint8_t read  (event_clock_t cycles, uint_least8_t addr) = 0;
};

#endif // _acid64_cmd_h_