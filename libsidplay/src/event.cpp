/***************************************************************************
                          event.cpp  -  Event schdeduler (based on alarm
                                        from Vice)
                             -------------------
    begin                : Wed May 9 2001
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


#include "event.h"

// Usefull to prevent clock overflowing
void EventContext::timeWarp (int cycles)
{
    Event *e = m_pendingEvents;
    while (e != NULL)
    {
        e->m_clk += cycles;
        e = e->m_next;
    }
    m_pendingEventClk += cycles;
    m_eventClk        += cycles;
}

void EventContext::reset (void)
{    // Remove all events
    Event *e = m_pendingEvents;
    while (e != NULL)
    {
        e->m_pending = false;
        e = e->m_next;
    }
    m_pendingEvents   = NULL;
    m_pendingEventClk = m_eventClk = m_schedClk = 0;
}
