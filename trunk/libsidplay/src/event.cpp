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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.3  2001/09/15 13:03:50  s_a_white
 *  timeWarp now zeros m_eventClk instead of m_pendingEventClk which
 *  fixes a inifinite loop problem when driving libsidplay1.
 *
 ***************************************************************************/


#include "event.h"
#define EVENT_TIMEWARP_COUNT 0x0FFFFF

EventContext::EventContext (const char * const name)
:m_name(name),
 m_pendingEvents(NULL),
 m_timeWarp(this)
{
    reset ();
}

// Usefull to prevent clock overflowing
void EventContext::timeWarp ()
{
    Event *e = m_pendingEvents;
    if (e != NULL)
    {
        event_clock_t cycles = m_eventClk;
        do
        {   // Reduce all event clocks and clip them
            // so none go negative
            event_clock_t clk = e->m_clk;
            e->m_clk = 0;
            if (clk >= cycles)
                e->m_clk = clk - cycles;
            e = e->m_next;
        } while (e != NULL);
        m_pendingEventClk = m_pendingEvents->m_clk;
    }
    m_eventClk = 0;
    // Re-schedule the next timeWarp
    schedule (&m_timeWarp, EVENT_TIMEWARP_COUNT);
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
    timeWarp ();
}

// Add event to ordered pending queue
void EventContext::schedule (Event *event, event_clock_t cycles)
{
    uint clk = m_eventClk + cycles;
    if (event->m_pending)
        cancel (event);
    event->m_pending = true;
    event->m_clk     = clk;

    {   // Now put in the correct place so we don't need to keep
        // searching the list later.
        Event *e     = m_pendingEvents;
        Event *ePrev = NULL;
        for (;e != NULL; ePrev = e, e = e->m_next)
        {
            if (e->m_clk > clk)
                break;
        }
        event->m_next = e;
        event->m_prev = NULL;

        if (e != NULL)
            e->m_prev = event;
        if (ePrev != NULL)
        {
            ePrev->m_next = event;
            event->m_prev = ePrev;
        }
        else
        {   // At front
            m_pendingEventClk = clk;
            m_pendingEvents   = event;
        }
    }
}

// Cancel a pending event
void EventContext::cancel (Event *event)
{
    if (event == m_pendingEvents)
        cancelPending (*event);
    else if (event->m_pending)
    {
        event->m_pending = false;
        // Remove event from pending list
        if (event->m_prev)
            event->m_prev->m_next = event->m_next;
        if (event->m_next)
            event->m_next->m_prev = event->m_prev;
    }
}
