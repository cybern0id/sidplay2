/***************************************************************************
                          event.h  -  Event scheduler (based on alarm
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

#ifndef _event_h_
#define _event_h_

#include <stdio.h>
#include "sidtypes.h"

typedef uint event_clock_t;
#define EVENT_CONTEXT_MAX_PENDING_EVENTS 0x100

class EventContext;
class Event
{
private:
    friend  EventContext;
    const   char * const m_name;
    event_clock_t m_clk;

    /* Index into the pending event list.  If < 0 the
     * event is not pending.  */
    bool m_pending;

    /* Link to the next and previous events in the
       list.  */
    Event *m_next, *m_prev;

public:
    Event(const char * const name)
        : m_name(name),
          m_pending(false) {}

    virtual void event (void) = 0;
};


class EventContext
{
private:
    const   char * const m_name;
    event_clock_t m_eventClk, m_schedClk;
    Event  *m_pendingEvents;
    uint    m_pendingEventClk;

private:
    void dispatch (void)
    {
        Event &e = *m_pendingEvents;
        cancel (m_pendingEvents);
        //printf ("Event \"%s\"\n", e.m_name);
        e.event ();
    }

public:
    EventContext (const char * const name)
        : m_name(name),
          m_eventClk(0),
          m_schedClk(0),
          m_pendingEvents(NULL) {}

    void reset    (void);
    void timeWarp (int cycles);

    void schedule (Event *event, event_clock_t cycles)
    {
        uint clk = m_eventClk + cycles;
        if (event->m_pending)
            cancel (event);
        event->m_pending = true;
        event->m_clk     = clk;

        // Now put in the correct place so we don't need to keep
        // searching the list later.
        for (;;)
        {
            Event *e = m_pendingEvents;

            if (e == NULL)
            {   // New pending list
                event->m_prev = NULL;
                event->m_next = NULL;
                m_pendingEventClk = clk;
                m_pendingEvents   = event;
            } else {
                while (e->m_next != NULL)
                {
                    if (e->m_clk > clk)
                        break;
                    e = e->m_next;
                }

                if (!e->m_next)
                {
                    if (e->m_clk <= clk)
                    {   // Insert at end
                        e->m_next     = event;
                        event->m_prev = e;
                        event->m_next = NULL;
                        break;
                    }
                }

                // Inserting in middle or front
                // of existing list
                event->m_prev = e->m_prev;
                event->m_next = e;
                if (e->m_prev)
                    e->m_prev->m_next = event;
                else
                {   // At front
                    m_pendingEventClk = clk;
                    m_pendingEvents   = event;
                }
                e->m_prev = event;
            }
            break;
        }
    }

    void cancel (Event *event)
    {
        if (!event->m_pending)
            return;
        event->m_pending = false;

        // Remove event from pending list
        if (event->m_prev)
            event->m_prev->m_next = event->m_next;
        if (event->m_next)
            event->m_next->m_prev = event->m_prev;

        if (event == m_pendingEvents)
        {
            m_pendingEvents = event->m_next;
            if (m_pendingEvents != NULL)
                m_pendingEventClk = m_pendingEvents->m_clk;
        }
    }

    void clock (event_clock_t delta = 1)
    {
        m_schedClk  += delta;
        m_eventClk  += delta;
        if (m_pendingEvents != NULL)
        {
            // Dispatch events which have fired
            while (m_eventClk >= m_pendingEventClk)
            {
                dispatch ();
                if (m_pendingEvents == NULL)
                    break;
            }
        }

        while (m_eventClk > 0xfffff)
            timeWarp (-0xfffff);
    }

    event_clock_t getTime (void)
    {   return m_schedClk; }

    event_clock_t getTime (event_clock_t clock)
    {
        event_clock_t clk = m_schedClk - clock;
        if (clock <= m_schedClk)
            return clk;
        return ~clk + 1;
    } 
};

#endif // _event_h_
