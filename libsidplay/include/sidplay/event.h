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

typedef uint_fast32_t event_clock_t;
#define EVENT_CONTEXT_MAX_PENDING_EVENTS 0x100

class EventContext;
class SID_EXTERN Event
{
private:
    friend  EventContext;
    const   char * const m_name;
    event_clock_t m_clk;

    /* This variable is set by the event context
       when it is scheduled */
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


class SID_EXTERN EventContext
{
private:
    const   char * const m_name;
    event_clock_t m_eventClk, m_schedClk;
    Event  *m_pendingEvents;
    uint    m_pendingEventClk;

    class SID_EXTERN EventTimeWarp: public Event
    {
    private:
        EventContext &m_eventContext;

        void event (void)
        {
            m_eventContext.timeWarp ();
        }

    public:
        EventTimeWarp (EventContext *context)
        :Event("Time Warp"),
         m_eventContext(*context)
        {;}
    } m_timeWarp;
    friend EventTimeWarp;

private:
    void timeWarp (void);
    void dispatch (void)
    {
        Event &e = *m_pendingEvents;
        cancelPending (e);
        //printf ("Event \"%s\"\n", e.m_name);
        e.event ();
    }

    void cancelPending (Event &event)
    {
        event.m_pending = false;
        m_pendingEvents = event.m_next;
        if (!m_pendingEvents)
            return;
        m_pendingEvents->m_prev = NULL;
        m_pendingEventClk = m_pendingEvents->m_clk;
    }

public:
    EventContext  (const char * const name);
    void cancel   (Event *event);
    void reset    (void);
    void schedule (Event *event, event_clock_t cycles);

    void clock (event_clock_t delta = 1)
    {
        m_schedClk  += delta;
        m_eventClk  += delta;
        while (m_pendingEvents != NULL)
        {   // Dispatch events which have fired
            if (m_eventClk < m_pendingEventClk)
                break;
            dispatch ();
        }
    }

    event_clock_t getTime (void) const
    {   return m_schedClk; }
    event_clock_t getTime (event_clock_t clock) const
    {   return m_schedClk - clock; }
};

#endif // _event_h_
