/***************************************************************************
                          mos6526.h  -  CIA timer to produce interrupts
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
 *  Revision 1.6  2001/07/14 16:46:59  s_a_white
 *  Sync with sidbuilder class project.
 *
 *  Revision 1.5  2001/07/14 13:03:33  s_a_white
 *  Now uses new component classes and event generation.
 *
 *  Revision 1.4  2001/03/25 19:50:16  s_a_white
 *  Timer B counts timer Aer a underflows correc
 *
 *  Revision 1.3  2001/03/23 23:20:29  s_a_white
 *  Removed redundant reset prototype.
 *
 *  Revision 1.2  2001/03/22 22:41:45  s_a_white
 *  Replaced tab characters
 *
 *  Revision 1.1  2001/03/21 22:41:45  s_a_white
 *  Non faked CIA emulation with NMI support.  Removal of Hacked VIC support
 *  off CIA timer.
 *
 *  Revision 1.7  2001/03/09 23:44:30  s_a_white
 *  Integrated more 6526 features.  All timer modes and interrupts correctly
 *  supported.
 *
 *  Revision 1.6  2001/02/21 22:07:10  s_a_white
 *  Prevent re-triggering of interrupt if it's already active.
 *
 *  Revision 1.5  2001/02/13 21:00:01  s_a_white
 *  Support for real interrupts.
 *
 *  Revision 1.3  2000/12/11 18:52:12  s_a_white
 *  Conversion to AC99
 *
 ***************************************************************************/

#ifndef _mos6526_h_
#define _mos6526_h_

#include "component.h"
#include "event.h"

class MOS6526: public component
{
private:
    static const char *credit;

protected:
    uint8_t regs[0x10];
    bool    cnt_high;

    // Timer A
    uint8_t cra, cra_latch;
    uint_least16_t ta, ta_latch;
    int  ta_state;

    // Timer B
    uint8_t crb;
    uint_least16_t tb, tb_latch;

    uint8_t icr, idr; // Interrupt Control Register
    event_clock_t m_accessClk;
    EventContext &event_context;

    class EventTa: public Event
    {
    private:
        MOS6526 &m_cia;
        void event (void) {m_cia.ta_event ();}

    public:
        EventTa (MOS6526 *cia)
            :Event("CIA Timer A"),
             m_cia(*cia) {}
    } event_ta;

    /*
    class EventStateMachineA: public Event
    {
    private:
        MOS6526 &m_cia;
        void event (void) {m_cia.cra_event ();}

    public:
        EventStateMachineA (MOS6526 *cia)
            :Event("CIA Timer A (State Machine)"),
             m_cia(*cia) {}
    } event_stateMachineA;
*/
    class EventTb: public Event
    {
    private:
        MOS6526 &m_cia;
        void event (void) {m_cia.tb_event ();}

    public:
        EventTb (MOS6526 *cia)
            :Event("CIA Timer B"),
             m_cia(*cia) {}
    } event_tb;

    friend class EventTa;
//    friend class EventStateMachineA;
    friend class EventTb;

protected:
    MOS6526 (EventContext *context);
    void ta_event  (void);
    void tb_event  (void);
    void trigger   (int irq);
//    void stateMachineA_event (void);

    // Environment Interface
    virtual void interrupt (const bool state) = 0;
    
public:
    // Component Standard Calls
    void    reset (void);
    uint8_t read  (const uint_least8_t addr);
    void    write (const uint_least8_t addr, const uint8_t data);
    const   char *credits (void) {return credit;}
};

#endif // _mos6526_h_

