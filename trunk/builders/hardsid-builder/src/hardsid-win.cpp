/***************************************************************************
             hardsid.cpp  -  Hardsid support interface.
                             Created from Jarnos original
                             Sidplay2 patch
                             -------------------
    begin                : Fri Dec 15 2000
    copyright            : (C) 2000-2002 by Simon White
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
 *  Revision 1.4  2002/02/17 17:24:51  s_a_white
 *  Updated for new reset interface.
 *
 *  Revision 1.3  2002/01/30 00:29:18  s_a_white
 *  Added realtime delays even when there is no accesses to
 *  the sid.  Prevents excessive CPU usage.
 *
 *  Revision 1.2  2002/01/29 21:47:35  s_a_white
 *  Constant fixed interval delay added to prevent emulation going fast when
 *  there are no writes to the sid.
 *
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 ***************************************************************************/

#include <stdio.h>
#include "config.h"
#include "hardsid.h"
#include "hardsid-emu.h"


extern HsidDLL2 hsid2;
const  uint HardSID::voices = HARDSID_VOICES;
uint   HardSID::sid = 0;
char   HardSID::credit[];


HardSID::HardSID (sidbuilder *builder)
:sidemu(builder),
 Event("HardSID Delay"),
 m_eventContext(NULL),
 m_instance(sid++),
 m_status(false)
{   
    *m_errorBuffer = '\0';
    if (m_instance >= hsid2.Devices ())
    {
        sprintf (m_errorBuffer, "HARDSID WARNING: System dosen't have enough SID chips.");
        return;
    }

    m_status = true;
    reset ();
    // Unmute all the voices
    hsid2.MuteAll (m_instance, false);
}


HardSID::~HardSID()
{
    sid--;
}

uint8_t HardSID::read (uint_least8_t addr)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk);
    m_accessClk += cycles;

    while (cycles > 0xFFFF)
    {
        hsid2.Delay ((BYTE) m_instance, 0xFFFF);
        cycles -= 0xFFFF;
    }

    return hsid2.Read ((BYTE) m_instance, (WORD) cycles,
                       (BYTE) addr);
}

void HardSID::write (uint_least8_t addr, uint8_t data)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk);
    m_accessClk += cycles;

    while (cycles > 0xFFFF)
    {
        hsid2.Delay ((BYTE) m_instance, 0xFFFF);
        cycles -= 0xFFFF;
    }

    hsid2.Write ((BYTE) m_instance, (WORD) cycles,
                 (BYTE) addr, (BYTE) data);
}

void HardSID::reset (uint8_t volume)
{   // Ok if no fifo, otherwise need hardware
    // reset to clear out fifo.
    
    m_accessClk = 0;
    hsid2.Reset ((BYTE) m_instance, volume);
    if (m_eventContext != NULL)
        m_eventContext->schedule (this, HARDSID_DELAY_CYCLES);
}

void HardSID::voice (uint_least8_t num, uint_least8_t volume,
                     bool mute)
{
    if (num >= voices)
        return;
    hsid2.Mute ((BYTE) m_instance, (BYTE) num, (BOOL) mute);
}

// Set execution environment and lock sid to it
bool HardSID::lock (c64env *env)
{
    if (env == NULL)
    {
        hsid2.Unlock (m_instance);
        m_eventContext->cancel (this);
        m_eventContext = NULL;
    }
    else
    {
        if (hsid2.Lock (m_instance) == FALSE)
            return false;
        m_eventContext = &env->context ();
        m_eventContext->schedule (this, HARDSID_DELAY_CYCLES);
    }
    return true;
}

void HardSID::event (void)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk);
    m_accessClk += cycles;
    if (cycles)
        hsid2.Delay ((BYTE) m_instance, (WORD) cycles);
    m_eventContext->schedule (this, HARDSID_DELAY_CYCLES);
}

// Disable/Enable SID filter
void HardSID::filter (bool enable)
{
    hsid2.Filter ((BYTE) m_instance, (BOOL) enable);
}

void HardSID::flush(void)
{
    hsid2.Flush ((BYTE) m_instance);
}