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
 m_eventContext(NULL),
 m_instance(sid++),
 m_status(false),
 m_locked(false)
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

uint8_t HardSID::read (const uint_least8_t addr)
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

void HardSID::write (const uint_least8_t addr, const uint8_t data)
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

void HardSID::reset (void)
{   // Ok if no fifo, otherwise need hardware
    // reset to clear out fifo.
    
    m_accessClk = 0;
    hsid2.Reset ((BYTE) m_instance);
}

void HardSID::voice (const uint_least8_t num, const uint_least8_t volume,
                     const bool mute)
{
    if (num >= voices)
        return;
    hsid2.Mute ((BYTE) m_instance, (BYTE) num, (BOOL) mute);
}

// Set execution environment and lock sid to it
void HardSID::lock (c64env *env)
{
    if (env == NULL)
    {
        m_locked = false;
        m_eventContext = NULL;
    }
    else
    {
        m_locked = true;
        m_eventContext = &env->context ();
    }
}

// Disable/Enable SID filter
void HardSID::filter (const bool enable)
{
    hsid2.Filter ((BYTE) m_instance, (BOOL) enable);
}

void HardSID::flush(void)
{
    // Not implemented
}
