/***************************************************************************
                          acid64-emu.cpp
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

#include "config.h"

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#include "acid64-builder_p.h"
#include "acid64-cmd.h"
#include "acid64-emu.h"

static const maxDelayCycles = 0xffff;

Acid64Emu::Acid64Emu (Acid64BuilderPrivate &builder, Acid64Cmd &cmd)
:CoEmulation<ISidEmulation>("Acid64Emu", builder.iunknown())
,CoAggregate<ISidMixer>(*iunknown())
,Event("Acid64 Max Delay")
,m_cmd(cmd)
,m_context(NULL)
,m_phase(EVENT_CLOCK_PHI1)
,m_status(true)
{
    m_error = "N/A";
    reset (0);
}

// Standard component options
const char *Acid64Emu::credits (void)
{
    SidIPtr<ISidBuilder> builder(this->builder());
    return builder->credits ();
}

void Acid64Emu::reset (uint8_t)
{
    m_accessClk = 0;
}

uint8_t Acid64Emu::read (uint_least8_t addr)
{
    event_clock_t cycles = m_context->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;
    return m_cmd.read (cycles, addr);
}

void Acid64Emu::write (uint_least8_t addr, uint8_t data)
{
    event_clock_t cycles = m_context->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;
    return m_cmd.write (cycles, addr, data);
}

void Acid64Emu::event (void)
{
    event_clock_t cycles = m_context->getTime (m_accessClk, m_phase);
    m_cmd.delay (cycles);
    schedule (*m_context, maxDelayCycles, m_phase);
}

bool Acid64Emu::lock (c64env *env)
{
    if (env == 0)
    {
        if (!m_context)
            return false;
        m_context = 0;
    }
    else
    {
        if (m_context)
            return false;
        m_context = &env->context ();
    }
    return true;
}

int_least32_t Acid64Emu::output (uint_least8_t)
{
    return 0;
}

void Acid64Emu::volume (uint_least8_t, uint_least8_t)
{
    // Not yet supported
}
    
void Acid64Emu::mute (uint_least8_t num, bool enable)
{
    //m_sid.mute (num, enable);
}

// Find the correct interface
bool Acid64Emu::_iquery (const Iid &iid, void **implementation)
{
    if (iid == ISidEmulation::iid())
        *implementation = static_cast<ISidEmulation *>(this);
    else if (iid == ISidMixer::iid())
        *implementation = static_cast<ISidMixer *>(this);
    else if (iid == ISidUnknown::iid())
        *implementation = static_cast<ISidEmulation *>(this);
    else if (iid == Acid64Emu::iid())
        *implementation = this;
    else
        return false;
    return true;
}
