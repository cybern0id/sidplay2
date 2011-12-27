/***************************************************************************
         resid-builder.cpp - ReSID builder class for creating/controlling
                             resids.
                             -------------------
    begin                : Wed Sep 5 2001
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

#include <algorithm>
#include <cstdio>
#include <sstream>

#include "config.h"
#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#include "acid64-builder.h"
#include "acid64-builder_p.h"
#include "acid64-emu.h"

Acid64Builder::Acid64Builder (const char * name, Acid64Cmd &cmd)
{
    m_impl = new Acid64BuilderPrivate (name, cmd);
    m_iptr = m_impl->iunknown (); // Will manage lifetime
}

uint Acid64Builder::create (uint sids)
{
    return m_impl->create (sids);
}

Acid64BuilderPrivate::Acid64BuilderPrivate (const char * name, Acid64Cmd &cmd)
:CoBuilder<ISidBuilder>(name)
,m_cmd(cmd)
,m_delay(0)
{
    m_error = "N/A";

    std::ostringstream out;
    out << "Acid64 Emulation V" VERSION " Engine:" << '\0';
    out << "\t(C) 2011 Simon White <sidplay2@yahoo.com>" << '\0';
    std::string s = out.str ();
    m_credits.resize (s.size()+1);
    memcpy (&m_credits[0], &s[0], s.size());
    m_credits[s.size()] = '\0';
}

Acid64BuilderPrivate::~Acid64BuilderPrivate (void)
{   // Remove all are SID emulations
    remove ();
}

// Create a new sid emulation.  Called by libsidplay2 only
uint Acid64BuilderPrivate::create (uint sids)
{
    m_status = true;

    // Check available devices
    uint count = devices (false);
    if (count && (count < sids))
        sids = count;

    try
    {
        for (count = 0; count < sids; count++)
        {
            std::auto_ptr<Acid64Emu> sid(new Acid64Emu(*this, m_cmd));
            m_sidobjs.push_back (sid.get()->iunknown());
            sid.release ();
        }
    }
    catch (...)
    {
        sprintf (m_errorBuffer, "%s ERROR: Unable to create ReSID object", iname ());
        m_error  = m_errorBuffer;
        m_status = false;
    }

    return count;
}

const char *Acid64BuilderPrivate::credits ()
{
    m_status = true;
    return &m_credits[0];
}


uint Acid64BuilderPrivate::devices (bool created)
{
    m_status = true;
    if (created)
        return (uint)m_sidobjs.size ();
    // Available devices
    return 1;
}

// Find a free SID of the required specs
ISidUnknown *Acid64BuilderPrivate::lock (c64env *env, sid2_model_t model)
{
    int size = m_sidobjs.size ();
    m_status = true;

    for (int i = 0; i < size; i++)
    {
        SidIPtr<Acid64Emu> sid = m_sidobjs[i];
        if (sid->lock (env))
            return sid->iunknown ();
    }
    // Unable to locate free SID
    m_status = false;
    sprintf (m_errorBuffer, "%s ERROR: No available SIDs to lock", iname ());
    return NULL;
}

// Allow something to use this SID
void Acid64BuilderPrivate::unlock (ISidUnknown &device)
{
    ISidUnknown *emulation = device.iunknown ();
    int size = m_sidobjs.size ();
    // Maek sure this is our SID
    for (int i = 0; i < size; i++)
    {
        if (m_sidobjs[i]->iunknown() == emulation)
        {   // Unlock it
            SidIPtr<Acid64Emu> sid = m_sidobjs[i];
            sid->lock (NULL);
            break;
        }
    }
}

// Remove all SID emulations.
void Acid64BuilderPrivate::remove ()
{
    m_sidobjs.clear();
}

// Find the correct interface
bool Acid64BuilderPrivate::_iquery (const Iid &iid, void **implementation)
{
    if (iid == ISidBuilder::iid())
        *implementation = static_cast<ISidBuilder *>(this);
    else if (iid == ISidUnknown::iid())
        *implementation = static_cast<ISidBuilder *>(this);
    else
        return false;
    return true;
}
