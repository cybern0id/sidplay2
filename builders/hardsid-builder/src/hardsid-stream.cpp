/***************************************************************************
      hardsid-stream.cpp  -  Hardsid stream handling.
                             -------------------
    begin                : Sat Mar 19 2005
    copyright            : (C) 2005 by Simon White
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

#include <linux/hardsid.h>
#include "hardsid-stream.h"


HardSIDStream::HardSIDStream(sidbuilder *builder)
:m_builder(builder),
 m_status(false),
 m_devUsed(0),
 m_devAvail(0)
{
    uint avail = HardSID::open (m_handle, m_errorBuffer);
    if (avail)
    {
        m_devAvail += avail;
        m_status    = true;
    }
}

HardSIDStream::~HardSIDStream()
{
    int size = m_sids.size ();
    for (int i = 0; i < size; i++)
        delete m_sids[i];
    HardSID::close (m_handle);
}

// Attempt to allocate a sid to this stream for external use.
// If sids are already allocated then provide one of those.
uint HardSIDStream::allocate (uint sids)
{
    if ((m_devUsed + sids) > m_devAvail)
    {   // Try to allocate more sids
        // @FIXME@
        sids = m_devAvail - m_devUsed;
    }

    for (uint i = 0; i < sids; i++)
    {
#   ifdef HAVE_EXCEPTIONS
        HardSID *sid = new(std::nothrow) HardSID(m_builder, m_devUsed + i,
                                                m_accessClk, m_handle);
#   else
        HardSID *sid = new HardSID(this, m_devUsed + i, m_accessClk, m_handle);
#   endif
        if (!sid)
        {
            int cnt = (int) (sids - i);
            //ioctl (m_handle, HSID_IOCTL_RELEASE, &cnt);
            sids = i;
            break;
        }
        m_sids.push_back(sid);
    }

    m_devUsed += sids;
    return sids;
}

// Change this sid to another model.  Note the sid is already
// allocated to us, we can never lose an allocated sid.  However
// it may become swapped with another programs sid unless we have
// locked it.  Re-allocation can happen at anytime when unlocked.
// The main reason for this is to allow auto sid version
// aquiring, but still guarentee we have a sid.
bool HardSIDStream::reallocate (HardSID *sid, sid2_model_t model)
{
    // @FIXME@
    if (model != SID2_MODEL_CORRECT)
    {
    }
    return false;
}

HardSID *HardSIDStream::lock (c64env *env, sid2_model_t model)
{
    int size = m_sids.size ();
    // Treat this to mean any sid model
    if (model == SID2_MODEL_CORRECT)
    {
        for (int i = 0; i < size; i++)
        {
            HardSID *sid = m_sids[i];
            if (sid->lock (env))
                return sid;
        }
    }
/*
    // Specific sid model
    else
    {
        for (int i = 0; i < size; i++)
        {
            HardSID *sid = m_sids[i];
            if ((m_sid->model () == model) && sid->lock (env))
                return sid;
        }
    }
*/
    return NULL;
}

void HardSIDStream::filter (bool enable)
{
    int size = m_sids.size ();
    for (int i = 0; i < size; i++)
        m_sids[i]->filter (enable);
}

void HardSIDStream::flush ()
{
    HardSID::flush (m_handle);
}
