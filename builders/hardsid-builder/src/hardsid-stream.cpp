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
 *  Revision 1.6  2006/06/29 19:12:18  s_a_white
 *  Seperate mixer interface from emulation interface.
 *
 *  Revision 1.5  2006/06/19 20:52:46  s_a_white
 *  Switch to new interfaces
 *
 *  Revision 1.4  2006/05/31 20:28:37  s_a_white
 *  Checking change laying around in local code.  Not sure why it is needed now.
 *
 *  Revision 1.3  2005/12/21 18:25:49  s_a_white
 *  Allow sids additional sids to be allocated (rather than just live with
 *  those that are provided on device open).
 *
 *  Revision 1.2  2005/03/22 19:10:27  s_a_white
 *  Converted windows hardsid code to work with new linux streaming changes.
 *  Windows itself does not yet support streaming in the drivers for synchronous
 *  playback to multiple sids (so cannot use MK4 to full potential).
 *
 *  Revision 1.1  2005/03/20 22:47:39  s_a_white
 *  Added synchronous stream support for MK4 styles hardware.
 *
 ***************************************************************************/

#include "hardsid-stream.h"


HardSIDStream::HardSIDStream(HardSIDBuilder *builder)
:m_builder(builder),
 m_status(false),
 m_handle(0),
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
        m_sids[i]->ifrelease ();
    HardSID::close (m_handle);
}

// Attempt to allocate a sid to this stream for external use.
// If sids are already allocated then provide one of those.
uint HardSIDStream::allocate (uint sids)
{
    for (uint i = 0; i < sids; i++)
    {   // Use up any pre-allocated sids first before allocating more
        if (m_devUsed >= m_devAvail)
        {
            if (!HardSID::allocate (m_handle))
            {
                sids = i;
                break;
            }
        }

#   ifdef HAVE_EXCEPTIONS
        HardSID *sid = new(std::nothrow) HardSID(m_builder, m_devUsed + 1,
                                                 m_accessClk, m_handle);
#   else
        HardSID *sid = new HardSID(m_builder, m_devUsed + 1, m_accessClk, m_handle);
#   endif
        if (!sid)
        {
            int cnt = (int) (sids - i);
            //ioctl (m_handle, HSID_IOCTL_RELEASE, &cnt);
            sids = i;
            break;
        }

        // Use if interface reference counting to delete object
        if_cast<ISidEmulation>(sid);

        m_devUsed++;
        m_sids.push_back(sid);
    }

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
