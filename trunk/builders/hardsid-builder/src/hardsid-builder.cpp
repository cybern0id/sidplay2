/***************************************************************************
         hardsid-builder.cpp - HardSID builder class for creating/controlling
                               HardSIDs.
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.14  2006/06/20 22:22:26  s_a_white
 *  Fuly support a COM style query interface.
 *
 *  Revision 1.13  2006/06/19 20:52:46  s_a_white
 *  Switch to new interfaces
 *
 *  Revision 1.12  2005/03/22 19:10:26  s_a_white
 *  Converted windows hardsid code to work with new linux streaming changes.
 *  Windows itself does not yet support streaming in the drivers for synchronous
 *  playback to multiple sids (so cannot use MK4 to full potential).
 *
 *  Revision 1.11  2005/03/20 22:52:22  s_a_white
 *  Add MK4 synchronous stream support.
 *
 *  Revision 1.10  2005/01/12 22:11:11  s_a_white
 *  Updated to support new ioctls so we can find number of installed sid devices.
 *
 *  Revision 1.9  2004/05/05 23:48:01  s_a_white
 *  Detect available sid devices on Unix system.
 *
 *  Revision 1.8  2003/10/18 13:31:58  s_a_white
 *  Improved hardsid.dll load failure error messages.
 *
 *  Revision 1.7  2003/06/27 07:07:00  s_a_white
 *  Use new hardsid.dll muting interface.
 *
 *  Revision 1.6  2002/10/17 18:37:43  s_a_white
 *  Exit unlock function early once correct sid is found.
 *
 *  Revision 1.5  2002/08/20 19:21:53  s_a_white
 *  Updated version information.
 *
 *  Revision 1.4  2002/08/09 18:11:35  s_a_white
 *  Added backwards compatibility support for older hardsid.dll.
 *
 *  Revision 1.3  2002/07/20 08:36:24  s_a_white
 *  Remove unnecessary and pointless conts.
 *
 *  Revision 1.2  2002/03/04 19:07:07  s_a_white
 *  Fix C++ use of nothrow.
 *
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 *
 ***************************************************************************/

#include <stdio.h>
#include "config.h"

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#include "hardsid-builder.h"
#include "hardsid-stream.h"

bool HardSIDBuilder::m_initialised = false;
uint HardSIDBuilder::m_count = 0;

HardSIDBuilder::HardSIDBuilder (const char * const name)
:SidBuilder<IHardSIDBuilder>(name)
{
    strcpy (m_errorBuffer, "N/A");

    if (!m_initialised)
    {   // Setup credits
        char *p = HardSID::credit;
        sprintf (p, "HardSID V%s Engine:", VERSION);
        p += strlen (p) + 1;
#ifdef HAVE_MSWINDOWS
        strcpy  (p, "\t(C) 1999-2002 Simon White <sidplay2@yahoo.com>");
#else
        strcpy  (p, "\t(C) 2001-2002 Jarno Paanenen <jpaana@s2.org>");
#endif
        p += strlen (p) + 1;
        *p = '\0';

        if (init () < 0)
            return;
        m_initialised = true;
    }
}

HardSIDBuilder::~HardSIDBuilder (void)
{   // Remove all are SID emulations
    remove ();
}

// Create a new sid emulation.  Called by libsidplay2 only
uint HardSIDBuilder::create (uint sids)
{
    uint count;
    m_status = true;

    // Check available devices
    count = devices (false);
    if (!m_status)
        goto HardSIDBuilder_create_error;
    if (count && (count < sids))
        sids = count;

    for (count = 0; count < sids; count++)
    {   // Can we add sid to existing stream?
        int i = m_streams.size();
        if (i > 0)
        {
            int allocated = m_streams[i-1]->allocate (sids - count);
            count += allocated;
            if (count >= sids)
                break;
        }

        // Create a new stream as no space in existing ones.
#   ifdef HAVE_EXCEPTIONS
        HardSIDStream *stream = new(std::nothrow) HardSIDStream(this);
#   else
        HardSIDStream *stream = new HardSIDStream(this);
#   endif
        // Memory alloc failed for new stream?
        if (!stream)
        {
            sprintf (m_errorBuffer, "%s ERROR: Unable to create HardSID stream", name ());
            goto HardSIDBuilder_create_error;
        }

        // stream init or sid alloc failed?
        if (!*stream || !stream->allocate(1))
        {
            strcpy (m_errorBuffer, stream->error ());
            delete stream;
            goto HardSIDBuilder_create_error;
        }
        m_streams.push_back (stream);
    }
    return count;

HardSIDBuilder_create_error:
    m_status = false;
    return count;
}

// Return the available devices or the used (created) devices.
uint HardSIDBuilder::devices (bool created)
{
    m_status = true;
    if (created)
    {
        uint count = 0;
        int  size  = m_streams.size ();
        for (int i = 0; i < size; i++)
            count += m_streams[i]->allocated ();
        return count;
    }

    return m_count;
}

const char *HardSIDBuilder::credits ()
{
    m_status = true;
    return HardSID::credit;
}

void HardSIDBuilder::flush(void)
{
    int size = m_streams.size ();
    for (int i = 0; i < size; i++)
        m_streams[i]->flush ();
}

void HardSIDBuilder::filter (bool enable)
{
    int size = m_streams.size ();
    m_status = true;
    for (int i = 0; i < size; i++)
        m_streams[i]->filter (enable);
}

// Find a free SID of the required type
ISidEmulation *HardSIDBuilder::lock (c64env *env, sid2_model_t model)
{
    HardSID *def = NULL;
    int size = m_streams.size ();
    m_status = true;

    // See if we can lock down an already allocated sid
    for (int i = 0; i < size; i++)
    {
        HardSIDStream *stream = m_streams[i];
        HardSID *sid = stream->lock (env, model);
        if (sid)
            return sid;
    }

    // See if we can reallocate a sid to the correct model
    for (int i = 0; i < size; i++)
    {
        HardSIDStream *stream = m_streams[i];
        HardSID *sid = stream->lock (env);
        if (sid)
        {
            if (stream->reallocate (sid, model))
                return sid;
            sid->lock (NULL);
            if (!def)
                def = sid;
        }
    }

    // No best match so use first available sid
    if (def)
    {
        def->lock (env);
        return def;
    }

    // Unable to locate free SID
    m_status = false;
    sprintf (m_errorBuffer, "%s ERROR: No available SIDs to lock", name ());
    return NULL;
}

// Allow something to use this SID
void HardSIDBuilder::unlock (ISidEmulation *device)
{
    HardSID *sid = (HardSID *) device;
    sid->lock (NULL);
}

// Remove all SID emulations.
void HardSIDBuilder::remove ()
{
    int size = m_streams.size ();
    for (int i = 0; i < size; i++)
        delete m_streams[i];
    m_streams.clear ();
}

// Find the number of sid devices.  We do not care about
// stuppid device numbering or drivers not loaded for the
// available nodes.
int HardSIDBuilder::init ()
{
    if (HardSID::init (m_errorBuffer))
        return -1;
    int ret = HardSID::devices (m_errorBuffer);
    if (ret < 0)
        return -1;
    m_count = (uint) ret;
    return 0;
}

// Find the correct interface
void HardSIDBuilder::ifquery (const InterfaceID &iid, void **implementation)
{
    if (iid == IID_IHardSIDBuilder)
        *implementation = static_cast<IHardSIDBuilder *>(this);
    else if (iid == IID_ISidBuilder)
        *implementation = static_cast<ISidBuilder *>(this);
    else if (iid == IID_IInterface)
        *implementation = static_cast<IInterface *>(this);
    else
    {
        *implementation = 0;
        return;
    }
    static_cast<IInterface *>(*implementation)->ifadd ();
}


// Entry point
IInterface *HardSIDBuilderCreate (const char * const name)
{
    IInterface *interface = 0;
#ifdef HAVE_EXCEPTIONS
    HardSIDBuilder *builder = new(nothrow) HardSIDBuilder(name);
#else
    HardSIDBuilder *builder = new HardSIDBuilder(name);
#endif
    if (builder)
        builder->ifquery (IF_QUERY(IInterface, &interface));
    return interface;
}
