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

#include "hardsid.h"
#include "hardsid-emu.h"


#ifdef HAVE_MSWINDOWS
//**************************************************************************
// Version 1 Interface
typedef BYTE (CALLBACK* HsidDLL1_InitMapper_t) (void);

HsidDLL2 hsid2 = {0};
#endif

uint HardSIDBuilder::m_instance = 0;

HardSIDBuilder::HardSIDBuilder (const char * const name)
:sidbuilder (name)
{
    strcpy (m_errorBuffer, "N/A");

    if (m_instance == 0)
    {   // Setup credits
        char *p = HardSID::credit;
        sprintf (p, "HardSID V1.00 Engine:");
        p += strlen (p) + 1;
#ifdef HAVE_MSWINDOWS
        strcpy  (p, "\tCopyright (C) 1999 Simon White <s_a_white@email.com>");
#else
        strcpy  (p, "\tCopyright (C) 2001-2002 Jarno Paanenen <jpaana@s2.org>");
#endif
        p += strlen (p) + 1;
        *p = '\0';

#ifdef HAVE_MSWINDOWS
        // Load Windows HardSID dll
        if (init () < 0)
            return;
#endif
    }
    m_instance++;
}

HardSIDBuilder::~HardSIDBuilder (void)
{   // Remove all are SID emulations
    remove ();
    m_instance--;
}

// Create a new sid emulation.  Called by libsidplay2 only
uint HardSIDBuilder::create (uint sids)
{
    uint   count;
    HardSID *sid = NULL;
    m_status     = true;

    // Check available devices
    count = devices (false);
    if (!m_status)
        goto HardSIDBuilder_create_error;
    if (count && (count < sids))
        sids = count;

    for (count = 0; count < sids; count++)
    {
#   ifdef HAVE_EXCEPTIONS
        sid = new(std::nothrow) HardSID(this);
#   else
        sid = new HardSID(this);
#   endif

        // Memory alloc failed?
        if (!sid)
        {
            sprintf (m_errorBuffer, "%s ERROR: Unable to create HardSID object", name ());
            goto HardSIDBuilder_create_error;
        }

        // SID init failed?
        if (!*sid)
        {
            strcpy (m_errorBuffer, sid->error ());
            goto HardSIDBuilder_create_error;
        }
        sidobjs.push_back (sid);
    }
    return count;

HardSIDBuilder_create_error:
    m_status = false;
    if (sid)
        delete sid;
    return count;
}

uint HardSIDBuilder::devices (bool created)
{
    m_status = true;
    if (created)
        return sidobjs.size ();

    // Available devices
    // @FIXME@ not yet supported on Linux
#ifdef HAVE_MSWINDOWS
    if (hsid2.Instance)
    {
        uint count = hsid2.Devices ();
        if (count == 0)
        {
            sprintf (m_errorBuffer, "HARDSID ERROR: No devices found (run HardSIDConfig)");
            m_status = false;
        }
        return count;
    }
#endif
    return 0;
}

const char *HardSIDBuilder::credits ()
{
    m_status = true;
    return HardSID::credit;
}

void HardSIDBuilder::flush(void)
{
    int size = sidobjs.size ();
    for (int i = 0; i < size; i++)
        ((HardSID*)sidobjs[i])->flush();
}

void HardSIDBuilder::filter (bool enable)
{
    int size = sidobjs.size ();
    m_status = true;
    for (int i = 0; i < size; i++)
    {
        HardSID *sid = (HardSID *) sidobjs[i];
        sid->filter (enable);
    }
}

// Find a free SID of the required specs
sidemu *HardSIDBuilder::lock (c64env *env, sid2_model_t model)
{
    int size = sidobjs.size ();
    m_status = true;

    for (int i = 0; i < size; i++)
    {
        HardSID *sid = (HardSID *) sidobjs[i];
        if (sid->lock ())
            continue;
        sid->lock  (env);
        sid->model (model);
        return sid;
    }
    // Unable to locate free SID
    m_status = false;
    sprintf (m_errorBuffer, "%s ERROR: No available SIDs to lock", name ());
    return NULL;
}

// Allow something to use this SID
void HardSIDBuilder::unlock (sidemu *device)
{
    int size = sidobjs.size ();
    // Maek sure this is our SID
    for (int i = 0; i < size; i++)
    {
        HardSID *sid = (HardSID *) sidobjs[i];
        if (sid != device)
            continue;
        // Unlock it
        sid->lock (NULL);
    }
}

// Remove all SID emulations.
void HardSIDBuilder::remove ()
{
    int size = sidobjs.size ();
    for (int i = 0; i < size; i++)
        delete sidobjs[i];
    sidobjs.clear();
}

#ifdef HAVE_MSWINDOWS

// Load the library and initialise the hardsid
int HardSIDBuilder::init ()
{
    HINSTANCE dll;
    HsidDLL1_InitMapper_t mapper;

    if (hsid2.Instance)
        return 0;

    m_status = false;
    dll = LoadLibrary("HARDSID.DLL");
    if (!dll)
    {
        sprintf (m_errorBuffer, "HARDSID ERROR: hardsid.dll not found or failed to initialise!");
        goto HardSID_init_error;
    }

    // Export Needed Version 1 Interface
    mapper = (HsidDLL1_InitMapper_t) GetProcAddress(dll, "InitHardSID_Mapper");

    if (mapper)
        mapper(); 
    else
    {
        sprintf (m_errorBuffer, "HARDSID ERROR: hardsid.dll is corrupt!");
        goto HardSID_init_error;
    }

    // Check for the Version 2 interface
    hsid2.Version = (HsidDLL2_Version_t) GetProcAddress(dll, "HardSID_Version");
    if (!hsid2.Version)
    {
        sprintf (m_errorBuffer, "HARDSID ERROR: hardsid.dll not V2");
        goto HardSID_init_error;
    }

    if ((hsid2.Version () >> 8) != 2)
    {
        sprintf (m_errorBuffer, "HARDSID ERROR: hardsid.dll not V2");
        goto HardSID_init_error;
    }

    // Export Needed Version 2 Interface
    hsid2.Delay    = (HsidDLL2_Delay_t)   GetProcAddress(dll, "HardSID_Delay");
    hsid2.Devices  = (HsidDLL2_Devices_t) GetProcAddress(dll, "HardSID_Devices");
    hsid2.Filter   = (HsidDLL2_Filter_t)  GetProcAddress(dll, "HardSID_Filter");
    hsid2.Flush    = (HsidDLL2_Flush_t)   GetProcAddress(dll, "HardSID_Flush");
    hsid2.Mute     = (HsidDLL2_Mute_t)    GetProcAddress(dll, "HardSID_Mute");
    hsid2.MuteAll  = (HsidDLL2_MuteAll_t) GetProcAddress(dll, "HardSID_MuteAll");
    hsid2.Read     = (HsidDLL2_Read_t)    GetProcAddress(dll, "HardSID_Read");
    hsid2.Reset    = (HsidDLL2_Reset_t)   GetProcAddress(dll, "HardSID_Reset");
    hsid2.Sync     = (HsidDLL2_Sync_t)    GetProcAddress(dll, "HardSID_Sync");
    hsid2.Write    = (HsidDLL2_Write_t)   GetProcAddress(dll, "HardSID_Write");
    hsid2.Instance = dll;
    m_status       = true;
    return 0;

HardSID_init_error:
    if (dll)
        FreeLibrary (dll);
    return -1;
}

#endif // HAVE_MSWINDOWS
