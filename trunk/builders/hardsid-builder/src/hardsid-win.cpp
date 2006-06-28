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
 *  Revision 1.19  2006/05/31 20:31:39  s_a_white
 *  Support passing of PAL/NTSC state to hardsid/catweasel to reduce de-tuning.
 *
 *  Revision 1.18  2006/03/02 23:56:54  s_a_white
 *  Make code build with new allocate function.  Just returning true will provide
 *  previous behaviour for Windows.
 *
 *  Revision 1.17  2005/10/02 07:47:52  s_a_white
 *  Add official call to enable support for alternative hardware (non hardsid).
 *
 *  Revision 1.16  2005/03/22 19:10:28  s_a_white
 *  Converted windows hardsid code to work with new linux streaming changes.
 *  Windows itself does not yet support streaming in the drivers for synchronous
 *  playback to multiple sids (so cannot use MK4 to full potential).
 *
 *  Revision 1.15  2004/06/26 15:35:05  s_a_white
 *  Switched hardcoded phases to use m_phase variable.
 *
 *  Revision 1.14  2004/06/26 11:17:28  s_a_white
 *  Changes to support new calling convention for event scheduler.
 *  Merged sidplay2/w volume/mute changes.
 *
 *  Revision 1.13  2004/04/15 15:19:31  s_a_white
 *  Only insert delays from the periodic event into output stream if
 *  HARDSID_DELAY_CYCLES has passed (removes unnecessary hw writes)
 *
 *  Revision 1.12  2004/03/18 20:46:41  s_a_white
 *  Fixed use of uninitialised variable m_phase.
 *
 *  Revision 1.11  2003/10/29 23:36:45  s_a_white
 *  Get clock wrt correct phase.
 *
 *  Revision 1.10  2003/06/27 18:35:19  s_a_white
 *  Remove unnecessary muting and add some initial support for the async dll.
 *
 *  Revision 1.9  2003/06/27 07:05:42  s_a_white
 *  Use new hardsid.dll muting interface.
 *
 *  Revision 1.8  2003/01/17 08:30:48  s_a_white
 *  Event scheduler phase support.
 *
 *  Revision 1.7  2002/10/17 18:36:43  s_a_white
 *  Prevent multiple unlocks causing a NULL pointer access.
 *
 *  Revision 1.6  2002/08/09 18:11:35  s_a_white
 *  Added backwards compatibility support for older hardsid.dll.
 *
 *  Revision 1.5  2002/07/20 08:36:24  s_a_white
 *  Remove unnecessary and pointless conts.
 *
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
#include <windows.h>
#include "config.h"
#include "hardsid.h"
#include "hardsid-emu.h"

#define HSID_VERSION_MIN (WORD) 0x0200
#define HSID_VERSION_204 (WORD) 0x0204
#define HSID_VERSION_207 (WORD) 0x0207
#define HSID_VERSION_208 (WORD) 0x0208

//**************************************************************************
// Version 1 Interface (used calls only)
typedef BYTE (CALLBACK* HsidDLL1_InitMapper_t) (void);

//**************************************************************************
// Version 2 Interface
typedef void (CALLBACK* HsidDLL2_Delay_t)   (BYTE deviceID, WORD cycles);
typedef BYTE (CALLBACK* HsidDLL2_Devices_t) (void);
typedef void (CALLBACK* HsidDLL2_Filter_t)  (BYTE deviceID, BOOL filter);
typedef void (CALLBACK* HsidDLL2_Flush_t)   (BYTE deviceID);
typedef void (CALLBACK* HsidDLL2_Mute_t)    (BYTE deviceID, BYTE channel, BOOL mute);
typedef void (CALLBACK* HsidDLL2_MuteAll_t) (BYTE deviceID, BOOL mute);
typedef void (CALLBACK* HsidDLL2_Reset_t)   (BYTE deviceID);
typedef BYTE (CALLBACK* HsidDLL2_Read_t)    (BYTE deviceID, WORD cycles, BYTE SID_reg);
typedef void (CALLBACK* HsidDLL2_Sync_t)    (BYTE deviceID);
typedef void (CALLBACK* HsidDLL2_Write_t)   (BYTE deviceID, WORD cycles, BYTE SID_reg, BYTE data);
typedef WORD (CALLBACK* HsidDLL2_Version_t) (void);

// Version 2.04 Extensions
typedef BOOL (CALLBACK* HsidDLL2_Lock_t)    (BYTE deviceID);
typedef void (CALLBACK* HsidDLL2_Unlock_t)  (BYTE deviceID);
typedef void (CALLBACK* HsidDLL2_Reset2_t)  (BYTE deviceID, BYTE volume);

// Version 2.07 Extensions
typedef void (CALLBACK* HsidDLL2_Mute2_t)   (BYTE deviceID, BYTE channel, BOOL mute, BOOL manual);

// Version 2.08 Extensions
typedef void (CALLBACK* HsidDLL2_OtherHardware_t) (void);

// Version 2.09 Extensions
typedef WORD (CALLBACK* HsidDLL2_Clock_t)   (BYTE deviceID, BYTE preset); // 0 = H/W, PAL = 1, NTSC = 2

struct HsidDLL2
{
    HINSTANCE          Instance;
    HsidDLL2_Clock_t   Clock;
    HsidDLL2_Delay_t   Delay;
    HsidDLL2_Devices_t Devices;
    HsidDLL2_Filter_t  Filter;
    HsidDLL2_Flush_t   Flush;
    HsidDLL2_Lock_t    Lock;
    HsidDLL2_Unlock_t  Unlock;
    HsidDLL2_Mute_t    Mute;
    HsidDLL2_Mute2_t   Mute2;
    HsidDLL2_MuteAll_t MuteAll;
    HsidDLL2_Reset_t   Reset;
    HsidDLL2_Reset2_t  Reset2;
    HsidDLL2_Read_t    Read;
    HsidDLL2_Sync_t    Sync;
    HsidDLL2_Write_t   Write;
    WORD               Version;
};

static HsidDLL2 hsid2 = {0};
char   HardSID::credit[];
static int hsid_device = 0;

HardSID::HardSID (HardSIDBuilder *builder, uint id, event_clock_t &accessClk,
                  hwsid_handle_t handle)
:SidEmulation<ISidEmulation,HardSIDBuilder>(builder),
 Event("HardSID Delay"),
 m_handle(handle),
 m_eventContext(NULL),
 m_accessClk(accessClk),
 m_phase(EVENT_CLOCK_PHI1),
 m_id(handle), // for now ignore id (as refers to stream id)
 m_locked(false)
{
    reset ();
}

HardSID::~HardSID()
{
}

uint8_t HardSID::read (uint_least8_t addr)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;

    while (cycles > 0xFFFF)
    {
        hsid2.Delay ((BYTE) m_id, 0xFFFF);
        cycles -= 0xFFFF;
    }

    return hsid2.Read ((BYTE) m_id, (WORD) cycles,
                       (BYTE) addr);
}

void HardSID::write (uint_least8_t addr, uint8_t data)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;

    while (cycles > 0xFFFF)
    {
        hsid2.Delay ((BYTE) m_id, 0xFFFF);
        cycles -= 0xFFFF;
    }

    hsid2.Write ((BYTE) m_id, (WORD) cycles,
                 (BYTE) addr, (BYTE) data);
}

void HardSID::reset (uint8_t volume)
{
    m_accessClk = 0;
    // Clear hardsid buffers
    hsid2.Flush ((BYTE) m_id);
    if (hsid2.Version >= HSID_VERSION_204)
        hsid2.Reset2 ((BYTE) m_id, volume);
    else
        hsid2.Reset  ((BYTE) m_id);
    hsid2.Sync ((BYTE) m_id);

    if (m_eventContext != NULL)
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES, m_phase);
}

void HardSID::volume (uint_least8_t num, uint_least8_t level)
{
    // Not yet implemented
}

void HardSID::mute (uint_least8_t num, bool mute)
{
    if (hsid2.Version >= HSID_VERSION_207)
        hsid2.Mute2 ((BYTE) m_id, (BYTE) num, (BOOL) mute, FALSE);
    else
        hsid2.Mute  ((BYTE) m_id, (BYTE) num, (BOOL) mute);
}

// Set execution environment and lock sid to it
bool HardSID::lock (c64env *env)
{
    if (env == NULL)
    {
        if (!m_locked)
            return false;
        if (hsid2.Version >= HSID_VERSION_204)
            hsid2.Unlock (m_id);
        m_locked = false;
        cancel ();
        m_eventContext = NULL;
    }
    else
    {
        if (m_locked)
            return false;
        if (hsid2.Version >= HSID_VERSION_204)
        {
            if (hsid2.Lock (m_id) == FALSE)
                return false;
        }
        m_locked = true;
        m_eventContext = &env->context ();
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES, m_phase);
    }
    return true;
}

void HardSID::event (void)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    if (cycles < HARDSID_DELAY_CYCLES)
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES - cycles, m_phase);
    else
    {
        m_accessClk += cycles;
        hsid2.Delay ((BYTE) m_id, (WORD) cycles);
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES, m_phase);
    }
}

// Disable/Enable SID filter
void HardSID::filter (bool enable)
{
    hsid2.Filter ((BYTE) m_id, (BOOL) enable);
}

// Load the library and initialise the hardsid
int HardSID::init (char *error)
{
    HINSTANCE dll;

    if (hsid2.Instance)
        return 0;

    dll = LoadLibrary("HARDSID.DLL");
    if (!dll)
    {
        DWORD err = GetLastError();
        if (err == ERROR_DLL_INIT_FAILED)
            sprintf (error, "HARDSID ERROR: hardsid.dll init failed!");
        else
            sprintf (error, "HARDSID ERROR: hardsid.dll not found!");
        goto HardSID_init_error;
    }

    {   // Export Needed Version 1 Interface
        HsidDLL1_InitMapper_t mapper;
        mapper = (HsidDLL1_InitMapper_t) GetProcAddress(dll, "InitHardSID_Mapper");

        if (mapper)
            mapper(); 
        else
        {
            sprintf (error, "HARDSID ERROR: hardsid.dll is corrupt!");
            goto HardSID_init_error;
        }
    }

    {   // Check for the Version 2 interface
        HsidDLL2_Version_t version;
        version = (HsidDLL2_Version_t) GetProcAddress(dll, "HardSID_Version");
        if (!version)
        {
            sprintf (error, "HARDSID ERROR: hardsid.dll not V2");
            goto HardSID_init_error;
        }
        hsid2.Version = version ();
    }

    {
        WORD version = hsid2.Version;
        if ((version >> 8) != (HSID_VERSION_MIN >> 8))
        {
            sprintf (error, "HARDSID ERROR: hardsid.dll not V%d", HSID_VERSION_MIN >> 8);
            goto HardSID_init_error;
        }

        if (version < HSID_VERSION_MIN)
        {
            sprintf (error, "HARDSID ERROR: hardsid.dll must be V%02u.%02u or greater",
                     HSID_VERSION_MIN >> 8, HSID_VERSION_MIN & 0xff);
            goto HardSID_init_error;
        }
    }

    // Export Needed Version 2 Interface
    hsid2.Delay    = (HsidDLL2_Delay_t)   GetProcAddress(dll, "HardSID_Delay");
    hsid2.Devices  = (HsidDLL2_Devices_t) GetProcAddress(dll, "HardSID_Devices");
    hsid2.Filter   = (HsidDLL2_Filter_t)  GetProcAddress(dll, "HardSID_Filter");
    hsid2.Flush    = (HsidDLL2_Flush_t)   GetProcAddress(dll, "HardSID_Flush");
    hsid2.MuteAll  = (HsidDLL2_MuteAll_t) GetProcAddress(dll, "HardSID_MuteAll");
    hsid2.Read     = (HsidDLL2_Read_t)    GetProcAddress(dll, "HardSID_Read");
    hsid2.Sync     = (HsidDLL2_Sync_t)    GetProcAddress(dll, "HardSID_Sync");
    hsid2.Write    = (HsidDLL2_Write_t)   GetProcAddress(dll, "HardSID_Write");

    if (hsid2.Version < HSID_VERSION_204)
        hsid2.Reset  = (HsidDLL2_Reset_t)  GetProcAddress(dll, "HardSID_Reset");
    else
    {
        hsid2.Lock   = (HsidDLL2_Lock_t)   GetProcAddress(dll, "HardSID_Lock");
        hsid2.Unlock = (HsidDLL2_Unlock_t) GetProcAddress(dll, "HardSID_Unlock");
        hsid2.Reset2 = (HsidDLL2_Reset2_t) GetProcAddress(dll, "HardSID_Reset2");
    }

    if (hsid2.Version < HSID_VERSION_207)
        hsid2.Mute   = (HsidDLL2_Mute_t)   GetProcAddress(dll, "HardSID_Mute");
    else
    {
        hsid2.Mute2  = (HsidDLL2_Mute2_t)  GetProcAddress(dll, "HardSID_Mute2");

        // Enable non hardsid hardware support
        HsidDLL2_OtherHardware_t otherHwEnable;
        otherHwEnable = (HsidDLL2_OtherHardware_t) GetProcAddress(dll, "HardSID_OtherHardware");
        if (otherHwEnable)
            otherHwEnable ();
    }

    hsid2.Clock = 0;
    if (hsid2.Version > HSID_VERSION_208)
        hsid2.Clock = (HsidDLL2_Clock_t) GetProcAddress(dll, "HardSID_Clock");

    hsid2.Instance = dll;
    return 0;

HardSID_init_error:
    if (dll)
        FreeLibrary (dll);
    return -1;
}

bool HardSID::allocate (hwsid_handle_t)
{
    return true;
}

// Open next available hardsid device.  For the newer drivers
// we will end up opening the same device multiple times
int HardSID::open (hwsid_handle_t &handle, char *error)
{
    if (hsid_device == hsid2.Devices ())
    {
        sprintf (error, "HARDSID WARNING: System dosen't have enough SID chips.");
        return -1;
    }
    handle = hsid_device++;
    return 1;
}

void HardSID::close (hwsid_handle_t)
{
    hsid_device--;
}

int HardSID::devices (char *error)
{
    if (hsid2.Instance)
    {
        uint count = hsid2.Devices ();
        if (count == 0)
            sprintf (error, "HARDSID ERROR: No devices found (run HardSIDConfig)");
        return count;
    }
    return 0;
}

void HardSID::flush(hwsid_handle_t handle)
{
    hsid2.Flush ((BYTE) handle);
}

void HardSID::clock(sid2_clock_t clk)
{
    if (hsid2.Clock)
    {
        if (clk == SID2_CLOCK_NTSC)
            hsid2.Clock ((BYTE) m_id, 2);
        else
            hsid2.Clock ((BYTE) m_id, 1);
    }
}
