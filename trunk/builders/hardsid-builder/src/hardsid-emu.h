/***************************************************************************
             hardsid-emu.h - Hardsid support interface.
                             -------------------
    begin                : Fri Dec 15 2000
    copyright            : (C) 2000-2002 by Simon White
                         : (C) 2001-2002 by Jarno Paananen
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
 ***************************************************************************/

#ifndef _hardsid_emu_h_
#define _hardsid_emu_h_

#include <sidplay/sidbuilder.h>
#include <sidplay/event.h>
#include "config.h"

#ifdef HAVE_MSWINDOWS

#include <windows.h>

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

struct HsidDLL2
{
    HINSTANCE          Instance;
    HsidDLL2_Delay_t   Delay;
    HsidDLL2_Devices_t Devices;
    HsidDLL2_Filter_t  Filter;
    HsidDLL2_Flush_t   Flush;
    HsidDLL2_Mute_t    Mute;
    HsidDLL2_MuteAll_t MuteAll;
    HsidDLL2_Reset_t   Reset;
    HsidDLL2_Read_t    Read;
    HsidDLL2_Sync_t    Sync;
    HsidDLL2_Write_t   Write;
    HsidDLL2_Version_t Version;
};

#endif // HAVE_MSWINDOWS

#define HARDSID_VOICES 3
// Approx 60ms
#define HARDSID_DELAY_CYCLES 60000

/***************************************************************************
 * HardSID SID Specialisation
 ***************************************************************************/
class HardSID: public sidemu, private Event
{
private:
    friend class HardSIDBuilder;

    // HardSID specific data
#ifdef HAVE_UNIX
    static         bool m_sidFree[16];
    int            m_handle;
#endif

    static const   uint voices;
    static         uint sid;
    static char    credit[100];


    // Generic variables
    EventContext  *m_eventContext;
    event_clock_t  m_accessClk;
    char           m_errorBuffer[100];

    // Must stay in this order
    bool           muted[HARDSID_VOICES];
    uint           m_instance;
    bool           m_status;
    bool           m_locked;    

public:
    HardSID  (sidbuilder *builder);
    ~HardSID ();

    // Standard component functions
    const char   *credits (void) {return credit;}
    void          reset   (void);
    uint8_t       read    (const uint_least8_t addr);
    void          write   (const uint_least8_t addr, const uint8_t data);
    const char   *error   (void) {return m_errorBuffer;}
    operator bool () const { return m_status; }

    // Standard SID functions
    int_least32_t output  (const uint_least8_t bits);
    void          filter  (const bool enable);
    void          model   (const sid2_model_t model) {;}
    void          voice   (const uint_least8_t num, const uint_least8_t volume,
                           const bool mute);
    void          gain    (const int_least8_t) {;}

    // HardSID specific
    void          flush   (void);

    // Must lock the SID before using the standard functions.
    void lock (c64env *env);
    bool lock (void) const { return m_locked; }

private:
    // Fixed interval timer delay to prevent sidplay2
    // shoot to 100% CPU usage when song nolonger
    // writes to SID.
    void event (void);
};

inline int_least32_t HardSID::output (const uint_least8_t bits)
{   // Not supported, should return samples off card...???
    return 0;
}

#endif // _hardsid_emu_h_
