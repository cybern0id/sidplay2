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
 *  Revision 1.14  2006/05/31 20:31:38  s_a_white
 *  Support passing of PAL/NTSC state to hardsid/catweasel to reduce de-tuning.
 *
 *  Revision 1.13  2005/12/21 18:25:49  s_a_white
 *  Allow sids additional sids to be allocated (rather than just live with
 *  those that are provided on device open).
 *
 *  Revision 1.12  2005/03/22 19:10:27  s_a_white
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
 *  Revision 1.9  2004/06/26 11:18:32  s_a_white
 *  Merged sidplay2/w volume/mute changes.
 *
 *  Revision 1.8  2004/03/18 20:50:21  s_a_white
 *  Indicate the 2.07 extensions.
 *
 *  Revision 1.7  2003/10/28 00:15:16  s_a_white
 *  Get time with respect to correct clock phase.
 *
 *  Revision 1.6  2003/06/27 07:08:17  s_a_white
 *  Use new hardsid.dll muting interface.
 *
 *  Revision 1.5  2002/08/09 18:11:35  s_a_white
 *  Added backwards compatibility support for older hardsid.dll.
 *
 *  Revision 1.4  2002/07/20 08:36:24  s_a_white
 *  Remove unnecessary and pointless conts.
 *
 *  Revision 1.3  2002/02/17 17:24:50  s_a_white
 *  Updated for new reset interface.
 *
 *  Revision 1.2  2002/01/29 21:47:35  s_a_white
 *  Constant fixed interval delay added to prevent emulation going fast when
 *  there are no writes to the sid.
 *
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 ***************************************************************************/

#ifndef _hardsid_emu_h_
#define _hardsid_emu_h_

#include <sidplay/imp/sidbuilder.h>
#include <sidplay/event.h>
#include "config.h"
#include "hardsid-builder.h"
#include "/home/swhite/CVSROOT/hardsid/src/hwsid.h"

#define HARDSID_VOICES 3
// Approx 60ms
#define HARDSID_DELAY_CYCLES 60000

/***************************************************************************
 * HardSID SID Specialisation
 ***************************************************************************/
class HardSID: public SidEmulation<ISidEmulation,HardSIDBuilder>,
               private Event
{
private:
    friend class HardSIDBuilder;

    // HardSID specific data
    hwsid_handle_t m_handle;
    hwsid_handle_t m_stream;
    static char    credit[100];

    // Generic variables
    EventContext  *m_eventContext;
    event_phase_t  m_phase;
    event_clock_t &m_accessClk;

    // Must stay in this order
    bool           muted[HARDSID_VOICES];
    uint           m_id;
    bool           m_locked;

public:
    HardSID  (HardSIDBuilder *builder, uint id, event_clock_t &accessClk,
              hwsid_handle_t handle);
    ~HardSID ();

    // Standard component functions
    void          ifquery (const InterfaceID &cid, void **implementation) {;}
    const char   *credits (void) {return credit;}
    void          reset   (uint8_t volume = 0);
    uint8_t       read    (uint_least8_t addr);
    void          write   (uint_least8_t addr, uint8_t data);
    const char   *error   (void) {return "";}

    // Standard SID functions
    void          clock   (sid2_clock_t clk);
    int_least32_t output  (uint_least8_t bits);
    void          filter  (bool enable);
    void          model   (sid2_model_t model) {;}
    void          volume  (uint_least8_t num, uint_least8_t level);
    void          mute    (uint_least8_t num, bool enable);
    void          gain    (int_least8_t) {;}

    // Must lock the SID before using the standard functions.
    bool          lock    (c64env *env);

private:
    // Fixed interval timer delay to prevent sidplay2
    // shoot to 100% CPU usage when song nolonger
    // writes to SID.
    void event (void);

public:
    // Support to obtain number of devices
    static int  init     (char *error);
    static int  open     (hwsid_handle_t &handle, char *error);
    static void close    (hwsid_handle_t handle);
    static int  devices  (char *error);
    static void flush    (hwsid_handle_t handle);
    static bool allocate (hwsid_handle_t handle);
};

inline int_least32_t HardSID::output (uint_least8_t bits)
{   // Not supported, should return samples off card...???
    return 0;
}

#endif // _hardsid_emu_h_
