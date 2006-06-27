/***************************************************************************
             hardsid.cpp  -  Hardsid support interface.
                             -------------------
    begin                : Fri Dec 15 2000
    copyright            : (C) 2001-2001 by Jarno Paananen
    email                : jpaana@s2.org
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
 *  Revision 1.23  2006/06/19 20:52:46  s_a_white
 *  Switch to new interfaces
 *
 *  Revision 1.22  2005/12/21 18:25:49  s_a_white
 *  Allow sids additional sids to be allocated (rather than just live with
 *  those that are provided on device open).
 *
 *  Revision 1.21  2005/03/22 19:17:00  s_a_white
 *  Small fix based on Windows hardsid build changes.
 *
 *  Revision 1.20  2005/03/22 19:10:28  s_a_white
 *  Converted windows hardsid code to work with new linux streaming changes.
 *  Windows itself does not yet support streaming in the drivers for synchronous
 *  playback to multiple sids (so cannot use MK4 to full potential).
 *
 *  Revision 1.19  2005/03/20 22:52:22  s_a_white
 *  Add MK4 synchronous stream support.
 *
 *  Revision 1.18  2005/01/14 16:01:28  s_a_white
 *  If the allocated returns an error it is because the call isn't supported
 *  (as is the case for older drivers), therefore is valid.
 *
 *  Revision 1.17  2005/01/12 22:11:11  s_a_white
 *  Updated to support new ioctls so we can find number of installed sid devices.
 *
 *  Revision 1.16  2004/11/04 12:34:42  s_a_white
 *  Newer versions of the hardsid driver allow /dev/sid to be opened multiple
 *  times rather than providing seperate /dev/sid<n> entries.
 *
 *  Revision 1.15  2004/06/26 15:35:25  s_a_white
 *  Switched code to use new scheduler interface.
 *
 *  Revision 1.14  2004/05/27 21:18:28  jpaana
 *  The filter ioctl was reversed
 *
 *  Revision 1.13  2004/05/05 23:48:01  s_a_white
 *  Detect available sid devices on Unix system.
 *
 *  Revision 1.12  2004/04/29 23:20:01  s_a_white
 *  Optimisation to polling hardsid delay write to only access the hardsid
 *  if really necessary.
 *
 *  Revision 1.11  2003/10/28 00:15:16  s_a_white
 *  Get time with respect to correct clock phase.
 *
 *  Revision 1.10  2003/01/20 16:25:25  s_a_white
 *  Updated for new event scheduler interface.
 *
 *  Revision 1.9  2002/10/17 18:36:43  s_a_white
 *  Prevent multiple unlocks causing a NULL pointer access.
 *
 *  Revision 1.8  2002/08/14 16:03:54  jpaana
 *  Fixed to compile with new HardSID::lock method
 *
 *  Revision 1.7  2002/07/20 08:36:24  s_a_white
 *  Remove unnecessary and pointless conts.
 *
 *  Revision 1.6  2002/02/17 17:24:51  s_a_white
 *  Updated for new reset interface.
 *
 *  Revision 1.5  2002/01/30 01:47:47  jpaana
 *  Read ioctl used wrong parameter type and delay ioctl takes uint, not uint*
 *
 *  Revision 1.4  2002/01/30 00:43:50  s_a_white
 *  Added realtime delays even when there is no accesses to
 *  the sid.  Prevents excessive CPU usage.
 *
 *  Revision 1.3  2002/01/29 21:47:35  s_a_white
 *  Constant fixed interval delay added to prevent emulation going fast when
 *  there are no writes to the sid.
 *
 *  Revision 1.2  2002/01/29 00:32:56  jpaana
 *  Use the new read and delay IOCTLs
 *
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 *
 ***************************************************************************/

#include "config.h"
#include "hardsid-emu.h"

#define HARDSID_SYNC_ID(id) ((id) << 24)
#define HARDSID_SYNC_ID_OPT(id) (HARDSID_SYNC_ID(id) | HSID_SID_ID_PRESENT)
char    HardSID::credit[];
static  int hsid_device  = 0;
static  int hsid_devices = 0;


HardSID::HardSID (HardSIDBuilder *builder, uint id, event_clock_t &accessClk,
                  hwsid_handle_t handle)
:SidEmulation<ISidEmulation,HardSIDBuilder>(builder),
 Event("HardSID Delay"),
 m_stream(handle),
 m_eventContext(NULL),
 m_accessClk(accessClk),
 m_phase(EVENT_CLOCK_PHI1),
 m_id(id),
 m_locked(false)
{
    m_handle = hwsid_create (m_stream);
    reset ();
}

HardSID::~HardSID ()
{
    hwsid_destroy (m_handle);
}

void HardSID::reset (uint8_t volume)
{
    for (uint i= 0; i < HARDSID_VOICES; i++)
        muted[i] = false;
    hwsid_reset (m_handle, volume);
    m_accessClk = 0;
    if (m_eventContext != NULL)
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES, m_phase);
}

uint8_t HardSID::read (uint_least8_t addr)
{
    event_clock_t cycles  = m_eventContext->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;

    hwsid_stream_delay (m_stream, cycles);
    return (uint8_t) hwsid_read (m_handle, addr);
}

void HardSID::write (uint_least8_t addr, uint8_t data)
{
    hwsid_handle_t stream = hwsid_stream (m_handle);
    event_clock_t cycles  = m_eventContext->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;

    hwsid_stream_delay (m_stream, cycles);
    hwsid_write (m_handle, addr, data);
}

void HardSID::volume (uint_least8_t num, uint_least8_t level)
{
    // Not yet implemented
}

void HardSID::mute (uint_least8_t num, bool mute)
{
    // Only have 3 voices!
    if (num >= HARDSID_VOICES)
        return;
    muted[num] = mute;
    
    int cmute = 0;
    for ( uint i = 0; i < HARDSID_VOICES; i++ )
        cmute |= (muted[i] << i);
    //    ioctl (m_handle, HSID_IOCTL_MUTE, HARDSID_SYNC_ID(m_id) | cmute);
}

void HardSID::event (void)
{   // When the hardsid is not actively being written to the drivers
    // buffering risks underflow.  Prevent this inactivity period from
    // becoming too great by allowing the driver to know how much idle
    // time has passed.
    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    if (cycles < HARDSID_DELAY_CYCLES)
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES - cycles, m_phase);
    else
    {
        uint delay = (uint) cycles;
        m_accessClk += cycles;
        hwsid_stream_delay (m_stream, delay);
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES, m_phase);
    }
}

void HardSID::filter(bool enable)
{
  //    ioctl (m_handle, HSID_IOCTL_NOFILTER, HARDSID_SYNC_ID(m_id) | !enable);
}

// (Un)Lock this device to pass the to an external program for
// configuration and use.  Requests for other device will not
// return ones already locked (inuse).
bool HardSID::lock(c64env* env)
{
    if( env == NULL )
    {
        if (!m_locked)
            return false;
        cancel ();
        m_locked = false;
        m_eventContext = NULL;
    }
    else
    {
        if (m_locked)
            return false;
        m_locked = true;
        m_eventContext = &env->context();
        schedule (*m_eventContext, HARDSID_DELAY_CYCLES, m_phase);
    }
    return true;
}

int HardSID::init (char *error)
{
    return 0;
}

bool HardSID::allocate (hwsid_handle_t handle)
{
//    if (ioctl (handle, HSID_IOCTL_ALLOCATE) < 0)
//        return false;
//    return true;
    return false;
}

// Open next available hardsid device.  For the newer drivers
// we will end up opening the same device multiple times
int HardSID::open (hwsid_handle_t &handle, char *error)
{
  //    char device[20];

    // New device driver support
  //    handle = ::open ("/dev/sid", O_RDWR);
  //    if (handle < 0)
  //    {   // Old device driver support
  //        int i = hsid_device;
  //        do
  //        {
  //            sprintf (device, "/dev/sid%u", i);
  //            handle = ::open (device, O_RDWR);
  //            if (handle >= 0)
  //                break;
  //            i = (i + 1) % hsid_devices;
  //        } while (i != hsid_device);
  //        if (handle >= 0)
  //            hsid_device = i + 1;
  //    }

  //    if (handle < 0)
  //    {
  //         sprintf (error, "HARDSID ERROR: Cannot access \"/dev/sid\" or \"%s\"", device);
  //         return -1;
  //    }

    // Check to see if a sid is allocated to the stream
    // Allow errors meaning call is not supported, so
    // must have a sid
    //    int avail = ioctl (handle, HSID_IOCTL_ALLOCATED);
    //    if (avail < 0)
    //        avail = 1;
    //    return avail;

    handle = hwsid_stream_create ();
    if (!handle)
    {
        sprintf (error, "HARDSID ERROR: Unable to create hw stream");
        return -1;
    }
    return 1;
}

void HardSID::close (hwsid_handle_t handle)
{
    if (handle)
        hwsid_stream_destroy (handle);
//    if (m_handle >= 0)
//        ::close (m_handle);
}

// Return available hardsid devices (in case of old hardsid
// driver it is a best guess) or -1 on error.
int HardSID::devices (char *error)
{
  /*
    int count = 0;

    // Try opening the /dev/sid as newer versions
    // have a different interface and provide available
    // sid count
    int fd = ::open ("/dev/sid", O_RDWR);
    for (;;)
    {
        if (fd >= 0)
        {
            count = ioctl (fd, HSID_IOCTL_DEVICES);
            ::close (fd);
            if (count >= 0)
                break;
            count = 1;
        }

        // Have an old style device driver.  Detect the number of
        // devices directly.  Gaps are ignored due to user
        // configuration problems, we will cope.
        DIR    *dir;
        dirent *entry;
        dir = opendir("/dev");
        if (!dir)
        {
            sprintf (error, "HARDSID ERROR: Unable to access /dev");
            return -1;
        }

        while ( (entry=readdir(dir)) )
        {
            // SID device
            if (strncmp ("sid", entry->d_name, 3))
                continue;
        
            // if it is truely one of ours then it will be
            // followed by numerics only
            const char *p = entry->d_name+3;
            int index     = 0;
            while (*p)
            {
                if (!isdigit (*p))
                    continue;
                index = index * 10 + (*p++ - '0');
            }
            index++;
            if (count < index)
                count = index;
        }
        closedir (dir);
    }

    hsid_devices = count;
    if (hsid_device >= hsid_devices)
        hsid_device = 0;
    return count;
  */
    return 1;
}

void HardSID::flush (hwsid_handle_t handle)
{
  //    ioctl(handle, HSID_IOCTL_FLUSH);
}

void HardSID::clock(sid2_clock_t /*clk*/)
{
//    if (clk == SID2_CLOCK_NTSC)
//        hsid2.Clock ((BYTE) m_id, 2);
//    else
//       hsid2.Clock ((BYTE) m_id, 1);
}
