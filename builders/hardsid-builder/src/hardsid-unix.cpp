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
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 *
 ***************************************************************************/

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include "config.h"
#include "hardsid-emu.h"

// Move these to common header file
#define HSID_IOCTL_RESET     _IOW('S', 0, int)
#define HSID_IOCTL_FIFOSIZE  _IOR('S', 1, int)
#define HSID_IOCTL_FIFOFREE  _IOR('S', 2, int)
#define HSID_IOCTL_SIDTYPE   _IOR('S', 3, int)
#define HSID_IOCTL_CARDTYPE  _IOR('S', 4, int)
#define HSID_IOCTL_MUTE      _IOW('S', 5, int)
#define HSID_IOCTL_NOFILTER  _IOW('S', 6, int)
#define HSID_IOCTL_FLUSH     _IO ('S', 7)
#define HSID_IOCTL_DELAY     _IOW('S', 8, int)
#define HSID_IOCTL_READ      _IOWR('S', 9, int)

bool       HardSID::m_sidFree[16] = {0};
const uint HardSID::voices = HARDSID_VOICES;
uint       HardSID::sid = 0;
char       HardSID::credit[];

HardSID::HardSID (sidbuilder *builder)
:sidemu(builder),
 m_handle(0),
 m_eventContext(NULL),
 m_instance(sid++),
 m_status(false),
 m_locked(false)
{
    uint num = 16;
    for ( uint i = 0; i < 16; i++ )
    {
        if(m_sidFree[i] == 0)
        {
            m_sidFree[i] = 1;
            num = i;
            break;
        }
    }

    // All sids in use?!?
    if ( num == 16 )
        return;

    m_instance = num;

    {
        char device[20];
        *m_errorBuffer = '\0';
        sprintf (device, "/dev/sid%u", m_instance);
        m_handle = open (device, O_RDWR);
        if (!m_handle)
        {
            sprintf (m_errorBuffer, "HARDSID ERROR: Require access to HardSID device \"%s\"\n", device);
            return;
        }
    }

    m_status = true;
    reset ();
}

HardSID::~HardSID()
{
    sid--;
    m_sidFree[m_instance] = 0;
    if (m_handle)
        close (m_handle);
}

void HardSID::reset (void)
{
    for (uint i= 0; i < voices; i++)
        muted[i] = false;
    ioctl(m_handle, HSID_IOCTL_RESET, 0x0f);
    m_accessClk = 0;
}

uint8_t HardSID::read (const uint_least8_t addr)
{
    if (!m_handle)
        return 0;

    event_clock_t cycles = m_eventContext->getTime (m_accessClk);
    m_accessClk += cycles;

    while ( cycles > 0xffff )
    {
        /* delay */
        ioctl(m_handle, HSID_IOCTL_DELAY, 0xffff);
        cycles -= 0xffff;
    }

    uint packet = (( cycles & 0xffff ) << 16 ) | (( addr & 0x1f ) << 8 );
    ioctl(m_handle, HSID_IOCTL_READ, &packet);

    cycles = 0;
    return (uint8_t) (packet & 0xff);
}

void HardSID::write (const uint_least8_t addr, const uint8_t data)
{
    if (!m_handle)
        return;

    event_clock_t cycles = m_eventContext->getTime (m_accessClk);
    m_accessClk += cycles;

    while ( cycles > 0xffff )
    {
        /* delay */
        ioctl(m_handle, HSID_IOCTL_DELAY, 0xffff);
        cycles -= 0xffff;
    }

    uint packet = (( cycles & 0xffff ) << 16 ) | (( addr & 0x1f ) << 8 )
        | (data & 0xff);
    cycles = 0;
    ::write (m_handle, &packet, sizeof (packet));
}

void HardSID::voice (const uint_least8_t num, const uint_least8_t volume,
                     const bool mute)
{
    // Only have 3 voices!
    if (num >= voices)
        return;
    muted[num] = mute;
    
    int cmute = 0;
    for ( uint i = 0; i < voices; i++ )
        cmute |= (muted[i] << i);
    ioctl (m_handle, HSID_IOCTL_MUTE, cmute);
}

void HardSID::filter(bool enable)
{
    ioctl (m_handle, HSID_IOCTL_NOFILTER, enable);
}

void HardSID::flush(void)
{
    ioctl(m_handle, HSID_IOCTL_FLUSH);
}

void HardSID::lock(c64env* env)
{
    if( env == NULL )
    {
        m_locked = false;
        m_eventContext = NULL;
    }
    else
    {
        m_locked = true;
        m_eventContext = &env->context();
    }
}
