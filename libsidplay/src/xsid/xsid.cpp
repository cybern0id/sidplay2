/***************************************************************************
                          xsid.cpp  -  Support for Playsids Extended
                                       Registers
                             -------------------
    begin                : Tue Jun 20 2000
    copyright            : (C) 2000 by Simon White
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

#include <string.h>
#include <stdio.h>
#include "xsid.h"


#ifndef XSID_USE_SID_VOLUME
// Convert from 4 bit resolution to 8 bits
/* Rev 2.0.5 (saw) - Removed for a more non-linear equivalent
   which better models the SIDS master volume register
const sbyte_sidt channel::sampleConvertTable[16] =
{
    '\x80', '\x91', '\xa2', '\xb3', '\xc4', '\xd5', '\xe6', '\xf7',
    '\x08', '\x19', '\x2a', '\x3b', '\x4c', '\x5d', '\x6e', '\x7f'
};
*/
const sbyte_sidt channel::sampleConvertTable[16] =
{
    '\x80', '\x94', '\xa9', '\xbc', '\xce', '\xe1', '\xf2', '\x03',
    '\x1b', '\x2a', '\x3b', '\x49', '\x58', '\x66', '\x73', '\x7f'
};
#endif

channel::channel ()
{
    memset (reg, 0, sizeof (reg));
    reset  ();
}

void channel::reset ()
{
    galVolume = 0; // This is left to free run until reset
    mode      = FM_NONE;
    // Set XSID to stopped state
    reg[convertAddr (0x1d)] = 0;
    free ();
}

inline void channel::clock (udword_sidt delta_t)
{   // Emulate a CIA timer
    if (!cycleCount) return;
    if (delta_t == 1)
    {   // Rev 1.4 (saw) - Optimisation to prevent _clock being
        // called un-necessarily.
#ifdef XSID_DEBUG
        cycles++;
#endif
        if (!--cycleCount)
            (this->*_clock) ();
        return;
    }

    // Slightly optimised clocking routine
    // for clock periods > than 1
    while (cycleCount <= delta_t)
    {   // Rev 1.4 (saw) - Changed to reflect change above
#ifdef XSID_DEBUG
        cycles  += cycleCount;
#endif
        delta_t -= cycleCount;
        (this->*_clock) ();
        // Check to see if the sequence finished
        if (!cycleCount) return;
    }
    
    // Reduce the cycleCount by whats left
#ifdef XSID_DEBUG
    cycles     += delta_t;
#endif
    cycleCount -= delta_t;
}

#ifndef XSID_USE_SID_VOLUME
inline sbyte_sidt channel::output ()
#else
inline ubyte_sidt channel::output ()
#endif
{
#ifdef XSID_DEBUG
    outputs++;
#endif
    return sample;
}

void channel::free ()
{
    active     = false;
    cycleCount = 0;
    _clock     = &channel::silence;
    silence ();
}

void channel::checkForInit ()
{   // Check to see mode of operation
    // See xsid documentation
    ubyte_sidt state = reg[convertAddr (0x1d)];

    if (!state) return;
    if (state == 0xFD)
    {   // We want xSID to stop
        reset ();
    }
    else if (state < 0xFC)
    {
        galwayInit ();
    }
    else // if (state >= 0xFC)
    {
	    sampleInit ();
    }
}

void channel::sampleInit ()
{
    if (active && (mode == FM_GALWAY))
        return;

#ifdef XSID_DEBUG
    printf ("XSID [%lu]: Sample Init\n", (unsigned long) this);

    if (active && (mode == FM_HEULS))
        printf ("XSID [%lu]: Stopping Playing Sample\n", (unsigned long) this);
#endif

    // Check all important parameters are legal
    _clock        = &channel::silence;
    volShift      = (ubyte_sidt) (0 - (sbyte_sidt) reg[convertAddr (0x1d)]) >> 1;
    reg[convertAddr (0x1d)] = 0;
    address       = ((uword_sidt) reg[convertAddr (0x1f)] << 8) | reg[convertAddr (0x1e)];
    samEndAddr    = ((uword_sidt) reg[convertAddr (0x3e)] << 8) | reg[convertAddr (0x3d)];
    if (samEndAddr <= address) return;
    samScale      = reg[convertAddr (0x5f)];
    samPeriod     = (((uword_sidt) reg[convertAddr (0x5e)] << 8) | reg[convertAddr (0x5d)]) >> samScale;
    if (!samPeriod) return;

    // Load the other parameters
    samNibble     = 0;
    samRepeat     = reg[convertAddr (0x3f)];
    samOrder      = reg[convertAddr (0x7d)];
    samRepeatAddr = ((uword_sidt) reg[convertAddr (0x7f)] << 8) | reg[convertAddr (0x7e)];
    cycleCount    = samPeriod;

    // Support Galway Samples, but that
    // mode it setup only when as Galway
    // Noise sequence begins
    if (mode == FM_NONE)
        mode  = FM_HUELS;
    active  = true;
    _clock  = &channel::sampleClock;
    sample  = sampleCalculate ();

#ifdef XSID_USE_SID_VOLUME
    changed = true;
    // Rev 2.0.5 (saw) - Added to track origin
    samMin  = sample;
    samMax  = sample;
#endif

#ifdef XSID_FULL_DEBUG
    printf ("XSID [%lu]: Sample Start Address:  0x%04x\n", (unsigned long) this, address);
    printf ("XSID [%lu]: Sample End Address:    0x%04x\n", (unsigned long) this, samEndAddr);
    printf ("XSID [%lu]: Sample Repeat Address: 0x%04x\n", (unsigned long) this, samRepeatAddr);
    printf ("XSID [%lu]: Sample Period: %u\n", (unsigned long) this, samPeriod);
    printf ("XSID [%lu]: Sample Repeat: %u\n", (unsigned long) this, samRepeat);
#endif

#ifdef XSID_DEBUG
    cycles  = 0;
    outputs = 0;
    printf ("XSID [%lu]: Sample Start\n", (unsigned long) this);
#endif
}

void channel::sampleClock ()
{
    cycleCount   = samPeriod;
    if (address >= samEndAddr)
    {
        if (samRepeat != 0xFF)
        {
            if (samRepeat)
                samRepeat--;
            else
                samRepeatAddr = address;
        }

        address      = samRepeatAddr;
        if (address >= samEndAddr)
        {
#ifdef XSID_DEBUG
            printf ("XSID [%lu]: Sample Stop (%lu Cycles, %lu Outputs)\n",
                (unsigned long) this, cycles, outputs);
#endif
#ifdef XSID_USE_SID_VOLUME
            // Rev 2.0.5 (saw) - Locate Origin
            sample  = (samMin + samMax) / 2;
            changed = true;
#endif
            free ();
            checkForInit ();
            return;
        }
    }

    // We have reached the required sample
    // So now we need to extract the right nibble
    sample     = sampleCalculate ();

#ifdef XSID_USE_SID_VOLUME
    // Rev 2.0.5 (saw) - Added to track origin
    if (sample < samMin)
        samMin = sample;
    else if (sample > samMax)
        samMax = sample;
    changed    = true;
#endif
    samNibble &= 1;
}

sbyte_sidt channel::sampleCalculate ()
{
    ubyte_sidt tempSample = envReadMemByte (address);
    if (samOrder == SO_LOWHIGH)
    {
        if (samScale == 0)
        {
            if (samNibble != 0)
                tempSample >>= 4;
        }
        // AND 15 further below.
    }
    else // if (samOrder == SO_HIGHLOW)
    {
        if (samScale == 0)
        {
            if (samNibble == 0)
                tempSample >>= 4;
        }
        else // if (samScale != 0)
            tempSample >>= 4;
        // AND 15 further below.
    }

    // Move to next address
    address   += samNibble;
    samNibble ^= 1;;
#ifndef XSID_USE_SID_VOLUME
    return sampleConvertTable[(tempSample & 0x0f) >> volShift];
#else
    return ((tempSample & 0x0f) >> volShift);
#endif
}

void channel::galwayInit()
{
    // If mode was HEULS, then it's
    // gonna become GALWAY and we should
    // not interrupt current sample
    mode = FM_GALWAY;
    if (active)
        return;

#ifdef XSID_DEBUG
    printf ("XSID [%lu]: Galway Init\n", (unsigned long) this);
#endif

    // Check all important parameters are legal
    _clock        = &channel::silence;
    galTones      = reg[convertAddr (0x1d)];
    reg[convertAddr (0x1d)] = 0;
    galInitLength = reg[convertAddr (0x3d)];
    if (!galInitLength) return;
    galLoopWait   = reg[convertAddr (0x3f)];
    if (!galLoopWait)   return;
    galNullWait   = reg[convertAddr (0x5d)];
    if (!galNullWait)   return;

    // Load the other parameters
    address  = ((uword_sidt) reg[convertAddr (0x1f)] << 8) | reg[convertAddr(0x1e)];
    volShift = reg[convertAddr (0x3e)] & 0x0f;
    mode     = FM_GALWAY;
    active   = true;

#ifndef XSID_USE_SID_VOLUME
    sample  = sampleConvertTable[galVolume];
#else
    sample  = galVolume;
    changed = true;
#endif

    galwayTonePeriod ();
    _clock  = &channel::galwayClock;

#ifdef XSID_DEBUG
    cycles  = 0;
    outputs = 0;
    printf ("XSID [%lu]: Galway Start\n", (unsigned long) this);
#endif
}

void channel::galwayClock ()
{
    if (--galLength)
    {
        cycleCount = samPeriod;
    }
    else if (galTones == 0xff)
    {   // The counter has completed
#ifdef XSID_DEBUG
        printf ("XSID [%lu]: Galway Stop (%lu Cycles, %lu Outputs)\n",
            (unsigned long) this, cycles, outputs);
        if (reg[convertAddr (0x1d)])
            printf ("XSID [%lu]: Starting Delayed Sequence\n");
#endif
        free ();
        checkForInit ();
        return;
    }
    else
    {
        galwayTonePeriod ();
    }

    // See Galway Example...
    galVolume += volShift;
    galVolume &= 0x0f;
#ifndef XSID_USE_SID_VOLUME
    sample  = sampleConvertTable[galVolume];
#else
    sample  = galVolume;
    changed = true;
#endif
}

void channel::galwayTonePeriod ()
{   // Calculate the number of cycles over which sample should last
    galLength  = galInitLength;
    samPeriod  = envReadMemByte (address + galTones);
    samPeriod *= galLoopWait;
    samPeriod += galNullWait;
    cycleCount = samPeriod;
    galTones--;
}

void channel::silence ()
{
#ifndef XSID_USE_SID_VOLUME
    sample = 0;
#endif
}


void XSID::write (uword_sidt addr, ubyte_sidt data)
{
    channel   *ch;
    ubyte_sidt tempAddr;

    // Make sure address is legal
    if ((addr & 0xfe8c) ^ 0x000c)
        return;

    ch = &ch4;
    if (addr & 0x0100)
        ch = &ch5;

    tempAddr = (ubyte_sidt) addr;
    ch->write (tempAddr, data);
#ifdef XSID_FULL_DEBUG
    printf ("XSID: Addr 0x%02x, Data 0x%02x\n", tempAddr, data);
#endif

    if (addr == 0x1d)
        ch->checkForInit ();
}

ubyte_sidt XSID::read (uword_sidt addr)
{
    channel   *ch;
    ubyte_sidt tempAddr;

    // Make sure address is legal
    if ((addr & 0xfe8c) ^ 0xd40c)
        return 0;
    if ((addr & 0x001f) < 0x001d)
    {   // Should never happen
        return 0;
    }

    ch = &ch4;
    if (addr & 0x0100)
        ch = &ch5;

    tempAddr = (ubyte_sidt) addr;
    return ch->read (tempAddr);
}

// This function is a mess but is remapped according
// to the mode.  It's kept like this so I don't have to
// keep updating 2 bits of code
#ifndef XSID_USE_SID_VOLUME
sdword_sidt XSID::output (ubyte_sidt bits)
{
    sdword_sidt sample = 0;
#else
ubyte_sidt XSID::output ()
{
    ubyte_sidt sample = 0;
#endif // XSID_USE_SID_VOLUME
    sample += ch4.output ();
    sample += ch5.output ();
#ifndef XSID_USE_SID_VOLUME
    sample <<= (bits - 8);
#endif
    // Automatically compensated for by C64 code
//    return (sample >> 1);
    return sample;
}

void XSID::reset ()
{
    ch4.reset ();
    ch5.reset ();
}

#ifdef XSID_USE_SID_VOLUME
void XSID::setSidVolume (bool cached)
{
    if (ch4.changed || ch5.changed)
    {
        ubyte_sidt volume = output () & 0x0f;
        volume |= (envReadMemDataByte (sidVolAddr, true) & 0xf0);
        ch4.changed = false;
        ch5.changed = false;
        envWriteMemByte (sidVolAddr, volume, false);
    }
}
#endif

void XSID::clock (udword_sidt delta_t)
{
#ifdef XSID_USE_SID_VOLUME
    bool wasRunning;
    wasRunning = ch4 || ch5;
#endif

    // Call optimised clock routine
    // Well it's better then just doing:
    // while (delta_t--)
    //     clock ();
    ch4.clock (delta_t);
    ch5.clock (delta_t);

#ifdef XSID_USE_SID_VOLUME
    if (ch4 || ch5)
        setSidVolume ();
    else if (wasRunning)
    {   // Rev 2.0.5 (saw) - Changed to restore volume different depending on mode
        // Normally after samples volume should be restored to half volume,
        // however, Galway Tunes sound horrible and seem to require setting back to
        // the original volume.  Setting back to the original volume for normal
        // samples can have nasty pulsing effects
        if (ch4.isGalway ())
        {
            ubyte_sidt sidVolume = envReadMemDataByte (sidVolAddr, true);
            envWriteMemByte (sidVolAddr, sidVolume);
        }
        else
            setSidVolume (true);
    }
#endif
}

void  XSID::setEnvironment (C64Environment *envp)
{
#ifdef XSID_USE_SID_VOLUME
    C64Environment::setEnvironment (envp);
#endif

    ch4.setEnvironment (envp);
    ch5.setEnvironment (envp);
}
