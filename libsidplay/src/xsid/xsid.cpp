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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.11  2001/03/09 22:27:13  s_a_white
 *  Speed optimisation update.
 *
 *  Revision 1.10  2001/03/01 23:45:58  s_a_white
 *  Combined both through sid and non-through sid modes.  Can be selected
 *  at runtime now.
 *
 *  Revision 1.9  2001/02/21 21:46:34  s_a_white
 *  0x1d = 0 now fixed.  Limit checking on sid volume.  This helps us determine
 *  even better what the sample offset should be (fixes Skate and Die).
 *
 *  Revision 1.8  2001/02/07 21:02:30  s_a_white
 *  Supported for delaying samples for frame simulation.  New alogarithm to
 *  better guess original tunes volume when playing samples.
 *
 *  Revision 1.7  2000/12/12 22:51:01  s_a_white
 *  Bug Fix #122033.
 *
 ***************************************************************************/

#include <string.h>
#include <stdio.h>
#include "xsid.h"


// Convert from 4 bit resolution to 8 bits
/* Rev 2.0.5 (saw) - Removed for a more non-linear equivalent
   which better models the SIDS master volume register
const int8_t XSID::sampleConvertTable[16] =
{
    '\x80', '\x91', '\xa2', '\xb3', '\xc4', '\xd5', '\xe6', '\xf7',
    '\x08', '\x19', '\x2a', '\x3b', '\x4c', '\x5d', '\x6e', '\x7f'
};
*/
const int8_t XSID::sampleConvertTable[16] =
{
    '\x80', '\x94', '\xa9', '\xbc', '\xce', '\xe1', '\xf2', '\x03',
    '\x1b', '\x2a', '\x3b', '\x49', '\x58', '\x66', '\x73', '\x7f'
};


channel::channel (XSID *p)
:xsid(*p)
{
    memset (reg, 0, sizeof (reg));
    active = true;
    reset  ();
}

void channel::reset ()
{
    galVolume  = 0; // This is left to free run until reset
    mode       = FM_NONE;
    free ();
}

void channel::free ()
{
    active      = false;
    cycleCount  = 0;
    _clock      = &channel::silence;
    sampleLimit = 0;
    // Set XSID to stopped state
    reg[convertAddr (0x1d)] = 0;
    silence ();
}

inline int8_t channel::output ()
{
    outputs++;
    return sample;
}

void channel::clock (uint_least16_t delta_t)
{   // Emulate a CIA timer
    if (!cycleCount)
        return;

    // Slightly optimised clocking routine
    // for clock periods > than 1
    while (cycleCount <= delta_t)
    {   // Rev 1.4 (saw) - Changed to reflect change above
        cycles  += cycleCount;
        delta_t -= cycleCount;
        (this->*_clock) ();
        // Check to see if the sequence finished
        if (!cycleCount)
            return;
    }
    
    // Reduce the cycleCount by whats left
    cycles     += delta_t;
    cycleCount -= delta_t;
}

void channel::checkForInit ()
{   // Check to see mode of operation
    // See xsid documentation
    uint8_t state = reg[convertAddr (0x1d)];

    if (!state)
        return;
    if (state == 0xfd)
    {
        if (!active)
            return;
        free (); // Stop
        // Calculate the sample offset
        xsid.sampleOffsetCalc ();
    }
    else if (state < 0xfc)
        galwayInit ();
    else // if (state >= 0xfc)
        sampleInit ();
}

void channel::sampleInit ()
{
    if (active && (mode == FM_GALWAY))
        return;

#ifdef XSID_DEBUG
    printf ("XSID [%lu]: Sample Init\n", (unsigned long) this);
    if (active && (mode == FM_HUELS))
        printf ("XSID [%lu]: Stopping Playing Sample\n", (unsigned long) this);
#endif

    // Check all important parameters are legal
    _clock        = &channel::silence;
    volShift      = (uint_least8_t) (0 - (int8_t) reg[convertAddr (0x1d)]) >> 1;
    reg[convertAddr (0x1d)] = 0;
    address       = ((uint_least16_t) reg[convertAddr (0x1f)] << 8) | reg[convertAddr (0x1e)];
    samEndAddr    = ((uint_least16_t) reg[convertAddr (0x3e)] << 8) | reg[convertAddr (0x3d)];
    if (samEndAddr <= address) return;
    samScale      = reg[convertAddr (0x5f)];
    samPeriod     = (((uint_least16_t) reg[convertAddr (0x5e)] << 8) | reg[convertAddr (0x5d)]) >> samScale;
    if (!samPeriod) return;

    // Load the other parameters
    samNibble     = 0;
    samRepeat     = reg[convertAddr (0x3f)];
    samOrder      = reg[convertAddr (0x7d)];
    samRepeatAddr = ((uint_least16_t) reg[convertAddr (0x7f)] << 8) | reg[convertAddr (0x7e)];
    cycleCount    = samPeriod;

    // Support Galway Samples, but that
    // mode it setup only when as Galway
    // Noise sequence begins
    if (mode == FM_NONE)
        mode  = FM_HUELS;

    _clock  = &channel::sampleClock;
    active  = true;
    cycles  = 0;
    outputs = 0;

    sampleLimit = 8 >> volShift;
    sample      = sampleCalculate ();
    changed     = true;

    // Calculate the sample offset
    xsid.sampleOffsetCalc ();

#ifdef XSID_DEBUG
#   if XSID_DEBUG > 1
    printf ("XSID [%lu]: Sample Start Address:  0x%04x\n", (unsigned long) this, address);
    printf ("XSID [%lu]: Sample End Address:    0x%04x\n", (unsigned long) this, samEndAddr);
    printf ("XSID [%lu]: Sample Repeat Address: 0x%04x\n", (unsigned long) this, samRepeatAddr);
    printf ("XSID [%lu]: Sample Period: %u\n", (unsigned long) this, samPeriod);
    printf ("XSID [%lu]: Sample Repeat: %u\n", (unsigned long) this, samRepeat);
    printf ("XSID [%lu]: Sample Order:  %u\n", (unsigned long) this, samOrder);
#   endif
    printf ("XSID [%lu]: Sample Start\n", (unsigned long) this);
#endif // XSID_DEBUG
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
        {   // The sequence has completed
            uint8_t *status = &reg[convertAddr (0x1d)];
            if (!*status)
                *status  = 0xfd;
            if (*status != 0xfd)
                active   = false;
#ifdef XSID_DEBUG
            printf ("XSID [%lu]: Sample Stop (%lu Cycles, %lu Outputs)\n",
                    (unsigned long) this, cycles, outputs);
            if (*status != 0xfd)
            {
                printf ("XSID [%lu]: Starting Delayed Sequence\n",
                        (unsigned long) this);
            }
#endif
            checkForInit ();
            return;
        }
    }

    // We have reached the required sample
    // So now we need to extract the right nibble
    sample  = sampleCalculate ();
    changed = true;
}

int8_t channel::sampleCalculate ()
{
    uint_least8_t tempSample = envReadMemByte (address, true);
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
    samNibble ^= 1;
    samNibble &= 1;
    return (int8_t) ((tempSample & 0x0f) - 0x08) >> volShift;
}

void channel::galwayInit()
{
    // If mode was HUELS, then it's
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
    address  = ((uint_least16_t) reg[convertAddr (0x1f)] << 8) | reg[convertAddr(0x1e)];
    volShift = reg[convertAddr (0x3e)] & 0x0f;
    mode     = FM_GALWAY;
    _clock   = &channel::galwayClock;
    active   = true;
    cycles   = 0;
    outputs  = 0;

    sampleLimit = 8;
    sample      = (int8_t) galVolume - 8;
    changed     = true;

    galwayTonePeriod ();

    // Calculate the sample offset
    xsid.sampleOffsetCalc ();

#ifdef XSID_DEBUG
    printf ("XSID [%lu]: Galway Start\n", (unsigned long) this);
#endif
}

void channel::galwayClock ()
{
    if (--galLength)
        cycleCount = samPeriod;
    else if (galTones == 0xff)
    {   // The sequence has completed
        uint8_t *status = &reg[convertAddr (0x1d)];
        if (!*status)
            *status  = 0xfd;
        if (*status != 0xfd)
            active   = false;
#ifdef XSID_DEBUG
        printf ("XSID [%lu]: Galway Stop (%lu Cycles, %lu Outputs)\n",
                (unsigned long) this, cycles, outputs);
        if (*status != 0xfd)
        {
            printf ("XSID [%lu]: Starting Delayed Sequence\n",
                    (unsigned long) this);
        }
#endif
        checkForInit ();
        return;
    }
    else
        galwayTonePeriod ();

    // See Galway Example...
    galVolume += volShift;
    galVolume &= 0x0f;
    sample     = (int8_t) galVolume - 8;
    changed    = true;
}

void channel::galwayTonePeriod ()
{   // Calculate the number of cycles over which sample should last
    galLength  = galInitLength;
    samPeriod  = envReadMemByte (address + galTones, true);
    samPeriod *= galLoopWait;
    samPeriod += galNullWait;
    cycleCount = samPeriod;
#if XSID_DEBUG > 2
    printf ("XSID [%lu]: Galway Settings\n", (unsigned long) this);
    printf ("XSID [%lu]: Length %u, LoopWait %u, NullWait %u\n",
        (unsigned long) this, galLength, galLoopWait, galNullWait);
    printf ("XSID [%lu]: Tones %u, Data %u\n",
        (unsigned long) this, galTones, envReadMemByte (address + galTones), true);
#endif
    galTones--;
}

void channel::silence ()
{
    sample = 0;
}


XSID::XSID ()
:ch4(this),
 ch5(this)
{
    setSidBaseAddr (0xd400);
    sidSamples (true);
    muted = false;
}

// Use Suppress to delay the samples and start them later
// Effectivly allows running samples in a frame based mode.
void XSID::suppress (bool enable)
{
    // @FIXME@: Mute Temporary Hack
    suppressed = enable | muted;
    if (!suppressed)
    {   // Get the channels running
        ch4.checkForInit ();
        ch5.checkForInit ();
    }
}

// By muting samples they will start and play the at the
// appropriate time but no sound is produced.  Un-muting
// will cause sound output from the current play position.
void XSID::mute (bool enable)
{
    // @FIXME@: Mute not properly implemented
    muted = suppressed = enable;
}

void XSID::write (uint_least16_t addr, uint8_t data)
{
    channel *ch;
    uint8_t tempAddr;

    // Make sure address is legal
    if ((addr & 0xfe8c) ^ 0x000c)
        return;

    ch = &ch4;
    if (addr & 0x0100)
        ch = &ch5;

    tempAddr = (uint8_t) addr;
    ch->write (tempAddr, data);
#if XSID_DEBUG > 1
    printf ("XSID: Addr 0x%02x, Data 0x%02x\n", tempAddr, data);
#endif

    if (addr == 0x1d)
    {
        if (suppressed)
        {
#if XSID_DEBUG
            printf ("XSID: Initialise Suppressed\n");
#endif
            return;
        }
        ch->checkForInit ();
    }
}

uint8_t XSID::read (uint_least16_t addr)
{
    channel *ch;
    uint8_t  tempAddr;

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

    tempAddr = (uint8_t) addr;
    return ch->read (tempAddr);
}

int8_t XSID::sampleOutput (void)
{
    int8_t sample;
    sample  = ch4.output ();
    sample += ch5.output ();
    // Automatically compensated for by C64 code
    //return (sample >> 1);
    return sample;
}

int_least32_t XSID::output (uint_least8_t bits)
{
    int_least32_t sample;
    if (_sidSamples)
        return 0;
    sample = sampleConvertTable[sampleOutput () + 8];
    return sample << (bits - 8);
}

void XSID::reset ()
{
    ch4.reset ();
    ch5.reset ();
    suppressed = false;
    wasRunning = false;
}

void XSID::setSidData0x18 (bool cached)
{
    uint8_t data = (sidData0x18 & 0xf0);
    data |= ((sampleOffset + sampleOutput ()) & 0x0f);

#ifdef XSID_DEBUG
    if ((sampleOffset + sampleOutput ()) > 0x0f)
    {
        printf ("XSID: Sample Wrapped [offset %u, sample %d]\n",
                sampleOffset, sampleOutput ());
    }
#   if XSID_DEBUG > 1
    printf ("XSID: Writing Sample to SID Volume [0x%02x]\n", data);
#   endif
#endif // XSID_DEBUG

    envWriteMemByte (sidAddr0x18, data, false);
}

void  XSID::setEnvironment (C64Environment *envp)
{
    C64Environment::setEnvironment (envp);
    ch4.setEnvironment (envp);
    ch5.setEnvironment (envp);
}

void XSID::sampleOffsetCalc (void)
{
    // Try to determine a sensible offset between voice
    // and sample volumes.
    uint_least8_t lower = ch4.limit () + ch5.limit ();
    uint_least8_t upper;

    // Both channels seem to be off.  Keep current offset!
    if (!lower)
        return;

    sampleOffset = sidData0x18 & 0x0f;

    // Is possible to compensate for both channels
    // set to 4 bits here, but should never happen.
    if (lower > 8)
        lower >>= 1;
    upper = 0x0f - lower + 1;

    // Check against limits
    if (sampleOffset < lower)
        sampleOffset = lower;
    else if (sampleOffset > upper)
        sampleOffset = upper;

#ifdef XSID_DEBUG
    printf ("XSID: Sample Offset %d based on channel(s) ", sampleOffset);
    if (ch4)
        printf ("4 ");
    if (ch5)
        printf ("5");
    printf ("\n");
#endif // XSID_DEBUG
}

bool XSID::updateSidData0x18 (uint8_t data)
{
    sidData0x18 = data;
    if (ch4 || ch5)
    {   // Force volume to be changed at next clock
        sampleOffsetCalc ();
        setSidData0x18   ();
#if XSID_DEBUG
        printf ("XSID: SID Volume Changed Externally (Corrected).\n");
#endif
        return true;
    }
    return false;
}
