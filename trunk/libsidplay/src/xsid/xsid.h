/***************************************************************************
                          xsid.h  -  Support for Playsids Extended
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
 *  Revision 1.8  2001/02/21 21:46:34  s_a_white
 *  0x1d = 0 now fixed.  Limit checking on sid volume.  This helps us determine
 *  even better what the sample offset should be (fixes Skate and Die).
 *
 *  Revision 1.7  2001/02/07 21:02:30  s_a_white
 *  Supported for delaying samples for frame simulation.  New alogarithm to
 *  better guess original tunes volume when playing samples.
 *
 *  Revision 1.6  2000/12/12 22:51:01  s_a_white
 *  Bug Fix #122033.
 *
 ***************************************************************************/

/*
Effectively there is only 1 channel, which can either perform Galway Noise
or Sampling.  However, to achieve all the effects on a C64, 2 sampling
channels are required.  No divide by 2 is required and is compensated for
automatically in the C64 machine code.

Confirmed by Warren Pilkington using the tune Turbo Outrun:
A new sample must interrupt an existing sample running on the same channel.

Confirmed by Michael Schwendt and Antonia Vera using the tune Game Over:
A Galway noise sequence cannot interrupt another.  However the last of
these new requested sequences will be played after the current sequence
ends.

Confirmed by Michael Schwendt using the tune Game Over:
Galway noise cannot interrupt a sample sequence.  However the last of
these new requested sequences will be played after the current sequence
ends.

Confirmed by Michael Schwendt using the tune Game Over:
A sample sequence cannot interrupt Galway Noise.  However the last of
these new requested sequences will be played after the current sequence
ends.

Lastly playing samples through the SIDs volume is not as clean as playing
them on their own channel.  Playing through the SID will effect the volume
of the other channels and this will be most noticable at low frequencies.
These effects are however be present in the oringial SID music.

Some SIDs put values directly into the volume register.  Others play samples
with respect to the current volume.  We can't for definate know which the author
has chosen originally.  We must just make a guess based on what the the volume
is initially at the start of a sample sequence and from the details xSID has been
programmed with.
*/

#ifndef _xsid_h_
#define _xsid_h_

#include "config.h"
#include "sidtypes.h"
#include "sidenv.h"

// XSID configuration settings
//#define XSID_DEBUG 1

// Support global debug option
#ifdef DEBUG
#   define XSID_DEBUG DEBUG
#endif

#if XSID_DEBUG
#   include <stdio.h>
#endif

class XSID;
class channel: public C64Environment
{
private:
    // General
    XSID &xsid;

    uint8_t  reg[0x10];
    enum    {FM_NONE = 0, FM_HUELS, FM_GALWAY} mode;
    bool     active;
    void    (channel::*_clock) (void);
    uint_least16_t address;
    uint_least16_t cycleCount; // Counts to zero and triggers!
    uint_least8_t  volShift;
    uint_least8_t  sampleLimit;
    int8_t         sample;
    bool           changed;

    // Sample Section
    uint_least8_t  samRepeat;
    uint_least8_t  samScale;
    enum {SO_LOWHIGH = 0, SO_HIGHLOW = 1};
    uint_least8_t  samOrder;
    uint_least8_t  samNibble;
    uint_least16_t samEndAddr;
    uint_least16_t samRepeatAddr;
    uint_least16_t samPeriod;

    // Galway Section
    uint_least8_t  galTones;
    uint_least8_t  galInitLength;
    uint_least8_t  galLength;
    uint_least8_t  galVolume;
    uint_least8_t  galLoopWait;
    uint_least8_t  galNullWait;

    // For Debugging
    uint_least32_t cycles;
    uint_least32_t outputs;

private:
    void free        (void);
    void silence     (void);
    void sampleInit  (void);
    void sampleClock (void);
    void galwayInit  (void);
    void galwayClock (void);

    // Compress address to not leave so many spaces
    uint_least8_t convertAddr(uint_least8_t addr)
    { return (((addr) & 0x3) | ((addr) >> 3) & 0x0c); }

public:
    channel (XSID *p);
    void    reset    (void);
    void    clock    (uint_least16_t delta_t);
    uint8_t read     (uint_least8_t  addr)
    { return reg[convertAddr (addr)]; }
    void    write    (uint_least8_t addr, uint8_t data)
    { reg[convertAddr (addr)] = data; }
    int8_t  output   (void);
    bool    isGalway (void)
    { return mode == FM_GALWAY; }

    bool hasChanged (void)
    {
        bool v  = changed;
        changed = false;
        return v;
    }

    uint_least8_t limit  (void)
    { return sampleLimit; }

    inline void   checkForInit     (void);
    inline int8_t sampleCalculate  (void);
    inline void   galwayTonePeriod (void);

    // Used to indicate if channel is running
    operator bool()  const { return (active); }
};


class XSID: public C64Environment
{
    friend channel;

private:
    channel ch4;
    channel ch5;
    bool    muted;
    bool    suppressed;

    uint_least16_t      sidAddr0x18;
    uint8_t             sidData0x18;
    bool                _sidSamples;
    int8_t              sampleOffset;
    static const int8_t sampleConvertTable[16];

private:
    void   checkForInit     (channel *ch);
    void   setSidData0x18   (bool cached = false);
    int8_t sampleOutput     (void);
    void   sampleOffsetCalc (void);

public:
    XSID ();

    // Standard calls
    void    reset    (void);
    uint8_t read     (uint_least16_t addr);
    void    write    (uint_least16_t addr, uint8_t data);
    void    clock    (uint_least16_t delta_t = 1);
    void    setEnvironment (C64Environment *envp);

    // Specialist calls
    int_least32_t output (uint_least8_t bits = 16);
    void    mute     (bool enable);
    bool    isMuted  (void) { return muted; }
    void    suppress (bool enable);

    void    sidSamples (bool enable)
    {   _sidSamples = enable; }
    void setSidBaseAddr (uint_least16_t addr)
    {   sidAddr0x18 = addr + 0x18; }
    // Return whether we care it was changed.
    bool updateSidData0x18 (uint8_t data);
};

#endif // _xsid_h_
