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

/*
Effectively there is only 1 channel, which can either perform Gawlay Noise
or Sampling.  However, to achieve all the effects on a C64, 2 sampling
channels are required.  No divide by 2 is required and is compensated for
automatically in the C64 machine code.

Confirmed by Warren Pilkington using the tune Turbo Outrun:
A new sample must interrupt an existing sample running on he same channel.

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
*/

#ifndef _xsid_h_
#define _xsid_h_

#include "config.h"
#include "sidtypes.h"
#include "sidenv.h"

// XSID configuration settings
#define XSID_USE_SID_VOLUME
//#define XSID_DEBUG
//#define XSID_FULL_DEBUG


// Support global debug options
#ifdef DEBUG
#   define XSID_DEBUG
#endif

#ifdef FULL_DEBUG
#   define XSID_FULL_DEBUG
#endif

#ifdef XSID_FULL_DEBUG
#   define XSID_DEBUG
#endif


class channel: public C64Environment
{
private:
    // General
    ubyte_sidt  reg[0x10];
    enum  {FM_NONE = 0, FM_HUELS, FM_GALWAY} mode;
    bool        active;
    uword_sidt  address;
    uword_sidt  cycleCount; // Counts to zero and triggers!
    ubyte_sidt  volShift;
    void  (channel::*_clock) (void);

    // Sample Section
    ubyte_sidt  samRepeat;
    ubyte_sidt  samScale;
    enum       {SO_LOWHIGH = 0, SO_HIGHLOW = 1};
    ubyte_sidt  samOrder;
    ubyte_sidt  samNibble;
    uword_sidt  samEndAddr;
    uword_sidt  samRepeatAddr;
    uword_sidt  samPeriod;

#ifdef XSID_USE_SID_VOLUME
    // Rev 2.0.5 (saw) - Added to locate sample origin
    ubyte_sidt  samMin;
    ubyte_sidt  samMax;
#endif

    // Galway Section
    ubyte_sidt  galTones;
    ubyte_sidt  galInitLength;
    ubyte_sidt  galLength;
    ubyte_sidt  galVolume;
    ubyte_sidt  galLoopWait;
    ubyte_sidt  galNullWait;

private:
    void   free        (void);
    void   silence     (void);
    void   sampleInit  (void);
    void   sampleClock (void);
    void   galwayInit  (void);
    void   galwayClock (void);

    // Compress address to not leave so many spaces
    ubyte_sidt convertAddr(ubyte_sidt addr)
    { return (((addr) & 0x3) | ((addr) >> 3) & 0x0c); }

public:
    channel (void);
    void       reset    (void);
    void       clock    (udword_sidt delta_t);
    ubyte_sidt read     (ubyte_sidt addr)
    { return reg[convertAddr (addr)]; }
    void       write    (ubyte_sidt addr, ubyte_sidt data)
    { reg[convertAddr (addr)] = data; }
    bool       isGalway (void)
    { return mode == FM_GALWAY; }

    inline void       checkForInit     (void);
    inline sbyte_sidt sampleCalculate  (void);
    inline void       galwayTonePeriod (void);

    // Used to indocate if channel is running
    operator bool()  const { return (active);  }
    bool operator!() const { return (!active); }

#ifdef XSID_DEBUG
private:
    udword_sidt cycles;
    udword_sidt outputs;
#endif

#ifdef XSID_USE_SID_VOLUME
public:
    bool        changed;
    sdword_sidt sample;
    ubyte_sidt  output (void);
#else
private:
    static const sbyte_sidt sampleConvertTable[16];

public:
    sdword_sidt sample;
    sbyte_sidt  output (void);
#endif
};


// Rev 2.0.3.  xSID now can properly access the environment
#ifndef XSID_USE_SID_VOLUME
class XSID
#else
class XSID: public C64Environment
#endif
{
private:
    channel     ch4;
    channel     ch5;
    udword_sidt sidVolAddr;

private:
    void checkForInit  (channel *ch);

public:
    void        reset  (void);
    ubyte_sidt  read   (uword_sidt addr);
    void        write  (uword_sidt addr, ubyte_sidt data);
    void        clock  (udword_sidt delta_t = 1);
    void        setEnvironment (C64Environment *envp);

#ifdef XSID_USE_SID_VOLUME
public:
    void setSIDAddress (udword_sidt addr)
    {
        sidVolAddr = addr + 0x18;
    }

    XSID ()
    {
        setSIDAddress (0xd400);
    }

private:
    void setSidVolume (bool cached = false);
    ubyte_sidt output ();
#else
public:
    // This provides standard 16 bit outputs
    sdword_sidt output (ubyte_sidt bits = 16);
#endif
};

#endif // _xsid_h_
