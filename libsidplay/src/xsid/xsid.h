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
*/

#ifndef _xsid_h_
#define _xsid_h_

#include "config.h"
#include "sidtypes.h"
#include "sidenv.h"

// XSID configuration settings
#define XSID_USE_SID_VOLUME
//#define XSID_DEBUG 1

// Support global debug option
#ifdef DEBUG
#   define XSID_DEBUG DEBUG
#endif

class channel: public C64Environment
{
private:
    // General
    uint8_t  reg[0x10];
    enum    {FM_NONE = 0, FM_HUELS, FM_GALWAY} mode;
    bool     active;
    void    (channel::*_clock) (void);
    uint_least16_t address;
    uint_least16_t cycleCount; // Counts to zero and triggers!
    uint_least8_t  volShift;

    // Sample Section
    uint_least8_t  samRepeat;
    uint_least8_t  samScale;
    enum {SO_LOWHIGH = 0, SO_HIGHLOW = 1};
    uint_least8_t  samOrder;
    uint_least8_t  samNibble;
    uint_least16_t samEndAddr;
    uint_least16_t samRepeatAddr;
    uint_least16_t samPeriod;

#ifdef XSID_USE_SID_VOLUME
    // Rev 2.0.5 (saw) - Added to locate sample origin
    uint_least8_t  samMin;
    uint_least8_t  samMax;
#endif

    // Galway Section
    uint_least8_t  galTones;
    uint_least8_t  galInitLength;
    uint_least8_t  galLength;
    uint_least8_t  galVolume;
    uint_least8_t  galLoopWait;
    uint_least8_t  galNullWait;

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
    channel (void);
    void    reset    (void);
    void    clock    (uint_least16_t delta_t);
    uint8_t read     (uint_least8_t  addr)
    { return reg[convertAddr (addr)]; }
    void    write    (uint_least8_t addr, uint8_t data)
    { reg[convertAddr (addr)] = data; }
    bool    isGalway (void)
    { return mode == FM_GALWAY; }

    inline void   checkForInit     (void);
    inline int8_t sampleCalculate  (void);
    inline void   galwayTonePeriod (void);

    // Used to indocate if channel is running
    operator bool()  const { return (active);  }
    bool operator!() const { return (!active); }

#ifdef XSID_DEBUG
private:
    uint_fast32_t cycles;
    uint_fast32_t outputs;
#endif

#ifdef XSID_USE_SID_VOLUME
public:
    bool    changed;
    uint8_t sample;
    uint8_t output (void);
#else
private:
    static const int8_t sampleConvertTable[16];

public:
    int_least32_t sample;
    int_least32_t output (void);
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
    channel ch4;
    channel ch5;
    bool    muted;
    uint_least16_t sidVolAddr;

private:
    void checkForInit  (channel *ch);

public:
    void    reset (void);
    void    mute  (bool enable)
    {   muted = enable; }
    uint8_t read  (uint_least16_t addr);
    void    write (uint_least16_t addr, uint8_t data);
    void    clock (uint_least16_t delta_t = 1);
    void    setEnvironment (C64Environment *envp);

#ifdef XSID_USE_SID_VOLUME
public:
    void setSIDAddress (uint_least16_t addr)
    {
        sidVolAddr = addr + 0x18;
    }

    XSID ()
    {
        setSIDAddress (0xd400);
        muted = false;
    }

private:
    void    setSidVolume (bool cached = false);
    uint8_t output ();
#else
public:
    // This provides standard 16 bit outputs
    int_least32_t output (uint_least8_t bits = 16);
#endif
};

#endif // _xsid_h_
