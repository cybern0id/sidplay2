/***************************************************************************
                          sidplay2.h  -  Public sidplay header
                             -------------------
    begin                : Fri Jun 9 2000
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

#ifndef _sidplay2_h_
#define _sidplay2_h_

#include "sidtypes.h"
#include "SidTune.h"


// Default settings
const uint_least32_t SID2_DEFAULT_SAMPLING_FREQ = 44100;
const uint_least8_t  SID2_DEFAULT_PRECISION     = 16;
const uint_least8_t  SID2_DEFAULT_OPTIMISATION  = 0;
const bool           SID2_DEFAULT_SID_SAMPLES   = true; // Samples through sid

// Maximum values
const uint_least8_t SID2_MAX_PRECISION    = 16;
const uint_least8_t SID2_MAX_OPTIMISATION = 2;

typedef enum {sid2_left  = 0, sid2_mono,  sid2_stereo, sid2_right} sid2_playback_t;
typedef enum {sid2_envPS = 0, sid2_envTP, sid2_envBS,  sid2_envR } sid2_env_t;
typedef enum {SID2_MOS6581, SID2_MOS8580} sid2_model_t;
typedef enum {SID2_CLOCK_CORRECT, SID2_CLOCK_PAL, SID2_CLOCK_NTSC} sid2_clock_t;

/* Environment Modes
sid_envps = Playsid
sid_envtp = Sidplay  - Transparent Rom
sid_envbs = Sidplay  - Bankswitching
sid_envr  = Sidplay2 - Real C64 Environment
*/

typedef struct
{
    char       *name;
    char       *version;
    SidTuneInfo tuneInfo;
    bool        filter;
    bool        extFilter;
    sid2_env_t  environment;
} sid2_playerInfo_t;

// Private Sidplayer
class player;
class SID_EXPORT sidplay2
{
private:
    player &sidplayer;

public:
    sidplay2 ();
    virtual ~sidplay2 ();

    void           configure    (sid2_playback_t mode, uint_least32_t samplingFreq, uint_least8_t precision, bool forceDualSid);
    void           stop         (void);
    void           pause        (void);
    uint_least32_t play         (void *buffer, uint_least32_t length);
    int            loadSong     (SidTune *tune);
    int            environment  (sid2_env_t env);
    int            fastForward  (uint_least8_t percent);
    void           getInfo      (sid2_playerInfo_t *info);
    void           optimisation (uint_least8_t level);
	int            loadFilter   (const sid_fc_t *cutoffs, uint_least16_t points);

    // Timer functions with respect to 10ths of a second
    uint_least32_t time    (void);
    uint_least32_t mileage (void);
    // Added new filter, model, and clockspeed functions
    void filter     (bool enabled);
    void extFilter  (bool enabled);
    void sidModel   (sid2_model_t model);
    void clockSpeed (sid2_clock_t clock, bool forced = true);
    void sidSamples (bool enable);

    // Rev 1.5 (saw) - Added error loop through support
    const char *getErrorString (void);

    operator bool()  const { return (&sidplayer ? true: false); }
    bool operator!() const { return (&sidplayer ? false: true); }
};

#endif // _sidplay2_h_
