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
#include "sidbuilder.h"

// Default settings
const uint_least32_t SID2_DEFAULT_SAMPLING_FREQ = 44100;
const uint_least8_t  SID2_DEFAULT_PRECISION     = 16;
const uint_least8_t  SID2_DEFAULT_OPTIMISATION  = 0;
const bool           SID2_DEFAULT_SID_SAMPLES   = true; // Samples through sid

// Maximum values
const uint_least8_t SID2_MAX_PRECISION    = 16;
const uint_least8_t SID2_MAX_OPTIMISATION = 2;

// Private Sidplayer
class Player;
class SID_EXTERN sidplay2
{
private:
    Player &sidplayer;

public:
    sidplay2 ();
    virtual ~sidplay2 ();

    const sid2_config_t &configure (void);
    const sid2_info_t   &info      (void);

    int            configure    (const sid2_config_t &cfg);
    const char    *error        (void);
    int            fastForward  (uint percent);
    int            loadSong     (SidTune *tune);
    void           pause        (void);
    uint_least32_t play         (void *buffer, uint_least32_t length);
    sid2_player_t  state        (void);
    void           stop         (void);
    void           debug        (bool enable);

    // Timer functions with respect to 10ths of a second
    uint_least32_t time    (void);
    uint_least32_t mileage (void);

    operator bool()  const { return (&sidplayer ? true: false); }
    bool operator!() const { return (&sidplayer ? false: true); }
};

#endif // _sidplay2_h_
