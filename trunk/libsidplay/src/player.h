/***************************************************************************
                          player.h  -  description
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 ***************************************************************************/

#ifndef _player_h_
#define _player_h_

#include "config.h"

#ifdef HAVE_MSWINDOWS
// This is the only place this should be defined!
#   undef  SID2_LIB_API
#   define SID2_LIB_API __declspec(dllexport)
#endif

#include "sidplay2.h"
#include "sidenv.h"
#include "mos6510/mos6510.h"
#include "mos6581/mos6581.h"
#include "xsid/xsid.h"
#include "fake6526.h"

class player: private C64Environment
{
private:
    //SID6510  cpu(6510, "Main CPU");
    SID6510  cpu;
    SID      sid;
    SID      sid2;
    XSID     xsid;
    fake6526 cia;

    // User Configuration Settings
    struct   SidTuneInfo tuneInfo;
    SidTune *myTune, *tune;
    uint8_t *ram, *rom;

    sid2_clock_t    _clockSpeed;
    sid2_env_t      _environment;
    const char     *_errorString;
    float64_t       _fastForwardFactor;
    bool            _forced;
    uint_least8_t   _optimiseLevel;
    uint_least32_t  _sampleCount;
    uint_least32_t  _samplingFreq;
    uint_least32_t  _mileage;
    uint_least32_t  _seconds;
    int_least32_t   _userLeftVolume;
    int_least32_t   _userRightVolume;
    enum  {_playing = 0, _paused, _stopped} playerState;

    // Internal Configuration Settings
    uint_least8_t   _channels;
    float64_t       _currentPeriod;
    bool            _forceDualSids;
    sid2_playback_t _playback;
    int             _precision;
    float64_t       _samplingPeriod;
    uint_least32_t  _scaleBuffer;
    int_least32_t   _leftVolume;
    int_least32_t   _rightVolume;

    // C64 environment settings
    float64_t       _cpuFreq;
    uint8_t         _bankReg;
    uint8_t         _initBankReg;
    uint_least16_t  _sidAddress[2];
    bool            _sidEnabled[2];
    bool            _filter;
    bool            _extFilter;
    sid2_model_t    _sidModel;

    // temp stuff -------------
    bool   isKernal;
    bool   isBasic;
    bool   isIO;
    inline void evalBankSelect (uint8_t data);
    void   c64_initialise      (void);
    // ------------------------

private:
    void clock          (void);
    int  initialise     (void);
    void initBankSelect (uint_least16_t addr);
    void nextSequence   (void);
    void mileageCorrect (void)
    {   // If just finished a song, round samples to correct mileage
        if (_sampleCount >= (_samplingFreq / 2))
            _mileage++;
        _sampleCount = 0;
    }

    uint8_t readMemByte_plain     (uint_least16_t addr, bool useCache);
    uint8_t readMemByte_playsid   (uint_least16_t addr, bool useCache);
    uint8_t readMemByte_sidplaytp (uint_least16_t addr, bool useCache);
    uint8_t readMemByte_sidplaybs (uint_least16_t addr, bool useCache);
    void    writeMemByte_playsid  (uint_least16_t addr, uint8_t data, bool useCache);
    void    writeMemByte_sidplay  (uint_least16_t addr, uint8_t data, bool useCache);

    // Use pointers to please requirements of all the provided
    // environments.
    uint8_t (player::*readMemByte)     (uint_least16_t, bool);
    void    (player::*writeMemByte)    (uint_least16_t, uint8_t, bool);
    uint8_t (player::*readMemDataByte) (uint_least16_t, bool);

    // Environment Function entry Points
    inline void    envReset           (void);
    inline uint8_t envReadMemByte     (uint_least16_t addr, bool useCache);
    inline void    envWriteMemByte    (uint_least16_t addr, uint8_t data, bool useCache);
    inline void    envTriggerIRQ      (void);
    inline void    envTriggerNMI      (void);
    inline void    envTriggerRST      (void);
    inline void    envClearIRQ        (void);
    inline bool    envCheckBankJump   (uint_least16_t addr);
    inline uint8_t envReadMemDataByte (uint_least16_t addr, bool useCache);

    // Rev 2.0.3 Added - New Mixer Routines
    void (player::*output) (uint_least16_t clock, void *buffer, uint_least32_t &count);

    // Rev 2.0.4 (saw) - Added to reduce code size
    int_least32_t monoOutGenericMonoIn     (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits);
    int_least32_t monoOutGenericStereoIn   (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits);
    int_least32_t monoOutGenericStereoRIn  (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits);
    int_least32_t stereoOutGenericMonoIn   (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits);
    int_least32_t stereoOutGenericStereoIn (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits,
                                            int_least32_t &sampleR);

    // 8 bit output
    void monoOut8MonoIn       (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void monoOut8StereoIn     (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void monoOut8StereoRIn    (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void stereoOut8MonoIn     (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void stereoOut8StereoIn   (uint_least16_t clock, void *buffer, uint_least32_t &count);

    // Rev 2.0.4 (jp) - Added 16 bit support
    void monoOut16MonoIn      (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void monoOut16StereoIn    (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void monoOut16StereoRIn   (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void stereoOut16MonoIn    (uint_least16_t clock, void *buffer, uint_least32_t &count);
    void stereoOut16StereoIn  (uint_least16_t clock, void *buffer, uint_least32_t &count);

private:
    friend sidplay2;
    player ();
    virtual ~player ();

    int            clockSpeed   (sid2_clock_t clock, bool forced);
    int            configure    (sid2_playback_t mode, uint_least32_t samplingFreq, int precision,
                                 bool forceDualSid);
    int            environment  (sid2_env_t env);
    int            fastForward  (uint_least8_t percent);
    void           getInfo      (sid2_playerInfo_t *info);
    int            loadSong     (const char * const title, const uint_least16_t songNumber);
    int            loadSong     (const uint_least16_t songNumber);
    int            loadSong     (SidTune *requiredTune);
    uint_least8_t  mileage      (void) { return _mileage + _seconds; }
    void           pause        (void);
    uint_least32_t play         (void *buffer, uint_least32_t length);
    void           stop         (void);
    uint_least32_t time         (void) { return _seconds; }

    void           optimisation (uint_least8_t level)
    {
        if (level > SID2_MAX_OPTIMISATION)
            level = SID2_MAX_OPTIMISATION;
        _optimiseLevel = level;
    }

    // Rev 2.0.4 (saw) - Added filter settings
    void filter (bool enabled)
    {
        _filter = enabled;
        sid.enable_filter  (_filter);
        sid2.enable_filter (_filter);
    }

    void extFilter (bool enabled)
    {
        _extFilter = enabled;
        sid.enable_external_filter  (_extFilter);
        sid2.enable_external_filter (_extFilter);
    }

    void sidModel (sid2_model_t model)
    {
        if (_sidModel == model)
	    return;
        _sidModel = model;
        if (_sidModel == SID2_MOS6581)
	{
            sid.set_chip_model  (MOS6581);
            sid2.set_chip_model (MOS6581);
        }
        else // if (_sidModel == SID2_MOS8580
	{
            sid.set_chip_model  (MOS8580);
            sid2.set_chip_model (MOS8580);
        }
    }
};

#endif // _player_h_
