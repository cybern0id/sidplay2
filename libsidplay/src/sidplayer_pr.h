/***************************************************************************
                          sidplayer_pr.h  -  description
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

#ifndef _sidplayer_pr_h_
#define _sidplayer_pr_h_

#include "config.h"
#include "sidplayer.h"
#include "sidenv.h"
#include "mos6510/mos6510.h"
#include "mos6581/mos6581.h"
#include "xsid/xsid.h"
#include "fake6526.h"

class sidplayer_pr: private C64Environment
{
private:
    //SID6510  cpu(6510, "Main CPU");
    SID6510  cpu;
    SID      sid;
    SID      sid2;
    XSID     xsid;
    fake6526 cia;
    SidTune *tune, *myTune;
    struct   SidTuneInfo tuneInfo;

    enum  {_playing = 0, _paused, _stopped} playerState;
    ubyte_sidt *ram;
    ubyte_sidt *rom;

    // Player configureation options
    env_sidt      _environment;
    playback_sidt _playback;
    udword_sidt   _samplingFreq;
    int           _precision;

    // C64 environment settings
    double        _cpuFreq;
    ubyte_sidt    _bankReg;

    // Rev 2.0.2 Added - Sidplay compatibility
    ubyte_sidt    _initBankReg;

    // Rev 2.0.3 Added - New Mixer
    udword_sidt   _leftVolume;
    udword_sidt   _rightVolume;
    bool          _sidEnabled[2];
    bool          _forceDualSids;
    int           _optimiseLevel;

    // Rev 2.0.4 (saw) - Added for new timer support
    udword_sidt   _scaleBuffer;
    double        _samplingPeriod;
    double        _currentPeriod;
    udword_sidt   _sampleCount;
    udword_sidt   _seconds;
    bool          _updateClock;
    udword_sidt   _channels;

    bool          _filter;
    bool          _extFilter;
    model_sidt    _sidModel;
    clock_sidt    _clockSpeed;

    // Rev 1.5 (saw) - Added to allow pass through of error
    // messages.
    const char   *_errorString;

    // temp stuff -------------
    bool   isKernal;
    bool   isBasic;
    bool   isIO;
    inline void evalBankSelect (ubyte_sidt data);
    void   c64_initialise      (void);
    // ------------------------

private:
    void       clock          (void);
    int        initialise     (void);
    void       initBankSelect (uword_sidt addr);
    void       nextSequence   (void);

    ubyte_sidt readMemByte_plain     (uword_sidt addr, bool useCache);
    ubyte_sidt readMemByte_playsid   (uword_sidt addr, bool useCache);
    ubyte_sidt readMemByte_sidplaytp (uword_sidt addr, bool useCache);
    ubyte_sidt readMemByte_sidplaybs (uword_sidt addr, bool useCache);
    void       writeMemByte_playsid  (uword_sidt addr, ubyte_sidt data, bool useCache);
    void       writeMemByte_sidplay  (uword_sidt addr, ubyte_sidt data, bool useCache);

    // Use pointers to please requirements of all the provided
    // environments.
    ubyte_sidt (sidplayer_pr::*readMemByte)     (uword_sidt, bool useCache);
    void       (sidplayer_pr::*writeMemByte)    (uword_sidt, ubyte_sidt, bool useCache);
    ubyte_sidt (sidplayer_pr::*readMemDataByte) (uword_sidt, bool useCache);

    // Environment Function entry Points
    inline void       envReset           (void);
    inline ubyte_sidt envReadMemByte     (uword_sidt addr, bool useCache);
    inline void       envWriteMemByte    (uword_sidt addr, ubyte_sidt data, bool useCache);
    inline void       envTriggerIRQ      (void);
    inline void       envTriggerNMI      (void);
    inline void       envTriggerRST      (void);
    inline void       envClearIRQ        (void);
    inline bool       envCheckBankJump   (uword_sidt addr);
    inline ubyte_sidt envReadMemDataByte (uword_sidt addr, bool useCache);

    // Rev 2.0.3 Added - New Mixer Routines
    void (sidplayer_pr::*output) (uword_sidt clock, void *buffer, udword_sidt &count);

    // Rev 2.0.4 (saw) - Added to reduce code size
    sdword_sidt monoOutGenericMonoIn     (uword_sidt clock, udword_sidt &count, ubyte_sidt bits);
    sdword_sidt monoOutGenericStereoIn   (uword_sidt clock, udword_sidt &count, ubyte_sidt bits);
    sdword_sidt monoOutGenericStereoRIn  (uword_sidt clock, udword_sidt &count, ubyte_sidt bits);
    sdword_sidt stereoOutGenericMonoIn   (uword_sidt clock, udword_sidt &count, ubyte_sidt bits);
    sdword_sidt stereoOutGenericStereoIn (uword_sidt clock, udword_sidt &count, ubyte_sidt bits, sdword_sidt &sampleR);

    // 8 bit output
    void monoOut8MonoIn      (uword_sidt clock, void *buffer, udword_sidt &count);
    void monoOut8StereoIn    (uword_sidt clock, void *buffer, udword_sidt &count);
    void monoOut8StereoRIn   (uword_sidt clock, void *buffer, udword_sidt &count);
    void stereoOut8MonoIn    (uword_sidt clock, void *buffer, udword_sidt &count);
    void stereoOut8StereoIn  (uword_sidt clock, void *buffer, udword_sidt &count);

    // Rev 2.0.4 (jp) - Added 16 bit support
    void monoOut16MonoIn      (uword_sidt clock, void *buffer, udword_sidt &count);
    void monoOut16StereoIn    (uword_sidt clock, void *buffer, udword_sidt &count);
    void monoOut16StereoRIn   (uword_sidt clock, void *buffer, udword_sidt &count);
    void stereoOut16MonoIn    (uword_sidt clock, void *buffer, udword_sidt &count);
    void stereoOut16StereoIn  (uword_sidt clock, void *buffer, udword_sidt &count);

private:
    friend sidplayer;
    sidplayer_pr ();
    virtual ~sidplayer_pr ();

    int         configure    (playback_sidt mode, udword_sidt samplingFreq, int precision, bool forceDualSid);
    void        stop         (void);
    void        pause        (void);
    udword_sidt play         (void *buffer, udword_sidt length);
    int         loadSong     (const char * const title, const uword_sidt songNumber);
    int         loadSong     (const uword_sidt songNumber);
    int         loadSong     (SidTune *requiredTune);
    int         environment  (env_sidt env);
    void        getInfo      (playerInfo_sidt *info);
    void        optimisation (int level)
    {
        if (level > SIDPLAYER_MAX_OPTIMISATION)
            level = SIDPLAYER_MAX_OPTIMISATION;
        _optimiseLevel = level;
    }

    // Rev 2.0.4 (saw) - Added new timer functions
    udword_sidt time         (void) { return _seconds; }
    bool        updateClock  (void)
    {
        bool update  = _updateClock;
        _updateClock = false;
        return update;
    }

    // Rev 2.0.4 (saw) - Added filter settings
    void filter    (bool enabled)
    {
        _filter = enabled;
        sid.enable_filter (_filter);
    }
    void extFilter (bool enabled)
    {
        _extFilter = enabled;
        sid2.enable_external_filter (_extFilter);
    }
    void sidModel  (model_sidt model)
    {
        _sidModel = model;
        if (_sidModel == SID_MOS6581)
            sid.set_chip_model (MOS6581);
        else // if (_sidModel == SID_MOS8580
            sid.set_chip_model (MOS8580);
    }
    void clockSpeed (clock_sidt clock)
    {
        _clockSpeed      = clock;
    }

    int  loadFilter      (long ini_fd);
    int  loadFilterType1 (long ini_fd);
    int  loadFilterType2 (long ini_fd);

    const ubyte_sidt *dumpRam (void)
    {   return ram; }
};

#endif // _sidplayer_pr_h_
