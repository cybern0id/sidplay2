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
 *  Revision 1.13  2001/07/14 12:50:58  s_a_white
 *  Support for credits and debuging.  External filter selection removed.  RTC
 *  and samples obtained in a more efficient way.  Support for component
 *  and sidbuilder classes.
 *
 *  Revision 1.12  2001/04/23 17:09:56  s_a_white
 *  Fixed video speed selection using unforced/forced and NTSC clockSpeeds.
 *
 *  Revision 1.11  2001/03/22 22:45:20  s_a_white
 *  Re-ordered initialisations to match defintions.
 *
 *  Revision 1.10  2001/03/21 23:28:12  s_a_white
 *  Support new component names.
 *
 *  Revision 1.9  2001/03/21 22:32:55  s_a_white
 *  Filter redefinition support.  VIC & NMI support added.
 *
 *  Revision 1.8  2001/03/08 22:48:33  s_a_white
 *  Sid reset on player destruction removed.  Now handled locally by the sids.
 *
 *  Revision 1.7  2001/03/01 23:46:37  s_a_white
 *  Support for sample mode to be selected at runtime.
 *
 *  Revision 1.6  2001/02/28 18:52:55  s_a_white
 *  Removed initBank* related stuff.
 *
 *  Revision 1.5  2001/02/21 21:41:51  s_a_white
 *  Added seperate ram bank to hold C64 player.
 *
 *  Revision 1.4  2001/02/07 20:56:46  s_a_white
 *  Samples now delayed until end of simulated frame.
 *
 *  Revision 1.3  2001/01/23 21:26:28  s_a_white
 *  Only way to load a tune now is by passing in a sidtune object.  This is
 *  required for songlength database support.
 *
 *  Revision 1.2  2001/01/07 15:58:37  s_a_white
 *  SID2_LIB_API now becomes a core define (SID_API).
 *
 *  Revision 1.1  2000/12/12 19:15:40  s_a_white
 *  Renamed from sidplayer
 *
 ***************************************************************************/

#ifndef _player_h_
#define _player_h_

#include "config.h"
#include "sidplay2.h"
#include "sidenv.h"
#include "c64env.h"
#include "c64/c64sid.h"
#include "c64/c64xsid.h"
#include "c64/c64cia.h"
#include "c64/c64vic.h"

#include "mos6510/mos6510.h"

class Player: private C64Environment, c64env
{
private:
    static const double CLOCK_FREQ_NTSC;
    static const double CLOCK_FREQ_PAL;
    static const double VIC_FREQ_PAL;
    static const double VIC_FREQ_NTSC;

    static const char  *TXT_PAL_VBI;
    static const char  *TXT_PAL_VBI_FIXED;
    static const char  *TXT_PAL_CIA;
    static const char  *TXT_PAL_UNKNOWN;
    static const char  *TXT_NTSC_VBI;
    static const char  *TXT_NTSC_VBI_FIXED;
    static const char  *TXT_NTSC_CIA;
    static const char  *TXT_NTSC_UNKNOWN;
    static const char  *TXT_NA;

    static const char  *ERR_CONF_WHILST_ACTIVE;
    static const char  *ERR_UNSUPPORTED_FREQ;
    static const char  *ERR_UNSUPPORTED_PRECISION;
    static const char  *ERR_MEM_ALLOC;
    static const char  *ERR_UNSUPPORTED_MODE;
    static const char  *ERR_FILTER_DEFINITION;
    static const char  *credit[10]; // 10 credits max

    //SID6510  cpu(6510, "Main CPU");
    SID6510 sid6510;
    MOS6510 mos6510;
    MOS6510 *cpu;
    // Sid objects to use.
    c64sid  mos6581_1;
    c64sid  mos6581_2;
    c64xsid xsid;
    c64cia1 cia;
    c64cia2 cia2;
    c64vic  vic;
    sidemu *sid;
    sidemu *sid2;

    sidbuilder *m_builder;

    class EventMixer: public Event
    {
    private:
        Player &m_player;
        void event (void) { m_player.mixer (); }

    public:
        EventMixer (Player *player)
        :Event("Mixer"),
         m_player(*player) {}
    } mixerEvent;
    friend EventMixer;

    class EventRTC: public Event
    {
    private:
        event_clock_t m_seconds;
        float64_t     m_period;
        float64_t     m_fclk;
        EventContext &m_eventContext;

        void    reset   (void)
        {   m_seconds  = 0; }

        void event (void)
        {
            event_clock_t cycles;
            m_seconds++;
            m_fclk += m_period;
            cycles  = (event_clock_t) (m_fclk);
            m_fclk -= cycles;
            m_eventContext.schedule (this, cycles);
        }

    public:
        EventRTC (EventContext *context)
        :Event("RTC"),
         m_eventContext(*context)
        {reset ();}

        event_clock_t getTime () {return m_seconds;}
        void          clock   (float64_t period)
        {
            event_clock_t cycles = (event_clock_t) period;
            reset ();
            m_period = period;
            m_fclk   = m_period - cycles;
            m_eventContext.schedule (this, cycles);
        }
    } rtc;

    // User Configuration Settings
    struct   SidTuneInfo tuneInfo;
    SidTune *_tune;
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
    volatile enum  {_playing = 0, _paused, _stopped} playerState;

    // Mixer settings
    float64_t      m_sampleClock;
    float64_t      m_samplePeriod;
    uint_least32_t m_sampleCount;
    uint_least32_t m_sampleIndex;
    char          *m_sampleBuffer;

    // Internal Configuration Settings
    volatile bool   m_running;
    uint_least8_t   _channels;
    bool            _digiChannel;
    bool            _forceDualSids;
    sid2_playback_t _playback;
    int             _precision;
    int_least32_t   _leftVolume;
    int_least32_t   _rightVolume;

    // C64 environment settings
    float64_t       _cpuFreq;
    uint8_t         _bankReg;
    uint_least16_t  _sidAddress[2];
    bool            _sidEnabled[2];
    bool            _filter;
    sid2_model_t    _sidModel;
    bool            _sidSamples;

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
    void nextSequence   (void);
    void mixer          (void);
    void mixerReset     (void);
    void mileageCorrect (void)
    {   // If just finished a song, round samples to correct mileage
        if (_sampleCount >= (_samplingFreq / 2))
            _mileage++;
        _sampleCount = 0;
    }

    uint8_t readMemByte_player    (uint_least16_t addr);
    uint8_t readMemByte_plain     (uint_least16_t addr);
    uint8_t readMemByte_playsid   (uint_least16_t addr);
    uint8_t readMemByte_sidplaytp (uint_least16_t addr);
    uint8_t readMemByte_sidplaybs (uint_least16_t addr);
    void    writeMemByte_plain    (uint_least16_t addr, uint8_t data);
    void    writeMemByte_playsid  (uint_least16_t addr, uint8_t data);
    void    writeMemByte_sidplay  (uint_least16_t addr, uint8_t data);

    // Use pointers to please requirements of all the provided
    // environments.
    uint8_t (Player::*m_readMemByte)    (uint_least16_t);
    void    (Player::*m_writeMemByte)   (uint_least16_t, uint8_t);
    uint8_t (Player::*m_readMemDataByte)(uint_least16_t);

    uint8_t  readMemRamByte (const uint_least16_t addr)
    {   return ram[addr]; }

    // Environment Function entry Points
    inline void    envReset           (void);
    inline uint8_t envReadMemByte     (uint_least16_t addr);
    inline void    envWriteMemByte    (uint_least16_t addr, uint8_t data);
    inline bool    envCheckBankJump   (uint_least16_t addr);
    inline uint8_t envReadMemDataByte (uint_least16_t addr);

    void   envLoadFile (char *file)
    {
        char name[0x100] = "e:/emulators/c64/games/prgs/";
        strcat (name, file);
        strcat (name, ".sid");
        _tune->load (name);
        stop ();
    }

    // Rev 2.0.3 Added - New Mixer Routines
    uint_least32_t (Player::*output) (char *buffer);

    // Rev 2.0.4 (saw) - Added to reduce code size
    int_least32_t monoOutGenericLeftIn   (uint_least8_t  bits);
    int_least32_t monoOutGenericStereoIn (uint_least8_t  bits);
    int_least32_t monoOutGenericRightIn  (uint_least8_t bits);

    // 8 bit output
    uint_least32_t monoOut8MonoIn       (char *buffer);
    uint_least32_t monoOut8StereoIn     (char *buffer);
    uint_least32_t monoOut8StereoRIn    (char *buffer);
    uint_least32_t stereoOut8MonoIn     (char *buffer);
    uint_least32_t stereoOut8StereoIn   (char *buffer);

    // Rev 2.0.4 (jp) - Added 16 bit support
    uint_least32_t monoOut16MonoIn      (char *buffer);
    uint_least32_t monoOut16StereoIn    (char *buffer);
    uint_least32_t monoOut16StereoRIn   (char *buffer);
    uint_least32_t stereoOut16MonoIn    (char *buffer);
    uint_least32_t stereoOut16StereoIn  (char *buffer);

    void extFilter (uint fc)
    {
        double cutoff = fc;
        mos6581_1.exfilter (cutoff);
        mos6581_1.exfilter (cutoff);
    }

    void interruptIRQ (const bool state);
    void interruptNMI (void);
    void interruptRST (void);

public:
    Player ();

    int            clockSpeed   (sid2_clock_t clock, bool forced);
    int            configure    (sid2_playback_t mode, uint_least32_t samplingFreq, int precision,
                                 bool forceDualSid);
    int            environment  (sid2_env_t env);
    int            fastForward  (uint_least8_t percent);
    void           getInfo      (sid2_playerInfo_t *info);
    int            loadSong     (SidTune *tune);
    uint_least8_t  mileage      (void) { return _mileage + _seconds; }
    void           pause        (void);
    uint_least32_t play         (void *buffer, uint_least32_t length);
    void           stop         (void);
    uint_least32_t time         (void) {return rtc.getTime (); }
    void           sidSamples   (bool enable);
    int            loadFilter   (const sid_fc_t *cutoffs, uint_least16_t points);
    const char   **credits      (void) {return credit;}
    void           emulation    (sidbuilder *builder);
    void           debug        (bool enable) { cpu->debug (enable); }

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
        sid->filter  (enabled);
        sid2->filter (enabled);

        if (m_builder)
        {   // Mirror settings in internal SIDs
            mos6581_1.filter (enabled);
            mos6581_2.filter (enabled);
        }
    }

    void sidModel (sid2_model_t model)
    {
        if (_sidModel == model)
            return;
        _sidModel = model;
        if (model != SID2_MODEL_CORRECT)
        {
            sid->model  (model);
            sid2->model (model);
            if (m_builder)
            {   // Mirror settings in internal SIDs
                mos6581_1.model (model);
                mos6581_2.model (model);
            }
        }
    }

    const char *getErrorString (void)
    {   return _errorString; }
};

inline void Player::interruptIRQ (const bool state)
{
    if (state)
    {
        cpu->triggerIRQ ();
        // Start the sample sequence
        xsid.suppress (false);
        xsid.suppress (true);
    }
    else
        cpu->clearIRQ ();
}

inline void Player::interruptNMI ()
{
    cpu->triggerNMI ();
}

inline void Player::interruptRST ()
{
    stop ();
}

#endif // _player_h_
