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
 *  Revision 1.16  2001/08/10 20:03:19  s_a_white
 *  Added RTC reset.
 *
 *  Revision 1.15  2001/07/25 17:01:13  s_a_white
 *  Support for new configuration interface.
 *
 *  Revision 1.14  2001/07/14 16:46:16  s_a_white
 *  Sync with sidbuilder class project.
 *
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

    static const char  *ERR_PSIDDRV_NO_SPACE; 
    static const char  *ERR_PSIDDRV_RELOC;

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
        EventContext &m_eventContext;
        event_clock_t m_seconds;
        float64_t     m_period;
        float64_t     m_fclk;

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
         m_eventContext(*context),
         m_seconds(0)
        {;}

        event_clock_t getTime () {return m_seconds;}

        void reset (void)
        {
            event_clock_t cycles = (event_clock_t) m_period;
            m_seconds = 0;
            m_fclk    = m_period - cycles;
            m_eventContext.schedule (this, cycles);
        }

        void clock (float64_t period)
        {
            m_period = period;
            reset ();
        }   
    } rtc;

    // User Configuration Settings
    sidbuilder   *m_builder;
    SidTuneInfo   m_tuneInfo;
    SidTune      *m_tune;
    uint8_t      *m_ram, *m_rom;
    sid2_info_t   m_info;
    sid2_config_t m_cfg;


    sid2_env_t      m_environment;
    const char     *m_errorString;
    float64_t       m_fastForwardFactor;
    uint_least32_t  m_mileage;
    int_least32_t   m_leftVolume;
    int_least32_t   m_rightVolume;
    volatile sid2_player_t m_playerState;
    volatile bool   m_running;

    // Mixer settings
    float64_t      m_sampleClock;
    float64_t      m_samplePeriod;
    uint_least32_t m_sampleCount;
    uint_least32_t m_sampleIndex;
    char          *m_sampleBuffer;

    // Internal Configuration Settings

    // C64 environment settings
    uint8_t        m_bankReg;
    uint_least16_t m_sidAddress[2];

    // temp stuff -------------
    bool   isKernal;
    bool   isBasic;
    bool   isIO;
    inline void evalBankSelect (uint8_t data);
    void   c64_initialise      (void);
    // ------------------------

private:
    float64_t clockSpeed     (sid2_clock_t clock, bool forced);
    int       environment    (sid2_env_t env);
    void      extFilter      (uint fc);
    int       initialise     (void);
    void      nextSequence   (void);
    void      mixer          (void);
    void      mixerReset     (void);
    void      mileageCorrect (void);
    void      sidEmulation   (sidbuilder *builder);
    void      sidFilter      (bool enable);
    int       sidFilterDef   (const sid_filter_t *filter);
    void      sidModel       (sid2_model_t model);
    void      sidSamples     (bool enable);

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
    {   return m_ram[addr]; }

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
        m_tune->load (name);
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

    void interruptIRQ (const bool state);
    void interruptNMI (void);
    void interruptRST (void);

    // PSID driver
    int  psidDrvInstall (void);
    void psidRelocAddr  (void);

public:
    Player ();

    const sid2_config_t &configure (void) { return m_cfg; }
    const sid2_info_t   &info      (void) { return m_info; }

    int            configure    (const sid2_config_t &cfg);
    int            fastForward  (uint percent);
    int            loadSong     (SidTune *tune);
    uint_least8_t  mileage      (void) { return m_mileage + time(); }
    void           pause        (void);
    uint_least32_t play         (void *buffer, uint_least32_t length);
    sid2_player_t  state        (void) { return m_playerState; }
    void           stop         (void);
    uint_least32_t time         (void) {return rtc.getTime (); }
    void           debug        (bool enable) { cpu->debug (enable); }
    const char    *error        (void) { return m_errorString; }
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
