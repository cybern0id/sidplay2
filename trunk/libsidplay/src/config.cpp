/***************************************************************************
                          config.cpp  -  Library Configuration Code
                             -------------------
    begin                : Fri Jul 27 2001
    copyright            : (C) 2001 by Simon White
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
 *  Revision 1.8  2001/09/20 20:33:54  s_a_white
 *  sid2 now gets correctly set to nullsid for a bad create call.
 *
 *  Revision 1.7  2001/09/20 19:34:11  s_a_white
 *  Error checking added for the builder create calls.
 *
 *  Revision 1.6  2001/09/17 19:02:38  s_a_white
 *  Now uses fixed point maths for sample output and rtc.
 *
 *  Revision 1.5  2001/09/01 11:13:56  s_a_white
 *  Fixes sidplay1 environment modes.
 *
 *  Revision 1.4  2001/08/20 18:24:50  s_a_white
 *  tuneInfo in the info structure now correctly has the sid revision setup.
 *
 *  Revision 1.3  2001/08/12 18:22:45  s_a_white
 *  Fixed bug in Player::sidEmulation call.
 *
 *  Revision 1.2  2001/07/27 12:51:40  s_a_white
 *  Removed warning.
 *
 *  Revision 1.1  2001/07/27 12:12:23  s_a_white
 *  Initial release.
 *
 ***************************************************************************/

#include "sid2types.h"
#include "player.h"

// An instance of this structure is used to transport emulator settings
// to and from the interface class.

int Player::config (const sid2_config_t &cfg)
{
    if (m_running)
    {
        m_errorString = ERR_CONF_WHILST_ACTIVE;
        goto Player_configure_error;
    }

    // Check for base sampling frequency
    if ((cfg.frequency < 4000) || (cfg.frequency > 96000))
    {   // Rev 1.6 (saw) - Added descriptive error
        m_errorString = ERR_UNSUPPORTED_FREQ;
        goto Player_configure_error;
    }

    // Check for legal precision
    switch (cfg.precision)
    {
    case 8:
    case 16:
    case 24:
        if (cfg.precision > SID2_MAX_PRECISION)
        {   // Rev 1.6 (saw) - Added descriptive error
            m_errorString = ERR_UNSUPPORTED_PRECISION;
            goto Player_configure_error;
        }
    break;

    default:
        // Rev 1.6 (saw) - Added descriptive error
        m_errorString = ERR_UNSUPPORTED_PRECISION;
        goto Player_configure_error;
    }
   
    // Only do these if we have a loaded tune
    if (m_tune)
    {
        float64_t cpuFreq;
        // External Setups
        if (sidCreate (cfg.sidEmulation, cfg.sidModel) < 0)
        {
            m_errorString      = cfg.sidEmulation->error ();
            m_cfg.sidEmulation = NULL;
            goto Player_configure_restore;
        }
        // Must be this order:
        // Determine clock speed
        cpuFreq = clockSpeed (cfg.clockSpeed, cfg.clockForced);
        // Fixed point conversion 16.16
        m_samplePeriod = (event_clock_t) (cpuFreq /
                         (float64_t) cfg.frequency *
                         (1 << 16) * m_fastForwardFactor);
        // Setup fake cia
        sid6526.clock ((uint_least16_t)(cpuFreq / VIC_FREQ_PAL + 0.5));
        if (m_tuneInfo.songSpeed  == SIDTUNE_SPEED_CIA_1A ||
            m_tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
        {
            sid6526.clock ((uint_least16_t)(cpuFreq / VIC_FREQ_NTSC + 0.5));
        }
        // Configure, setup and install C64 environment/events
        if (environment (cfg.environment) < 0)
            goto Player_configure_restore;
        // Start the real time clock event
        rtc.clock (cpuFreq);
    }
    sidSamples (cfg.sidSamples);

    // All parameters check out, so configure player.
    m_info.channels = 1;
    if (cfg.playback == sid2_stereo)
        m_info.channels++;

    m_sidAddress[0]  = m_tuneInfo.sidChipBase1;
    m_sidAddress[1]  = m_tuneInfo.sidChipBase2;

    // Only force dual sids if second wasn't detected
    if (!m_sidAddress[1] && cfg.forceDualSids)
        m_sidAddress[1] = 0xd500; // Assumed

    m_leftVolume  = cfg.leftVolume;
    m_rightVolume = cfg.rightVolume;

    if (cfg.playback != sid2_mono)
    {   // Try Spliting channels across 2 sids
        if (!m_sidAddress[1])
        {
            m_sidAddress[1] = m_sidAddress[0];

            // Mute Voices
            sid->voice  (0, 0, true);
            sid->voice  (2, 0, true);
            sid2->voice (1, 0, true);
            // 2 Voices scaled to unity from 4 (was !SID_VOL)
            //    m_leftVolume  *= 2;
            //    m_rightVolume *= 2;
            // 2 Voices scaled to unity from 3 (was SID_VOL)
            //        m_leftVolume  *= 3;
            //        m_leftVolume  /= 2;
            //    m_rightVolume *= 3;
            //    m_rightVolume /= 2;
        }

        if (cfg.playback == sid2_left)
            xsid.mute (true);
    }

    // Setup the audio side, depending on the audio hardware
    // and the information returned by sidtune
    switch (cfg.precision)
    {
    case 8:
        if (!m_sidAddress[1])
        {
            if (cfg.playback == sid2_stereo)
                output = &Player::stereoOut8MonoIn;
            else
                output = &Player::monoOut8MonoIn;
        }
        else
        {
            switch (cfg.playback)
            {
            case sid2_stereo: // Stereo Hardware
                output = &Player::stereoOut8StereoIn;
            break;

            case sid2_right: // Mono Hardware,
                output = &Player::monoOut8StereoRIn;
            break;

            case sid2_left:
                output = &Player::monoOut8MonoIn;
            break;

            case sid2_mono:
                output = &Player::monoOut8StereoIn;
            break;
            }
        }
    break;
            
    case 16:
        if (!m_sidAddress[1])
        {
            if (cfg.playback == sid2_stereo)
                output = &Player::stereoOut16MonoIn;
            else
                output = &Player::monoOut16MonoIn;
        }
        else
        {
            switch (cfg.playback)
            {
            case sid2_stereo: // Stereo Hardware
                output = &Player::stereoOut16StereoIn;
            break;

            case sid2_right: // Mono Hardware,
                output = &Player::monoOut16StereoRIn;
            break;

            case sid2_left:
                output = &Player::monoOut16MonoIn;
            break;

            case sid2_mono:
                output = &Player::monoOut16StereoIn;
            break;
            }
        }
    }

    // Update Configuration
    m_cfg = cfg;

    if (m_cfg.optimisation > SID2_MAX_OPTIMISATION)
        m_cfg.optimisation = SID2_MAX_OPTIMISATION;    
return 0;

Player_configure_restore:
    config (m_cfg);
Player_configure_error:
    return -1;
}

// Clock speed changes due to loading a new song
float64_t Player::clockSpeed (sid2_clock_t clock, bool forced)
{
    float64_t cpuFreq = SID2_CLOCK_PAL;

    // Mirror a real C64
    if (m_tuneInfo.playAddr == 0xffff)
        forced = true;

    // Detect the Correct Song Speed
    if (clock == SID2_CLOCK_CORRECT)
    {
        clock = SID2_CLOCK_PAL;
        if (m_tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
            clock = SID2_CLOCK_NTSC;
    }
    // If forced change song to be the requested speed
    else if (forced)
    {
        m_tuneInfo.clockSpeed = SIDTUNE_CLOCK_PAL;
        if (clock == SID2_CLOCK_NTSC)
            m_tuneInfo.clockSpeed = SIDTUNE_CLOCK_NTSC;
    }

    if (m_tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
        vic.chip (MOS6569);
    else // if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
        vic.chip (MOS6567R8);

    if (clock == SID2_CLOCK_PAL)
    {
        cpuFreq = CLOCK_FREQ_PAL;
        m_tuneInfo.speedString = TXT_PAL_VBI;
        if (m_tuneInfo.songSpeed == SIDTUNE_SPEED_CIA_1A)
            m_tuneInfo.speedString = TXT_PAL_CIA;
        else if (m_tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
            m_tuneInfo.speedString = TXT_PAL_VBI_FIXED;
    }
    else // if (clock == SID2_CLOCK_NTSC)
    {
        cpuFreq = CLOCK_FREQ_NTSC;
        m_tuneInfo.speedString = TXT_NTSC_VBI;
        if (m_tuneInfo.songSpeed == SIDTUNE_SPEED_CIA_1A)
            m_tuneInfo.speedString = TXT_NTSC_CIA;
        else if (m_tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
            m_tuneInfo.speedString = TXT_NTSC_VBI_FIXED;
    }

    // Check for real C64 environment
    if (m_tuneInfo.playAddr == 0xffff)
    {
        xsid.mute (true);
        m_tuneInfo.songSpeed   = SIDTUNE_SPEED_CIA_1A;
        m_tuneInfo.speedString = TXT_PAL_UNKNOWN;
        if (m_tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
            m_tuneInfo.speedString = TXT_NTSC_UNKNOWN;;
    }
    return cpuFreq;
}

int Player::environment (sid2_env_t env)
{
    // Environment already set?
    if (!(m_ram && (m_environment == env)))
    {   // Setup new player environment
        m_environment = env;
        if (m_ram)
        {
            if (m_ram == m_rom)
               delete [] m_ram;
            else
            {
               delete [] m_rom;
               delete [] m_ram;
            }
        }

#ifdef HAVE_EXCEPTIONS
        m_ram = new(nothrow) uint8_t[0x10000];
#else
        m_ram = new uint8_t[0x10000];
#endif

        // Setup the access functions to the environment
        // and the properties the memory has.
        if (m_environment == sid2_envPS)
        {   // Playsid has no roms and SID exists in ram space
            m_rom = m_ram;
            m_readMemByte     = &Player::readMemByte_player;
            m_writeMemByte    = &Player::writeMemByte_playsid;
            m_readMemDataByte = &Player::readMemByte_playsid;
        }
        else
        {
#ifdef HAVE_EXCEPTIONS
            m_rom = new(nothrow) uint8_t[0x10000];
#else
            m_rom = new uint8_t[0x10000];
#endif

            switch (m_environment)
            {
            case sid2_envTP:
                m_readMemByte     = &Player::readMemByte_player;
                m_writeMemByte    = &Player::writeMemByte_sidplay;
                m_readMemDataByte = &Player::readMemByte_sidplaytp;
            break;

            case sid2_envBS:
                m_readMemByte     = &Player::readMemByte_player;
                m_writeMemByte    = &Player::writeMemByte_sidplay;
                m_readMemDataByte = &Player::readMemByte_sidplaybs;
            break;

            case sid2_envR:
            default: // <-- Just to please compiler
                m_readMemByte     = &Player::readMemByte_player;
                m_writeMemByte    = &Player::writeMemByte_sidplay;
                m_readMemDataByte = &Player::readMemByte_sidplaybs;
            break;
            }
        }
    }

    // Check if tune is invalid
    if ((m_environment != sid2_envR) &&
        (m_tuneInfo.playAddr == 0xffff))
    {   // Unload song.
        load (NULL);
    }

    // Have to reload the song into memory as
    // everything has changed
    return initialise ();
}

// Integrate SID emulation from the builder class into
// libsidplay2
int Player::sidCreate (sidbuilder *builder, sid2_model_t model)
{
    sidemu *sid1 = xsid.emulation ();

    // If we are already using the emulation
    // then don't change
    if (builder == sid1->builder ())
        return 0;

	{   // Release old sids
		sidbuilder *b;
		b = sid1->builder ();
		if (b)
			b->unlock (sid1);
		b = sid2->builder ();
		if (b)
		   b->unlock (sid2);
    }

    if (!builder)
    {   // No sid
        sid1 = &nullsid;
        sid2 = &nullsid;
    }
    else
    {
        if (model == SID2_MODEL_CORRECT)
        {   // Base selection on song
            model = SID2_MOS6581;
            if ((m_tune->getInfo()).sidRev8580)
                model = SID2_MOS8580;
        }

        // Set the tunes sid model
        m_tuneInfo.sidRev8580 = false;
        if (model == SID2_MOS8580)
            m_tuneInfo.sidRev8580 = true;

        // Get first SID emulation
        sid1 = builder->lock (this, model);
        if (!sid1)
            sid1 = &nullsid;
        if (!*builder)
            return -1;

        // Get second SID emulation
        sid2 = builder->lock (this, model);
        if (!sid2)
            sid2 = &nullsid;
    }
    xsid.emulation (sid1);
    return 0;
}

void Player::sidSamples (bool enable)
{
    sidemu *sid1 = xsid.emulation ();
    xsid.sidSamples (enable);

    // Now balance voices
    if (enable)
    {
        sid1->gain (0);
        sid2->gain (0);
        xsid.gain (-100);
    } else {
        sid1->gain (-25);
        sid2->gain (-25);
        xsid.gain (-75);
    }
}
