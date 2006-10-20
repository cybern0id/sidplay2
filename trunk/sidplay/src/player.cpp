/***************************************************************************
                          player.cpp  -  Frontend Player
                             -------------------
    begin                : Sun Oct 7 2001
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
 *  Revision 1.39  2006/10/20 16:27:12  s_a_white
 *  Begin improving compatibility with old libraries
 *
 *  Revision 1.38  2006/10/16 21:50:45  s_a_white
 *  Merge verbose and quiet levels.
 *
 *  Revision 1.37  2006/07/07 18:44:09  s_a_white
 *  Display keys for non soundcard output sources.
 *
 *  Revision 1.36  2006/06/28 07:42:00  s_a_white
 *  Switch builders use to COM object use.
 *
 *  Revision 1.35  2005/11/30 22:49:48  s_a_white
 *  Add raw output support (--raw=<file>)
 *
 *  Revision 1.34  2005/06/10 18:40:16  s_a_white
 *  Mingw support.
 *
 *  Revision 1.33  2004/11/12 20:20:21  s_a_white
 *  Remove some unnecessary duplicate code.
 *
 *  Revision 1.32  2004/06/26 15:46:47  s_a_white
 *  Changes to support new calling convention for event scheduler.
 *
 *  Revision 1.31  2004/02/26 18:19:22  s_a_white
 *  Updates for VC7 (use real libstdc++ headers instead of draft ones).
 *
 *  Revision 1.30  2004/02/12 05:58:03  s_a_white
 *  Update argurements and help menu handling.
 *
 *  Revision 1.29  2004/01/31 17:07:45  s_a_white
 *  Support of specifing max sids writes forming sid2crc and experimental
 *  TSID2 library support.
 *
 *  Revision 1.28  2003/12/15 23:34:49  s_a_white
 *  Timebase for timers now obtainable from engine.
 *
 *  Revision 1.27  2003/10/18 12:45:12  s_a_white
 *  no message
 *
 *  Revision 1.26  2003/07/16 06:54:06  s_a_white
 *  Added sid2crc support.
 *
 *  Revision 1.25  2003/06/27 21:07:40  s_a_white
 *  Fixed problem whereby the audio buffer size was ever only calculated once.
 *
 *  Revision 1.24  2003/02/23 08:59:20  s_a_white
 *  Displayed keyboard checks at high quiet levels.   At these quiet levels we
 *  are most likely being run from within another program.
 *
 *  Revision 1.23  2003/02/22 09:41:59  s_a_white
 *  Made crcs be printed for every subtune automatically.
 *
 *  Revision 1.22  2003/02/20 18:50:44  s_a_white
 *  sid2crc support.
 *
 *  Revision 1.21  2003/01/20 16:26:18  s_a_white
 *  Updated for new event scheduler interface.
 *
 *  Revision 1.20  2003/01/14 20:37:10  s_a_white
 *  Updated previous song select timeout to 4 seconds.
 *
 *  Revision 1.19  2003/01/08 08:45:14  s_a_white
 *  Don't configure an emulation if it failed to be created.
 *
 *  Revision 1.18  2002/08/19 17:04:28  s_a_white
 *  Added timeout to decide if previous key should select previous song or
 *  restart current song.
 *
 *  Revision 1.17  2002/06/16 19:44:21  s_a_white
 *  Changing resid filter works again.
 *
 *  Revision 1.16  2002/04/18 22:57:28  s_a_white
 *  Fixed use of track looping/single when creating audio files.
 *
 *  Revision 1.15  2002/03/11 18:02:56  s_a_white
 *  Display errors like sidplay1.
 *
 *  Revision 1.14  2002/03/04 19:30:15  s_a_white
 *  Fix C++ use of nothrow.
 *
 *  Revision 1.13  2002/01/30 00:34:15  s_a_white
 *  Printing builder error message instead of not enough memory.
 *
 *  Revision 1.12  2002/01/29 08:11:43  s_a_white
 *  TSID filename fix
 *
 *  Revision 1.11  2002/01/28 19:40:50  s_a_white
 *  Added TSID support.
 *
 *  Revision 1.10  2002/01/16 19:54:56  s_a_white
 *  Fixed -b command arg with UNKNOWN songlengh.
 *
 *  Revision 1.9  2002/01/16 19:28:55  s_a_white
 *  Now now wraps at 100th minute.
 *
 *  Revision 1.8  2002/01/10 19:39:46  s_a_white
 *  Fixed default to switch to please solaris compiler.
 *
 *  Revision 1.7  2001/12/11 19:38:13  s_a_white
 *  More GCC3 Fixes.
 *
 *  Revision 1.6  2001/12/07 18:22:33  s_a_white
 *  Player quit fixes.
 *
 *  Revision 1.5  2001/12/05 22:22:48  s_a_white
 *  Added playerFast flag.
 *
 *  Revision 1.4  2001/12/03 20:00:24  s_a_white
 *  sidSamples no longer forced for hardsid.
 *
 *  Revision 1.3  2001/12/03 19:17:34  s_a_white
 *  Corrected spelling of BUILDER.
 *
 *  Revision 1.2  2001/12/01 20:16:23  s_a_white
 *  Player changed to ConsolePlayer.
 *
 *  Revision 1.1  2001/11/27 19:10:44  s_a_white
 *  Initial Release.
 *
 ***************************************************************************/

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <iomanip>
#include "config.h"

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#include "player.h"
#include "keyboard.h"

// Previous song select timeout (3 secs)
#define SID2_PREV_SONG_TIMEOUT 4

// Depreciated
#ifdef HAVE_RESID_BUILDER
#   include <sidplay/builders/resid.h>
const char ConsolePlayer::RESID_ID[]   = "ReSID";
#endif
#ifdef HAVE_HARDSID_BUILDER
#   include <sidplay/builders/hardsid.h>
const char ConsolePlayer::HARDSID_ID[] = "HardSID";
#endif



ConsolePlayer::ConsolePlayer (const char * const name)
:Event("External Timer\n"),
 m_name(name),
 m_tune(0),
 m_state(playerStopped),
 m_outfile(NULL),
 m_context(NULL),
 m_verboseLevel(0),
 m_crc(0),
 m_cpudebug(false)
{   // Other defaults
    m_filename       = "";
    m_filter.enabled = true;
    m_driver.device  = NULL;
    m_timer.start    = 0;
    m_timer.length   = 0; // FOREVER
    m_timer.valid    = false;
    m_track.first    = 0;
    m_track.selected = 0;
    m_track.loop     = false;
    m_track.single   = false;
    m_speed.current  = 1;
    m_speed.max      = 32;

    // Read default configuration
    m_iniCfg.read ();
    m_engCfg = m_engine.config ();

    {   // Load ini settings
        IniConfig::audio_section     audio     = m_iniCfg.audio();
        IniConfig::emulation_section emulation = m_iniCfg.emulation();

        // INI Configuration Settings
        m_engCfg.clockForced    = emulation.clockForced;
        m_engCfg.clockSpeed     = emulation.clockSpeed;
        m_engCfg.clockDefault   = SID2_CLOCK_PAL;
        m_engCfg.frequency      = audio.frequency;
        m_engCfg.optimisation   = emulation.optimiseLevel;
        m_engCfg.playback       = audio.playback;
        m_engCfg.precision      = audio.precision;
        m_engCfg.sidModel       = emulation.sidModel;
        m_engCfg.sidDefault     = SID2_MOS6581;
        m_engCfg.sidSamples     = emulation.sidSamples;
        m_filter.enabled        = emulation.filter;
    }

    createOutput (OUT_NULL, NULL);
    createSidEmu (EMU_NONE);
}


// Create the output object to process sound buffer
bool ConsolePlayer::createOutput (OUTPUTS driver, const SidTuneInfo *tuneInfo)
{
    char *name = NULL;
    const char *title = m_outfile;

    // Remove old audio driver
    m_driver.null.close ();
    m_driver.selected = &m_driver.null;
    if (m_driver.device != NULL)
    {
        if (m_driver.device != &m_driver.null)
            delete m_driver.device;
        m_driver.device = NULL;
    }

    // Create audio driver
    switch (driver)
    {
    case OUT_NULL:
        m_driver.device = &m_driver.null;
        title = "";
    break;

    case OUT_SOUNDCARD:
#ifdef HAVE_WAV_ONLY
        m_driver.device = NULL;
#elif defined(HAVE_EXCEPTIONS)
        m_driver.device = new(std::nothrow) AudioDriver;
#else
        m_driver.device = new AudioDriver;
#endif
    break;

    case OUT_WAV:
#ifdef HAVE_EXCEPTIONS
        m_driver.device = new(std::nothrow) WavFile;
#else
        m_driver.device = new WavFile;
#endif
    break;

    case OUT_RAW:
#ifdef HAVE_EXCEPTIONS
        m_driver.device = new(std::nothrow) RawFile;
#else
        m_driver.device = new RawFile;
#endif
    break;

    default:
        break;
    }

    // Audio driver failed
    if (!m_driver.device)
    {
        m_driver.device = &m_driver.null;
        displayError (ERR_NOT_ENOUGH_MEMORY);
        return false;
    }

    // Generate a name for the wav file
    if (title == NULL)
    {
        size_t length, i;
        title  = tuneInfo->dataFileName;
        length = strlen (title);
        i      = length;
        while (i > 0)
        {
            if (title[--i] == '.')
                break;
        }
        if (!i) i = length;

#ifdef HAVE_EXCEPTIONS
        name = new(std::nothrow) char[i + 10];
#else
        name = new char[i + 10];
#endif
        if (!name)
        {
            displayError (ERR_NOT_ENOUGH_MEMORY);
            return false;
        }

        strcpy (name, title);
        // Prevent extension ".sid.wav"
        name[i] = '\0';

        // Change name based on subtune
        if (tuneInfo->songs > 1)
            sprintf (&name[i], "[%u]", tuneInfo->currentSong);
        strcat (&name[i], m_driver.device->extension ());
        title = name;
    }

    // Configure with user settings
    m_driver.cfg.frequency = m_engCfg.frequency;
    m_driver.cfg.precision = m_engCfg.precision;
    m_driver.cfg.channels  = 1; // Mono
    m_driver.cfg.bufSize   = 0; // Recalculate
    if (m_engCfg.playback == sid2_stereo)
        m_driver.cfg.channels = 2;

    {   // Open the hardware
        bool err = false;
        if (m_driver.device->open (m_driver.cfg, title) == NULL)
            err = true;
        // Can't open the same driver twice
        if (driver != OUT_NULL)
        {
            if (m_driver.null.open (m_driver.cfg, title) == NULL)
                err = true;;
        }
        if (name != NULL)
            delete [] name;
        if (err)
            return false;
    }

    // See what we got
    m_engCfg.frequency = m_driver.cfg.frequency;
    m_engCfg.precision = m_driver.cfg.precision;
    switch (m_driver.cfg.channels)
    {
    case 1:
        if (m_engCfg.playback == sid2_stereo)
            m_engCfg.playback  = sid2_mono;
        break;
    case 2:
        if (m_engCfg.playback != sid2_stereo)
            m_engCfg.playback  = sid2_stereo;
        break;
    default:
        cerr << m_name << ": " << "ERROR: " << m_driver.cfg.channels
             << " audio channels not supported" << endl;
        return false;
    }
    return true;
}


// Create the sid emulation
bool ConsolePlayer::createSidEmu (SIDEMUS emu)
{
    // Remove old driver and emulation
    if (m_engCfg.sidEmulation)
    {
#ifdef HAVE_SID2_COM
        ISidBuilder
#else // Depreciared interface
        sidbuilder
#endif
        *builder = m_engCfg.sidEmulation;
        m_engCfg.sidEmulation = 0;
        m_engine.config (m_engCfg);
#ifdef HAVE_SID2_COM
        builder->ifrelease ();
#else // Depreciared interface
        delete builder;
#endif

    }

    // Now setup the sid emulation
    switch (emu)
    {
#ifdef HAVE_RESID_BUILDER
    case EMU_RESID:
    {
        ReSIDBuilder *rs;
#ifdef HAVE_SID2_COM
        if ( ReSIDBuilderCreate ("", IF_QUERY(ReSIDBuilder, &rs)) )
        {
            ISidBuilder *builder = 0;
            rs->ifquery (IF_QUERY(ISidBuilder, &builder));
            m_engCfg.sidEmulation = builder;
#else // Depreciated interface
#   ifdef HAVE_EXCEPTIONS
        rs = new(std::nothrow) ReSIDBuilder( RESID_ID );
#   else
        rs = new ReSIDBuilder( RESID_ID );
#   endif
        if (rs)
        {
            m_engCfg.sidEmulation = rs;
#endif // HAVE_SID2_COM
            if (!*rs) goto EMU_RESID_ERROR;

            // Setup the emulation
            rs->create ((m_engine.info ()).maxsids);
            if (!*rs) goto EMU_RESID_ERROR;
            rs->filter (m_filter.enabled);
            if (!*rs) goto EMU_RESID_ERROR;
            rs->sampling (m_driver.cfg.frequency);
            if (!*rs) goto EMU_RESID_ERROR;
            if (m_filter.enabled && m_filter.definition)
            {   // Setup filter
                rs->filter (m_filter.definition.provide ());
                if (!*rs) goto EMU_RESID_ERROR;
            }
#ifdef HAVE_SID2_COM
            rs->ifrelease ();
#endif
        }
        break;
    EMU_RESID_ERROR:
#ifdef HAVE_SID2_COM
        rs->ifrelease ();
#endif
        goto createSidEmu_error;
    }
#endif // HAVE_RESID_BUILDER

#ifdef HAVE_HARDSID_BUILDER
    case EMU_HARDSID:
    {
        HardSIDBuilder *hs;
#ifdef HAVE_SID2_COM
        if ( HardSIDBuilderCreate ("", IF_QUERY(HardSIDBuilder, &hs)) )
        {
            ISidBuilder *builder = 0;
            hs->ifquery (IF_QUERY(ISidBuilder, &builder));
            m_engCfg.sidEmulation = builder;
#else // Depreciated interface
#   ifdef HAVE_EXCEPTIONS
        hs = new(std::nothrow) HardSIDBuilder( HARDSID_ID );
#   else
        hs = new HardSIDBuilder( HARDSID_ID );
#   endif
        if (hs)
        {
            m_engCfg.sidEmulation = hs;
#endif // HAVE_SID2_COM
            if (!*hs) goto EMU_HARDSID_ERROR;

            // Setup the emulation
            hs->create ((m_engine.info ()).maxsids);
            if (!*hs) goto EMU_HARDSID_ERROR;
            hs->filter (m_filter.enabled);
            if (!*hs) goto EMU_HARDSID_ERROR;
#ifdef HAVE_SID2_COM
            hs->ifrelease ();
#endif
        }
        break;
    EMU_HARDSID_ERROR:
#ifdef HAVE_SID2_COM
        hs->ifrelease ();
#endif
        goto createSidEmu_error;
    }
#endif // HAVE_HARDSID_BUILDER

    default:
        // Emulation Not yet handled
        // This default case results in the default
        // emulation
        break;
    }

    if (!m_engCfg.sidEmulation)
    {
        if (emu > EMU_DEFAULT)
        {   // No sid emulation?
            displayError (ERR_NOT_ENOUGH_MEMORY);
            return false;
        }
    }
    return true;

createSidEmu_error:
    displayError (m_engCfg.sidEmulation->error ());
#ifdef HAVE_SID2_COM
    (m_engCfg.sidEmulation)->ifrelease ();
#else // Depreciated Interface
    delete m_engCfg.sidEmulation;
#endif
    m_engCfg.sidEmulation = 0;
    return false;
}


bool ConsolePlayer::open (void)
{
    const SidTuneInfo *tuneInfo;

    if ((m_state & ~playerFast) == playerRestart)
    {
        if (m_verboseLevel > -2)
            cerr << endl;
        if (m_state & playerFast)
            m_driver.selected->reset ();
        m_state = playerStopped;
    }

    // Select the required song
    m_track.selected = m_tune.selectSong (m_track.selected);
    if (m_engine.load (&m_tune) < 0)
    {
        displayError (m_engine.error ());
        return false;
    }

    // Get tune details
    tuneInfo = (m_engine.info ()).tuneInfo;
    if (!m_track.single)
        m_track.songs = tuneInfo->songs;
    if (!createOutput (m_driver.output, tuneInfo))
        return false;
    if (!createSidEmu (m_driver.sid))
        return false;

    // Configure engine with settings
    if (m_engine.config (m_engCfg) < 0)
    {   // Config failed
        displayError (m_engine.error ());
        return false;
    }

    // Start the player.  Do this by fast
    // forwarding to the start position
    m_driver.selected = &m_driver.null;
    m_speed.current   = m_speed.max;
    m_engine.fastForward (100 * m_speed.current);

    // As yet we don't have a required songlength
    // so try the songlength database
    if (!m_timer.valid)
    {
        int_least32_t length = m_database.length (m_tune);
        if (length > 0)
            m_timer.length = length;
    }

    // Set up the play timer
    m_context = (m_engine.info()).eventContext;
    m_timer.stop = m_timer.length;

    if (m_timer.valid)
    {   // Length relative to start
        m_timer.stop += m_timer.start;
    }
    else
    {   // Check to make start time dosen't exceed end
        if (m_timer.stop & (m_timer.start >= m_timer.stop))
        {
            displayError ("ERROR: Start time exceeds song length!");
            return false;
        }
    }

    m_timer.current = ~0;
    m_state = playerRunning;

    // Update display
    menu  ();
    event ();
    return true;
}

void ConsolePlayer::close ()
{
    m_engine.stop   ();
    if (m_state == playerExit)
    {   // Natural finish
        emuflush ();
        if (m_driver.file)
            cerr << (char) 7; // Bell
    }
    else // Destroy buffers
        m_driver.selected->reset ();

    // Shutdown drivers, etc
    createOutput    (OUT_NULL, NULL);
    createSidEmu    (EMU_NONE);
    m_engine.load   (NULL);
    m_engine.config (m_engCfg);

    if (m_verboseLevel > -2)
    {   // Correctly leave ansi mode and get prompt to
        // end up in a suitable location
        if ((m_iniCfg.console ()).ansi)
            cerr << '\x1b' << "[0m";
#ifndef HAVE_MSWINDOWS
        cerr << endl;
#endif
    }
}

// Flush any hardware sid fifos so all music is played
void ConsolePlayer::emuflush ()
{   // Eventually need an interface to flush all hardware
    // seperate to a specific interface
#ifdef HSID_SID2_COM
#   ifdef HAVE_HARDSID_BUILDER
    HardSIDBuilder *hs;
    if ( (m_engCfg.sidEmulation)->ifquery (IF_QUERY(HardSIDBuilder, &hs)) )
    {
        hs->flush ();
        hs->ifrelease ();
    }
#   endif // HAVE_HARDSID_BUILDER
#else // Depreciated Interface
    switch (m_driver.sid)
    {
#   ifdef HAVE_HARDSID_BUILDER
    case EMU_HARDSID:
        ((HardSIDBuilder *)m_engCfg.sidEmulation)->flush ();
        break;
#   endif // HAVE_HARDSID_BUILDER
    default:
        break;
    }
#endif // HSID_SID2_COM
}


// Out play loop to be externally called
bool ConsolePlayer::play ()
{
    void *buffer = m_driver.selected->buffer ();
    uint_least32_t length = m_driver.cfg.bufSize;

    if (m_state == playerRunning)
    {
        // Fill buffer
        uint_least32_t ret;
        ret = m_engine.play (buffer, length);
        if (ret < length)
        {
            if (m_engine.state () != sid2_stopped)
            {
                m_state = playerError;
                return false;
            }
            return false;
        }
    }

    switch (m_state)
    {
    case playerRunning:
        m_driver.selected->write ();
        // Deliberate run on
    case playerPaused:
        // Check for a keypress (approx 250ms rate, but really depends
        // on music buffer sizes).  Don't do this for high quiet levels
        // as chances are we are under remote control.
        if ((m_verboseLevel > -2) && _kbhit ())
            decodeKeys ();
        return true;
    default:
        if (m_verboseLevel > -2)
            cerr << endl;
        if (m_crc)
        {
            const SidTuneInfo *tuneInfo = (m_engine.info ()).tuneInfo;
            cout << std::setw(8) << std::setfill('0') << std::hex
                 << (m_engine.info ()).sid2crc << " : " << std::dec
                 << m_filename << " - song " << tuneInfo->currentSong
                 << "/" << tuneInfo->songs << endl;
        }
        m_engine.stop ();
#if HAVE_TSID == 1
        if (m_tsid)
        {
            m_tsid.addTime ((int) m_timer.current, m_track.selected,
                            m_filename);
        }
#elif HAVE_TSID == 2
        if (m_tsid)
        {
            int_least32_t length;
            char md5[SIDTUNE_MD5_LENGTH + 1];
            m_tune.createMD5 (md5);
            length = m_database.length (md5, m_track.selected);
            // ignore errors
            if (length < 0)
                length = 0;
            m_tsid.addTime (md5, m_filename, (uint) m_timer.current,
                            m_track.selected, (uint) length);
        }
#endif
        break;
    }
    return false;
}


void ConsolePlayer::stop ()
{
    m_state = playerStopped;
    m_engine.stop ();
}


// External Timer Event
void ConsolePlayer::event (void)
{
    uint_least32_t seconds = m_engine.time() / m_engine.timebase();
    if (m_verboseLevel >= 0)
    {
        cerr << "\b\b\b\b\b" << std::setw(2) << std::setfill('0')
             << ((seconds / 60) % 100) << ':' << std::setw(2)
             << std::setfill('0') << (seconds % 60) << std::flush;
    }

    if (seconds != m_timer.current)
    {
        m_timer.current = seconds;

        // Handle exiting on crc completion
        if (m_crc && (m_crc == (m_engine.info ()).sid2crcCount))
            m_timer.stop = seconds;

        if (seconds == m_timer.start)
        {   // Switch audio drivers.
            m_driver.selected = m_driver.device;
            memset (m_driver.selected->buffer (), 0, m_driver.cfg.bufSize);
            m_speed.current = 1;
            m_engine.fastForward (100);
            if (m_cpudebug)
                m_engine.debug (true, NULL);
        }
        else if (m_timer.stop && (seconds == m_timer.stop))
        {
            m_state = playerExit;
            for (;;)
            {
                if (m_track.single)
                    return;
                // Move to next track
                m_track.selected++;
                if (m_track.selected > m_track.songs)
                    m_track.selected = 1;
                if (m_track.selected == m_track.first)
                    return;
                m_state = playerRestart;
                break;
            }
            if (m_track.loop)
                m_state = playerRestart;
        }
    }

    // Units in C64 clock cycles
    m_context->schedule (this, 900000, EVENT_CLOCK_PHI1);
}


void ConsolePlayer::displayError (const char *error)
{
    cerr << m_name << ": " << error << endl;
}


// Keyboard handling
void ConsolePlayer::decodeKeys ()
{
    int action;

    // All keys useless when saving to files
    if (m_driver.output != OUT_SOUNDCARD)
        return;

    do
    {
        action = keyboard_decode ();
        if (action == A_INVALID)
            continue;

        switch (action)
        {
        case A_RIGHT_ARROW:
            m_state = playerFastRestart;
            if (!m_track.single)
            {
                m_track.selected++;
                if (m_track.selected > m_track.songs)
                    m_track.selected = 1;
            }
        break;

        case A_LEFT_ARROW:
            m_state = playerFastRestart;
            if (!m_track.single)
            {   // Only select previous song if less than timeout
                // else restart current song
                if ((m_engine.time() / m_engine.timebase()) < SID2_PREV_SONG_TIMEOUT)
                {
                    m_track.selected--;
                    if (m_track.selected < 1)
                        m_track.selected = m_track.songs;
                }
            }
        break;

        case A_UP_ARROW:
            m_speed.current *= 2;
            if (m_speed.current > m_speed.max)
                m_speed.current = m_speed.max;
            m_engine.fastForward (100 * m_speed.current);
        break;

        case A_DOWN_ARROW:
            m_speed.current = 1;
            m_engine.fastForward (100);
        break;

        case A_HOME:
            m_state = playerFastRestart;
            m_track.selected = 1;
        break;

        case A_END:
            m_state = playerFastRestart;
            m_track.selected = m_track.songs;
        break;

        case A_PAUSED:
            if (m_state == playerPaused)
            {
                cerr << "\b\b\b\b\b\b\b\b\b";
                // Just to make sure PAUSED is removed from screen
                cerr << "         ";
                cerr << "\b\b\b\b\b\b\b\b\b";
                m_state  = playerRunning;
            }
            else
            {
                cerr << " [PAUSED]";
                m_state = playerPaused;
                m_driver.selected->pause ();
            }
        break;

        case A_QUIT:
            m_state = playerFastExit;
            return;
        break;
        }
    } while (_kbhit ());
}
