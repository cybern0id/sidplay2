/***************************************************************************
                          args.cpp  -  Command Line Parameters
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
 *  Revision 1.11  2003/02/23 08:53:11  s_a_white
 *  Option none nolonger uses the audio hardware (removing realtime delays).
 *  New option nosid allows library profiling without the sid emulation.
 *
 *  Revision 1.10  2003/02/20 18:50:43  s_a_white
 *  sid2crc support.
 *
 *  Revision 1.9  2002/11/06 19:08:46  s_a_white
 *  Added --none to command line for selecting no sid (debug purposes).
 *
 *  Revision 1.8  2002/09/23 21:49:58  s_a_white
 *  Display error message on engine configuration failure.
 *
 *  Revision 1.7  2002/04/18 22:57:28  s_a_white
 *  Fixed use of track looping/single when creating audio files.
 *
 *  Revision 1.6  2002/03/11 18:02:56  s_a_white
 *  Display errors like sidplay1.
 *
 *  Revision 1.5  2002/01/29 08:11:43  s_a_white
 *  TSID filename fix
 *
 *  Revision 1.4  2002/01/29 00:49:49  jpaana
 *  Fixed a typo on in the TSID code
 *
 *  Revision 1.3  2002/01/28 19:40:50  s_a_white
 *  Added TSID support.
 *
 *  Revision 1.2  2001/12/01 20:16:23  s_a_white
 *  Player changed to ConsolePlayer.
 *
 *  Revision 1.1  2001/11/27 19:13:24  s_a_white
 *  Initial Release.
 *
 ***************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <iostream.h>
//using std::endl;
#include "player.h"

#if defined(HAVE_SGI)
#   define DISALLOW_16BIT_SOUND
#   define DISALLOW_STEREO_SOUND
#endif

// Convert time from integer
bool ConsolePlayer::parseTime (const char *str, uint_least32_t &time)
{
    char *sep;
    uint_least32_t _time;

    // Check for empty string
    if (*str == '\0')
        return false;

    sep = strstr (str, ":");
    if (!sep)
    {   // User gave seconds
        _time = atoi (str);
    }
    else
    {   // Read in MM:SS format
        int val;
        *sep = '\0';
        val  = atoi (str);
        if (val < 0 || val > 99)
            return false;
        _time = (uint_least32_t) val * 60;
        val   = atoi (sep + 1);
        if (val < 0 || val > 59)
            return false;
        _time += (uint_least32_t) val;
    }

    time = _time;
    return true;
}

// Parse command line arguments
bool ConsolePlayer::args (int argc, const char *argv[])
{
    int  infile = 0;
    int  i      = 0;
    bool err    = false;

    if (argc == 0) // at least one argument required
    {
        displayArgs ();
        return false;
    }

    // default arg options
    m_driver.output = OUT_SOUNDCARD;
    m_driver.file   = false;
    m_driver.sid    = EMU_RESID;

    // parse command line arguments
    while ((i < argc) && (argv[i] != NULL))
    {
        if ((argv[i][0] == '-') && (argv[i][1] != '\0'))
        {
            if (argv[i][1] == 'b')
            {
                if (!parseTime (&argv[i][2], m_timer.start))
                    err = true;
            }
            else if (strcmp (&argv[i][1], "fd") == 0)
            {   // Override sidTune and enable the second sid
                m_engCfg.forceDualSids = true;
            }
            else if (strcmp (&argv[i][1], "fs") == 0)
            {   // Force samples through soundcard instead of SID
                m_engCfg.sidSamples = false;
            }
            else if (argv[i][1] == 'f')
            {
                if (argv[i][2] == '\0')
                    err = true;
				m_engCfg.frequency = (uint_least32_t) atoi (argv[i]+2);
            }

            // Player Mode (Environment) Options ----------
            else if (strcmp (&argv[i][1], "mb") == 0)
            {   // Bankswitching
                m_engCfg.environment = sid2_envBS;
            }
            else if (strcmp (&argv[i][1], "mr") == 0)
            {   // Real C64
                m_engCfg.environment = sid2_envR;
            }
            else if (strcmp (&argv[i][1], "mt") == 0)
            {   // Transparent Rom
                m_engCfg.environment = sid2_envTP;
            }
            else if (argv[i][1] == 'm')
            {   // PlaySID
                m_engCfg.environment = sid2_envPS;
            }

            // New/No filter options
            else if (strncmp (&argv[i][1], "nf", 2) == 0)
            {
                if (argv[i][3] == '\0')
                    m_filter.enabled = false;
                else
                {   // New filter
                    // This line will open an existing file
                    m_filter.enabled = true;
                    m_filter.definition.read (&(argv[i][3]));
                    if (!m_filter.definition)
                    {
                        displayError (m_filter.definition.error ());
                        return false;
                    }
                }
            }

            // Newer sid (8580)
            else if (strcmp (&argv[i][1], "ns") == 0)
            {
                m_engCfg.sidModel = SID2_MOS8580;
            }

            // Track options
            else if (strncmp (&argv[i][1], "ols", 3) == 0)
            {
                m_track.loop   = true;
                m_track.single = true;
                m_track.first  = atoi(&argv[i][4]);
            }
            else if (strncmp (&argv[i][1], "ol", 2) == 0)
            {
                m_track.loop  = true;
                m_track.first = atoi(&argv[i][3]);
            }
            else if (strncmp (&argv[i][1], "os", 2) == 0)
            {
                m_track.single = true;
                m_track.first  = atoi(&argv[i][3]);
            }
            else if (argv[i][1] == 'o')
            {   // User forgot track number ?
                if (argv[i][2] == '\0')
                    err = true;
                m_track.first = atoi(&argv[i][2]);
            }

            else if (argv[i][1] == 'O')
            {
                if (argv[i][2] != '\0')
                    m_engCfg.optimisation = atoi(&argv[i][2]);
            }

            else if (argv[i][1] == 'p')
            {   // User forgot precision
                if (argv[i][2] == '\0')
                    err = true;
                {
                    uint_least8_t precision = atoi(&argv[i][2]);
                    if (precision <= 8)
                        precision = 8;
                    else if (precision <= 16)
                        precision = 16;
                    else
                        precision = 24;

                    if (precision > SID2_MAX_PRECISION)
                        precision = SID2_MAX_PRECISION;
                    m_engCfg.precision = precision;
                }
            }

            else if (argv[i][1] == 'q')
            {
                if (argv[i][2] == '\0')
                    m_quietLevel = 1;
                else
                    m_quietLevel = atoi(&argv[i][2]);
            }

            // Stereo Options
            else if (strcmp (&argv[i][1], "sl") == 0)
            {   // Left Channel
                m_engCfg.playback = sid2_left;
            }
            else if (strcmp (&argv[i][1], "sr") == 0)
            {   // Right Channel
                m_engCfg.playback = sid2_right;
            }
            else if (argv[i][1] == 's')
            {   // Stereo Playback
                m_engCfg.playback = sid2_stereo;
            }

            else if (argv[i][1] == 't')
            {
                if (!parseTime (&argv[i][2], m_timer.length))
                    err = true;
                m_timer.valid = true;
            }

            // Video/Verbose Options
            else if (strcmp (&argv[i][1], "vnf") == 0)
            {
                m_engCfg.clockForced = true;
                m_engCfg.clockSpeed  = SID2_CLOCK_NTSC;
            }
            else if (strcmp (&argv[i][1], "vpf") == 0)
            {
                m_engCfg.clockForced = true;
                m_engCfg.clockSpeed  = SID2_CLOCK_PAL;
            }
            else if (strcmp (&argv[i][1], "vf") == 0)
            {
                m_engCfg.clockForced = true;
            }
            else if (strcmp (&argv[i][1], "vn") == 0)
            {
                m_engCfg.clockSpeed  = SID2_CLOCK_NTSC;
            }
            else if (strcmp (&argv[i][1], "vp") == 0)
            {
                m_engCfg.clockSpeed  = SID2_CLOCK_PAL;
            }
            else if (argv[i][1] == 'v')
            {
                if (argv[i][2] == '\0')
                    m_verboseLevel = 1;
                else
                    m_verboseLevel = atoi(&argv[i][2]);
            }
            else if (strcmp (&argv[i][1], "-crc") == 0)
            {
                m_crc = true;
                m_engCfg.powerOnDelay = 0;
            }

            // File format conversions
            else if (argv[i][1] == 'w')
            {
                m_driver.output = OUT_WAV;
                m_driver.file   = true;
                if (argv[i][2] != '\0')
                    m_outfile = &argv[i][2];
            }
            else if (strncmp (&argv[i][1], "-wav", 4) == 0)
            {
                m_driver.output = OUT_WAV;
                m_driver.file   = true;
                if (argv[i][5] != '\0')
                    m_outfile = &argv[i][5];
            }
            else if (strncmp (&argv[i][1], "-au", 3) == 0)
            {
                m_driver.output = OUT_AU;
                m_driver.file   = true;
                if (argv[i][4] != '\0')
                    m_outfile = &argv[i][4];
            }

            // Hardware selection
#ifdef HAVE_HARDSID_BUILDER
            else if (strcmp (&argv[i][1], "-hardsid") == 0)
            {
                m_driver.sid    = EMU_HARDSID;
                m_driver.output = OUT_NULL;
            }
#endif // HAVE_HARDSID_BUILDER
            // These two are for debug
            else if (strcmp (&argv[i][1], "-none") == 0)
            {
                m_driver.sid    = EMU_NONE;
                m_driver.output = OUT_NULL;
            }

            else if (strcmp (&argv[i][1], "-nosid") == 0)
            {
                m_driver.sid = EMU_NONE;
            }

            else
            {
                err = true;
            }

        }
        else
        {   // Reading file name
            if (infile == 0)
                infile = i;
            else
                err = true;
        }

        if (err)
        {
            displayArgs (argv[i]);
            return false;
        }

        i++;  // next index
    }

    // Load the tune
    m_filename = argv[infile];
    m_tune.load (m_filename);
    if (!m_tune)
    {
        displayError ((m_tune.getInfo ()).statusString);
        return false;
    }

    // If filename specified we can only convert one song
    if (m_outfile != NULL)
        m_track.single = true;

    // Can only loop if not creating audio files
    if (m_driver.output > OUT_SOUNDCARD)
        m_track.loop = false;

    // Check to see if we are trying to generate an audio file
    // whilst using a hardware emulation
    if (m_driver.file && (m_driver.sid >= EMU_HARDSID))
    {
        displayError ("ERROR: Cannot generate audio files using hardware emulations");
        return false;
    }
    
    // Select the desired track
    m_track.first    = m_tune.selectSong (m_track.first);
    m_track.selected = m_track.first;
    if (m_track.single)
        m_track.songs = 1;

    // If user provided no time then load songlength database
    // and set default lengths incase it's not found in there.
    {
        if (m_driver.file && m_timer.valid && !m_timer.length)
        {   // Time of 0 provided for wav generation
            displayError ("ERROR: -t0 invalid in record mode");
            return false;
        }
        if (!m_timer.valid)
        {
            const char *database = (m_iniCfg.sidplay2()).database;
            m_timer.length = (m_iniCfg.sidplay2()).playLength;
            if (m_driver.file)
                m_timer.length = (m_iniCfg.sidplay2()).recordLength;
            if (database && (*database != '\0'))
            {   // Try loading the database specificed by the user
                if (m_database.open (database) < 0)
                {
                    displayError (m_database.error ());
                    return false;
                }
            }
        }
    }

    if (!m_filter.definition)
        m_filter.definition = m_iniCfg.filter (m_engCfg.sidModel);

#ifdef HAVE_TSID
    // Set TSIDs base directory
    if (!m_tsid.setBaseDir(true))
    {
        displayError (m_tsid.getError ());
        return false;
    }
#endif

    // Configure engine with settings
    if (m_engine.config (m_engCfg) < 0)
    {   // Config failed
        displayError (m_engine.error ());
        return false;
    }
    return true;
}


void ConsolePlayer::displayArgs (const char *arg)
{
    ostream &out = arg ? cerr : cout;

    if (arg)
        out << "Option Error: " << arg << endl;
    else
        out << "Syntax: " << m_name << " [-<option>...] <datafile>" << endl;

    out << "Options:" << endl
        << " --help|-h    display this screen" << endl

        << " -b<num>      set start time in [m:]s format (default 0)" << endl

        << " -f<num>      set frequency in Hz (default: "
        << SID2_DEFAULT_SAMPLING_FREQ << ")" << endl
        << " -fd          force dual sid environment" << endl
        << " -fs          force samples to a channel (default: uses sid)" << endl

// Old options are hidden
//        << " -m           PlaySID Compatibility Mode (read the docs!)" << endl
//        << " -mt          Sidplays Transparent Rom Mode" << endl
        << " -m<b|r>      mode switch <Bankswitching | Real C64 (default)>" << endl

        << " -nf[filter]  no/new SID filter emulation" << endl
        << " -ns          MOS 8580 waveforms (default: MOS 6581)" << endl

        << " -o<l|s>      looping and/or single track" << endl
        << " -o<num>      start track (default: preset)" << endl
        << " -O<num>      optimisation level, max is " << (uint) (SID2_MAX_OPTIMISATION - 1)
        << " (default: " << (uint) SID2_DEFAULT_OPTIMISATION << ')' << endl

        << " -p<num>      set bit precision for samples. "
        << "(default: " << (uint) SID2_DEFAULT_PRECISION << ")" << endl

#if !defined(DISALLOW_STEREO_SOUND)
        << " -s[l|r]      stereo sid support or [left/right] channel only" << endl
#endif

        << " -t<num>      set play length in [m:]s format (0 is endless)" << endl

        << " -<v|q>       verbose or quiet (no time display) output" << endl
        << " -v[p|n][f]   set VIC PAL/NTSC clock speed (default: defined by song)" << endl
        << "              Use 'f' to force the clock by preventing speed fixing" << endl

        << " -w[name]     create wav file (default: <datafile>[n].wav)" << endl
#ifdef HAVE_HARDSID_BUILDER
        << " --hardsid    enable hardsid support" << endl
#endif
        << endl
        // Changed to new homepage address
        << "Home Page: http://sidplay2.sourceforge.net/" << endl;
//        << "Mail comments, bug reports, or contributions to <sidplay2@email.com>." << endl;
}
