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
 *  Revision 1.2  2001/12/01 20:16:23  s_a_white
 *  Player changed to ConsolePlayer.
 *
 *  Revision 1.1  2001/11/27 19:13:24  s_a_white
 *  Initial Release.
 *
 ***************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "player.h"

#if defined(HAVE_SGI)
#   define DISALLOW_16BIT_SOUND
#   define DISALLOW_STEREO_SOUND
#endif

// Convert time from integer
bool ConsolePlayer::parseTime (char *str, uint_least32_t &time)
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
bool ConsolePlayer::args (int argc, char *argv[])
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
            if (strncmp (&argv[i][1], "b", 1) == 0)
            {
                if (!parseTime (&argv[i][2], m_timer.start))
                    err = true;
            }
            else if (strncmp (&argv[i][1], "fd", 2) == 0)
            {   // Override sidTune and enable the second sid
                m_engCfg.forceDualSids = true;
            }
            else if (strncmp (&argv[i][1], "fs", 2) == 0)
            {   // Force samples through soundcard instead of SID
                m_engCfg.sidSamples = false;
            }
            else if (strncmp (&argv[i][1], "f", 1) == 0)
            {
                if (argv[i][2] == '\0')
                    err = true;
				m_engCfg.frequency = (uint_least32_t) atoi (argv[i]+2);
            }

            // Player Mode (Environment) Options ----------
            else if (strncmp (&argv[i][1], "mb", 2) == 0)
            {   // Bankswitching
                m_engCfg.environment = sid2_envBS;
            }
            else if (strncmp (&argv[i][1], "mr", 2) == 0)
            {   // Real C64
                m_engCfg.environment = sid2_envR;
            }
            else if (strncmp (&argv[i][1], "mt", 2) == 0)
            {   // Transparent Rom
                m_engCfg.environment = sid2_envTP;
            }
            else if (strncmp (&argv[i][1], "m", 1) == 0)
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
            else if (strncmp (&argv[i][1], "ns", 2) == 0)
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
            else if (strncmp (&argv[i][1], "o", 1) == 0)
            {   // User forgot track number ?
                if (argv[i][2] == '\0')
                    err = true;
                m_track.first = atoi(&argv[i][2]);
            }

            else if (strncmp (&argv[i][1], "O", 1) == 0)
            {
                if (argv[i][2] != '\0')
                    m_engCfg.optimisation = atoi(&argv[i][2]);
            }

            else if (strncmp (&argv[i][1], "p", 1) == 0)
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

            else if (strncmp (&argv[i][1], "q", 1) == 0)
            {
                if (argv[i][2] == '\0')
                    m_quietLevel = 1;
                else
                    m_quietLevel = atoi(&argv[i][2]);
            }

            // Stereo Options
            else if (strncmp (&argv[i][1], "sl", 2) == 0)
            {   // Left Channel
                m_engCfg.playback = sid2_left;
            }
            else if (strncmp (&argv[i][1], "sr", 2) == 0)
            {   // Right Channel
                m_engCfg.playback = sid2_right;
            }
            else if (strncmp (&argv[i][1], "s", 1) == 0)
            {   // Stereo Playback
                m_engCfg.playback = sid2_stereo;
            }

            else if (strncmp (&argv[i][1], "t", 1) == 0)
            {
                if (!parseTime (&argv[i][2], m_timer.length))
                    err = true;
                m_timer.valid = true;
            }

            // Video/Verbose Options
            else if (strncmp (&argv[i][1], "vnf", 3) == 0)
            {
                m_engCfg.clockForced = true;
                m_engCfg.clockSpeed  = SID2_CLOCK_NTSC;
            }
            else if (strncmp (&argv[i][1], "vpf", 3) == 0)
            {
                m_engCfg.clockForced = true;
                m_engCfg.clockSpeed  = SID2_CLOCK_PAL;
            }
            else if (strncmp (&argv[i][1], "vf", 2) == 0)
            {
                m_engCfg.clockForced = true;
            }
            else if (strncmp (&argv[i][1], "vn", 2) == 0)
            {
                m_engCfg.clockSpeed  = SID2_CLOCK_NTSC;
            }
            else if (strncmp (&argv[i][1], "vp", 2) == 0)
            {
                m_engCfg.clockSpeed  = SID2_CLOCK_PAL;
            }
            else if (strncmp (&argv[i][1], "v", 1) == 0)
            {
                if (argv[i][2] == '\0')
                    m_verboseLevel = 1;
                else
                    m_verboseLevel = atoi(&argv[i][2]);
            }

            // File format conversions
            else if (strncmp (&argv[i][1], "w", 1) == 0)
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
            else if (strncmp (&argv[i][1], "-hardsid", 8) == 0)
            {
                m_driver.sid    = EMU_HARDSID;
                m_driver.output = OUT_NULL;
            }
#endif // HAVE_HARDSID_BUILDER

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
            displayArgs ();
            return false;
        }

        i++;  // next index
    }

    // Load the tune
    m_tune.load (argv[infile]);
    if (!m_tune)
    {
        cerr << m_name << "\n" << (m_tune.getInfo ()).statusString << endl;
        return false;
    }

    // Check to see if we are trying to generate an audio file
    // whilst using a hardware emulation
    if (m_driver.file && (m_driver.sid >= EMU_HARDSID))
    {
        cerr << m_name << "\n" << "ERROR: Cannot generate audio files using hardware emulations" << endl;
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
            cerr << m_name << "\nERROR: -t0 invalid in record mode" << endl;
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
                    cerr << m_name << "\n" << m_database.error () << endl;
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
        cerr << m_name << "\n" << m_tsid.getError (); << endl;
        return false;
    }
#endif
    return true;
}


void ConsolePlayer::displayArgs ()
{
    cout 
        << "Syntax: " << m_name << " [-<option>...] <datafile>" << endl
        << "Options:" << endl
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
