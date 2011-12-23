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
 *  Revision 1.29  2007/01/27 10:20:49  s_a_white
 *  Updated to use better COM emulation interface.
 *
 *  Revision 1.28  2006/10/30 19:32:06  s_a_white
 *  Switch sidplay2 class to iinterface.
 *
 *  Revision 1.27  2006/10/29 23:05:58  s_a_white
 *  Add -q for compatibily (is marked depreciated, remove later).
 *
 *  Revision 1.26  2006/10/28 10:12:18  s_a_white
 *  Update to new COM style interface.
 *
 *  Revision 1.25  2006/10/20 17:36:52  s_a_white
 *  Source now source compatible with old sidplay-libs
 *
 *  Revision 1.24  2006/10/17 22:23:34  s_a_white
 *  Add some support for print options (feature request #1109764)
 *
 *  Revision 1.23  2006/10/16 21:44:42  s_a_white
 *  Merge verbose and quiet levels.  Prevent quiet level (verbose -2) accessing
 *  the keyboard in anyway (for background operation).
 *
 *  Revision 1.22  2006/06/27 20:14:55  s_a_white
 *  Switch to new COM style builder classes.
 *
 *  Revision 1.21  2005/12/02 19:25:30  s_a_white
 *  Fix bug reported by Patrick Mauritz in patch 1282585 (modification of
 *  constant string).
 *
 *  Revision 1.20  2005/11/30 22:49:48  s_a_white
 *  Add raw output support (--raw=<file>)
 *
 *  Revision 1.19  2005/06/10 18:40:16  s_a_white
 *  Mingw support.
 *
 *  Revision 1.18  2004/05/05 23:49:20  s_a_white
 *  Remove the hardsid option from the args list if not sid devices are available.
 *
 *  Revision 1.17  2004/03/01 00:16:56  s_a_white
 *  ostream is part of std namespace.
 *
 *  Revision 1.16  2004/02/26 18:19:22  s_a_white
 *  Updates for VC7 (use real libstdc++ headers instead of draft ones).
 *
 *  Revision 1.15  2004/02/12 05:58:03  s_a_white
 *  Update argurements and help menu handling.
 *
 *  Revision 1.14  2004/01/31 17:07:44  s_a_white
 *  Support of specifing max sids writes forming sid2crc and experimental
 *  TSID2 library support.
 *
 *  Revision 1.13  2003/10/28 08:44:44  s_a_white
 *  Force being able to select MOS6581 from command line.
 *
 *  Revision 1.12  2003/06/27 21:09:00  s_a_white
 *  Better error checking on args and now displays invalid arguments.
 *
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
#include <cstring>
#include <iostream>
#include <vector>
using std::cout;
using std::cerr;
using std::endl;
#include "player.h"

#ifdef HAVE_HARDSID_BUILDER
#   include <sidplay/builders/hardsid.h>
#endif

#if defined(HAVE_SGI)
#   define DISALLOW_16BIT_SOUND
#   define DISALLOW_STEREO_SOUND
#endif


// Convert time from integer
bool ConsolePlayer::parseTime (const char *timeStr, uint_least32_t &time)
{
    // Check for empty string
    if (*timeStr == '\0')
        return false;

    try
    {
        const char *sep;
        uint_least32_t _time;
        std::vector<char> str(strlen(timeStr)+1);

        strcpy (&str[0], timeStr);
        sep = strstr (&str[0], ":");
        if (!sep)
        {   // User gave seconds
            _time = atoi (&str[0]);
        }
        else
        {   // Read in MM:SS format
            char min[3] = {'\0', '\0', '\0'};
            int  val;

            if (str[0] == ':')
                return false;
            min[0]  = str[0];
            if (str[1] != ':')
            {
                min[1]  = str[1];
                if (str[2] != ':')
                    return false;
            }

            val = atoi (min);
            if (val < 0 || val > 99)
                return false;
            _time = (uint_least32_t) val * 60;
            val   = atoi (sep + 1);
            if (val < 0 || val > 59)
                return false;
            _time += (uint_least32_t) val;
        }

        time = _time;
    }
    catch (...)
    {
        return false;
    }
    return true;
}

// Parse command line arguments
int ConsolePlayer::args (int argc, const char *argv[])
{
    int  infile = 0;
    int  i      = 0;
    bool err    = false;

    enum
    {
        PRINT_NONE    = 0,
        PRINT_TITLE   = 1,
        PRINT_AUTHOR  = 2,
        PRINT_RELEASE = 3,
        PRINT_CLOCK   = 4,
        PRINT_MODEL   = 5
    } print = PRINT_NONE;

    if (argc == 0) // at least one argument required
    {
        displayArgs ();
        return -1;
    }

    // default arg options
#ifdef HAVE_WAV_ONLY
    m_driver.output = OUT_WAV;
    m_driver.file   = true;
#else
    m_driver.output = OUT_SOUNDCARD;
    m_driver.file   = false;
#endif
    m_driver.sid    = EMU_RESID;

    // parse command line arguments
    while ((i < argc) && (argv[i] != NULL))
    {
        if ((argv[i][0] == '-') && (argv[i][1] != '\0'))
        {
            // help options
            if ((argv[i][1] == 'h') || !strcmp(&argv[i][1], "-help"))
            {
                displayArgs ();
                return 0;
            }
            else if (!strcmp(&argv[i][1], "-help-debug"))
            {
                displayDebugArgs ();
                return 0;
            }

            else if (argv[i][1] == 'b')
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
                        return -1;
                    }
                }
            }

            // Newer sid (8580)
            else if (strncmp (&argv[i][1], "ns", 2) == 0)
            {
                switch (argv[i][3])
                {
                case '\0':
                case '1':
                    m_engCfg.sidModel = SID2_MOS8580;
                    break;
                // No new sid so use old one (6581)
                case '0':
                    m_engCfg.sidModel = SID2_MOS6581;
                    break;
                default:
                    err = true;
                }
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
            // Depreciated
            else if (argv[i][1] == 'q')
            {
                if (argv[i][2] == '\0')
                    m_verboseLevel = -1;
                else
                    m_verboseLevel = -atoi(&argv[i][2]);
            }
            else if (strncmp (&argv[i][1], "-crc", 4) == 0)
            {
                m_crc = ~0;
                if (argv[i][5] == '=')
                    m_crc = (uint_least32_t) atoi(&argv[i][6]);
            }
            else if (strncmp (&argv[i][1], "-delay=", 7) == 0)
            {
                m_engCfg.powerOnDelay = (uint_least16_t) atoi(&argv[i][8]);
            }

            // File format conversions
            else if (argv[i][1] == 'w')
            {
                m_driver.output = OUT_WAV;
                m_driver.file   = true;
                if (argv[i][2] != '\0')
                    m_outfile = &argv[i][2];
            }
            else if (strncmp (&argv[i][1], "-wav=", 5) == 0)
            {
                m_driver.output = OUT_WAV;
                m_driver.file   = true;
                if (argv[i][6] != '\0')
                    m_outfile = &argv[i][6];
            }
            else if (strncmp (&argv[i][1], "-au=", 4) == 0)
            {
                m_driver.output = OUT_AU;
                m_driver.file   = true;
                if (argv[i][5] != '\0')
                    m_outfile = &argv[i][5];
            }
            else if (strncmp (&argv[i][1], "-raw=", 5) == 0)
            {
                m_driver.output = OUT_RAW;
                m_driver.file   = true;
                if (argv[i][6] != '\0')
                    m_outfile = &argv[i][6];
            }

            // Hardware selection
#ifdef HAVE_HARDSID_BUILDER
            else if (strcmp (&argv[i][1], "-hardsid") == 0)
            {
                m_driver.sid    = EMU_HARDSID;
                m_driver.output = OUT_NULL;
            }
#endif // HAVE_HARDSID_BUILDER

            // These are for debug
            else if (strcmp (&argv[i][1], "-none") == 0)
            {
                m_driver.sid    = EMU_NONE;
                m_driver.output = OUT_NULL;
            }
            else if (strcmp (&argv[i][1], "-nosid") == 0)
            {
                m_driver.sid = EMU_NONE;
            }
            else if (strcmp (&argv[i][1], "-cpu-debug") == 0)
            {
                m_cpudebug = true;
            }

// Print options
            else if (strcmp (&argv[i][1], "-print-title") == 0)
                print = PRINT_TITLE;
            else if (strcmp (&argv[i][1], "-print-author") == 0)
                print = PRINT_AUTHOR;
            else if (strcmp (&argv[i][1], "-print-release") == 0)
                print = PRINT_RELEASE;
            else if (strcmp (&argv[i][1], "-print-clock") == 0)
                print = PRINT_CLOCK;
            else if (strcmp (&argv[i][1], "-print-model") == 0)
                print = PRINT_MODEL;

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
            return -1;
        }

        i++;  // next index
    }

    // Load the tune
    m_filename = argv[infile];
    m_tune.load (m_filename);
    if (!m_tune)
    {
        displayError ((m_tune.getInfo ()).statusString);
        return -1;
    }

    // Select the desired track
    m_track.first    = m_tune.selectSong (m_track.first);
    m_track.selected = m_track.first;

    if (print != PRINT_NONE)
    {
        const SidTuneInfo &tuneInfo = m_tune.getInfo ();
        if (!tuneInfo.musPlayer && (tuneInfo.numberOfInfoStrings == 3))
        {
            if (print == PRINT_TITLE)
                cout << tuneInfo.infoString[0];
            if (print == PRINT_AUTHOR)
                cout << tuneInfo.infoString[1];
            if (print == PRINT_RELEASE)
                cout << tuneInfo.infoString[2];
        }
        if (print == PRINT_CLOCK)
        {
            if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
                cout << "PAL";
            else if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
                cout << "NTSC";
        }
        if (print == PRINT_MODEL)
        {
            if (tuneInfo.sidModel1 == SIDTUNE_SIDMODEL_6581)
                cout << "6581";
            else if (tuneInfo.sidModel1 == SIDTUNE_SIDMODEL_8580)
                cout << "8580";
        }
        cout << std::flush; // cerr << endl can overtake
        return 0;
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
        return -1;
    }
    
    if (m_track.single)
        m_track.songs = 1;

    // CRC handling (remove random behaviour)
    if (m_crc)
    {
        m_engCfg.powerOnDelay = 0;
        m_engCfg.sid2crcCount = m_crc;
    }

    // If user provided no time then load songlength database
    // and set default lengths incase it's not found in there.
    {
        if (m_driver.file && m_timer.valid && !m_timer.length)
        {   // Time of 0 provided for wav generation
            displayError ("ERROR: -t0 invalid in record mode");
            return -1;
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
                    return -1;
                }
            }
        }
    }

    if (!m_filter.definition)
        m_filter.definition = m_iniCfg.filter (m_engCfg.sidModel);

#if HAVE_TSID == 1
    // Set TSIDs base directory
    if (!m_tsid.setBaseDir(true))
    {
        displayError (m_tsid.getError ());
        return -1;
    }
#endif

    // Configure engine with settings
    if (m_engine->config (m_engCfg) < 0)
    {   // Config failed
        displayError (m_engine->error ());
        return -1;
    }
    return 1;
}


void ConsolePlayer::displayArgs (const char *arg)
{
    std::ostream &out = arg ? cerr : cout;

    if (arg)
        out << "Option Error: " << arg << endl;
    else
        out << "Syntax: " << m_name << " [-<option>...] <datafile>" << endl;

    out << "Options:" << endl
        << " --help|-h    display this screen" << endl
        << " --help-debug debug help menu" << endl
        << " -b<num>      set start time in [m:]s format (default 0)" << endl

        << " -f<num>      set frequency in Hz (default: "
        << SID2_DEFAULT_SAMPLING_FREQ << ")" << endl
        << " -fd          force dual sid environment" << endl
        << " -fs          force samples to a channel (default: uses sid)" << endl

        << " -nf[filter]  no/new SID filter emulation" << endl
        << " -ns[0|1]     (no) MOS 8580 waveforms (default: from tune or cfg)" << endl

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

        << " -v[num]      verbose output (positive more, negative quieter)" << endl
        << " -v[p|n][f]   set VIC PAL/NTSC clock speed (default: defined by song)" << endl
        << "              Use 'f' to force the clock by preventing speed fixing" << endl

        << " -w[name]     create wav file (default: <datafile>[n].wav)" << endl;
#ifdef HAVE_HARDSID_BUILDER
    {
#ifdef HAVE_SID2_COM
        SidLazyIPtr<ISidUnknown>    unknown (HardSIDBuilderCreate (""));
        SidLazyIPtr<HardSIDBuilder> hs = unknown;
#else // Depreciated Interface
#   ifdef HAVE_EXCEPTIONS
        HardSIDBuilder *hs = new(std::nothrow) HardSIDBuilder( HARDSID_ID );
#   else
        HardSIDBuilder *hs = new HardSIDBuilder( HARDSID_ID );
#   endif
#endif // HAVE_SID2_COM
        if (hs)
        {
            if (hs->devices (false))
                out << " --hardsid    enable hardsid support" << endl;
#ifndef HAVE_SID2_COM
            delete hs;
#endif
	}
    }
#endif // HAVE_HARDSID_BUILDER
    out << endl
        // Changed to new homepage address
        << "Home Page: http://sidplay2.sourceforge.net/" << endl;
//        << "Mail comments, bug reports, or contributions to <sidplay2@email.com>." << endl;
}


void ConsolePlayer::displayDebugArgs ()
{
    std::ostream &out = cout;

    out << "Debug Options:" << endl
        << " --cpu-debug   display cpu register and assembly dumps" << endl
        << " --crc[=<num>] generate CRC for [<num>] sid writes (default: 0)" << endl
        << " --delay=<num> simulate c64 power on delay" << endl

        << " -m            PlaySID compatibility mode (read the docs!)" << endl
        << " -mt           Sidplays Transparent Rom mode" << endl
        << " -mb           Sidplays Bankswitching mode" << endl
        << " -mr           Real C64 mode (default)>" << endl

        << " --none        no audio output device" << endl
        << " --nosid       no sid emulation" << endl;
}
