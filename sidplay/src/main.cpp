/***************************************************************************
                          main.cpp  -  description
                             -------------------
    begin                : Fri Jun 2 2000
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
 *  Revision 1.18  2001/09/01 11:24:22  s_a_white
 *  Renamed configure to config.
 *
 *  Revision 1.17  2001/08/20 18:29:55  s_a_white
 *  SID Model now obtained from info structure.
 *
 *  Revision 1.16  2001/07/25 17:11:32  s_a_white
 *  Support new libsidplay2 configuration interface.
 *
 *  Revision 1.15  2001/07/14 12:38:19  s_a_white
 *  Added sid loop counter, to exit multi-sid tunes.  Added -b to set start
 *  of song.  0xffff songs now get reported as sys.  Support for sidbuilder
 *  classes.  !TODO! must tidy this file, it's getting too big.
 *
 *  Revision 1.14  2001/04/23 17:08:33  s_a_white
 *  Added extended video flag -v<n|p>[f].
 *
 *  Revision 1.13  2001/04/21 13:28:31  s_a_white
 *  Updated help information.
 *
 *  Revision 1.12  2001/04/21 10:28:22  s_a_white
 *  Fix -w flag to take an optional filename.
 *
 *  Revision 1.11  2001/03/27 19:35:33  s_a_white
 *  Moved default record length for wav files from main.cpp to IniConfig.cpp.
 *
 *  Revision 1.10  2001/03/27 19:00:49  s_a_white
 *  Default record and play lengths can now be set in the sidplay2.ini file.
 *
 *  Revision 1.9  2001/03/27 17:14:39  s_a_white
 *  Time length can be made INFINITE by using -t0 on the command line.
 *
 *  Revision 1.8  2001/03/26 18:14:20  s_a_white
 *  Removed debug code.
 *
 *  Revision 1.7  2001/03/21 22:54:55  s_a_white
 *  Support for ini config file and libsidutils tools.
 *
 *  Revision 1.6  2001/03/04 12:58:56  s_a_white
 *  Defualt environment changed to real.  Verbose info now printed correctly.
 *
 *  Revision 1.5  2001/03/01 23:47:00  s_a_white
 *  Support for sample mode to be selected at runtime.
 *
 *  Revision 1.4  2001/02/08 20:58:01  s_a_white
 *  Help screen bug fix for default precision and optimisation, which were printed
 *  as characters.
 *
 *  Revision 1.3  2001/01/23 22:54:24  s_a_white
 *  Prevents timer overwriting paused message.
 *
 *  Revision 1.2  2001/01/23 21:25:15  s_a_white
 *  Only way to load a tune now is by passing in a sidtune object.  This is
 *  required for songlength database support.
 *
 *  Revision 1.1  2001/01/08 16:41:42  s_a_white
 *  App and Library Seperation
 *
 *  Revision 1.20  2000/12/11 18:52:12  s_a_white
 *  Conversion to AC99
 *
 ***************************************************************************/

#include <fstream.h>
#include <iostream.h>
#include <iomanip.h>

#include <ctype.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Lots of sidTune stuff
// @TODO@: Tidy this code up
#include <sidplay/sidplay2.h>
#include <sidplay/utils/SidDatabase.h>
#include "audio/AudioDrv.h"
#include "keyboard.h"
#include "IniConfig.h"

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#if defined(__amigaos__)
#   define EXIT_ERROR_STATUS (20)
#else
#   define EXIT_ERROR_STATUS (-1)
#endif

#if defined(HAVE_SGI)
#   define DISALLOW_16BIT_SOUND
#   define DISALLOW_STEREO_SOUND
#endif

// Error and status message numbers.
enum
{
    ERR_SYNTAX = 0,
    ERR_NOT_ENOUGH_MEMORY,
    ERR_SIGHANDLER,
    ERR_FILE_OPEN
};

typedef enum {black, red, green, yellow, blue, magenta, cyan, white} textColour_t;
typedef enum {sid2_tableStart, sid2_tableMiddle, sid2_tableSeperator, sid2_tableEnd} sid2_table_t;

// Rev 1.13 (saw) - Grouped global variables
static class sidplayer_t
{
public:
    sidplay2            lib;
    AudioBase          *audioDrv;
    AudioBase          *useDrv;
    sid2_config_t       cfg;

    IniConfig           ini;
    SidDatabase         database;
    SidFilter           filter;

    // Rev 1.11 (saw) - Bug fix for Ctrl C exiting
    volatile bool       fastExit;
    uint_least8_t       quietLevel;
    uint_least16_t      selectedSong;
    uint_least16_t      startSong;
    uint_least16_t      songs;
    bool                looping;
    bool                singleTrack;
    bool                restart;
    bool                paused;
    uint_least8_t       speed;
    const uint_least8_t speedMax;

public:
    sidplayer_t::sidplayer_t ()
      :speedMax(32)
    {
        audioDrv     = NULL;
        useDrv       = NULL;
        fastExit     = false;
        // (ms) Opposite of verbose output.
        // 1 = no time display
        quietLevel   = 0;
        selectedSong = 0;
        restart      = false;    
        paused       = false;
        speed        = 1;
        looping      = false;
        singleTrack  = false;

        // Read configuration settings
        cfg = lib.config ();
        ini.read ();
    }
} player;


// Function prototypes
#define SAFE_DELETE(x) if (x) {delete x; x = 0;}
static void displayError  (char *arg0, uint num);
static void displaySyntax (char *arg0);
static void sighandler    (int signum);
static void cleanup       (bool fast);
static void textColour    (textColour_t colour, bool bold);
static void displayTable  (sid2_table_t table);
static void decodeKeys ();
static bool parseTime     (char *str, uint_least32_t &time);

// Rev 2.0.4 (saw) - Added for better MAC support
static inline int_least32_t generateMusic (AudioConfig &cfg, void *buffer);

int main(int argc, char *argv[])
{
    sid2_env_t      playerMode    = sid2_envR;
    bool            wavOutput     = false;
    uint            sidFile       = 0;
    char           *wavName       = NULL;
    void           *nextBuffer    = NULL;
    uint_least32_t  runtime       = 0;
    bool            timeValid     = false;
    bool            verboseOutput = false;
    bool            force2SID     = false;
    Audio_Null      nullDrv;

    // Rev 1.9 (saw) - Default now obtained from sidplayer.h
    AudioConfig  audioCfg;
    SidTuneMod   tune (0);
    SidTuneInfo  tuneInfo;
    sid2_info_t  info;

    // (ms) Incomplete...
    // Fastforward/Rewind Patch
    uint_least32_t  starttime  = 0;
    IniConfig::sidplay2_section  sidplay2  = player.ini.sidplay2();

    {   // Load ini settings
        IniConfig::audio_section     audio     = player.ini.audio();
        IniConfig::emulation_section emulation = player.ini.emulation();

        // INI Configuration Settings
        player.cfg.clockForced  = emulation.clockForced;
        player.cfg.clockSpeed   = emulation.clockSpeed;
        player.cfg.frequency    = audio.frequency;
        player.cfg.optimisation = emulation.optimiseLevel;
        player.cfg.playback     = audio.playback;
        player.cfg.precision    = audio.precision;
        player.cfg.sidFilter    = emulation.filter;
        player.cfg.sidModel     = emulation.sidModel;
        player.cfg.sidSamples   = emulation.sidSamples;
    }

    if (argc < 2) // at least one argument required
    {
        displayError (argv[0], ERR_SYNTAX);
        goto main_error;
    }

    {   // parse command line arguments
        int i = 1;
        while ((i < argc) && (argv[i] != NULL))
        {
            uint x = 0;
            if ((argv[i][0] == '-') && (argv[i][1] != '\0'))
            {
                x++;
                switch (argv[i][x++])
                {
                case 'b':
                    if (!parseTime (&argv[i][x], starttime))
                        break;
                    // Show that complete string was parsed
                    while (argv[i][x] != '\0')
                        x++;
                break;

                case 'f':
                    switch (argv[i][x++])
                    {
                    case '\0':
                        // User forgot frequency number
                        x = 0;
                    break;

                    case 'd':
                        // Override sidTune and enable the second sid
                        player.cfg.forceDualSids = true;
                    break;

                    case 's':
                        // Force samples through soundcard instead of SID
                        player.cfg.sidSamples = false;
                    break;

                    default:
                        player.cfg.frequency = atoi(argv[i] + x - 1);
                        // Show that all string was processed
                        while (argv[i][x] != '\0')
                            x++;
                    break;
                    }
                break;
        
                // Player Mode (Environment) Options
                case 'm':
                    switch (argv[i][x++])
                    {
                    case '\0':
                        player.cfg.environment = sid2_envPS;
                        x--;
                    break;

                    case 't':
                        player.cfg.environment = sid2_envTP;
                    break;

                    case 'b':
                        player.cfg.environment = sid2_envBS;
                    break;

                    case 'r':
                        player.cfg.environment = sid2_envR;
                    break;

                    default:
                        x = 0;
                    break;
                    }
                break;

                // New/No options
                case 'n':
                    switch (argv[i][x++])
                    {
                    // Filter options
                    case 'f':
                        if (argv[i][x] == '\0')
                        {   // Disable filter
                            player.cfg.sidFilter = false;
                            break;
                        }

                        {   // New filter
                            // This line will open an existing file
                            player.filter.read (&(argv[i][x]));
                            if (!player.filter)
                            {
                                cerr << argv[0] << "\n" << player.filter.error () << endl;
                                goto main_error;
                            }
                            player.cfg.sidFilterDef = player.filter.definition ();

                            // Show that complete string was parsed
                            while (argv[i][x] != '\0')
                                x++;
                        }
                    break;

                    // Newer sid (8580)
                    case 's':
                        if (argv[i][x] == '\0')
                        {   // Select newer sid
                            // Rev 1.18 (MiKiL) - Changed from MOS6581
                            player.cfg.sidModel = SID2_MOS8580;
                            break;
                        }
                    break;

                    default:
                        x = 0;
                    break;
                    }
                break;

                case 'o':
                    // Looping
                    if (argv[i][x] == 'l')
                    {
                        player.looping = true;
                        if (argv[i][++x] == '\0')
                            break;
                    }

                    // Play selected track only
                    if (argv[i][x] == 's')
                    {
                        player.singleTrack = true;
                        if (argv[i][++x] == '\0')
                            break;
                    }

                    if (argv[i][x] == '\0')
                    {   // User forgot track number
                        x = 0;
                        break;
                    }

                    player.selectedSong = atoi(argv[i] + x);
                    // Show that all string was processed
                    while (argv[i][x] != '\0')
                        x++;
                break;

                case 'O':
                    if (argv[i][x] == '\0')
                    {   // User optimisation level
                        x = 0;
                        break;
                    }

                    player.cfg.optimisation = atoi(argv[i] + x);
                    // Show that all string was processed
                    while (argv[i][x] != '\0')
                        x++;
                break;

                case 'p':
                    if (argv[i][x] == '\0')
                    {
                        // User forgot precision
                        x = 0;
                    }

                    {
                        uint_least8_t precision = atoi(argv[i] + x);
                        if (precision <= 8)
                            precision = 8;
                        else if (precision <= 16)
                            precision = 16;
                        else
                            precision = 24;

                        if (precision > SID2_MAX_PRECISION)
                            precision = SID2_MAX_PRECISION;
                        player.cfg.precision = precision;

                        // Show that all string was processed
                        while (argv[i][x] != '\0')
                            x++;
                    }
                break;

                case 'q':
                    // Later introduce incremental mode.
                    if (argv[i][x] == '\0')
                        ++player.quietLevel;
                break;

                // Stereo Options
                case 's':
                    switch (argv[i][x++])
                    {
                    case '\0':
                        // Stereo Playback
                        player.cfg.playback = sid2_stereo;
                        x--;
                    break;

                    case 'l':
                        // Left Channel
                        player.cfg.playback = sid2_left;
                    break;

                    case 'r':
                        // Right Channel
                        player.cfg.playback = sid2_right;
                    break;

                    default:
                        x = 0;
                    break;
                    }
                break;

                case 't':
                    if (!parseTime (&argv[i][x], runtime))
                        break;
                    timeValid = true;
                    // Show that complete string was parsed
                    while (argv[i][x] != '\0')
                        x++;
                break;

                // Video Options
                case 'v':
                    switch (argv[i][x++])
                    {
                    case '\0':
                        verboseOutput = true;
                        x--;
                    break;

                    case 'n':
                        player.cfg.clockSpeed = SID2_CLOCK_NTSC;
                        if (argv[i][x] == 'f')
                        {
                            x++;
                            player.cfg.clockForced = true;
                        }
                    break;

                    case 'p':
                        player.cfg.clockSpeed = SID2_CLOCK_PAL;
                        if (argv[i][x] == 'f')
                        {
                            x++;
                            player.cfg.clockForced = true;
                        }
                    break;

                    case 'f':
                        player.cfg.clockForced = true;
                    break;

                    default:
                        x = 0;
                    break;
                    }
                break;

                case 'w':
                    wavOutput = true;
                    if (argv[i][x] != '\0')
                    {
                        wavName = &argv[i][x];
                        while (argv[i][x] != '\0')
                            x++;
                    }
                break;

                default:
                    x = 0;
                break;
                }

                // Make sure all all string was checked
                if (argv[i][x] != '\0')
                {
                    displaySyntax (argv[0]);
                    goto main_error;
                }
            }
            else
            {   // Reading file name
                if (!sidFile)
                {
                    sidFile = i;
                }
                else
                {
                    displayError (argv[0], ERR_SYNTAX);
                    goto main_error;
                }
            }
            i++;  // next index
        }
    }

    if (sidFile == 0)
    {   // Neither file nor stdin.
        displaySyntax(argv[0]);
        goto main_error;
    }

    audioCfg.channels       = 1; // Mono
    if (player.cfg.playback == sid2_stereo)
        audioCfg.channels   = 2;
    audioCfg.frequency = player.cfg.frequency;
    audioCfg.precision = player.cfg.precision;
    
    if (!wavOutput)
    {
        // Open Audio Driver
#ifdef HAVE_EXCEPTIONS
        player.audioDrv = new(nothrow) AudioDriver;
#else
        player.audioDrv = new AudioDriver;
#endif
        if (!player.audioDrv)
        {
            displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
            goto main_error;
        }

        nextBuffer = player.audioDrv->open (audioCfg, "");
    }

    // Check to make sure that hardware supports stereo
    if (player.cfg.playback == sid2_stereo)
    {
        if (audioCfg.channels  != 2)
            player.cfg.playback = sid2_mono;
    }

    tune.load (argv[sidFile]);
    if (!tune)
    {
        cerr << argv[0] << "\n" << (tune.getInfo ()).statusString << endl;
        goto main_error;
    }        

    player.startSong = (tune.getInfo ()).startSong;
    player.songs     = (tune.getInfo ()).songs;

    // Play all tunes if no track specifically asked for
    if (!player.selectedSong)
        player.singleTrack = false;
    else
    {
        player.selectedSong += (player.startSong - 1);
        if (player.selectedSong > player.songs)
            player.selectedSong -= player.songs;
        if (player.singleTrack)
        {
            player.songs     = 1;
            player.startSong = player.selectedSong;
        }
    }

    // Load the filter
    if (!player.cfg.sidFilterDef)
        player.cfg.sidFilterDef = player.ini.filter (player.cfg.sidModel);

    // Configure Emulation
    if (player.lib.config (player.cfg) < 0)
    {
        cerr << argv[0] << "\n" << player.lib.error () << endl;
        goto main_error;
    }

    // Load song length database
    if (!timeValid)
        player.database.open (sidplay2.database);

main_restart:
    player.selectedSong = tune.selectSong (player.selectedSong);
    if (!tune)
    {
        cerr << argv[0] << "\n" << player.lib.error () << endl;
        goto main_error;
    }

    if (player.lib.loadSong (&tune) == -1)
    {
        cerr << argv[0] << "\n" << player.lib.error () << endl;
        goto main_error;
    }

    // See if we can get the songs length
    if (!timeValid)
    {
        int_least32_t ret = player.database.length (tune);
        if (ret > 0)
            runtime = ret;
        else if (!wavOutput)
            runtime = sidplay2.playLength;
    }

    // Rev 1.12 (saw) Moved to allow modification of wav filename
    // based on subtune
    tune.getInfo (tuneInfo);

    if (wavOutput)
    {
        WavFile *wavFile    = 0;
        bool     deleteName = false;

        if (!wavName)
        {
            // Generate a name for the wav file
            size_t  length, i;

            length = strlen (tuneInfo.dataFileName);
            i      = length;
            while (i > 0)
            {
                if (tuneInfo.dataFileName[--i] == '.')
                    break;
            }
            if (!i) i = length;
        
#ifdef HAVE_EXCEPTIONS
            wavName = new(nothrow) char[i + 10];
#else
            wavName = new char[i + 10];
#endif
            if (!wavName)
            {
                displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
                goto main_error;
            }

            deleteName = true;
            strcpy (wavName, tuneInfo.dataFileName);
            // Prevent extension ".sid.wav"
            wavName[i] = '\0';

            // Rev 1.12 (saw) - Modified to change wav name based on subtune
            // Now we have a name
            if (tuneInfo.songs > 1)
                sprintf (&wavName[i], "[%u]", tuneInfo.currentSong);
            strcat (&wavName[i], ".wav");
        }

        // lets create the wav object
#ifdef HAVE_EXCEPTIONS
        wavFile = new(nothrow) WavFile;
#else
        wavFile = new WavFile;
#endif

        if (!wavFile)
        {
            if (deleteName)
                SAFE_DELETE (wavName);
            displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
            goto main_error;
        }

        player.audioDrv = wavFile;
        nextBuffer =  wavFile->open (audioCfg, wavName, true);
        if (deleteName)
            SAFE_DELETE (wavName);

        if (!runtime)
            runtime = sidplay2.recordLength;
    }

    if (!nextBuffer)
    {
        cerr << argv[0] << "\n" << player.audioDrv->getErrorString () << endl;
        goto main_error;
    }

    nullDrv.open (audioCfg, "");

    if (timeValid)
    {   // Run time is relative to start time
        runtime += starttime;
    }
    else
    {   // Check to make start time dosen't exceed end
        if (runtime & (starttime >= runtime))
        {
            cerr << argv[0] << "\n" << "ERROR: Start time exceeds song length!" << endl;
            goto main_error;
        }
    }

    info = player.lib.info ();
    tuneInfo = *info.tuneInfo;
    // cerr << (char) 12 << '\b'; // New Page
    if ((player.ini.console ()).ansi)
    {
        cerr << '\x1b' << "[40m";  // Background black
        cerr << '\x1b' << "[2J";   // Clear screen
        cerr << '\x1b' << "[0;0H"; // Move cursor to 0,0
    }

    displayTable (sid2_tableStart);
    displayTable (sid2_tableMiddle);
    textColour (red, true);
    cerr << "   SID";
    textColour (blue, true);
    cerr << "PLAY";
    textColour (white, true);
    cerr << " - Music Player and C64 SID Chip Emulator" << endl;
    displayTable (sid2_tableMiddle);
    textColour (white, false);
    cerr << setw(19) << "Sidplay" << " V" << VERSION << ", ";
    cerr << (char) toupper (*info.name);
    cerr << info.name + 1 << " V" << info.version << endl;

    displayTable (sid2_tableSeperator); 
    if (tuneInfo.numberOfInfoStrings == 3)
    {
        displayTable (sid2_tableMiddle);
        textColour   (cyan, true);
        cerr << " Name         : ";
        textColour   (magenta, true);
        cerr << tuneInfo.infoString[0] << endl;
        displayTable (sid2_tableMiddle);
        textColour   (cyan, true);
        cerr << " Author       : ";
        textColour   (magenta, true);
        cerr << tuneInfo.infoString[1] << endl;
        displayTable (sid2_tableMiddle);
        textColour   (cyan, true);
        cerr << " Copyright    : ";
        textColour   (magenta, true);
        cerr << tuneInfo.infoString[2] << endl;
    }
    else
    {
        for (int infoi = 0; infoi < tuneInfo.numberOfInfoStrings; infoi++)
        {
            displayTable (sid2_tableMiddle);
            textColour   (cyan, true);
            cerr << " Description  : ";
            textColour   (magenta, true);
            cerr << tuneInfo.infoString[infoi] << endl;
        }
    }

    displayTable (sid2_tableSeperator);
    if (verboseOutput)
    {
        displayTable (sid2_tableMiddle);
        textColour   (green, true);
        cerr << " File format  : ";
        textColour   (white, true);
        cerr << tuneInfo.formatString << endl;
        displayTable (sid2_tableMiddle);
        textColour   (green, true);
        cerr << " Filename(s)  : ";
        textColour   (white, true);
        cerr << tuneInfo.dataFileName << endl;
        // Second file is only sometimes present
        if (tuneInfo.infoFileName[0])
        {
            displayTable (sid2_tableMiddle);
            textColour   (green, true);
            cerr << "              : ";
            textColour   (white, true);
            cerr << tuneInfo.infoFileName << endl;
        }
        displayTable (sid2_tableMiddle);
        textColour   (green, true);
        cerr << " Condition    : ";
        textColour   (white, true);
        cerr << tuneInfo.statusString << endl;
    }

    displayTable (sid2_tableMiddle);
    textColour   (green, true);
    cerr << " Playlist     : ";
    textColour   (white, true);

    {   // This will be the format used for playlists
        int i = 1;        
        if (!player.singleTrack)
        {
             i  = player.selectedSong;
            i -= (player.startSong - 1);
            if (i < 1)
                i += player.songs;
        }
        cerr << i << '/' << player.songs;
        cerr << " (tune " << tuneInfo.currentSong << '/'
         << tuneInfo.songs << '['
             << tuneInfo.startSong << "])";
    }

    if (player.looping)
        cerr << " [LOOPING]";
    cerr << endl;

    if (verboseOutput)
    {
        displayTable (sid2_tableMiddle);
        textColour   (green, true);
        cerr << " Song Speed   : ";
        textColour   (white, true);
        cerr << tuneInfo.speedString << endl;
    }

    displayTable (sid2_tableMiddle);
    textColour   (green, true);
    cerr << " Song Length  : ";
    textColour   (white, true);
    if (runtime)
        cerr << setw(2) << setfill('0') << ((runtime / 60) % 100)
             << ':' << setw(2) << setfill('0') << (runtime % 60) << endl;
    else if (timeValid)
        cerr << "FOREVER" << endl;
    else
        cerr << "UNKNOWN" << endl;

    if (verboseOutput)
    {
        displayTable (sid2_tableSeperator);
        displayTable (sid2_tableMiddle);
        textColour   (yellow, true);
        cerr << " Addresses    : " << hex;
        cerr.setf(ios::uppercase);
        textColour   (white, false);
        cerr << "DRIVER=$" << setw(4) << setfill('0') << info.driverAddr;
        cerr << ", LOAD=$" << setw(4) << setfill('0') << tuneInfo.loadAddr;
        cerr << "-$"       << setw(4) << setfill('0') << tuneInfo.loadAddr +
            (tuneInfo.c64dataLen - 1) << endl;
        displayTable (sid2_tableMiddle);
        textColour   (yellow, true);
        cerr << "              : ";
        textColour   (white, false);
        if (tuneInfo.playAddr == 0xffff)
            cerr << " SYS=$" << setw(4) << setfill('0') << tuneInfo.initAddr;
        else
        {
            cerr << "INIT  =$" << setw(4) << setfill('0') << tuneInfo.initAddr;
            cerr << ", PLAY=$" << setw(4) << setfill('0') << tuneInfo.playAddr;
        }
        cerr << dec << endl;
        cerr.unsetf(ios::uppercase);

        displayTable (sid2_tableMiddle);
        textColour   (yellow, true);
        cerr << " SID Details  : ";
        textColour   (white, false);
        cerr << "Filter = "
             << ((player.cfg.sidFilter == true) ? "Yes" : "No");
        cerr << ", Model = "
             << ((info.tuneInfo->sidRev8580) ? "8580" : "6581")
             << endl;
        displayTable (sid2_tableMiddle);
        textColour   (yellow, true);
        cerr << " Environment  : ";
        textColour   (white, false);
        switch (player.cfg.environment)
        {
        case sid2_envPS:
            cerr << "PlaySID (PlaySID-specific rips)" << endl;
        break;
        case sid2_envTP:
            cerr << "Transparent ROM" << endl;
        break;
        case sid2_envBS:
            cerr << "Bank Switching" << endl;
        break;
        case sid2_envR:  // When it happens
            cerr << "Real C64 (default)" << endl;
        break;
        }
    }
    displayTable (sid2_tableEnd);
/*
    cerr << "Credits:\n";
    const char **p;
    const char  *credit;
    p = player.lib.credits ();
    while (*p)
    {
        credit = *p;
        while (*credit)
        {
            cerr << credit << endl;
            credit += strlen (credit) + 1;
        }
        cerr << endl;
        p++;
    }
*/
    if (!wavOutput)
        cerr << "Playing, press ^C to stop...";
    else
        cerr << "Creating WAV file, please wait...";

    // Get all the text to the screen so music playback
    // is not disturbed.
    if ( player.quietLevel == 1 )
        cerr << endl;
    else
        cerr << "00:00";
    cerr << flush;

    // Install signal error handlers
    if ((signal (SIGINT,  &sighandler) == SIG_ERR)
     || (signal (SIGABRT, &sighandler) == SIG_ERR)
     || (signal (SIGTERM, &sighandler) == SIG_ERR))
    {
        displayError(argv[0], ERR_SIGHANDLER);
        goto main_error;
    }

#ifdef HAVE_UNIX
    // Configure terminal to allow direct access to key events
    keyboard_enable_raw ();
#endif // HAVE_UNIX

    // Debug hack
    if (starttime == 0)
    {
        player.lib.debug (true);
        player.useDrv = player.audioDrv;
    }
    else
    {
        player.useDrv = &nullDrv;
//        player.speed  = 32;
//        player.lib.fastForward (100 * player.speed);
    }

    // Play loop
    nextBuffer    = player.useDrv->buffer ();
    player.paused = false;

    FOREVER
    {
        int_least32_t ret;
        ret = generateMusic (audioCfg, nextBuffer);
        // Check for ^C
        if (player.fastExit)
            break;

        nextBuffer = player.useDrv->write ();
        if (ret   != 0)
        {   // -1 means end
            if (ret < 0)
                break;
            if ((uint_least32_t) ret >= runtime)
                break;
            if ((uint_least32_t) ret == starttime)
            {
                player.lib.debug (true);
                nextBuffer    = player.useDrv->buffer ();
                player.useDrv = player.audioDrv;
                player.speed  = 1;
                player.lib.fastForward (100 * player.speed);
            }
        }
    }

#ifdef HAVE_UNIX
    keyboard_disable_raw ();
#endif

    // Install signal error handlers
    if ((signal (SIGINT,  SIG_DFL) == SIG_ERR)
     || (signal (SIGABRT, SIG_DFL) == SIG_ERR)
     || (signal (SIGTERM, SIG_DFL) == SIG_ERR))
    {
        displayError(argv[0], ERR_SIGHANDLER);
        goto main_error;
    }

    if (!player.fastExit)
    {
        bool fastReset = true;

        if (!player.restart)
        {
            if (!(wavOutput || timeValid))
            {   // It's annoying to exit at end of song when there is lots
                // of subtunes and we were skipping through and accidently waited
                // too long on a short track.  As a result increment naturally to
                // the next tune during normal playback.
                for (;;)
                {
                    fastReset = false;
                    if (player.singleTrack)
                        break;
                    player.selectedSong++;
                    if (player.selectedSong > tuneInfo.songs)
                        player.selectedSong = 1;
                    if (player.startSong == player.selectedSong)
                        break;
                    player.restart = true;
                    break;
                }

                if (player.looping)
                    player.restart = true;
            }
        }
            

        // Allow the same or another tune to be played
        if (player.restart)
        {   // Allows user to skip songs immediately, but prevents
            // the ends of songs benig cutoff through natural song playback
            if (fastReset)
                nextBuffer = player.audioDrv->reset();
            if (wavOutput)
                player.audioDrv->close();
            cerr << endl << endl;
            player.restart = false;
            goto main_restart;
        }
    }

#ifndef HAVE_MSWINDOWS
    cerr << endl;
#endif

    // Clean up
    cleanup (player.fastExit);
    if (wavOutput && !player.fastExit)
        cerr << (char) 7; // Bell
return EXIT_SUCCESS;

main_error:
    cleanup (true);
    return EXIT_ERROR_STATUS;
}

int_least32_t generateMusic (AudioConfig &cfg, void *buffer)
{
    static uint_least32_t currentSecs = (uint_least32_t) -1;
    uint_least32_t        seconds     = currentSecs;

    if (!player.paused)
    {   // Fill buffer
        if (!player.lib.play (buffer, cfg.bufSize))
            return -1;
        // Check to see if the clock requires updating
        seconds = player.lib.time();
    }
    else
    {   // For looping audio systems, we must silence audio
        memset (buffer, 0, cfg.bufSize);
    }

    // Check for a keypress (approx 250ms rate, but really depends
    // on music buffer sizes)
    if (_kbhit ())
    {
        decodeKeys ();
        if (player.restart)
        {
            currentSecs = (uint_least32_t) -1;
            return -1;
        }
    }

    if (player.paused)
        return 0;

    if (currentSecs != seconds)
    {
        if (!player.quietLevel)
        {
            cerr << "\b\b\b\b\b" << setw(2) << setfill('0') << ((seconds / 60) % 100)
            << ':' << setw(2) << setfill('0') << (seconds % 60) << flush;
        }
        return (currentSecs = seconds);
    }
    
    return 0;
}


void sighandler (int signum)
{
    switch (signum)
    {
    case SIGINT:
    case SIGABRT:
    case SIGTERM:
        // Rev 1.11 (saw) - Bug fix for Ctrl C exiting
        player.fastExit = true;
    break;
    default: break;
    }
}


void displayError (char* arg0, uint num)
{
    cerr << arg0 << ": ";

    switch (num)
    {
    case ERR_SYNTAX:
        cerr << "command line syntax error" << endl
             << "Try `" << arg0 << " --help' for more information." << endl;
    break;

    case ERR_NOT_ENOUGH_MEMORY:
        cerr << "ERROR: Not enough memory." << endl;
    break;

    case ERR_SIGHANDLER:
        cerr << "ERROR: Could not install signal handler." << endl;
    break;

    case ERR_FILE_OPEN:
        cerr << "ERROR: Could not open file for binary input." << endl;
    break;

    default: break;
    }
}

void displaySyntax (char* arg0)
{
    cout 
        << "Syntax: " << arg0 << " [-<option>...] <datafile>" << endl
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

        << " -ol          looping" << endl
        << " -o[s]<num>   start track [or selected track only] (default: preset)" << endl
        // Rev 1.7 (saw) - Changed max printed optimisation
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
        << endl
        // Changed to new homepage address
        << "Home Page: http://sidplay2.sourceforge.net/" << endl;
//        << "Mail comments, bug reports, or contributions to <sidplay2@email.com>." << endl;
}


void cleanup (bool fast)
{
    if (player.audioDrv)
    {   // Rev 1.11 (saw) - We are stoping via Ctrl C so
        // for speed flush all buffers
        if (fast)
            (void) player.audioDrv->reset();
        delete player.audioDrv;
    }
    if ((player.ini.console ()).ansi)
        cerr << '\x1b' << "[0m";
    cerr << endl;
}


void textColour (textColour_t colour, bool bold)
{
    if ((player.ini.console ()).ansi)
    {
        char *mode = "";

        switch (colour)
        {
        case black:   mode = "30"; break;
        case red:     mode = "31"; break;
        case green:   mode = "32"; break;
        case yellow:  mode = "33"; break;
        case blue:    mode = "34"; break;
        case magenta: mode = "35"; break;
        case cyan:    mode = "36"; break;
        case white:   mode = "37"; break;
        }

        if (bold)
            cerr << '\x1b' << "[1;40;" << mode << 'm';
        else
            cerr << '\x1b' << "[0;40;" << mode << 'm';
    }
}


void displayTable (sid2_table_t table)
{
    const uint tableWidth = 54;

    textColour (white, true);
    switch (table)
    {
    case sid2_tableStart:
        cerr << (player.ini.console ()).topLeft << setw(tableWidth)
             << setfill ((player.ini.console ()).horizontal) << ""
             << (player.ini.console ()).topRight;
    break;

    case sid2_tableMiddle:
        cerr << setw(tableWidth + 1) << setfill(' ') << ""
             << (player.ini.console ()).vertical << '\r'
             << (player.ini.console ()).vertical;
    return;

    case sid2_tableSeperator:
        cerr << (player.ini.console ()).junctionRight << setw(tableWidth)
             << setfill ((player.ini.console ()).horizontal) << ""
             << (player.ini.console ()).junctionLeft;
    break;

    case sid2_tableEnd:
        cerr << (player.ini.console ()).bottomLeft << setw(tableWidth)
             << setfill ((player.ini.console ()).horizontal) << ""
             << (player.ini.console ()).bottomRight;
    break;
    }

    // Move back to begining of row and skip first char
    cerr << "\n";
}

// Rev 1.13 (saw) - Added to allow user to change subtune
void decodeKeys ()
{
    uint_least16_t songs = player.songs;
    int action;

    do
    {
        action = keyboard_decode ();
        if (action == A_INVALID)
            continue;

        switch (action)
        {
        case A_RIGHT_ARROW:
            player.restart = true;
            if (!player.singleTrack)
            {
                player.selectedSong++;
                if (player.selectedSong > songs)
                    player.selectedSong = 1;
            }
        break;

        case A_LEFT_ARROW:
            player.restart = true;
            if (!player.singleTrack)
            {
                player.selectedSong--;
                if (player.selectedSong < 1)
                    player.selectedSong = songs;
            }
        break;

        case A_UP_ARROW:
            player.speed *= 2;
            if (player.speed > player.speedMax)
                player.speed = player.speedMax;
            player.lib.fastForward (100 * player.speed);
        break;

        case A_DOWN_ARROW:
            player.speed = 1;
            player.lib.fastForward (100);
        break;

        case A_HOME:
            player.restart = true;
            player.selectedSong = 1;
        break;

        case A_END:
            player.restart      = true;
            player.selectedSong = songs;
        break;

        case A_PAUSED:
            player.paused ^= true;
            if (player.paused)
                cerr << " [PAUSED]";
            else
            {
                cerr << "\b\b\b\b\b\b\b\b\b";
                // Just to make sure PAUSED is removed from screen
                cerr << "         ";
                cerr << "\b\b\b\b\b\b\b\b\b";
            }
        break;

        case A_QUIT:
            player.fastExit = true;
            return;
        break;
        }
    } while (_kbhit ());
}

bool parseTime (char *str, uint_least32_t &time)
{
    char *sep;
    uint_least32_t _time;
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
