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
static class sid2_player_t
{
public:
    sidplay2            lib;
    AudioBase          *audioDrv;
    // Rev 1.11 (saw) - Bug fix for Ctrl C exiting
    volatile bool       fastExit;
    uint_least8_t       quietLevel;
    uint_least16_t      selectedSong;
    bool                restart;
    sid2_playerInfo_t   info;
    bool                paused;
    uint_least8_t       speed;
    const uint_least8_t speedMax;
    IniConfig           ini;
    SidDatabase         database;
    SidFilter           userFilter;
    const sid_filter_t *filter;

public:
    sid2_player_t::sid2_player_t ()
      :speedMax(32)
    {
        audioDrv     = 0;
        fastExit     = false;
        // (ms) Opposite of verbose output.
        // 1 = no time display
        quietLevel   = 0;
        selectedSong = 0;
        restart      = false;    
        paused       = false;
        speed        = 1;
        filter       = NULL;
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

// Rev 2.0.4 (saw) - Added for better MAC support
static inline int_least32_t generateMusic (AudioConfig &cfg, void *buffer);

int main(int argc, char *argv[])
{
    sid2_env_t      playerMode    = sid2_envR;
    bool            wavOutput     = false;
    uint            sidFile       = 0;
	char           *wavName       = 0;
    void           *nextBuffer    = NULL;
    uint_least32_t  runtime       = 0;
    bool            timeValid     = false;
    bool            verboseOutput = false;
    bool            force2SID     = false;

    // Rev 1.9 (saw) - Default now obtained from sidplayer.h
    AudioConfig     audioCfg;
    SidTuneMod      tune (0);
    struct          SidTuneInfo tuneInfo;

    IniConfig::sidplay2_section  sidplay2;
    IniConfig::audio_section     audio;
    IniConfig::emulation_section emulation;

    // (ms) Incomplete...
    // Fastforward/Rewind Patch
    uint_least32_t  starttime  = 0;

    // Load ini settings
    player.ini.read ();
    sidplay2      = player.ini.sidplay2();
    audio         = player.ini.audio();
    emulation     = player.ini.emulation();

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
                case 'f':
                    switch (argv[i][x++])
                    {
                    case '\0':
                        // User forgot frequency number
                        x = 0;
                    break;

                    case 'd':
                        // Override sidTune and enable the second sid
                        force2SID = true;
                    break;

                    case 's':
                        // Force samples through soundcard instead of SID
                        emulation.sidSamples = false;
                    break;

                    default:
                        audio.frequency = atoi(argv[i] + x - 1);
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
                        playerMode = sid2_envPS;
                        x--;
                    break;

                    case 't':
                        playerMode = sid2_envTP;
                    break;

                    case 'b':
                        playerMode = sid2_envBS;
                    break;

                    case 'r':
                        playerMode = sid2_envR;
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
                    // External Filter Options
                    case 'e':
                        if (argv[i][x] == '\0')
                        {   // Disable filter
                            emulation.extFilter = false;
                            break;
                        }
                    break;

                    // Filter options
                    case 'f':
                        if (argv[i][x] == '\0')
                        {   // Disable filter
                            emulation.filter = false;
                            break;
                        }

                        {   // New filter
                            // This line will open an existing file
                            player.userFilter.read (&(argv[i][x]));
                            if (!player.userFilter)
                            {
                                cerr << argv[0] << "\n" << player.userFilter.getErrorString () << endl;
                                goto main_error;
                            }
                            player.filter = player.userFilter.definition ();

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
                            emulation.sidModel = SID2_MOS8580;
                            break;
                        }
                    break;

                    default:
                        x = 0;
                    break;
                    }
                break;

                case 'o':
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

                    emulation.optimiseLevel = atoi(argv[i] + x);
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
                        audio.precision = precision;

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
                        audio.playback = sid2_stereo;
                        x--;
                    break;

                    case 'l':
                        // Left Channel
                        audio.playback = sid2_left;
                    break;

                    case 'r':
                        // Right Channel
                        audio.playback = sid2_right;
                    break;

                    default:
                        x = 0;
                    break;
                    }
                break;

                case 't':
                {
                    char *sep;
                    bool start = false;
                    uint_least32_t time;
                    if ((argv[i][x]) == 's')
                    {
                        x++;
                        start = true;
                    }

                    sep = strstr (argv[i] + x, ":");
                    if (!sep)
                    {   // User gave seconds
                        time = atoi (argv[i] + x);
                    }
                    else
                    {   // Read in MM:SS format
                        *sep = '\0';
                        int val;
                        val  = atoi (argv[i] + x);
                        if (val < 0 || val > 99)
                            break;
                        time = (uint_least32_t) val * 60;
                        val  = atoi (sep + 1);
                        if (val < 0 || val > 59)
                            break;
                        time += (uint_least32_t) val;
                    }

                    if (!start)
                    {
                        timeValid = true;
                        runtime   = time;
                    }
                    else
                        starttime = time;

                    // Show that complete string was parsed
                    while (argv[i][x] != '\0')
                        x++;
                }
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
                        emulation.clockSpeed = SID2_CLOCK_NTSC;
                    break;

                    case 'p':
                        emulation.clockSpeed = SID2_CLOCK_PAL;
                    break;

                    case 'f':
                        emulation.clockForced = true;
                    break;

                    default:
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

    audioCfg.channels     = 1; // Mono
    if (audio.playback   == sid2_stereo)
        audioCfg.channels = 2;
    audioCfg.frequency = audio.frequency;
    audioCfg.precision = audio.precision;
    
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

        nextBuffer = (char *) player.audioDrv->open (audioCfg);
    }

    // Check to make sure that hardware supports stereo
    if (audio.playback == sid2_stereo)
    {
        if (audioCfg.channels != 2)
            audio.playback = sid2_mono;
    }

    tune.load (argv[sidFile]);
    if (!tune)
    {
        cerr << argv[0] << "\n" << (tune.getInfo ()).statusString << endl;
        goto main_error;
    }        

    // Load the filter
    if (!player.filter)
        player.filter = player.ini.filter (emulation.sidModel);

    if (player.filter)
    {
        if (player.lib.loadFilter (player.filter->fc, player.filter->points) == -1)
        {
            cerr << argv[0] << "\n" << player.lib.getErrorString () << endl;
            goto main_error;
        }
    }

    // Configure Emulation
    player.lib.clockSpeed   (emulation.clockSpeed, emulation.clockForced);
    player.lib.configure    (audio.playback, audio.frequency, audio.precision, force2SID);
    player.lib.optimisation (emulation.optimiseLevel);
    player.lib.sidModel     (emulation.sidModel);
    player.lib.sidSamples   (emulation.sidSamples);
    player.lib.filter       (emulation.filter);
    player.lib.extFilter    (emulation.extFilter);
    if (player.lib.environment (playerMode) == -1)
    {
        cerr << argv[0] << "\n" << player.lib.getErrorString () << endl;
        goto main_error;
    }

    // Load song length database
    if (!timeValid)
        player.database.open (sidplay2.database);

main_restart:
    player.selectedSong = tune.selectSong (player.selectedSong);
    if (!tune)
    {
        cerr << argv[0] << "\n" << player.lib.getErrorString () << endl;
        goto main_error;
    }

    if (player.lib.loadSong (&tune) == -1)
    {
        cerr << argv[0] << "\n" << player.lib.getErrorString () << endl;
        goto main_error;
    }

    // See if we can get the songs length
    if (!timeValid)
    {
        int_least32_t ret = player.database.getSongLength (tune);
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
        nextBuffer = (char *) wavFile->open (audioCfg, wavName, true);
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

    player.lib.getInfo (&player.info);
    tuneInfo = player.info.tuneInfo;
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
    cerr << (char) toupper (*player.info.name);
    cerr << player.info.name + 1 << " V" << player.info.version << endl;

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
    cerr << " Setting Song : ";
    textColour   (white, true);
    cerr << tuneInfo.currentSong
         << " out of "         << tuneInfo.songs
         << " (default = "     << tuneInfo.startSong << ')' << endl;

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
        cerr << "Load=$"   << setw(4) << setfill('0') << tuneInfo.loadAddr;
        cerr << ", Init=$" << setw(4) << setfill('0') << tuneInfo.initAddr;
        cerr << ", Play=$" << setw(4) << setfill('0') << tuneInfo.playAddr;
        cerr << dec << endl;
        cerr.unsetf(ios::uppercase);

        displayTable (sid2_tableMiddle);
        textColour   (yellow, true);
        cerr << " SID Filters  : ";
        textColour   (white, false);
        cerr << "Internal=";
        cerr << ((player.info.filter == true) ? "Yes" : "No");
        cerr << ", External=";
        cerr << ((player.info.extFilter == true) ? "Yes" : "No") << endl;
        displayTable (sid2_tableMiddle);
        textColour   (yellow, true);
        cerr << " Environment  : ";
        textColour   (white, false);
        switch (player.info.environment)
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

    // Play loop
    player.paused = false;
    FOREVER
    {
        int_least32_t ret;
        ret = generateMusic (audioCfg, nextBuffer);
        // Check for ^C
        if (player.fastExit)
            break;

        nextBuffer = player.audioDrv->write ();
        if (ret   != 0)
        {   // -1 means end
            if (ret < 0)
                break;
            if ((uint_least32_t) ret == runtime)
                break;
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
                uint_least16_t songs = player.info.tuneInfo.songs;
                if (songs > 1)
                {   // Has multiple subtunes, so move onto next one
                    player.restart = true;
                    fastReset      = false;
                    player.selectedSong++;
                    if (player.selectedSong > songs)
                        player.selectedSong = 1;
                }
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
    cerr 
        << "Syntax: " << arg0 << " [-<option>...] <datafile>" << endl
        << "Options:" << endl
        << " --help|-h    display this screen" << endl

        << " -f<num>      set frequency in Hz (default: "
        << SID2_DEFAULT_SAMPLING_FREQ << ")" << endl
        << " -fd          force dual sid environment" << endl
        << " -fs          force samples to a channel (default: uses sid)" << endl

#if !defined(DISALLOW_STEREO_SOUND)
        << " -s[l|r]      stereo sid support or [left/right] channel only" << endl
#endif

// Old options are hidden
//        << " -m           PlaySID Compatibility Mode (read the docs!)" << endl
//        << " -mt          Sidplays Transparent Rom Mode" << endl
//        << " -mb          Sidplays Bankswitching Mode (default)" << endl
//        << " -mr          Sidplay2s Real C64 Emulation Mode" << endl

        << " -ne          no external SID filter emulation" << endl
        << " -nf          no SID filter emulation" << endl
        << " -ns          MOS 8580 waveforms (default: MOS 6581)" << endl

        << " -o<num>      select track number (default: preset)" << endl
        // Rev 1.7 (saw) - Changed max printed optimisation
        << " -O<num>      optimisation level, max is " << (uint) (SID2_MAX_OPTIMISATION - 1)
        << " (default: " << (uint) SID2_DEFAULT_OPTIMISATION << ')' << endl

        << " -p<num>      set bit precision for samples. "
        << "(default: " << (uint) SID2_DEFAULT_PRECISION << ")" << endl

        << " -q           quiet (= no time display) (EXPERIMENTAL)" << endl
        << " -t<num>      set play length in [m:]s format (0 is endless)" << endl

        << " -v           verbose output" << endl
        << " -vf          force song speed by preventing speed fixing" << endl
        << " -v<p|n>      set VIC PAL/NTSC clock speed (default: defined by song)" << endl

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
        cerr << '+' << setw(tableWidth) << setfill ('-') << "" << '+';
    break;

    case sid2_tableMiddle:
        cerr << setw(tableWidth + 1) << setfill(' ') << "" << "|\r|";
    return;

    case sid2_tableSeperator:
        cerr << '|' << setw(tableWidth) << setfill ('-') << "" << '|';
    break;

    case sid2_tableEnd:
        cerr << '+' << setw(tableWidth) << setfill ('-') << "" << '+';
    break;
    }

    // Move back to begining of row and skip first char
    cerr << "\n";
}

// Rev 1.13 (saw) - Added to allow user to change subtune
void decodeKeys ()
{
    uint_least16_t songs;
    int action;
    songs = player.info.tuneInfo.songs;

    do
    {
        action = keyboard_decode ();
        if (action == A_INVALID)
            continue;

        switch (action)
        {
        case A_RIGHT_ARROW:
            player.restart = true;
            player.selectedSong++;
            if (player.selectedSong > songs)
                player.selectedSong = 1;
        break;

        case A_LEFT_ARROW:
            player.restart = true;
            player.selectedSong--;
            if (player.selectedSong < 1)
                player.selectedSong = songs;
        break;

        case A_UP_ARROW:
        player.speed *= 2;
            if (player.speed > player.speedMax)
            player.speed = player.speedMax;
        player.lib.fastForward (100 / player.speed);
    break;

        case A_DOWN_ARROW:
        player.speed = 1;
        player.lib.fastForward (100);
    break;

        case A_HOME:
            player.restart      = true;
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
