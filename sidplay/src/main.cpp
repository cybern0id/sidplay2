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
#include "audio/AudioDrv.h"
#include "keyboard.h"

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
    }
} player;


// Function prototypes
static void displayError  (char *arg0, uint num);
static void displaySyntax (char *arg0);
static void sighandler    (int signum);
static void cleanup       (bool fast);
static void displayTable  (sid2_table_t table);
static void decodeKeys ();

// Rev 2.0.4 (saw) - Added for better MAC support
static inline int_least32_t generateMusic (AudioConfig &cfg, void *buffer);

int main(int argc, char *argv[])
{
    sid2_playback_t playback      = sid2_mono;
    sid2_env_t      playerMode    = sid2_envBS;
    bool            wavOutput     = false;
    uint            sidFile       = 0;
    void           *nextBuffer    = NULL;
    uint_least32_t  runtime       = 0;
    bool            verboseOutput = false;
    bool            force2SID     = false;
    // Rev 1.9 (saw) - Default now obtained from sidplayer.h
    uint_least8_t   optimiseLevel = SID2_DEFAULT_OPTIMISATION;
    sid2_clock_t    clockSpeed    = SID2_CLOCK_CORRECT;
    bool            clockForced   = false;
    AudioConfig     audioCfg;
    SidTune         tune (0);
    struct          SidTuneInfo tuneInfo;

    // (ms) Incomplete...
    // Fastforward/Rewind Patch
    uint_least32_t  starttime     = 0;
  
    audioCfg.frequency = SID2_DEFAULT_SAMPLING_FREQ;;
    audioCfg.precision = SID2_DEFAULT_PRECISION;
    audioCfg.channels  = 1; // Mono

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

                    default:
                        audioCfg.frequency = atoi(argv[i] + x - 1);
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
                            player.lib.extFilter (false);
                            break;
                        }
                    break;

                    // Filter options
                    case 'f':
                        if (argv[i][x] == '\0')
                        {   // Disable filter
                            player.lib.filter (false);
                            break;
                        }
                    break;

                    // Newer sid (8580)
                    case 's':
                        if (argv[i][x] == '\0')
                        {   // Select newer sid
                            // Rev 1.18 (MiKiL) - Changed from MOS6581
                            player.lib.sidModel (SID2_MOS8580);
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

                    optimiseLevel = atoi(argv[i] + x);
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
                        audioCfg.precision = precision;

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
                        playback = sid2_stereo;
                        audioCfg.channels = 2; // Stereo
                        x--;
                    break;

                    case 'l':
                        // Left Channel
                        playback = sid2_left;
                        audioCfg.channels = 1; // Mono
                    break;

                    case 'r':
                        // Right Channel
                        playback = sid2_right;
                        audioCfg.channels = 1; // Mono
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
                        runtime   = time;
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
                        clockSpeed = SID2_CLOCK_NTSC;
                    break;

                    case 'p':
                        clockSpeed = SID2_CLOCK_PAL;
                    break;

                    case 'f':
                        clockForced = true;
                    break;

                    default:
                    break;
                    }
                break;

                case 'w':
                    wavOutput = true;
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
    if (playback == sid2_stereo)
    {
        if (audioCfg.channels != 2)
            playback = sid2_mono;
    }

    tune.load (argv[sidFile]);
    if (!tune)
    {
        cerr << argv[0] << " " << (tune.getInfo ()).statusString << endl;
        goto main_error;
    }        

    player.lib.clockSpeed   (clockSpeed, clockForced);
    player.lib.configure    (playback, audioCfg.frequency, audioCfg.precision, force2SID);
    player.lib.optimisation (optimiseLevel);
    if (player.lib.environment (playerMode) == -1)
    {
        cerr << argv[0] << " " << player.lib.getErrorString () << endl;
        goto main_error;
    }

main_restart:
    player.selectedSong = tune.selectSong (player.selectedSong);
    if (!tune)
    {
        cerr << argv[0] << " " << player.lib.getErrorString () << endl;
        goto main_error;
    }

    if (player.lib.loadSong (&tune) == -1)
    {
        cerr << argv[0] << " " << player.lib.getErrorString () << endl;
        goto main_error;
    }

    // Rev 1.12 (saw) Moved to allow modification of wav filename
    // based on subtune
    tune.getInfo (tuneInfo);

    if (wavOutput)
    {
        // Generate a name for the wav file
        WavFile *wavFile = 0;
        char    *wavName = 0;
        size_t  length, i;

        length = strlen (argv[sidFile]);
        i      = length;
        while (i > 0)
        {
            if (argv[sidFile][--i] == '.')
                break;
        }
        if (!i) i = length;
        
        // Rev 1.12 (saw - Create wav filename
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

        strcpy (wavName, argv[sidFile]);
        // Rev 1.16 (saw) - BugFix to prevent extension ".sid.wav"
        wavName[i] = '\0';

        // Rev 1.12 (saw) - Modified to change wav name based on subtune
        // Now we have a name
        if (tuneInfo.songs > 1)
            sprintf (&wavName[i], "[%u]", tuneInfo.currentSong);
        strcat (&wavName[i], ".wav");
        // lets create the wav object
#ifdef HAVE_EXCEPTIONS
        wavFile = new(nothrow) WavFile;
#else
        wavFile = new WavFile;
#endif

        if (!wavFile)
        {
            delete wavName;
            displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
            goto main_error;
        }

        player.audioDrv = wavFile;
        nextBuffer = (char *) wavFile->open (audioCfg, wavName, true);
        delete wavName;

        if (!runtime)
        {   // Can't have endless runtime for Wav Output
            // Use default of 3 mins 30 secs
            runtime = 3 * 60 + 30;
        }
    }

    if (!nextBuffer)
    {
        cerr << argv[0] << " " << player.audioDrv->getErrorString () << endl;
        goto main_error;
    }

    player.lib.getInfo (&player.info);
    // cerr << (char) 12 << '\b'; // New Page
    displayTable (sid2_tableStart);
    displayTable (sid2_tableMiddle);
    cerr << "   SIDPLAY - Music Player and C64 SID Chip Emulator" << endl;
    displayTable (sid2_tableMiddle);
    cerr << setw(19) << "Sidplay V" << VERSION << ", ";
    cerr << (char) toupper (*player.info.name);
    cerr << player.info.name + 1 << " V" << player.info.version << endl;

    displayTable (sid2_tableSeperator); 
    if (tuneInfo.numberOfInfoStrings == 3)
    {
        displayTable (sid2_tableMiddle);
        cerr << " Name         : " << tuneInfo.infoString[0] << endl;
        displayTable (sid2_tableMiddle);
        cerr << " Author       : " << tuneInfo.infoString[1] << endl;
        displayTable (sid2_tableMiddle);
        cerr << " Copyright    : " << tuneInfo.infoString[2] << endl;
    }
    else
    {
        for (int infoi = 0; infoi < tuneInfo.numberOfInfoStrings; infoi++)
        {
            displayTable (sid2_tableMiddle);
            cerr << " Description  : " << tuneInfo.infoString[infoi] << endl;
        }
    }

    displayTable (sid2_tableSeperator);
    if (verboseOutput)
    {
        displayTable (sid2_tableMiddle);
        cerr << " File format  : " << tuneInfo.formatString << endl;
        displayTable (sid2_tableMiddle);
        cerr << " Filename(s)  : " << tuneInfo.dataFileName << endl;
        // Second file is only sometimes present
        if (tuneInfo.infoFileName[0])
        {
            displayTable (sid2_tableMiddle);
            cerr << "              : " << tuneInfo.infoFileName << endl;
        }
        displayTable (sid2_tableMiddle);
        cerr << " Condition    : " << tuneInfo.statusString << endl;
    }

    displayTable (sid2_tableMiddle);
    cerr << " Setting Song : " << tuneInfo.currentSong
         << " out of "         << tuneInfo.songs
         << " (default = "     << tuneInfo.startSong << ')' << endl;

    if (verboseOutput)
    {
        displayTable (sid2_tableMiddle);
        cerr << " Song Speed   : " << tuneInfo.speedString << endl;
    }

    displayTable (sid2_tableMiddle);
    if (runtime)
        cerr << " Song Length  : " << setw(2) << setfill('0') << ((runtime / 60) % 100)
             << ':' << setw(2) << setfill('0') << (runtime % 60) << endl;
    else
        cerr << " Song Length  : UNKNOWN" << endl;

    if (verboseOutput)
    {
        displayTable (sid2_tableSeperator);
        displayTable (sid2_tableMiddle);
        cerr << " Addresses    : ";
        cerr << "Load=$" << hex << setw(4) << setfill('0')
             << tuneInfo.loadAddr;
        cerr << ", Init=$";
        cerr << hex << setw(4) << setfill('0')
             << tuneInfo.initAddr;
        cerr << ", Play=$";
        cerr << hex << setw(4) << setfill('0')
             << tuneInfo.playAddr << dec << endl;

        displayTable (sid2_tableMiddle);
        cerr << " SID Filters  : ";
        cerr << "Internal=";
        cerr << ((player.info.filter == true) ? "Yes" : "No");
        cerr << ", External=";
        cerr << ((player.info.extFilter == true) ? "Yes" : "No") << endl;
        displayTable (sid2_tableMiddle);
        switch (player.info.environment)
        {
        case sid2_envPS:
            cerr << " Environment  : PlaySID (PlaySID-specific rips)" << endl;
        break;
        case sid2_envTP:
            cerr << " Environment  : Transparent ROM" << endl;
        break;
        case sid2_envBS:
            cerr << " Environment  : Bank Switching (default)" << endl;
        break;
        case sid2_envR:  // When it happens
            cerr << " Environment  : Real C64 (default)" << endl;
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

    // Rev 1.13 (saw) - Allow restart of keys
    if (player.restart)
    {
        nextBuffer = player.audioDrv->reset();
        if (wavOutput)
            player.audioDrv->close();
        cerr << endl << endl;
        player.restart = false;
        goto main_restart;
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
        << " -vp          set VIC PAL clock speed (default: defined by song)" << endl
        << " -vn          set VIC NTSC clock speed (default: defined by song)" << endl

        << " -w           create wav file (default: <datafile>.wav)" << endl
//        << " -w<name>     explicitly defines wav output name" << endl
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
    cerr << endl;
}


void displayTable (sid2_table_t table)
{
    const uint tableWidth = 54;

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
