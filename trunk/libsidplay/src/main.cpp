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
#include <fstream.h>
#include <iostream.h>
#include <iomanip.h>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Lots of sidTune stuff
// @TODO@: Tidy this code up
#include "sidplayer.h"
#include "audio/AudioDrv.h"

#ifdef HAVE_MSWINDOWS
#   include <conio.h>
#endif

// Rev 1.13 (saw) - Unix console handling headers
#ifdef HAVE_UNIX
#   include <ctype.h>
#   include <termios.h>
#   include <sys/time.h>
#   include <sys/types.h>
#   include <unistd.h>
int _kbhit (void);
int _getch (void);
#endif

#define KEY_UP    'H'
#define KEY_DOWN  'P'
#define KEY_LEFT  'K'
#define KEY_RIGHT 'M'

#ifdef SID_HAVE_EXCEPTIONS
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
    ERR_SYNTAX,
    ERR_NOT_ENOUGH_MEMORY,
    ERR_SIGHANDLER,
    ERR_FILE_OPEN
};

typedef enum {sid_tableStart, sid_tableMiddle, sid_tableSeperator, sid_tableEnd} table_sidt;

// Rev 1.13 (saw) - Grouped global variables
static class player_t
{
public:
    sidplayer       lib;
    AudioBase      *audioDrv;
    // Rev 1.11 (saw) - Bug fix for Ctrl C exiting
    volatile bool   fastExit;
    int             quietLevel;
    uword_sidt      selectedSong;
    bool            restart;
    playerInfo_sidt info;
#ifdef HAVE_UNIX
    struct termios  term;
#endif // HAVE_UNIX

public:
    player_t::player_t ()
    {
        audioDrv     = 0;
        fastExit     = false;
        // (ms) Opposite of verbose output.
        // 1 = no time display
        quietLevel   = 0;
        selectedSong = 0;
        restart      = false;    
    }
} player;


// Function prototypes
static void displayError  (char *arg0, int num);
static void displaySyntax (char *arg0);
// Rev 2.0.4 (saw) - Added for better MAC support
static inline sdword_sidt generateMusic (AudioConfig &cfg, void *buffer);
static void sighandler    (int signum);
static void cleanup       (bool fast);
static void displayTable  (table_sidt table);
static void decodeKeys ();

int main(int argc, char *argv[])
{
    playback_sidt   playback      = sid_mono;
    env_sidt        playerMode    = sid_envBS;
    bool            wavOutput     = false;
    int             sidFile       = 0;
    ubyte_sidt     *nextBuffer    = NULL;
    udword_sidt     runtime       = 0;
    bool            verboseOutput = false;
    bool            force2SID     = false;
    // Rev 1.9 (saw) - Default now obtained from sidplayer.h
    int             optimiseLevel = SIDPLAYER_DEFAULT_OPTIMISATION;
    clock_sidt      clockSpeed    = SID_TUNE_CLOCK;
    AudioConfig     audioCfg;
    struct          SidTuneInfo *tuneInfo;

    // (ms) Incomplete...
    // Fastforward/Rewind Patch
    udword_sidt   starttime     = 0;
  
    audioCfg.frequency = SIDPLAYER_DEFAULT_SAMPLING_FREQ;;
    audioCfg.precision = SIDPLAYER_DEFAULT_PRECISION;
    audioCfg.channels  = 1; // Mono

    // Rev 1.17 (saw) - Moved to fix Linux console lockup.
    // Rev 1.13 (saw) - Configure Unix terminal to allow direct access to keypresses
#ifdef HAVE_UNIX
    {   // set to non canonical mode, echo off, ignore signals
        struct termios current;
        // save current terminal settings */
        tcgetattr (STDOUT_FILENO, &current);

        // set to non canonical mode, echo off, ignore signals
        player.term = current;
        current.c_lflag &= ~(ECHO | ICANON | IEXTEN);
        current.c_cc[VMIN] = 1;
        current.c_cc[VTIME] = 0;
        tcsetattr (STDOUT_FILENO, TCSAFLUSH, &current);
    }
#endif // HAVE_UNIX

    // Install signal error handlers
    if ((signal (SIGINT,  &sighandler) == SIG_ERR)
     || (signal (SIGABRT, &sighandler) == SIG_ERR)
     || (signal (SIGTERM, &sighandler) == SIG_ERR))
    {
        displayError(argv[0], ERR_SIGHANDLER);
        goto main_error;
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
            int x = 0;
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
                        // Override sudTune and enable the second sid
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
                        playerMode = sid_envPS;
                        x--;
                    break;

                    case 't':
                        playerMode = sid_envTP;
                    break;

                    case 'b':
                        playerMode = sid_envBS;
                    break;

                    default:
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
                            player.lib.sidModel (SID_MOS8580);
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
                        int precision = atoi(argv[i] + x);
                        if (precision <= 8)
                            precision = 8;
                        else if (precision <= 16)
                            precision = 16;
                        else
                            precision = 24;

                        if (precision > SIDPLAYER_MAX_PRECISION)
                            precision = SIDPLAYER_MAX_PRECISION;
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
                        // Select Dual SIDS
                        playback = sid_stereo;
                        audioCfg.channels = 2; // Stereo
                        x--;
                    break;

                    case 'l':
                        // Left Channel
                        playback = sid_left;
                        audioCfg.channels = 1; // Mono
                    break;

                    case 'r':
                        // Right Channel
                        playback = sid_right;
                        audioCfg.channels = 1; // Mono
                    break;

                    default:
                    break;
                    }
                break;

                case 't':
                {
                    char *sep;
                    bool start = false;
                    udword_sidt time;
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
                        time = (udword_sidt) val * 60;
                        val  = atoi (sep + 1);
                        if (val < 0 || val > 59)
                            break;
                        time += (udword_sidt) val;
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

                case 'v':
                    switch (argv[i][x++])
                    {
                    case '\0':
                        // Select Dual SIDS
                        verboseOutput = true;
                        x--;
                    break;

                    case 'n':
                        // Select NTSC
                        clockSpeed = SID_NTSC;
                    break;

                    case 'p':
                        // Select NTSC
                        clockSpeed = SID_PAL;
                    break;

                    default:
                    break;
                    }
                break;

                case 'w':
                    wavOutput = true;
                break;

                default:
                    // Rev 2.0.3 (saw): Added to fix switch bug
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
#ifdef SID_HAVE_EXCEPTIONS
        player.audioDrv = new(nothrow) AudioDriver;
#else
        player.audioDrv = new AudioDriver;
#endif
        if (!player.audioDrv)
        {
            displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
            goto main_error;
        }

        nextBuffer = (ubyte_sidt *) player.audioDrv->open (audioCfg);
    }

    // Check to make sure that hardware supports stereo
    if (playback == sid_stereo)
    {
        if (audioCfg.channels != 2)
            playback = sid_mono;
    }

    player.lib.clockSpeed   (clockSpeed);
    player.lib.configure    (playback, audioCfg.frequency, audioCfg.precision, force2SID);
    player.lib.optimisation (optimiseLevel);
    player.lib.environment  (playerMode);

main_restart:
    player.restart = false;

    if (player.lib.loadSong (argv[sidFile], player.selectedSong) == -1)
    {
        // Rev 1.7 (saw) - Changed to print error message provided
        // from the player
        cerr << argv[0] << " " << player.lib.getErrorString () << endl;
        goto main_error;
    }

    // Rev 1.12 (saw) Moved to allow modification of wav filename
    // based on subtune
    player.lib.getInfo (&player.info);
    tuneInfo = &player.info.tuneInfo;
    player.selectedSong = tuneInfo->currentSong;

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
#ifdef SID_HAVE_EXCEPTIONS
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
        if (tuneInfo->songs > 1)
            sprintf (&wavName[i], "[%u]", (unsigned int) tuneInfo->currentSong);
        strcat (&wavName[i], ".wav");
        // lets create the wav object
#ifdef SID_HAVE_EXCEPTIONS
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
        nextBuffer = (ubyte_sidt *) wavFile->open (audioCfg, wavName, true);
        delete wavName;

        if (!runtime)
        {   // Can't have endless runtime for Wav Output
            // Use default of 3 mins 30 secs
            runtime = 3 * 60 + 30;
        }
    }

    if (!nextBuffer)
    {
        cout << argv[0] << " " << player.audioDrv->getErrorString () << endl;
        goto main_error;
    }

    // cout << (char) 12 << '\b'; // New Page
    displayTable (sid_tableStart);
    displayTable (sid_tableMiddle);
    cout << "  SIDPLAY - Music Player and C64 SID Chip Emulator" << endl;
    displayTable (sid_tableMiddle);
    cout << setw(24) << "sidplay V2.0.0," << " "
         << player.info.name << " V" << player.info.version << endl;

    displayTable (sid_tableSeperator); 
    if (tuneInfo->numberOfInfoStrings == 3)
    {
        displayTable (sid_tableMiddle);
        cout << "Name         : " << tuneInfo->infoString[0] << endl;
        displayTable (sid_tableMiddle);
        cout << "Author       : " << tuneInfo->infoString[1] << endl;
        displayTable (sid_tableMiddle);
        cout << "Copyright    : " << tuneInfo->infoString[2] << endl;
    }
    else
    {
        for (int infoi = 0; infoi < tuneInfo->numberOfInfoStrings; infoi++)
        {
            displayTable (sid_tableMiddle);
            cout << "Description  : " << tuneInfo->infoString[infoi] << endl;
        }
    }

    displayTable (sid_tableSeperator);
    if (verboseOutput)
    {
        displayTable (sid_tableMiddle);
        cout << "File format  : " << tuneInfo->formatString << endl;
        displayTable (sid_tableMiddle);
        cout << "Filename(s)  : " << tuneInfo->dataFileName << endl;
        // Second file is only sometimes present
        if (tuneInfo->infoFileName[0])
        {
            displayTable (sid_tableMiddle);
            cout << "             : " << tuneInfo->infoFileName << endl;
        }
        displayTable (sid_tableMiddle);
        cout << "Condition    : " << tuneInfo->statusString << endl;
    }

    displayTable (sid_tableMiddle);
    cout << "Setting Song : " << tuneInfo->currentSong
         << " out of "        << tuneInfo->songs
         << " (default = "    << tuneInfo->startSong << ')' << endl;

    if (verboseOutput)
    {
        displayTable (sid_tableMiddle);
        cout << "Song Speed   : " << tuneInfo->speedString << endl;
    }

    displayTable (sid_tableMiddle);
    if (runtime)
        cout << "Song Length  : " << setw(2) << setfill('0') << ((runtime / 60) % 100)
             << ':' << setw(2) << setfill('0') << (runtime % 60) << endl;
    else
        cout << "Song Length  : UNKNOWN" << endl;

    if (verboseOutput)
    {
        displayTable (sid_tableSeperator);
        displayTable (sid_tableMiddle);
        cout << "Addresses    : ";
        cout << "Load=$" << hex << setw(4) << setfill('0')
             << tuneInfo->loadAddr;
        cout << ", Init=$";
        cout << hex << setw(4) << setfill('0')
             << tuneInfo->initAddr;
        cout << ", Play=$";
        cout << hex << setw(4) << setfill('0')
             << tuneInfo->playAddr << dec << endl;

        displayTable (sid_tableMiddle);
        cout << "SID Filters  : ";
        cout << "Internal=";
        cout << ((player.info.filter == true) ? "Yes" : "No");
        cout << ", External=";
        cout << ((player.info.extFilter == true) ? "Yes" : "No") << endl;
        displayTable (sid_tableMiddle);
        switch (player.info.environment)
        {
        case sid_envPS:
            cout << "Environment  : PlaySID (PlaySID-specific rips)" << endl;
        break;
        case sid_envTP:
            cout << "Environment  : Transparent ROM" << endl;
        break;
        case sid_envBS:
            cout << "Environment  : Bank Switching (Default)" << endl;
        break;
        case sid_envR:  // When it happens
            cout << "Environment  : Real C64 (Default)" << endl;
        break;
        }
    }
    displayTable (sid_tableEnd);

    if (!wavOutput)
        cout << "Playing, press ^C to stop..." << flush;
    else
        cout << "Creating WAV file, please wait..." << flush;

    // Get all the text to the screen so music playback
    // is not disturbed.
    if ( player.quietLevel == 1 )
        cout << endl << flush;
    else
        cout << "00:00" << flush;

    {   // Play loop
        FOREVER
        {
            sdword_sidt ret;
            ret        = generateMusic (audioCfg, nextBuffer);
            nextBuffer = (ubyte_sidt *) player.audioDrv->write ();

            // Rev 1.14 - Changed -1 means end
            if (ret != 0)
            {
                if (ret < 0)
                    break;
                if ((udword_sidt) ret == runtime)
                    break;
            }
        }
    }

    // Rev 1.13 (saw) - Allow restart of keys
    if (player.restart)
    {
        nextBuffer = (ubyte_sidt *) player.audioDrv->reset();
        if (wavOutput)
            player.audioDrv->close();
        cout << endl << endl;
        goto main_restart;
    }

#ifndef HAVE_MSWINDOWS
    cout << endl;
#endif

    // Clean up
    cleanup (player.fastExit);
    if (wavOutput && !player.fastExit)
        cout << (char) 7; // Bell
    return EXIT_SUCCESS;

main_error:
    cleanup (true);
    return EXIT_ERROR_STATUS;
}

sdword_sidt generateMusic (AudioConfig &cfg, void *buffer)
{
	static udword_sidt currentSecs = (udword_sidt) -1;
	udword_sidt seconds;

	// Fill buffer
    // Rev 1.11 (saw) - Bug fix for Ctrl C exiting
    if (!player.lib.play (buffer, cfg.bufSize) || player.fastExit)
        return -1;

    // Check to see if the clock requires updating
	seconds = player.lib.time();
    if (currentSecs != seconds)
    {
        if (!player.quietLevel)
        {
            cout << "\b\b\b\b\b" << setw(2) << setfill('0') << ((seconds / 60) % 100)
            << ':' << setw(2) << setfill('0') << (seconds % 60) << flush;
        }

        // Check for a keypress
        if (_kbhit ())
        {
            decodeKeys ();
            if (player.restart || player.fastExit)
			{
				currentSecs = (udword_sidt) -1;
                return -1;
			}
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


void displayError (char* arg0, int num)
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

        << " -f<num>      set frequency in Hz (default: "
        << SIDPLAYER_DEFAULT_SAMPLING_FREQ << ")" << endl
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
        << " -O<num>      optimisation level, max is " << (SIDPLAYER_MAX_OPTIMISATION - 1)
        << " (default: " << SIDPLAYER_DEFAULT_OPTIMISATION << ')' << endl

        << " -p<num>      set bit precision for samples. "
        << "(default: " << SIDPLAYER_DEFAULT_PRECISION << ")" << endl

        << " -q           quiet (= no time display) (EXPERIMENTAL)" << endl
        << " -t<num>      set play length in [m:]s format (0 is endless)" << endl

        << " -v           verbose output" << endl
        << " -vp          set VIC PAL clock speed (default: defined by song)" << endl
        << " -vn          set VIC NTSC clock speed (default: defined by song)" << endl

        << " -w           create wav file (default: <datafile>.wav)" << endl
//        << " -w<name>     explicitly defines wav output name" << endl
        << endl
        // Rev 1.7 (saw) - Changed to new hompage address
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
    cout << endl;

#ifdef HAVE_UNIX
    // Rev 1.13 (saw) - Restore old terminal settings
    tcsetattr (STDOUT_FILENO, TCSAFLUSH, &player.term);
#endif
}


void displayTable (table_sidt table)
{
    const int tableWidth = 52;

    switch (table)
    {
    case sid_tableStart:
        cout << '+' << setw(tableWidth) << setfill ('-') << "" << '+';
    break;

    case sid_tableMiddle:
        cout << setw(tableWidth + 1) << setfill(' ') << "" << "|\r|";
    return;

    case sid_tableSeperator:
        cout << '|' << setw(tableWidth) << setfill ('-') << "" << '|';
    break;

    case sid_tableEnd:
        cout << '+' << setw(tableWidth) << setfill ('-') << "" << '+';
    break;
    }

    // Move back to begining of row and skip first char
    cout << "\n";
}

// Rev 1.13 (saw) - Added to allow user to change subtune
void decodeKeys ()
{
    uword_sidt songs;
    int ch;
    songs = player.info.tuneInfo.songs;

	do
	{
		ch = _getch ();
		if (!ch)
		{   // Extended Key
			ch = _getch ();
			switch (ch)
			{
			case KEY_RIGHT:
				player.restart = true;
				player.selectedSong++;
				if (player.selectedSong > songs)
					player.selectedSong = 1;
			break;

			case KEY_LEFT:
				player.restart = true;
				player.selectedSong--;
				if (player.selectedSong < 1)
					player.selectedSong = songs;
			break;
			}
		}
		else
		{   // Normal Key
			switch (tolower (ch))
			{
			case '6':
			case '>':
			case '.':
				player.restart = true;
				player.selectedSong++;
				if (player.selectedSong > songs)
					player.selectedSong = 1;
			break;
			case '4':
			case '<':
			case ',':
				player.restart = true;
				player.selectedSong--;
				if (player.selectedSong < 1)
					player.selectedSong = songs;
			break;
			case '\x1b':
				if (!_kbhit ())
				{   // If more keys then we have an escape sequence
				    player.fastExit = true;
				    return;
				}
			break;
			}
		}
	} while (_kbhit ());
}


// Rev 1.13 (saw) - Simulate Standard Microsoft Extensions under Unix
#ifdef HAVE_UNIX
int _kbhit (void)
{   // Set no delay
    static struct timeval tv = {0, 0};
    fd_set rdfs;

    // See if key has been pressed
    FD_ZERO (&rdfs);
    FD_SET  (STDIN_FILENO, &rdfs);
    if (select (STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv) > 0)
        return 1;
    return 0;
}

int _getch (void)
{
    char ch = 0;
    int  ret;
    ret = read (STDIN_FILENO, &ch, 1);
    if (ret == -1)
        return -1;
    return ch;
}
#endif // HAVE_LINUX
