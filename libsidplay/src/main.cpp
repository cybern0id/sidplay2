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

// Global variables
static sidplayer  player;
static AudioBase *audioDrv = NULL;
int quietLevel;

// Function prototypes
static void displayError  (char *arg0, int num);
static void displaySyntax (char *arg0);
// Rev 2.0.4 (saw) - Added for better MAC support
static inline bool generateMusic (AudioConfig &cfg, void *buffer);
static void sighandler    (int signum);
static void cleanup (void);

int main(int argc, char *argv[])
{
    uword_sidt    selectedSong = 0;
    playback_sidt playback     = sid_mono;
    env_sidt      playerMode   = sid_envBS;

    bool          wavOutput     = false;
    int           sidFile       = 0;
    ubyte_sidt   *nextBuffer    = NULL;
    udword_sidt   runtime       = 0;
    bool          verboseOutput = false;
    bool          force2SID     = false;
    int           i             = 1;
    int           optimiseLevel = 0;
    clock_sidt    clockSpeed    = SID_TUNE_CLOCK;

    // (ms) Opposite of verbose output.
    // 1 = no time display
    quietLevel = 0;

	// (ms) Incomplete...
    // Fastforward/Rewind Patch
    udword_sidt      starttime     = 0;

    // Install signal error handlers
    if ((signal (SIGINT,  &sighandler) == SIG_ERR)
     || (signal (SIGABRT, &sighandler) == SIG_ERR)
     || (signal (SIGTERM, &sighandler) == SIG_ERR))
    {
		displayError(argv[0], ERR_SIGHANDLER);
        goto main_error;
    }

    // New...
    AudioConfig   audioCfg;

    audioCfg.frequency = SIDPLAYER_DEFAULT_SAMPLING_FREQ;;
    audioCfg.precision = SIDPLAYER_DEFAULT_PRECISION;
    audioCfg.channels  = 1; // Mono

    if (argc < 2) // at least one argument required
    {
        displayError (argv[0], ERR_SYNTAX);
        goto main_error;
    }

    // parse command line arguments
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
                        player.extFilter (false);
                        break;
                    }
                break;

                // Filter options
                case 'f':
                    if (argv[i][x] == '\0')
                    {   // Disable filter
                        player.filter (false);
                        break;
                    }
                break;

                // Newer sid (8580)
                case 's':
                    if (argv[i][x] == '\0')
                    {   // Select newer sid
                        player.sidModel (SID_MOS6581);
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

                selectedSong = atoi(argv[i] + x);
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
                    ++quietLevel;
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

    if (sidFile == 0)
    {   // Neither file nor stdin.
        displaySyntax(argv[0]);
        exit(0);
    }

    if (!wavOutput)
    {
        // Open Audio Driver
#ifdef SID_HAVE_EXCEPTIONS
        audioDrv = new(nothrow) AudioDriver;
#else
        audioDrv = new AudioDriver;
#endif
        if (!audioDrv)
        {
            displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
            goto main_error;
        }

        nextBuffer = (ubyte_sidt *) audioDrv->open (audioCfg);
        if (!nextBuffer)
        {
            cout << argv[0] << " " << audioDrv->getErrorString () << endl;
            goto main_error;
        }
    }

    // Check to make sure that hardware supports stereo
    if (playback == sid_stereo)
    {
        if (audioCfg.channels != 2)
            playback = sid_mono;
    }

    player.clockSpeed   (clockSpeed);
    player.configure    (playback, audioCfg.frequency, audioCfg.precision, force2SID);
    player.optimisation (optimiseLevel);
    player.environment  (playerMode);
    if (player.loadSong (argv[sidFile], selectedSong) == -1)
    {
        // Rev 1.7 (saw) - Changed to print error message provided
        // from the player
        cerr << argv[0] << " " << player.getErrorString () << endl;
        goto main_error;
    }

    if (wavOutput)
    {
        // Generate a name for the wav file
        char wavName[0x100];
        int  length;
        WavFile *wavFile;

        strcpy (wavName, argv[sidFile]);
        length = strlen (wavName);
        i      = length;
        while (i-- > 0)
        {
            if (wavName[i] == '.')
                break;
        }

        if (!i)
            i = length;

        // Now we have a name
        strcpy (&wavName[i], ".wav");
        // lets create the wav object
#ifdef SID_HAVE_EXCEPTIONS
        wavFile = new(nothrow) WavFile;
#else
        wavFile = new WavFile;
#endif

        if (!wavFile)
        {
            displayError (argv[0], ERR_NOT_ENOUGH_MEMORY);
            goto main_error;
        }

        audioDrv   = wavFile;
        nextBuffer = (ubyte_sidt *) wavFile->open (audioCfg, wavName, true);
        if (!nextBuffer)
        {
            cout << argv[0] << " " << wavFile->getErrorString () << endl;
            goto main_error;
        }

        if (!runtime)
        {   // Can't have endless runtime for Wav Output
            // Use default of 3 mins 30 secs
            runtime = 3 * 60 + 30;
        }
    }

    playerInfo_sidt playerInfo;
    player.getInfo (&playerInfo);

    cout << "SIDPLAY - Music Player and C64 SID Chip Emulator" << endl;
    cout << "--------------------------------------------------" << endl;
    cout.setf (ios::left);

//    if (verboseOutput)
//        cout << setw(12) << "sidplay" << " : V2.0.0" << endl;
	
    cout << setw(12) << playerInfo.name
         << " : V" << playerInfo.version << endl;
    cout.setf (ios::internal);

    if (verboseOutput)
    {
        cout << "File format  : " << playerInfo.tuneInfo.formatString << endl;
        cout << "Filenames    : " << playerInfo.tuneInfo.dataFileName << ", ";
        cout << playerInfo.tuneInfo.infoFileName << endl;
        cout << "Condition    : " << playerInfo.tuneInfo.statusString << endl;
	}

    cout << "Setting Song : " << playerInfo.tuneInfo.currentSong
         << " out of "        << playerInfo.tuneInfo.songs
         << " (default = "    << playerInfo.tuneInfo.startSong << ')' << endl;

    if (verboseOutput)
		cout << "Song speed   : " << playerInfo.tuneInfo.speedString << endl;

    if (runtime)
        cout << "Song Length  : " << setw(2) << setfill('0') << ((runtime / 60) % 100)
		     << ':' << setw(2) << setfill('0') << (runtime % 60) << endl;
    else
        cout << "Song Length  : UNKNOWN" << endl;
    cout << "--------------------------------------------------" << endl;

    if (playerInfo.tuneInfo.numberOfInfoStrings == 3)
    {
        cout << "Name         : " << playerInfo.tuneInfo.infoString[0] << endl;
        cout << "Author       : " << playerInfo.tuneInfo.infoString[1] << endl;
        cout << "Copyright    : " << playerInfo.tuneInfo.infoString[2] << endl;
    }
    else
    {
        for (int infoi = 0; infoi < playerInfo.tuneInfo.numberOfInfoStrings; infoi++)
            cout << "Description  : " << playerInfo.tuneInfo.infoString[infoi] << endl;
    }
    cout << "--------------------------------------------------" << endl;

    if (verboseOutput)
    {
        cout << "Load address : $" << hex << setw(4) << setfill('0')
             << playerInfo.tuneInfo.loadAddr << endl;
        cout << "Init address : $" << hex << setw(4) << setfill('0')
             << playerInfo.tuneInfo.initAddr << endl;
        cout << "Play address : $" << hex << setw(4) << setfill('0')
             << playerInfo.tuneInfo.playAddr << dec << endl;

		cout << "SID Filter   : " << ((playerInfo.filter == true) ? "Yes" : "No") << endl;
		switch (playerInfo.environment)
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
        cout << "--------------------------------------------------" << endl;
    }

    if (!wavOutput)
        cout << "Playing, press ^C to stop..." << flush;
    else
        cout << "Creating WAV file, please wait..." << flush;

    // Get all the text to the screen so music playback
    // is not disturbed.
    player.playLength (runtime);
    if ( quietLevel == 1 )
    {
        cout << endl;
    }
    else
    {
        cout << setw(2) << setfill('0') << ((runtime / 60) % 100)
             << ':' << setw(2) << setfill('0') << (runtime % 60) << flush;
    }

    // Play loop
    bool keepPlaying;
    FOREVER
    {
        keepPlaying = generateMusic (audioCfg, nextBuffer);
        nextBuffer  = (ubyte_sidt *) audioDrv->write ();

        if (!keepPlaying)
        {   // Check to see if we haven't got an
            // infinite play length
            udword_sidt secs = player.time ();
            if (runtime && !secs)
                break;
        }
    }

    // Clean up
    cleanup ();
    return EXIT_SUCCESS;

main_error:
    cleanup ();
    exit (EXIT_ERROR_STATUS);
}

bool generateMusic (AudioConfig &cfg, void *buffer)
{   // Fill buffer
    if (!player.play (buffer, cfg.bufSize))
        return false;

    // Check to see if the clock requires updating
    if ( player.updateClock ())
    {
        udword_sidt seconds = player.time();
        if ( !quietLevel )
        {
            cout << "\b\b\b\b\b" << setw(2) << setfill('0') << ((seconds / 60) % 100)
            << ':' << setw(2) << setfill('0') << (seconds % 60) << flush;
		}
        // Commented out temporarily for fastforward/rewind support.
        //if (!seconds)
            return false;
    }

    return true;
}


void sighandler (int signum)
{
    switch (signum)
    {
    case SIGINT:
    case SIGABRT:
    case SIGTERM:
        {
        if (audioDrv)
            audioDrv->reset();
        cleanup ();
        exit (EXIT_SUCCESS);
        }
    default:
        break;
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

    default:
        break;
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
//        << "Mail comments, bug reports, or contributions to <sidplay2@email.com>." << endl;
        // Rev 1.7 (saw) - Changed to new hompage address
        << "Home Page: http://sidplay2.sourceforge.net/" << endl;
}

void cleanup (void)
{
    if (audioDrv)
        delete audioDrv;
    cout << endl;
}
