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
 *  Revision 1.21  2001/11/21 18:55:25  s_a_white
 *  Sidplay2 0.8 new frontend interface.
 *
 *  Revision 1.19  2001/09/01 11:48:42  s_a_white
 *  Help moved from cerr to cout.
 *
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
 *  Help screen bug fix for default precision and optimisation, which were
 *  printed as characters.
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

#include <signal.h>
#include "player.h"

#if defined(__amigaos__)
#   define EXIT_ERROR_STATUS (20)
#else
#   define EXIT_ERROR_STATUS (-1)
#endif

// Function prototypes
static void sighandler (int signum);
static Player *g_player;

int main(int argc, char *argv[])
{
    Player player(argv[0]);
    g_player = &player;

    // Decode the command line args
    if (!player.args (argc - 1, &argv[1]))
        goto main_error;

main_restart:
    if (!player.open ())
        return EXIT_ERROR_STATUS;

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
    FOREVER
    {
        if (!player.play ())
            break;
    }

#ifdef HAVE_UNIX
    keyboard_disable_raw ();
#endif

    // Restore default signal error handlers
    if ((signal (SIGINT,  SIG_DFL) == SIG_ERR)
     || (signal (SIGABRT, SIG_DFL) == SIG_ERR)
     || (signal (SIGTERM, SIG_DFL) == SIG_ERR))
    {
        displayError(argv[0], ERR_SIGHANDLER);
        goto main_error;
    }

    if (player.restart())
        goto main_restart;
    player.close ();
    return EXIT_SUCCESS;

main_error:
    player.close ();
    return EXIT_ERROR_STATUS;
}


void sighandler (int signum)
{
    switch (signum)
    {
    case SIGINT:
    case SIGABRT:
    case SIGTERM:
        // Exit now!
        g_player->stop ();
    break;
    default: break;
    }
}


void displayError (const char *arg0, uint num)
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
