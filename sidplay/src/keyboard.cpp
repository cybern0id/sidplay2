/***************************************************************************
                          keyboard.cpp  -  Keyboard decoding
                             -------------------
    begin                : Thur Dec 7 2000
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
 *  Revision 1.1  2000/12/12 19:13:15  s_a_white
 *  New keyboard handling routines.
 *
 ***************************************************************************/

#include "keyboard.h"

#ifdef HAVE_UNIX
// Unix console headers
#   include <ctype.h>
#   include <termios.h>
#   include <sys/time.h>
#   include <sys/types.h>
#   include <unistd.h>
int _getch (void);
#endif

#define MAX_CMDLEN 10
#define ESC '\033'

// Special Extended Key Definitions
enum
{
    KEY_LEFT  = 'K',
    KEY_RIGHT = 'M',
    KEY_HOME  = 'G',
    KEY_END   = 'O'
};

#define EX_KEY '\340'

static char keytable[] =
{
    // Windows Special Cursors
#ifdef HAVE_MSWINDOWS
    EX_KEY, KEY_RIGHT,0,    A_RIGHT_ARROW,
    EX_KEY, KEY_LEFT,0,     A_LEFT_ARROW,
    EX_KEY, KEY_HOME,0,     A_HOME,
    EX_KEY, KEY_END,0,      A_END,
#endif

#ifdef HAVE_UNIX
    // Linux Special Keys
    ESC,'[','C',0,          A_RIGHT_ARROW,
    ESC,'[','D',0,          A_LEFT_ARROW,
    ESC,'[','A',0,          A_UP_ARROW,
    ESC,'[','B',0,          A_DOWN_ARROW,
    // Hmm, in consile there:
    ESC,'[','1','~',0,      A_HOME,
    ESC,'[','4','~',0,      A_END,
    // But in X there:
    ESC,'[','H',0,          A_HOME,
    ESC,'[','F',0,          A_END,

    ESC,'[','1','0',0,      A_INVALID,
    ESC,'[','2','0',0,      A_INVALID,
#endif

    // General Keys
    '6',0,                  A_RIGHT_ARROW,
    '4',0,                  A_LEFT_ARROW,
    '7',0,                  A_HOME,
    '1',0,                  A_END,
    'p',0,                  A_PAUSED,
    'P',0,                  A_PAUSED,
    ESC,ESC,0,              A_QUIT,

    // Old Keys
    '>',0,                  A_RIGHT_ARROW,
    '<',0,                  A_LEFT_ARROW,
    '.',0,                  A_RIGHT_ARROW,
    ',',0,                  A_LEFT_ARROW,

    0,                      A_END_LIST
};


/*
 * Search a single command table for the command string in cmd.
 */
static int keyboard_search (char *cmd)
{
    register char *p;
    register char *q;
    register int   a;

    for (p = keytable, q = cmd;;  p++, q++)
    {
        if (*p == *q)
        {
            /*
             * Current characters match.
             * If we're at the end of the string, we've found it.
             * Return the action code, which is the character
             * after the null at the end of the string
             * in the command table.
             */
            if (*p == '\0')
            {
                a = *++p & 0377;
                while (a == A_SKIP)
                    a = *++p & 0377;
                if (a == A_END_LIST)
                {
                    /*
                     * We get here only if the original
                     * cmd string passed in was empty ("").
                     * I don't think that can happen,
                     * but just in case ...
                     */
                    break;
                }
                return (a);
            }
        } else if (*q == '\0')
        {
            /*
             * Hit the end of the user's command,
             * but not the end of the string in the command table.
             * The user's command is incomplete.
             */
            return (A_PREFIX);
        } else
        {
            /*
             * Not a match.
             * Skip ahead to the next command in the
             * command table, and reset the pointer
             * to the beginning of the user's command.
             */
            if (*p == '\0' && p[1] == A_END_LIST)
            {
                /*
                 * A_END_LIST is a special marker that tells 
                 * us to abort the cmd search.
                 */
                break;
            }
            while (*p++ != '\0')
                continue;
            while (*p == A_SKIP)
                p++;
            q = cmd-1;
        }
    }
    /*
     * No match found in the entire command table.
     */
    return (A_INVALID);
}

int keyboard_decode ()
{
    char cmd[MAX_CMDLEN+1], c;
    int  nch = 0;
    int  action;

    /*
     * Collect characters in a buffer.
     * Start with the one we have, and get more if we need them.
     */
    c = _getch();
    if (c == '\0')
        c = '\340'; // 224
    else if (c == ESC)
    {
        cmd[nch++] = c;
        if (_kbhit ())
            c = _getch ();
    }

    for (;;)
    {
        cmd[nch++] = c;
        cmd[nch]   = '\0';
        action     = keyboard_search (cmd);

        if (action != A_PREFIX)
            break;
        if (!_kbhit ())
            break;
        c = _getch ();
    }
    return action;
}

// Simulate Standard Microsoft Extensions under Unix
#ifdef HAVE_UNIX
int _kbhit (void)
{   // Set no delay
    static struct timeval tv = {0, 0};
    fd_set rdfs;

    // See if key has been pressed
    FD_ZERO (&rdfs);
    FD_SET  (STDERR_FILENO, &rdfs);
    if (select  (STDERR_FILENO + 1, &rdfs, NULL, NULL, &tv) <= 0)
        return 0;
    if (FD_ISSET (STDERR_FILENO, &rdfs))
        return 1;
    return 0;
}

int _getch (void)
{
    char ch = 0;
    int  ret;
    ret = read (STDERR_FILENO, &ch, 1);
    if (ret <= 0)
        return -1;
    return ch;
}

// Set keyboard to raw mode to getch will work
static termios term;
void keyboard_enable_raw ()
{
    // set to non canonical mode, echo off, ignore signals
    struct termios current;
    // save current terminal settings
    tcgetattr (STDERR_FILENO, &current);

    // set to non canonical mode, echo off, ignore signals
    term = current;
    current.c_lflag &= ~(ECHO | ICANON | IEXTEN);
    current.c_cc[VMIN] = 1;
    current.c_cc[VTIME] = 0;
    tcsetattr (STDERR_FILENO, TCSAFLUSH, &current);
}

void keyboard_disable_raw ()
{
    // Restore old terminal settings
    tcsetattr (STDERR_FILENO, TCSAFLUSH, &term);
}

#endif // HAVE_LINUX
