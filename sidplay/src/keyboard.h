/***************************************************************************
                          keyboard.h  -  Keyboard decoding
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

#include "config.h"

#ifdef HAVE_MSWINDOWS
#   include <conio.h>
#endif

#ifdef HAVE_UNIX
    int _kbhit (void);
#endif

enum
{   // Standard Commands
    A_PREFIX = 1,
    A_SKIP,
    A_END_LIST,
    A_INVALID,

    // Custom Commands
    A_LEFT_ARROW,
    A_RIGHT_ARROW,
    A_UP_ARROW,
    A_DOWN_ARROW,
    A_HOME,
    A_END,
    A_PAUSED,
    A_QUIT
};

int  keyboard_decode      ();
void keyboard_enable_raw  ();
void keyboard_disable_raw ();
