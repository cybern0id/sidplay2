/***************************************************************************
                          config.h  -  Redirect to real config.h
                             -------------------
    begin                : Tues Dec 4 2001
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
 ***************************************************************************/

#if defined(HAVE_UNIX)
#   include "../unix/config.h"
#elif defined(HAVE_WINDOWS)
#   include "../win/VC/config.h"
#else
#   error Platform not supported!
#endif
