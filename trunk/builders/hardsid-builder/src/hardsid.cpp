/***************************************************************************
                 hardsid.cpp - Redirects to real hardsid driver.
                               -------------------
    begin                : Mon 28 Jan 2002
    copyright            : (C) 2002 by Simon White
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
 *
 ***************************************************************************/

#if defined(HAVE_UNIX)
#   include "../unix/hardsid.cpp"
#elif defined(HAVE_MSWINDOWS)
#   include "../win/hardsid.cpp"
#else
#   error Platform not supported!
#endif
