/***************************************************************************
                          mos6581.h  -  Just redirects to the current SID
                                        emulation.  Currently this is reSID
                             -------------------
    begin                : Thu Jul 20 2000
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
 *  Revision 1.3  2000/12/11 19:02:10  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

#include "config.h"

// Rev 1.2 (saw) - Changed to allow resid to be in more than one location
#ifdef HAVE_LOCAL_RESID
#   include "resid/sid.h"
#else
#   ifdef HAVE_USER_RESID
#       include "sid.h"
#   else
#       include <resid/sid.h>
#   endif
#endif
