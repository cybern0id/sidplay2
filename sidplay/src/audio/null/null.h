/***************************************************************************
                          null.h  -  NULL audio driver used for hardsid
                                     and songlength detection
                             -------------------
    begin                : Mon Nov 6 2000
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
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 ***************************************************************************/

#ifndef audio_null_h_
#define audio_null_h_

#include "config.h"
#ifdef   HAVE_HARDSID
#   ifndef AudioDriver
#   define AudioDriver Audio_Null
#   endif
#endif

#include "../AudioBase.h"

class Audio_Null: public AudioBase
{	
private:  // ------------------------------------------------------- private
    bool isOpen;

public:  // --------------------------------------------------------- public
    Audio_Null();
    ~Audio_Null();

    void *open  (AudioConfig &cfg);
    void  close ();	
    void *reset ();
    void *write ();
};

#endif // audio_null_h_
