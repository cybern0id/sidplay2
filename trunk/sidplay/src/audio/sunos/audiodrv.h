/***************************************************************************
                          audiodrv.h  -  SunOS sound support
                             -------------------
    begin                : Sat Jul 8 2000
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
 *  Revision 1.4  2000/12/11 19:08:32  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

// --------------------------------------------------------------------------
// SPARCstation specific audio interface.
// --------------------------------------------------------------------------

#include "config.h"
#ifdef HAVE_SUNOS

#ifndef _audiodrv_h_
#define _audiodrv_h_
#define AUDIO_HAVE_DRIVER

#include "../AudioBase.h"

class AudioDriver: public AudioBase
{
private:  // ------------------------------------------------------- private
    static const char AUDIODEVICE[];
    void   outOfOrder ();
    int    _audiofd;

public:  // --------------------------------------------------------- public
    AudioDriver();
    ~AudioDriver();

    void *open (AudioConfig &cfg);
	
    // Free and close opened audio device and reset any variables that
    // reflect the current state of the driver.
    void close();
	
    // Flush output stream.
    // Rev 1.3 (saw) - Changed, see AudioBase.h	
    void *reset ();
    void *write ();		
};

#endif // _audiodrv_h_
#endif // HAVE_SUNOS
