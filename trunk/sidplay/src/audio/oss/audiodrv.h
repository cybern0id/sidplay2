/***************************************************************************
                          audiodrv.h  -  OSS sound support
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
 *  Revision 1.3  2000/12/11 19:09:12  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

// --------------------------------------------------------------------------
// ``Open Sound System (OSS)'' specific audio driver interface.
// --------------------------------------------------------------------------

#include "config.h"
#ifdef HAVE_OSS

#ifndef _audiodrv_h_
#define _audiodrv_h_
#define AUDIO_HAVE_DRIVER

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#if defined(HAVE_LINUX_SOUNDCARD_H)
  #include <linux/soundcard.h>
#elif defined(HAVE_MACHINE_SOUNDCARD_H)
  #include <machine/soundcard.h>
#elif defined(HAVE_SOUNDCARD_H)
  #include <soundcard.h>
#else
  #error Audio driver not supported.
#endif

#include "../AudioBase.h"

class AudioDriver: public AudioBase
{	
private:  // ------------------------------------------------------- private
    static   const char AUDIODEVICE[];
    volatile int   _audiofd;

    bool _swapEndian;
    void outOfOrder ();

public:  // --------------------------------------------------------- public
    AudioDriver();
    ~AudioDriver();

    void *open  (AudioConfig &cfg);
    void  close ();
    // Rev 1.2 (saw) - Changed, see AudioBase.h	
    void *reset ()
    {
        if (_audiofd != (-1))
        {
            if (ioctl (_audiofd, SNDCTL_DSP_RESET, 0) != (-1))
                return _sampleBuffer;
        }
        return NULL;
    }
    void *write ();
};

#endif // _audiodrv_h_
#endif // HAVE_OSS
