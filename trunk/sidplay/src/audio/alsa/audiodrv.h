// --------------------------------------------------------------------------
// Advanced Linux Sound Architecture (ALSA) specific audio driver interface.
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 ***************************************************************************/

#include "config.h"
#ifdef HAVE_ALSA

#ifndef _audiodrv_h_
#define _audiodrv_h_
#define AUDIO_HAVE_DRIVER

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/asoundlib.h>

#include "../AudioBase.h"

class AudioDriver: public AudioBase
{	
private:  // ------------------------------------------------------- private
    snd_pcm_t * _audioHandle;

    void outOfOrder ();

public:  // --------------------------------------------------------- public
    AudioDriver();
    ~AudioDriver();

    void *open  (AudioConfig &cfg);
    void  close ();
    // Rev 1.2 (saw) - Changed, see AudioBase.h	
    void *reset ();
    void *write ();
};

#endif // _audiodrv_h_
#endif // HAVE_ALSA
