// --------------------------------------------------------------------------
// SGI/Irix specific audio interface.
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 ***************************************************************************/

#include "config.h"
#ifdef HAVE_IRIX

#ifndef _audiodrv_h_
#define _audiodrv_h_
#define AUDIO_HAVE_DRIVER

#include "../AudioBase.h"

#if defined(HAVE_AUDIO_H) && defined(HAVE_DMEDIA_AUDIO_H)
  #include <audio.h>
  #include <dmedia/audio.h>
#else
  #error Audio driver not supported.
#endif

class AudioDriver: public AudioBase
{
private:  // ------------------------------------------------------- private
    void   outOfOrder ();
    ALport _audio;
    ALconfig _config;

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
#endif // HAVE_IRIX
