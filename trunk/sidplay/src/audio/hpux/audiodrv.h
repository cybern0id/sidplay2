// --------------------------------------------------------------------------
// HPPA/HPUX specific audio interface.
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 ***************************************************************************/

#include "config.h"
#ifdef HAVE_HPUX

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
#endif // HAVE_HPUX


