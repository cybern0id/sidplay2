// --------------------------------------------------------------------------
// HPPA/HPUX specific audio interface.
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 ***************************************************************************/

#ifndef audio_hpux_h_
#define audio_hpux_h_

#include "config.h"
#ifdef   HAVE_HPUX
#   ifndef AudioDriver
#   define AudioDriver Audio_HPUX
#   endif
#endif

#include "../AudioBase.h"


class Audio_HPUX: public AudioBase
{
private:  // ------------------------------------------------------- private
    static const char AUDIODEVICE[];
    void   outOfOrder ();
    int    _audiofd;

public:  // --------------------------------------------------------- public
    Audio_HPUX();
    ~Audio_HPUX();

    void *open (AudioConfig &cfg);
	
    // Free and close opened audio device and reset any variables that
    // reflect the current state of the driver.
    void close();
	
    // Flush output stream.
    // Rev 1.3 (saw) - Changed, see AudioBase.h	
    void *reset ();
    void *write ();		
};

#endif // HAVE_HPUX
#endif // audio_hpux_h_
