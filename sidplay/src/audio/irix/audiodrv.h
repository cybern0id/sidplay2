// --------------------------------------------------------------------------
// SGI/Irix specific audio interface.
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.4  2001/10/30 23:35:35  s_a_white
 *  Added pause support.
 *
 *  Revision 1.3  2001/01/23 17:50:59  s_a_white
 *  Removed duplicate #endif.
 *
 *  Revision 1.2  2001/01/18 18:36:16  s_a_white
 *  Support for multiple drivers added.  C standard update applied (There
 *  should be no spaces before #)
 *
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 ***************************************************************************/

#ifndef audio_irix_h_
#define audio_irix_h_

#include "config.h"
#ifdef   HAVE_IRIX

#ifndef AudioDriver
#define AudioDriver Audio_Irix
#endif

#include "../AudioBase.h"

#if defined(HAVE_AUDIO_H) && defined(HAVE_DMEDIA_AUDIO_H)
#   include <audio.h>
#   include <dmedia/audio.h>
#else
#   error Audio driver not supported.
#endif


class Audio_Irix: public AudioBase
{
private:  // ------------------------------------------------------- private
    void   outOfOrder ();
    ALport _audio;
    ALconfig _config;

public:  // --------------------------------------------------------- public
    Audio_Irix();
    ~Audio_Irix();

    void *open (AudioConfig &cfg, const char *name);
	
    // Free and close opened audio device and reset any variables that
    // reflect the current state of the driver.
    void close();
	
    // Flush output stream.
    // Rev 1.3 (saw) - Changed, see AudioBase.h	
    void *reset ();
    void *write ();		
    void  pause () {;}
};

#endif // HAVE_IRIX
#endif // audio_irix_h_
