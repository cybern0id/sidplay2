// --------------------------------------------------------------------------
// HPPA/HPUX specific audio interface. (very poor)
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 ***************************************************************************/

#include "audiodrv.h"
#ifdef   HAVE_HPUX

#ifdef SID_HAVE_EXCEPTIONS
#   include <new>
#endif

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>

#if defined(HAVE_SYS_AUDIO_H)
#   include <sys/audio.h>
#else
#   error Audio driver not supported.
#endif

const char Audio_HPUX::AUDIODEVICE[] = "/dev/audio";

Audio_HPUX::Audio_HPUX()
{
    outOfOrder();
}

Audio_HPUX::~Audio_HPUX()
{
    close();
}

void Audio_HPUX::outOfOrder()
{
    // Reset everything.
    _errorString = "None";
    _audiofd     = (-1);
}

void *Audio_HPUX::open (AudioConfig& cfg)
{
    // Copy input parameters. May later be replaced with driver defaults.
    _settings = cfg;

    if ((_audiofd =::open (AUDIODEVICE,O_WRONLY,0)) == (-1))
    {
        perror (AUDIODEVICE);
        _errorString = "ERROR: Could not open audio device.\n       See standard error output.";
        return 0;
    }

    // Choose the nearest possible frequency.
    int dbrifreqs[] =
    {
      5512, 6615, 8000, 9600, 11025, 16000, 18900, 22050, 27428, 32000,
      44100, 48000, 0
    };
    int dbrifsel      = 0;
    int dbrifreqdiff  = 100000;
    int dbrifrequency = _settings.frequency;
    do
    {
        int dbrifreqdiff2 = _settings.frequency  - dbrifreqs[dbrifsel];
        dbrifreqdiff2 < 0 ? dbrifreqdiff2 = 0 - dbrifreqdiff2 : dbrifreqdiff2 += 0;
        if (dbrifreqdiff2 < dbrifreqdiff)
        {
            dbrifreqdiff  = dbrifreqdiff2;
            dbrifrequency = dbrifreqs[dbrifsel];
        }
        dbrifsel++;
    }  while ( dbrifreqs[dbrifsel] != 0 );

    _settings.frequency = dbrifrequency;

    if ( ( ioctl(_audiofd, AUDIO_SET_DATA_FORMAT, AUDIO_FORMAT_LINEAR16BIT ) ) < 0)
    {
        perror (AUDIODEVICE);
        _errorString = "ERROR: Could not set sample format.\n       See standard error output.";
	goto open_error;
    }

    if (ioctl(_audiofd, AUDIO_SET_CHANNELS, _settings.channels) < 0 )
    {
        perror (AUDIODEVICE);
        _errorString = "ERROR: Could not set mono/stereo.\n       See standard error output.";
	goto open_error;
   } 

    if (ioctl(_audiofd, AUDIO_SET_SAMPLE_RATE,_settings.frequency)< 0)
    {
        perror (AUDIODEVICE);
        _errorString = "ERROR: Could not set sample rate.\n       See standard error output.";
	goto open_error;
   } 
 
    // Setup internal Config
    _settings.encoding  = AUDIO_SIGNED_PCM;
    _settings.bufSize   = _settings.frequency;
    _settings.precision = 16; // No other modes supported by the HW
    // Update the users settings
    getConfig (cfg);

    // Allocate memory same size as buffer
#ifdef SID_HAVE_EXCEPTIONS
    _sampleBuffer = new(nothrow) ubyte_sidt[_settings.bufSize];
#else
    _sampleBuffer = new ubyte_sidt[_settings.bufSize];
#endif

    _errorString = "OK";
    return (void *) _sampleBuffer;

open_error:
    ::close(_audiofd);
    _audiofd = (-1);
    return 0;
}

void *Audio_HPUX::reset()
{
    // Flush output stream.
    if (_audiofd != (-1))
    {
        return _sampleBuffer;
    }
    return NULL;
}

void Audio_HPUX::close ()
{
    if (_audiofd != (-1))
    {
        ::close (_audiofd);
        outOfOrder ();
    }
}

void *Audio_HPUX::write ()
{
    if (_audiofd != (-1))
    {
        ::write (_audiofd, (char*) _sampleBuffer, _settings.bufSize);
        return _sampleBuffer;
    }

    _errorString = "ERROR: Device not open.";
    return 0;
}

#endif // HAVE_HPUX
