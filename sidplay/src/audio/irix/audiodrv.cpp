// --------------------------------------------------------------------------
// SGI/Irix specific audio interface. (very poor)
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.2  2001/01/18 18:36:16  s_a_white
 *  Support for multiple drivers added.  C standard update applied (There
 *  should be no spaces before #)
 *
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 ***************************************************************************/

#include "audiodrv.h"
#ifdef   HAVE_IRIX

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#include <stdio.h>

#include "audiodrv.h"

Audio_Irix::Audio_Irix()
{
    outOfOrder();
}

Audio_Irix::~Audio_Irix()
{
    close();
}

void Audio_Irix::outOfOrder()
{
    // Reset everything.
    _errorString = "None";
    _audio       = NULL;
}

void *Audio_Irix::open (AudioConfig& cfg)
{
    // Copy input parameters. May later be replaced with driver defaults.
    _settings = cfg;

    long chpars[] = {AL_OUTPUT_RATE, 0};

    // Frequency
    chpars[1] = _settings.frequency;
    ALsetparams(AL_DEFAULT_DEVICE, chpars, 2);
    ALgetparams(AL_DEFAULT_DEVICE, chpars, 2);
    _settings.frequency = chpars[1];
    
    _config = ALnewconfig();

    // Set sample format
    ALsetsampfmt(_config, AL_SAMPFMT_TWOSCOMP);
    // encoding = SIDEMU_SIGNED_PCM;  // unlike other systems

    // Mono output
    ALsetchannels(_config, AL_MONO);

    // 8-bit sampleSize
    ALsetwidth(_config, AL_SAMPLE_8);

    if((_audio = ALopenport("SIDPLAY2 sound", "w", _config)) == NULL)
    {
        perror("AUDIO:");
        _errorString = "ERROR: Could not open audio device.\n       See standard error output.";
        ALfreeconfig(_config);
        return 0;
    }

    // Allocate sound buffers
    int blockSize = _settings.frequency;
    ALsetqueuesize(_config,blockSize);

    // Setup internal Config
    _settings.channels  = 1;
    _settings.encoding  = AUDIO_SIGNED_PCM;
    _settings.bufSize   = blockSize;
    _settings.precision = 8;
    // Update the users settings
    getConfig (cfg);

    // Allocate memory same size as buffer
#ifdef HAVE_EXCEPTIONS
    _sampleBuffer = new(nothrow) ubyte_sidt[blockSize];
#else
    _sampleBuffer = new ubyte_sidt[blockSize];
#endif

    _errorString = "OK";
    return (void *) _sampleBuffer;
}

void *Audio_Irix::reset()
{
    // Flush output stream.
    if (_audio != NULL)
    {
        return _sampleBuffer;
    }
    return NULL;
}

void Audio_Irix::close ()
{
    if (_audio != NULL)
    {
        ALcloseport(_audio);
        ALfreeconfig(_config);
        _audio  = NULL;
        _config = NULL;
        outOfOrder ();
    }
}

void *Audio_Irix::write ()
{
    if (_audio != NULL)
    {
        ALwritesamps(_audio, (char*)_sampleBuffer, _settings.bufSize);
        return _sampleBuffer;
    }

    _errorString = "ERROR: Device not open.";
    return 0;
}

#endif // HAVE_IRIX
