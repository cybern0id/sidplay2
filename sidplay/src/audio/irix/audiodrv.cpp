// --------------------------------------------------------------------------
// SGI/Irix specific audio interface. (very poor)
// --------------------------------------------------------------------------
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 ***************************************************************************/

#include "config.h"
#ifdef HAVE_IRIX


#ifdef SID_HAVE_EXCEPTIONS
#include <new>
#endif

#include <stdio.h>

#include "audiodrv.h"

AudioDriver::AudioDriver()
{
    outOfOrder();
}

AudioDriver::~AudioDriver()
{
    close();
}

void AudioDriver::outOfOrder()
{
    // Reset everything.
    _errorString = "None";
    _audio       = NULL;
}

void *AudioDriver::open (AudioConfig& cfg)
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
    // configure thinks we have exception support, but for some reason
    // the compiler thinks otherwise
#undef SID_HAVE_EXCEPTIONS
#ifdef SID_HAVE_EXCEPTIONS
    _sampleBuffer = new(nothrow) ubyte_sidt[blockSize];
#else
    _sampleBuffer = new ubyte_sidt[blockSize];
#endif

    _errorString = "OK";
    return (void *) _sampleBuffer;
}

void *AudioDriver::reset()
{
    // Flush output stream.
    if (_audio != NULL)
    {
        return _sampleBuffer;
    }
    return NULL;
}

void AudioDriver::close ()
{
    if (_audio != NULL)
    {
        ALcloseport(_audio);
        ALfreeconfig(_config);
        _audio = NULL;
	_config = NULL;
        outOfOrder ();
    }
}

void *AudioDriver::write ()
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
