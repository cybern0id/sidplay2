/***************************************************************************
                          sidplayer_pr.cpp  -  description
                             -------------------
    begin                : Fri Jun 9 2000
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
#include <string.h>

#define  _sidplayer_pr_cpp_
#include "sidplayer_pr.h"

#ifdef SID_HAVE_EXCEPTIONS
#   include <new>
#endif

#include "config.h"

#define SIDPLAYER_CLOCK_FREQ_NTSC 1022727.14
#define SIDPLAYER_CLOCK_FREQ_PAL   985248.4
#define SIDPLAYER_VIC_FREQ_PAL         50.0
#define SIDPLAYER_VIC_FREQ_NTSC        60.0

// These texts are used to override the sidtune settings.
static const char SIDPLAYER_TXT_PAL_VBI[]  = "50 Hz VBI (PAL FORCED)";
static const char SIDPLAYER_TXT_PAL_CIA[]  = "CIA 1 Timer A (PAL FORCED)";
static const char SIDPLAYER_TXT_NTSC_VBI[] = "60 Hz VBI (NTSC FORCED)";
static const char SIDPLAYER_TXT_NTSC_CIA[] = "CIA 1 Timer A (NTSC FORCED)";
static const char SIDPLAYER_TXT_NA[]       = "NA";

// Rev 1.6 (saw) - Added Error Strings
static const char SIDPLAYER_ERR_CONF_WHILST_ACTIVE[]    = "SIDPLAYER ERROR: Trying to configure player whilst active.";
static const char SIDPLAYER_ERR_UNSUPPORTED_FREQ[]      = "SIDPLAYER ERROR: Unsupported sampling frequency.";
static const char SIDPLAYER_ERR_UNSUPPORTED_PRECISION[] = "SIDPLAYER ERROR: Unsupported sample precision.";
static const char SIDPLAYER_ERR_MEM_ALLOC[]             = "SIDPLAYER ERROR: Memory Allocation Failure.";
static const char SIDPLAYER_ERR_UNSUPPORTED_MODE[]      = "SIDPLAYER ERROR: Unsupported Environment Mode (Coming Soon).";

// Set the ICs environment variable to point to
// this sidplayer_pr
sidplayer_pr::sidplayer_pr (void)
{   // Set the ICs to use this environment
    cpu.setEnvironment  (this);
    cia.setEnvironment  (this);
    xsid.setEnvironment (this);

    //----------------------------------------------
    // SID Initialise
    // These are optional
    // Emulation type selectable
    filter    (true);
    extFilter (true);
    // Emulation type selectable
    sidModel  (SID_MOS6581);

    //----------------------------------------------
    // Emulation type selectable
    sid2.set_chip_model(MOS6581);

    // Set default settings for system
    myTune = tune  = NULL;
    ram    = (rom  = NULL);
    _environment   = sid_envBS;
    playerState    = _stopped;
    _channels      = 1;

    // Rev 2.0.4 (saw) - Added
    configure (sid_mono, SIDPLAYER_DEFAULT_SAMPLING_FREQ,
               SIDPLAYER_DEFAULT_PRECISION, false);
    // Rev 1.7 (saw) - Fixed
    _optimiseLevel = SIDPLAYER_DEFAULT_OPTIMISATION;

    // Rev 2.0.4 (saw) - Added Force Clock Speed
    _clockSpeed    = SID_TUNE_CLOCK;

    // Temp @TODO@
    _leftVolume    = 255;
    _rightVolume   = 255;

    // Rev 1.6 (saw) - Added variable initialisation
    _errorString = SIDPLAYER_TXT_NA;
}

sidplayer_pr::~sidplayer_pr ()
{   // Remove the loaded song
    if (myTune != NULL)
        delete myTune;
}

int sidplayer_pr::configure (playback_sidt playback, udword_sidt samplingFreq, int precision, bool forceDualSids)
{
    if (playerState != _stopped)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = SIDPLAYER_ERR_CONF_WHILST_ACTIVE;
        return -1;
    }

    // Check for bas sampling frequency
    if (!samplingFreq)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = SIDPLAYER_ERR_UNSUPPORTED_FREQ;
        return -1;
    }

    // Check for legal precision
    switch (precision)
    {
    case 8:
    case 16:
    case 24:
        if (precision > SIDPLAYER_MAX_PRECISION)
        {   // Rev 1.6 (saw) - Added descriptive error
            _errorString = SIDPLAYER_ERR_UNSUPPORTED_PRECISION;
            return -1;
        }
        _precision = precision;
    break;

    default:
        // Rev 1.6 (saw) - Added descriptive error
        _errorString = SIDPLAYER_ERR_UNSUPPORTED_PRECISION;
        return -1;
    }

    _playback       = playback;
    _channels       = 1;
    if (_playback == sid_stereo)
        _channels   = 2;
    _samplingFreq   = samplingFreq;
    // Added Rev 2.0.3
    _forceDualSids  = forceDualSids;
    // Rev 2.0.4 (saw) - Added for 16 bit and new timer
    _scaleBuffer    = (precision / 8);
    _samplingPeriod = _cpuFreq / (double) samplingFreq;
    return 0;
}

// Stops the emulation routine
void sidplayer_pr::stop (void)
{
    playerState = _stopped;
}

void sidplayer_pr::pause (void)
{
    playerState = _paused;
}

// Makes the next sequence of notes available.  For sidplay compatibility
// this function should be called from trigger IRQ event
void sidplayer_pr::nextSequence ()
{   // Check to see if the play address has been provided or whether
    // we should pick it up from an IRQ vector
    uword_sidt playAddr = tuneInfo.playAddr;

    // We have to reload the new play address
    if (!playAddr)
    {
        if (isKernal)
        {   // Setup the entry point from hardware IRQ
            playAddr = ((uword_sidt) ram[0x0315] << 8) | ram[0x0314];
        }
        else
        {   // Setup the entry point from software IRQ
            playAddr = ((uword_sidt) ram[0xfffe] << 8) | ram[0xffff];
        }
    }
    else
        evalBankSelect (_initBankReg);

    // Setup the entry point and restart the cpu
    ram[0xfffc] = (rom[0xfffc] = (ubyte_sidt)  playAddr);
    ram[0xfffd] = (rom[0xfffd] = (ubyte_sidt) (playAddr >> 8));
    cpu.reset ();
}

udword_sidt sidplayer_pr::play (void *buffer, udword_sidt length)
{
    udword_sidt count = 0;
    uword_sidt  clock = 0;

    // Make sure a tune is loaded
    if (!tune)
        return 0;

    // Change size from generic units to native units
    length /= _scaleBuffer;

    // Start the player loop
    playerState = _playing;
    while (playerState == _playing)
    {   // For sidplay compatibility the cpu must be idle
        // when the play routine exists.  The cpu will stay
        // idle until an interrupt occurs
        while (!cpu.SPWrapped)
        {
            cpu.clock ();
            if (_optimiseLevel < 2)
                break;
        }

        if (!_optimiseLevel)
        {   // Sids currently have largest cpu overhead, so have been moved
            // and are now only clocked when an output it required
            // Only clock second sid if we want to hear right channel or
            // stereo.  However having this results in better playback
            if (_sidEnabled[0])
                sid.clock ();
            if (_sidEnabled[1])
                sid2.clock ();
            xsid.clock ();
        }

        cia.clock  ();
        clock++;

        // Check to see if we need a new sample from reSID
        _currentPeriod++;
        if (_currentPeriod < _samplingPeriod)
            continue;

        // Rev 2.0.3 Changed - Using new mixer routines
        (this->*output) (clock, buffer, count);

        // Check to see if the buffer is full and if so return
        // so the samples can be played
        if (count >= length)
        {
            count = length;
            goto sidplayer_pr_play_updateTimer;
        }

        _currentPeriod -= _samplingPeriod;
        clock = 0;
    }

    if (playerState == _stopped)
    {
        initialise ();
        return 0;
    }

sidplayer_pr_play_updateTimer:
    // Calculate the current air time
    playerState   = _paused;
    _sampleCount += (count / _channels);

    while (_sampleCount >= _samplingFreq)
    {   // Calculate play time
        _updateClock  = true;
		_sampleCount -= _samplingFreq;
        _seconds++;
    }
    // Change size from native units to generic units
    return count * _scaleBuffer;
}

int sidplayer_pr::loadSong (const char * const title, const uword_sidt songNumber)
{
    // My tune is a tune which belongs and
    // is fully controlled be sidplayer_pr
    // so try to remove it
    if (myTune != NULL)
        delete myTune;

    // Create new sid tune object and load song
#ifdef SID_HAVE_EXCEPTIONS
    myTune = new(nothrow) SidTune(title);
#else
    myTune = new SidTune(title);
#endif
    // Make sure the memory was allocated
    if (!myTune)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = SIDPLAYER_ERR_MEM_ALLOC;
        return -1;
    }

    // Make sure the tune loaded correctly
    if (!(*myTune))
    {   // Rev 1.6 (saw) - Allow loop through errors
        _errorString = (myTune->getInfo ()).statusString;
        return -1;
    }
    myTune->selectSong(songNumber);
    return loadSong (myTune);
};

int sidplayer_pr::loadSong (SidTune *requiredTune)
{
    tune   = requiredTune;
    tune->getInfo(tuneInfo);

    // Determine if first sid is enabled
    _sidEnabled[0] = true;
	if (_playback == sid_right)
        _sidEnabled[0] = false;

    // Disable second sid from read/writes in memory
    // accesses.  The help preventing breaking of songs
    // which deliberately use SID mirroring.
    if (tuneInfo.sidChipBase2 || _forceDualSids)
        _sidEnabled[1] = true;
    else
        _sidEnabled[1] = false;

    // Setup the audio side, depending on the audio hardware
    // and the information returned by sidtune
    switch (_precision)
    {
    case 8:
        if (!_sidEnabled[1])
        {
            if (_playback == sid_stereo)
                output = &sidplayer_pr::stereoOut8MonoIn;
            else
                output = &sidplayer_pr::monoOut8MonoIn;
        }
        else
        {
            switch (_playback)
            {
            case sid_stereo: // Stereo Hardware
                output = &sidplayer_pr::stereoOut8StereoIn;
            break;

            case sid_right: // Mono Hardware,
                output = &sidplayer_pr::monoOut8StereoRIn;
            break;

            case sid_left:
                output = &sidplayer_pr::monoOut8MonoIn;
            break;

            case sid_mono:
                output = &sidplayer_pr::monoOut8StereoIn;
            break;
            }
        }
    break;
            
    case 16:
        if (!_sidEnabled[1])
        {
            if (_playback == sid_stereo)
                output = &sidplayer_pr::stereoOut16MonoIn;
            else
                output = &sidplayer_pr::monoOut16MonoIn;
        }
        else
        {
            switch (_playback)
            {
            case sid_stereo: // Stereo Hardware
                output = &sidplayer_pr::stereoOut16StereoIn;
            break;

            case sid_right: // Mono Hardware,
                output = &sidplayer_pr::monoOut16StereoRIn;
            break;

            case sid_left:
                output = &sidplayer_pr::monoOut16MonoIn;
            break;

            case sid_mono:
                output = &sidplayer_pr::monoOut16StereoIn;
            break;
            }
        }
    }

    // Check if environment has not initialised or
    // the user has asked to a different one.
    // This call we initalise the player
    if (!ram)
        return environment (_environment);

    // Initialise the player
    return initialise ();
}

void sidplayer_pr::getInfo (playerInfo_sidt *info)
{
    info->name        = PACKAGE;
    info->version     = VERSION;
    info->filter      = _filter;
    info->extFilter   = _extFilter;
    info->tuneInfo    = tuneInfo;
    info->environment = _environment;
}

int sidplayer_pr::initialise ()
{   // Now read the sub tune into memory
    ubyte_sidt AC = tuneInfo.currentSong - 1;
    envReset ();
    if (!tune->placeSidTuneInC64mem (ram))
    {   // Rev 1.6 (saw) - Allow loop through errors
        _errorString = tuneInfo.statusString;
        return -1;
    }

    // Rev 2.0.4 (saw) - Added for new time ounter
    _currentPeriod  = 0;
    _sampleCount    = 0;
    // Clock speed changing due to loading a new song
    _samplingPeriod = _cpuFreq / (double) _samplingFreq;

    // Setup the Initial entry point
    uword_sidt initAddr = tuneInfo.initAddr;
    initBankSelect (initAddr);
    ram[0xfffc] = (rom[0xfffc] = (ubyte_sidt)  initAddr);
    ram[0xfffd] = (rom[0xfffd] = (ubyte_sidt) (initAddr >> 8));
    cpu.reset (AC, 0, 0);

    // Initialise the song
    while (!cpu.SPWrapped)
    {
        cpu.clock ();
#ifdef DEBUG
        cpu.DumpState ();
#endif // DEBUG
    }

    // Check to make sure the play address is legal
    uword_sidt playAddr = tuneInfo.playAddr;
	// Rev 1.8 (saw) - (playAddr == initAddr) is no longer
	// treated as an init loop that sets it's own irq
	// handler
    //if ((playAddr == 0xffff) || (playAddr == initAddr))
    if (playAddr == 0xffff)
        tuneInfo.playAddr = 0;
    initBankSelect (playAddr);
    _initBankReg = _bankReg;

    // Get the next sequence of notes
    nextSequence ();
    playerState  = _stopped;
    // Rev 1.11 - Added to cause timer update for 0:00
    // Performance related issue!
    _updateClock = true;
    return 0;
}

int sidplayer_pr::environment (env_sidt env)
{
    if (playerState != _stopped)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = SIDPLAYER_ERR_CONF_WHILST_ACTIVE;
        return -1;
    }

    // Not supported yet
    if (env == sid_envR)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = SIDPLAYER_ERR_UNSUPPORTED_MODE;
        return -1;
    }

    // Environment already set?
    if (_environment == env)
        if (ram) return 0;

    // Setup new player environment
    _environment = env;
    if (ram)
    {
        if (ram == rom)
           delete [] ram;
        else
        {
           delete [] rom;
           delete [] ram;
        }
    }

#ifdef SID_HAVE_EXCEPTIONS
    ram = new(nothrow) ubyte_sidt[0x10000];
#else
    ram = new ubyte_sidt[0x10000];
#endif

    // Setup the access functions to the environment
    // and the properties the memory has.
    if (_environment == sid_envPS)
    {   // Playsid has no roms and SID exists in ram space
        rom             = ram;
        readMemByte     = &sidplayer_pr::readMemByte_plain;
        writeMemByte    = &sidplayer_pr::writeMemByte_playsid;
        readMemDataByte = &sidplayer_pr::readMemByte_playsid;
    }
    else
    {
#ifdef SID_HAVE_EXCEPTIONS
        rom = new(nothrow) ubyte_sidt[0x10000];
#else
        rom = new ubyte_sidt[0x10000];
#endif

        switch (_environment)
        {
        case sid_envTP:
            readMemByte     = &sidplayer_pr::readMemByte_plain;
            writeMemByte    = &sidplayer_pr::writeMemByte_sidplay;
            readMemDataByte = &sidplayer_pr::readMemByte_sidplaytp;
        break;

        case sid_envBS:
            readMemByte     = &sidplayer_pr::readMemByte_plain;
            writeMemByte    = &sidplayer_pr::writeMemByte_sidplay;
            readMemDataByte = &sidplayer_pr::readMemByte_sidplaybs;
        break;

        case sid_envR:
        default: // <-- Just to please compiler
            readMemByte     = &sidplayer_pr::readMemByte_sidplaybs;
            writeMemByte    = &sidplayer_pr::writeMemByte_sidplay;
            readMemDataByte = &sidplayer_pr::readMemByte_sidplaybs;
        break;
        }
    }

    // Have to reload the song into memory as
    // everything has changed
    if (tune)
        return initialise ();

    return 0;
}


//-------------------------------------------------------------------------
// Temporary hack till real bank switch code added

//  Input: A 16-bit effective address
// Output: A default bank-select value for $01.
void sidplayer_pr::initBankSelect (uword_sidt addr)
{
    ubyte_sidt data;
    if (_environment == sid_envPS)
        data = 4;  // RAM only, but special I/O mode
    else
    {
        if (addr < 0xa000)
            data = 7;  // Basic-ROM, Kernal-ROM, I/O
        else if (addr  < 0xd000)
            data = 6;  // Kernal-ROM, I/O
        else if (addr >= 0xe000)
            data = 5;  // I/O only
        else
            data = 4;  // RAM only
    }

    evalBankSelect (data);
    ram[1] = data;
}

void sidplayer_pr::evalBankSelect (ubyte_sidt data)
{   // Determine new memory configuration.
    isBasic  = ((data & 3) == 3);
    isIO     = ((data & 7) >  4);
    isKernal = ((data & 2) != 0);
    _bankReg = data;
#ifdef DEBUG
    ram[1]   = data;
#endif // DEBUG
}

ubyte_sidt sidplayer_pr::readMemByte_plain (uword_sidt addr, bool useCache)
{
    return ram[addr];
}

ubyte_sidt sidplayer_pr::readMemByte_playsid (uword_sidt addr, bool useCache)
{
    uword_sidt tempAddr = (addr & 0xfc1f);

    // Not SID ?
    if (( tempAddr & 0xff00 ) != 0xd400 )
    {
        if ( (addr&0xfff0) == 0xdc00 )   // (ms) CIA 1
            return cia.read(addr&0x0f);
        else
            return rom[addr];
    }
    else
    {   // $D41D/1E/1F, $D43D/, ... SID not mirrored
        if (( tempAddr & 0x00ff ) >= 0x001d )
        {
            return xsid.read (addr);
        }
        else // (Mirrored) SID.
        {
            // Read real sid for these
            if (_sidEnabled[1])
            {   // Support dual sid
                if ((addr & 0xff00) == 0xd500)
                {
                    if (useCache)
                        return rom[addr];
                    return sid2.read ((ubyte_sidt) addr);
                }
            }
            if (useCache)
                return rom[tempAddr];
            return sid.read ((ubyte_sidt) tempAddr);
        }
    }

    // Bank Select Register Value DOES NOT
    // get to ram
    if (addr == 0x0001) return _bankReg;
    return ram[addr];
}

ubyte_sidt sidplayer_pr::readMemByte_sidplaytp(uword_sidt addr, bool useCache)
{
    if (addr < 0xD000)
    {
        return ram[addr];
    }
    else
    {
        // Get high-nibble of address.
        switch (addr >> 12)
        {
        case 0xd:
            if (isIO)
                return readMemByte_playsid (addr, useCache);
            else
                return ram[addr];
        break;
        case 0xe:
        case 0xf:
        default:  // <-- just to please the compiler
              return ram[addr];
        }
    }
}
    	
ubyte_sidt sidplayer_pr::readMemByte_sidplaybs (uword_sidt addr, bool useCache)
{
    if (addr < 0xA000)
    {
        if (addr == 0x0001) return _bankReg;
        return ram[addr];
    }
    else
    {
        // Get high-nibble of address.
        switch (addr >> 12)
        {
        case 0xa:
        case 0xb:
            if (isBasic)
                return rom[addr];
            else
                return ram[addr];
        break;
        case 0xc:
            return ram[addr];
        break;
        case 0xd:
            if (isIO)
                return readMemByte_playsid (addr, useCache);
            else
                return ram[addr];
        break;
        case 0xe:
        case 0xf:
        default:  // <-- just to please the compiler
          if (isKernal)
              return rom[addr];
          else
              return ram[addr];
        }
    }
}

void sidplayer_pr::writeMemByte_playsid (uword_sidt addr, ubyte_sidt data, bool useCache)
{
    if (addr == 0x0001)
    {   // Determine new memory configuration.
        evalBankSelect (data);
        return;
    }

    if ((addr & 0xff00) == 0xdc00)
    {
        addr &= 0x000f;
        cia.write ((ubyte_sidt) addr, data);
        return;
    }

    // Check whether real SID or mirrored SID.
    uword_sidt tempAddr = (addr & 0xfc1f);

    // Not SID ?
    if (( tempAddr & 0xff00 ) != 0xd400 )
    {
        ram[addr] = data;
        return;
    }

    // $D41D/1E/1F, $D43D/3E/3F, ...
    // Map to real address to support PlaySID
    // Extended SID Chip Registers.
    if (( tempAddr & 0x00ff ) >= 0x001d )
    {
        xsid.write (addr - 0xd400, data);
    }
    else // Mirrored SID.
    {   // SID.
        // Convert address to that acceptable by resid
        if (_sidEnabled[1])
        {   // Support dual sid
            if ((addr & 0xff00) == 0xd500)
            {
                if (useCache)
                    rom[addr] = data;
                sid2.write ((ubyte_sidt) addr, data);
                return;
            }
        }
        if (useCache)
            rom[tempAddr] = data;
        sid.write ((ubyte_sidt) tempAddr, data);
    }
}

void sidplayer_pr::writeMemByte_sidplay (uword_sidt addr, ubyte_sidt data, bool useCache)
{
    if (addr < 0xA000)
    {
        if (addr == 0x0001)
        {
            evalBankSelect (data);
            return;
        }
        ram[addr] = data;
    }
    else
    {
        // Get high-nibble of address.
        switch (addr >> 12)
        {
        case 0xa:
        case 0xb:
        case 0xc:
            ram[addr] = data;
        break;
        case 0xd:
            if (isIO)
                writeMemByte_playsid (addr, data, useCache);
            else
                ram[addr] = data;
        break;
        case 0xe:
        case 0xf:
        default:  // <-- just to please the compiler
            ram[addr] = data;
        }
    }
}

// --------------------------------------------------
// These must be available for use:
void sidplayer_pr::envReset (void)
{
    cpu.reset ();
    sid.reset ();
    sid2.reset ();
    cia.reset ();

    // Initalise Memory
    memset (ram, 0, 0x10000);
    memset (rom, 0, 0x10000);
    memset (rom + 0xE000, RTIn, 0x2000);
    if (_environment != sid_envPS)
        memset (rom + 0xA000, RTSn, 0x2000);

    ram[0] = 0x2F;
    // defaults: Basic-ROM on, Kernal-ROM on, I/O on
    evalBankSelect(0x07);
    // fake VBI-interrupts that do $D019, BMI ...
    rom[0x0d019] = 0xff;

    // Select speed description string.
    if (_clockSpeed == SID_PAL)
    {
        tuneInfo.clockSpeed      = SIDTUNE_CLOCK_PAL;
        if (tuneInfo.songSpeed  == SIDTUNE_SPEED_VBI)
            tuneInfo.speedString = SIDPLAYER_TXT_PAL_VBI;
        else // if (tuneInfo.songSpeed == SIDTUNE_SPEED_CIA_1A)
            tuneInfo.speedString = SIDPLAYER_TXT_PAL_CIA;
    }
    else if (_clockSpeed == SID_NTSC)
    {
        tuneInfo.clockSpeed      = SIDTUNE_CLOCK_NTSC;
        if (tuneInfo.songSpeed  == SIDTUNE_SPEED_VBI)
            tuneInfo.speedString = SIDPLAYER_TXT_NTSC_VBI;
        else // if (tuneInfo.songSpeed == SIDTUNE_SPEED_CIA_1A)
            tuneInfo.speedString = SIDPLAYER_TXT_NTSC_CIA;
    }

    // Set the CIA Timer
    if (tuneInfo.songSpeed == SIDTUNE_SPEED_VBI)
    {
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
        {
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_PAL;
            cia.reset ((uword_sidt) (_cpuFreq / SIDPLAYER_VIC_FREQ_PAL + 0.5));
        }
        else // SIDTUNE_CLOCK_NTSC
        {
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_NTSC;
            cia.reset ((uword_sidt) (_cpuFreq / SIDPLAYER_VIC_FREQ_NTSC + 0.5));
        }

        cia.write (0x0e, 0x01); // Start the timer
        cia.locked = true;
        ram[0x02a6] = 1; // PAL
    }
    else // SIDTUNE_SPEED_CIA_1A
    {
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_PAL;
        else // SIDTUNE_CLOCK_NTSC
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_NTSC;

        cia.reset ((uword_sidt) (_cpuFreq / SIDPLAYER_VIC_FREQ_NTSC + 0.5));
        cia.write (0x0e, 0x01); // Start the timer
        ram[0x02a6] = 0; // NTSC
    }

    // @TODO@ Enabling these causes SEG FAULT
    // software vectors
    ram[0x0314] = 0x31; // IRQ to $EA31
    ram[0x0315] = 0xea;
    ram[0x0316] = 0x66; // BRK to $FE66
    ram[0x0317] = 0xfe;
    ram[0x0318] = 0x47; // NMI to $FE47
    ram[0x0319] = 0xfe;

    // hardware vectors
    rom[0xfffa] = 0x43; // NMI to $FE43
    rom[0xfffb] = 0xfe;
    rom[0xfffc] = 0xe2; // RESET to $FCE2
    rom[0xfffd] = 0xfc;
    rom[0xfffe] = 0x48; // IRQ to $FF48
    rom[0xffff] = 0xff;
	
    if (_environment == sid_envPS)
    {
	    // (ms) $FFFE: JMP $FF48 -> $FF48: JMP ($0314)
        rom[0xff48] = 0x6c;
        rom[0xff49] = 0x14;
        rom[0xff4a] = 0x03;
    }

    // Set Master Output Volume to fix some bad songs
    sid.write  (0x18, 0x0f);
    sid2.write (0x18, 0x0f);
}

ubyte_sidt sidplayer_pr::envReadMemByte (uword_sidt addr, bool useCache)
{   // Read from plain only to prevent execution of rom code
    return (this->*(readMemByte)) (addr, useCache);
}

void sidplayer_pr::envWriteMemByte (uword_sidt addr, ubyte_sidt data, bool useCache)
{   // Writes must be passed to env version.
    (this->*(writeMemByte)) (addr, data, useCache);
}

void sidplayer_pr::envTriggerIRQ (void)
{   // Load the next note sequence
    nextSequence ();
}

void sidplayer_pr::envTriggerNMI (void)
{   // NOT DEFINED
    ;
}

void sidplayer_pr::envTriggerRST (void)
{   // NOT DEFINED
    ;
}

void sidplayer_pr::envClearIRQ (void)
{   // NOT DEFINED
    ;
}

ubyte_sidt sidplayer_pr::envReadMemDataByte (uword_sidt addr, bool useCache)
{   // Read from plain only to prevent execution of rom code
    return (this->*(readMemDataByte)) (addr, useCache);
}

bool sidplayer_pr::envCheckBankJump (uword_sidt addr)
{
    switch (_environment)
    {
    case sid_envBS:
        if (addr >= 0xA000)
        {
            // Get high-nibble of address.
            switch (addr >> 12)
            {
            case 0xa:
            case 0xb:
                if (isBasic)
                    return false;
            break;

            case 0xc:
            break;

            case 0xd:
                if (isIO)
                    return false;
            break;

            case 0xe:
            case 0xf:
            default:  // <-- just to please the compiler
               if (isKernal)
                    return false;
            break;
            }
        }
    break;

    case sid_envTP:
        if ((addr >= 0xd000) && isKernal)
            return false;
    break;

    default:
    break;
    }

    return true;
}




//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
// Redirection to private version of sidplayer (This method is called Cheshire Cat)
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
sidplayer::sidplayer ()
{
#ifdef SID_HAVE_EXCEPTIONS
    player = new(nothrow) sidplayer_pr;
#else
    player = new sidplayer_pr;
#endif
}

sidplayer::~sidplayer ()
{   if (player) delete player; }

void sidplayer::configure (playback_sidt mode, udword_sidt samplingFreq, int precision, bool forceDualSid)
{   player->configure (mode, samplingFreq, precision, forceDualSid); }

void sidplayer::stop (void)
{   player->stop (); }

void sidplayer::pause (void)
{   player->pause (); }

udword_sidt sidplayer::play (void *buffer, udword_sidt length)
{   return player->play (buffer, length); }

int sidplayer::loadSong (const char * const title, const uword_sidt songNumber)
{   return player->loadSong (title, songNumber); }

int sidplayer::loadSong (SidTune *requiredTune)
{   return player->loadSong (requiredTune); }

void sidplayer::environment (env_sidt env)
{   player->environment (env); }

void sidplayer::getInfo (playerInfo_sidt *info)
{   player->getInfo (info); }

void sidplayer::optimisation (int level)
{   player->optimisation (level); }

udword_sidt sidplayer::time (void)
{   return player->time (); }

bool sidplayer::updateClock (void)
{   return player->updateClock (); }

void sidplayer::filter (bool enabled)
{   player->filter (enabled); }

void sidplayer::extFilter (bool enabled)
{   player->extFilter (enabled); }

void sidplayer::sidModel (model_sidt model)
{   player->sidModel (model); }

void sidplayer::clockSpeed (clock_sidt clock)
{   player->clockSpeed (clock); }

// Rev 1.6 (saw) - Added to obtain error messages
// from the player
const char *sidplayer::getErrorString (void)
{   return player->_errorString; }
