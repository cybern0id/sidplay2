/***************************************************************************
                          player.cpp  -  Main Library Code
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.12  2001/02/21 21:43:10  s_a_white
 *  Now use VSID code and this handles interrupts much better!  The whole
 *  initialise sequence has been modified to support this.
 *
 *  Revision 1.11  2001/02/13 21:01:14  s_a_white
 *  Support for real interrupts.  C64 Initialisation routine now run from player::play
 *  instead of player::initialise.  Prevents lockups if init routine does not return.
 *
 *  Revision 1.10  2001/02/08 17:21:14  s_a_white
 *  Initial SID volumes not being stored in cache.  Fixes Dulcedo Cogitationis.
 *
 *  Revision 1.9  2001/02/07 20:56:46  s_a_white
 *  Samples now delayed until end of simulated frame.
 *
 *  Revision 1.8  2001/01/23 21:26:28  s_a_white
 *  Only way to load a tune now is by passing in a sidtune object.  This is
 *  required for songlength database support.
 *
 *  Revision 1.7  2001/01/07 15:13:39  s_a_white
 *  Hardsid update to mute sids when program exits.
 *
 *  Revision 1.6  2000/12/21 22:48:27  s_a_white
 *  Re-order voices for mono to stereo conversion to match sidplay1.
 *
 *  Revision 1.5  2000/12/14 23:53:36  s_a_white
 *  Small optimisation update, and comment revision.
 *
 *  Revision 1.4  2000/12/13 17:56:24  s_a_white
 *  Interrupt vector address changed from 0x315 to 0x314.
 *
 *  Revision 1.3  2000/12/13 12:00:25  mschwendt
 *  Corrected order of members in member initializer-list.
 *
 *  Revision 1.2  2000/12/12 22:50:15  s_a_white
 *  Bug Fix #122033.
 *
 ***************************************************************************/

#include <string.h>
#include "config.h"
#include "sidendian.h"
#include "player.h"

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

const double player::CLOCK_FREQ_NTSC = 1022727.14;
const double player::CLOCK_FREQ_PAL  = 985248.4;
const double player::VIC_FREQ_PAL    = 50.0;
const double player::VIC_FREQ_NTSC   = 60.0;

// These texts are used to override the sidtune settings.
const char  *player::TXT_PAL_VBI  = "50 Hz VBI (PAL)";
const char  *player::TXT_PAL_CIA  = "CIA 1 Timer A (PAL)";
const char  *player::TXT_NTSC_VBI = "60 Hz VBI (NTSC)";
const char  *player::TXT_NTSC_CIA = "CIA 1 Timer A (NTSC)";
const char  *player::TXT_NA       = "NA";

// Error Strings
const char  *player::ERR_CONF_WHILST_ACTIVE    = "SIDPLAYER ERROR: Trying to configure player whilst active.";
const char  *player::ERR_UNSUPPORTED_FREQ      = "SIDPLAYER ERROR: Unsupported sampling frequency.";
const char  *player::ERR_UNSUPPORTED_PRECISION = "SIDPLAYER ERROR: Unsupported sample precision.";
const char  *player::ERR_MEM_ALLOC             = "SIDPLAYER ERROR: Memory Allocation Failure.";
const char  *player::ERR_UNSUPPORTED_MODE      = "SIDPLAYER ERROR: Unsupported Environment Mode (Coming Soon).";

// Set the ICs environment variable to point to
// this player
player::player (void)
// Set default settings for system
:_tune (NULL),
 ram    (NULL), rom  (NULL),
 _clockSpeed        (SID2_CLOCK_CORRECT),
 _environment       (sid2_envBS),
 _errorString       (TXT_NA),
 _fastForwardFactor (1.0),
 _forced            (true),
 _optimiseLevel     (SID2_DEFAULT_OPTIMISATION),
 _sampleCount       (0),
 _samplingFreq      (SID2_DEFAULT_SAMPLING_FREQ),
 _mileage           (0),
 _seconds           (0),
 _userLeftVolume    (255),
 _userRightVolume   (255),
 playerState        (_stopped)
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
    sidModel  (SID2_MOS6581);
    //----------------------------------------------

    // Rev 2.0.4 (saw) - Added
    configure (sid2_mono, SID2_DEFAULT_SAMPLING_FREQ,
               SID2_DEFAULT_PRECISION, false);
}

player::~player ()
{   // Mute sids (Important for Hardsid)
    sid.reset  ();
    sid2.reset ();
}

int player::clockSpeed (sid2_clock_t clock, bool forced)
{
    if (playerState == _playing)
        return -1;

    _clockSpeed  = clock;
    _forced      = forced;
    // Other paremters set later
    if (!_tune)
        return 0;

    // Refresh Information
    _tune->getInfo (tuneInfo);

    // Detect the Correct Song Speed
    if (clock == SID2_CLOCK_CORRECT)
    {
        clock = SID2_CLOCK_PAL;
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
        clock = SID2_CLOCK_NTSC;
    }
    // If forced change song to be the requested speed
    else if (_forced)
    {
        tuneInfo.clockSpeed = SIDTUNE_CLOCK_PAL;
        if (clock == SID2_CLOCK_NTSC)
        tuneInfo.clockSpeed = SIDTUNE_CLOCK_NTSC;
    }

    // Set clock and select speed description string.
    if (clock == SID2_CLOCK_PAL)
    {
        _cpuFreq = CLOCK_FREQ_PAL;
        tuneInfo.speedString     = TXT_PAL_VBI;
        if (tuneInfo.songSpeed  == SIDTUNE_SPEED_CIA_1A)
            tuneInfo.speedString = TXT_PAL_CIA;
    }
    else if (clock == SID2_CLOCK_NTSC)
    {
        _cpuFreq = CLOCK_FREQ_NTSC;
        tuneInfo.speedString     = TXT_NTSC_VBI;
        if (tuneInfo.songSpeed  == SIDTUNE_SPEED_CIA_1A)
            tuneInfo.speedString = TXT_NTSC_CIA;
    }

    // Set the VBI Timer
    if (tuneInfo.songSpeed == SIDTUNE_SPEED_VBI)
    {
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
            cia.reset ((uint_least16_t) (_cpuFreq / VIC_FREQ_PAL + 0.5));
        else // SIDTUNE_CLOCK_NTSC
            cia.reset ((uint_least16_t) (_cpuFreq / VIC_FREQ_NTSC + 0.5));
        cia.write (0x0e, 0x01); // Start the timer
        cia.locked = true;
    }
    else // SIDTUNE_SPEED_CIA_1A
    {   // Don't reset the CIA if already running.  If we do,
        // will mess up the song.
        if (playerState == _stopped)
            cia.reset ();
    }

    // Clock speed changes due to loading a new song
    _currentPeriod  = 0;
    _samplingPeriod = _cpuFreq / (float64_t) _samplingFreq;
    _sampleCount    = 0;

    return 0;
}

int player::configure (sid2_playback_t playback, uint_least32_t samplingFreq, int precision, bool forceDualSids)
{
    if (playerState == _playing)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = ERR_CONF_WHILST_ACTIVE;
        return -1;
    }

    // Check for base sampling frequency
    if ((samplingFreq < 4000) || (samplingFreq > 96000))
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = ERR_UNSUPPORTED_FREQ;
        return -1;
    }

    // Check for legal precision
    switch (precision)
    {
    case 8:
    case 16:
    case 24:
        if (precision > SID2_MAX_PRECISION)
        {   // Rev 1.6 (saw) - Added descriptive error
            _errorString = ERR_UNSUPPORTED_PRECISION;
            return -1;
        }
    break;

    default:
        // Rev 1.6 (saw) - Added descriptive error
        _errorString = ERR_UNSUPPORTED_PRECISION;
        return -1;
    }

    // Fix the mileage counter if just finished another song.
    mileageCorrect ();

    // Do the actual configuration
    _playback       = playback;
    _channels       = 1;
    if (_playback == sid2_stereo)
        _channels++;
    _samplingFreq   = samplingFreq;
    _forceDualSids  = forceDualSids;
    _precision      = precision;
    _scaleBuffer    = (_precision / 8);
    _samplingPeriod = _cpuFreq / (float64_t) samplingFreq;
    _leftVolume     = _userLeftVolume;
    _rightVolume    = _userRightVolume;

    _sidAddress[0]  = 0xd400;
    _sidAddress[1]  = tuneInfo.sidChipBase2;

    // Only force dual sids if second wasn't detected
    if (!_sidAddress[1] && _forceDualSids)
        _sidAddress[1] = 0xd500; // Assumed

    if (_playback != sid2_mono)
    {   // Try Spliting channels across 2 sids
        if (!_sidAddress[1])
        {
            _sidAddress[1] = _sidAddress[0];

            // Mute Voices
            sid.mute  (1, true);
            sid2.mute (0, true);
            sid2.mute (2, true);
#ifndef XSID_USE_SID_VOLUME
            // 2 Voices scaled to unity from 4
            //    _leftVolume  *= 2;
            //    _rightVolume *= 2;
#else
            // 2 Voices scaled to unity from 3
            //        _leftVolume  *= 3;
            //        _leftVolume  /= 2;
            //    _rightVolume *= 3;
            //    _rightVolume /= 2;
#endif // XSID_USE_SID_VOLUME
        }

        if (_playback == sid2_left)
            xsid.mute (true);
    }

    // Setup the audio side, depending on the audio hardware
    // and the information returned by sidtune
    switch (_precision)
    {
    case 8:
        if (!_sidAddress[1])
        {
            if (_playback == sid2_stereo)
                output = &player::stereoOut8MonoIn;
            else
                output = &player::monoOut8MonoIn;
        }
        else
        {
            switch (_playback)
            {
            case sid2_stereo: // Stereo Hardware
                output = &player::stereoOut8StereoIn;
            break;

            case sid2_right: // Mono Hardware,
                output = &player::monoOut8StereoRIn;
            break;

            case sid2_left:
                output = &player::monoOut8MonoIn;
            break;

            case sid2_mono:
                output = &player::monoOut8StereoIn;
            break;
            }
        }
    break;
            
    case 16:
        if (!_sidAddress[1])
        {
            if (_playback == sid2_stereo)
                output = &player::stereoOut16MonoIn;
            else
                output = &player::monoOut16MonoIn;
        }
        else
        {
            switch (_playback)
            {
            case sid2_stereo: // Stereo Hardware
                output = &player::stereoOut16StereoIn;
            break;

            case sid2_right: // Mono Hardware,
                output = &player::monoOut16StereoRIn;
            break;

            case sid2_left:
                output = &player::monoOut16MonoIn;
            break;

            case sid2_mono:
                output = &player::monoOut16StereoIn;
            break;
            }
        }
    }

    // Not really necessary, but improve performance
    _sidEnabled[0] = true;
    _sidEnabled[1] = (_sidAddress[1] != 0);
    if (_playback == sid2_right)
        _sidEnabled[0] = false;
    else if (_playback == sid2_left)
        _sidEnabled[1] = false;

    return 0;
}

int player::environment (sid2_env_t env)
{
    if (playerState != _stopped)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = ERR_CONF_WHILST_ACTIVE;
        return -1;
    }

    // Not supported yet
//    if (env == sid2_envR)
//    {   // Rev 1.6 (saw) - Added descriptive error
//        _errorString = ERR_UNSUPPORTED_MODE;
//        return -1;
//    }

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

#ifdef HAVE_EXCEPTIONS
    ram = new(nothrow) uint8_t[0x10000];
#else
    ram = new uint8_t[0x10000];
#endif

    // Setup the access functions to the environment
    // and the properties the memory has.
    if (_environment == sid2_envPS)
    {   // Playsid has no roms and SID exists in ram space
        rom             = ram;
        readMemByte     = &player::readMemByte_player;
        writeMemByte    = &player::writeMemByte_playsid;
        readMemDataByte = &player::readMemByte_playsid;
    }
    else
    {
#ifdef HAVE_EXCEPTIONS
        rom = new(nothrow) uint8_t[0x10000];
#else
        rom = new uint8_t[0x10000];
#endif

        switch (_environment)
        {
        case sid2_envTP:
            readMemByte     = &player::readMemByte_player;
            writeMemByte    = &player::writeMemByte_sidplay;
            readMemDataByte = &player::readMemByte_sidplaytp;
        break;

        case sid2_envBS:
            readMemByte     = &player::readMemByte_player;
            writeMemByte    = &player::writeMemByte_sidplay;
            readMemDataByte = &player::readMemByte_sidplaybs;
        break;

        case sid2_envR:
        default: // <-- Just to please compiler
            readMemByte     = &player::readMemByte_player;
            writeMemByte    = &player::writeMemByte_sidplay;
            readMemDataByte = &player::readMemByte_sidplaybs;
        break;
        }
    }

    // Have to reload the song into memory as
    // everything has changed
    if (_tune)
        return initialise ();

    return 0;
}

int player::fastForward (uint_least8_t percent)
{
    if (playerState == _playing)
    {   // Rev 1.6 (saw) - Added descriptive error
        _errorString = ERR_CONF_WHILST_ACTIVE;
        return -1;
    }

    if ((percent < 1) || (percent > 100))
    {
        _errorString = "SIDPLAYER ERROR: Percentage value out of range";
        return -1;
    }
    _fastForwardFactor = 100.0 / (float64_t) percent;
    return 0;
}

void player::getInfo (sid2_playerInfo_t *info)
{
    info->name        = PACKAGE;
    info->version     = VERSION;
    info->filter      = _filter;
    info->extFilter   = _extFilter;
    info->tuneInfo    = tuneInfo;
    info->environment = _environment;
}

int player::initialise ()
{
    playerState = _stopped;
    // Fix the mileage counter if just finished another song.
    mileageCorrect ();
    _mileage += _seconds;
    _seconds  = 0;

    envReset ();
    if (!_tune->placeSidTuneInC64mem (ram))
    {   // Rev 1.6 (saw) - Allow loop through errors
        _errorString = tuneInfo.statusString;
        return -1;
    }

    // Code must not be bigger than zero page (256 bytes)
    uint8_t psid_driver[] = {
#       include "player.bin"
    };

    /* Install PSID driver code. */
    memcpy (psidDrv, psid_driver, sizeof(psid_driver));

    /* Install interrupt vectors in both ROM and RAM. */
    memcpy (&ram[0xfffa], psid_driver, 6);
    memcpy (&rom[0xfffa], psid_driver, 6);
    memcpy (&ram[0x0312], &psid_driver[0x0d], 8);
    ram[0x0311] = JMPw;

    // Setup the Initial entry point
    uint_least16_t playAddr = tuneInfo.playAddr;

    // Check to make sure the play address is legal
    // (playAddr == initAddr) is no longer treated as an
    // init loop that sets it's own irq handler
    //if ((playAddr == 0xffff) || (playAddr == initAddr))
    if (playAddr == 0xffff)
	{   // Indicates sid requiring real C64 environment
        // so speed flag.
		cia.locked = false;
		xsid.mute (true);
        playAddr   = 0;
    }

    // Tell C64 about song
    endian_little16 (&psidDrv[0x06], tuneInfo.initAddr);
    endian_little16 (&psidDrv[0x08], playAddr);
    psidDrv[0x0a] = (uint8_t) tuneInfo.currentSong;
    if (tuneInfo.songSpeed == SIDTUNE_SPEED_VBI)
        psidDrv[0x0b] = 0;
    else // SIDTUNE_SPEED_CIA_1A
        psidDrv[0x0b] = 1;

    // Will get done later if can't now
    if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
        ram[0x02a6] = 0;
    else // SIDTUNE_CLOCK_NTSC
        ram[0x02a6] = 1;

    cpu.reset ();
    return 0;
}

int player::loadSong (SidTune *tune)
{
    _tune = tune;
    _tune->getInfo(tuneInfo);

    // Un-mute all voices
    xsid.mute (false);
    reg8 i = 3;
    while (i--)
    {
        sid.mute  (i, false);
        sid2.mute (i, false);
    }

    // Must re-configure on fly for stereo support!
    (void) configure (_playback, _samplingFreq, _precision, _forceDualSids);

    // Check if environment has not initialised or
    // the user has asked to a different one.
    // This call we initalise the player
    if (!ram)
        return environment (_environment);

    // Initialise the player
    return initialise ();
}

// Makes the next sequence of notes available.  For sidplay compatibility
// this function should be called from trigger IRQ event
void player::nextSequence ()
{
    if (cpu) // Real IRQ
        cpu.triggerIRQ ();
    else // CPU is sleeping, so restart it.
    {   // Setup the entry point from hardware IRQ
        if (isKernal)
			memcpy (&rom[0xfffc], &rom[0xfffe], 2);
		else
			memcpy (&ram[0xfffc], &ram[0xfffe], 2);
        cpu.reset ();
    }

    // Paged CPU mode moved to interrupt
    xsid.suppress (false);
    xsid.suppress (true);
/*
    if (_optimiseLevel > 1)
    {
        while (cpu)
            cpu.clock ();
        xsid.suppress (false);
    }
*/
}

void player::pause (void)
{
    if (playerState != _stopped)
        playerState  = _paused;
}

uint_least32_t player::play (void *buffer, uint_least32_t length)
{
    uint_least32_t count = 0;
    uint_least16_t clock = 0;
    //    uint_least8_t  factor = 0;

    // Make sure a _tune is loaded
    if (!_tune)
        return 0;

    // Change size from generic units to native units
    length /= _scaleBuffer;

    // Start the player loop
    playerState = _playing;

    while (playerState == _playing)
    {   // For sidplay compatibility the cpu must be idle
        // when the play routine exists.  The cpu will stay
        // idle until an interrupt occurs
        if (cpu)
            cpu.clock ();
//            if (!cpu)
//                xsid.suppress (false);
//        }

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
      //        factor++;
      //            _optimiseLevel = 0;
      //            if (factor == 2)
      //        {
                count = length;
                goto player_play_updateTimer;
        //            }
        //            count = 0;
        }

        _currentPeriod -= _samplingPeriod;
        clock = 0;
    }

    if (playerState == _stopped)
    {
        initialise ();
        return 0;
    }

player_play_updateTimer:
    // Calculate the current air time
    playerState   = _paused;
    _sampleCount += (count / _channels);
    // Calculate play time
    _seconds     += (_sampleCount / _samplingFreq);
    _sampleCount %= _samplingFreq;
    // Change size from native units to generic units
    return count * _scaleBuffer;
}

void player::stop (void)
{
    playerState = _stopped;
}



//-------------------------------------------------------------------------
// Temporary hack till real bank switching code added

/*
//  Input: A 16-bit effective address
// Output: A default bank-select value for $01.
void player::initBankSelect (uint_least16_t addr)
{
    uint8_t data;
    if (_environment == sid2_envPS)
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
}
*/

void player::evalBankSelect (uint8_t data)
{   // Determine new memory configuration.
    isBasic  = ((data & 3) == 3);
    isIO     = ((data & 7) >  4);
    isKernal = ((data & 2) != 0);
    _bankReg = data;
}

uint8_t player::readMemByte_player (uint_least16_t addr, bool useCache)
{
    if (useCache)
    {   // Support bad samples in Sidplay Modes
        return ram[addr];
    }

    usePsidDrv = false;
    if (addr < 0x0100)
        usePsidDrv = true;

    if (_environment == sid2_envR)
        return readMemByte_sidplaybs (addr, useCache);

    return readMemByte_plain (addr, useCache);
}

uint8_t player::readMemByte_plain (uint_least16_t addr, bool useCache)
{   // Access the Protected PSID Driver
    if (usePsidDrv && (addr < 0x0100))
        return psidDrv[addr];
    // Bank Select Register Value DOES NOT get to ram
    if (addr == 0x0001)
        return _bankReg;
    return ram[addr];
}

uint8_t player::readMemByte_playsid (uint_least16_t addr, bool useCache)
{
    uint_least16_t tempAddr = (addr & 0xfc1f);

    // Not SID ?
    if (( tempAddr & 0xff00 ) != 0xd400 )
    {
        if ( (addr&0xff00) == 0 )
            return readMemByte_plain (addr, false);
        if ( (addr&0xfff0) == 0xdc00 )   // (ms) CIA 1
            return cia.read(addr&0x0f);
        return rom[addr];
    }

    // $D41D/1E/1F, $D43D/, ... SID not mirrored
    if (( tempAddr & 0x00ff ) >= 0x001d )
        return xsid.read (addr);
    else // (Mirrored) SID.
    {
        // Read real sid for these
        if ((addr & 0xff00) == _sidAddress[1])
        {
            if (useCache)
                return rom[addr];
            return sid2.read ((uint8_t) addr);
        }
        if (useCache)
            return rom[tempAddr];
        return sid.read ((uint8_t) tempAddr);
    }
    return 0;
}

uint8_t player::readMemByte_sidplaytp(uint_least16_t addr, bool useCache)
{
    if (addr < 0xD000)
        return readMemByte_plain (addr, false);
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
        
uint8_t player::readMemByte_sidplaybs (uint_least16_t addr, bool useCache)
{
    if (addr < 0xA000)
        return readMemByte_plain (addr, false);
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

void player::writeMemByte_plain (uint_least16_t addr, uint8_t data, bool useCache)
{   // Access the Protected PSID Driver
    if (usePsidDrv && (addr < 0x0100))
    {   // Writes to PSID drivers memory must effect banks!
        psidDrv[addr] = data;
        if (addr == 0x0001)
            goto player_writeMemByte_plain_bankSelect;
        return;
    }

    if (addr == 0x0001)
    {   // Determine new memory configuration.
player_writeMemByte_plain_bankSelect:
        evalBankSelect (data);
        return;
    }

    ram[addr] = data;
}

void player::writeMemByte_playsid (uint_least16_t addr, uint8_t data, bool useCache)
{
    uint_least16_t tempAddr = (addr & 0xfc1f);

    // Not SID ?
    if (( tempAddr & 0xff00 ) != 0xd400 )
    {
        if ( (addr&0xff00) == 0 )
        {
            writeMemByte_plain (addr, data, false);
            return;
        }

        if ( (addr&0xfff0) == 0xdc00 )   // (ms) CIA 1
        {
            cia.write(addr&0x0f, data);
            return;
        }

        rom[addr] = data;
        return;
    }

    // $D41D/1E/1F, $D43D/3E/3F, ...
    // Map to real address to support PlaySID
    // Extended SID Chip Registers.
    if (( tempAddr & 0x00ff ) >= 0x001d )
        xsid.write (addr - 0xd400, data);
    else // Mirrored SID.
    {   // SID.
        // Convert address to that acceptable by resid
        // Support dual sid
        if ((addr & 0xff00) == _sidAddress[1])
        {
            sid2.write ((uint8_t) addr, data);
            // Prevent samples getting to both sids
            // if they are exist at same address for
            // mono to stereo sid conversion.
            if (!useCache)
                return;
            else
                rom[addr] = data;

            // Prevent other sid register write accessing
            // the other sid if not doing mono to stereo
            // conversion.
            if (_sidAddress[1] != _sidAddress[0])
                return;
        }

        if (useCache)
        {
            rom[tempAddr] = data;
#ifdef XSID_USE_SID_VOLUME
            if ((uint8_t) tempAddr == 0x18)
            {   // Check if xsid wants to trap the change
                if (xsid.volumeUpdated (data))
                    return;
            }
#endif // XSID_USE_SID_VOLUME
        }
        sid.write ((uint8_t) tempAddr, data);
    }
}

void player::writeMemByte_sidplay (uint_least16_t addr, uint8_t data, bool useCache)
{
    if (addr < 0xA000)
        writeMemByte_plain (addr, data, false);
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
void player::envReset (void)
{
    cpu.reset ();
    sid.reset ();
    sid2.reset ();
    cia.reset ();

    // Initalise Memory
    memset (ram, 0, 0x10000);
    memset (rom, 0, 0x10000);
    memset (rom + 0xE000, RTSn, 0x2000);
    if (_environment != sid2_envPS)
        memset (rom + 0xA000, RTSn, 0x2000);

    ram[0] = 0x2F;
    // defaults: Basic-ROM on, Kernal-ROM on, I/O on
    evalBankSelect(0x07);
    // fake VBI-interrupts that do $D019, BMI ...
    rom[0x0d019] = 0xff;

    // Select speed description string.
    (void) clockSpeed (_clockSpeed, _forced);

    // @TODO@ Enabling these causes SEG FAULT
    // software vectors
    endian_little16 (&ram[0x0314], 0xEA31); // IRQ
    endian_little16 (&ram[0x0316], 0xFE66); // BRK
    endian_little16 (&ram[0x0318], 0xFE47); // NMI


    // hardware vectors
    endian_little16 (&rom[0xfffa],  0xFE43); // NMI
    endian_little16 (&rom[0xfffc],  0xFCE2); // RESET
    endian_little16 (&rom[0xfffe],  0xFE48); // IRQ
    
	// Install some basic rom functionality

    /* EA31 IRQ return: jmp($0312). */
    rom [0xea31] = JMPw;
    rom [0xea32] = 0x7e;
    rom [0xea33] = 0xea;

    rom [0xea7e] = NOPn;
    rom [0xea7f] = NOPn;
    rom [0xea80] = NOPn;

    rom [0xea81] = JMPi;
    rom [0xea82] = 0x12;
    rom [0xea83] = 0x03;

    // Is this still needed?  Really some of Dags player should be put
	// into rom as it's only doing what the normal code should do.
	// However what will playsid mode make of this...
    if (_environment == sid2_envPS)
    {
        // (ms) IRQ ($FFFE) comes here and we do JMP ($0314)
        rom[0xff48] = JMPi;
        rom[0xff49] = 0x14;
        rom[0xff4a] = 0x03;
    }

    // Set master volume to fix some bad songs
    if (_sidEnabled[0])
        writeMemByte_playsid (_sidAddress[0] + 0x18, 0x0f, true);
    if (_sidEnabled[1])
        writeMemByte_playsid (_sidAddress[1] + 0x18, 0x0f, true);
}

uint8_t player::envReadMemByte (uint_least16_t addr, bool useCache)
{   // Read from plain only to prevent execution of rom code
    return (this->*(readMemByte)) (addr, useCache);
}

void player::envWriteMemByte (uint_least16_t addr, uint8_t data, bool useCache)
{   // Writes must be passed to env version.
    (this->*(writeMemByte)) (addr, data, useCache);
}

void player::envTriggerIRQ (void)
{   // Load the next note sequence
    nextSequence ();
}

void player::envTriggerNMI (void)
{   // NOT DEFINED
    ;
}

void player::envTriggerRST (void)
{   // NOT DEFINED
    ;
}

void player::envClearIRQ (void)
{
    cpu.clearIRQ ();
}

uint8_t player::envReadMemDataByte (uint_least16_t addr, bool useCache)
{   // Read from plain only to prevent execution of rom code
    return (this->*(readMemDataByte)) (addr, useCache);
}

bool player::envCheckBankJump (uint_least16_t addr)
{
    switch (_environment)
    {
    case sid2_envBS:
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

    case sid2_envTP:
        if ((addr >= 0xd000) && isKernal)
            return false;
    break;

    default:
    break;
    }

    return true;
}
