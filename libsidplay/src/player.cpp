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
 *  Revision 1.22  2001/07/14 12:56:15  s_a_white
 *  SID caching no longer needed. IC  components now run using event
 *  generation (based on VICE).  Handling of IRQs now more effecient.  All
 *  sidplay1 hacks either removed or moved to sid6510.  Fixed PAL/NTSC
 *  speeding fixing.  Now uses new component and sidbuilder classes.
 *
 *  Revision 1.21  2001/04/23 17:09:56  s_a_white
 *  Fixed video speed selection using unforced/forced and NTSC clockSpeeds.
 *
 *  Revision 1.20  2001/03/26 21:46:43  s_a_white
 *  Removed unused #include.
 *
 *  Revision 1.19  2001/03/25 19:48:13  s_a_white
 *  xsid.reset added.
 *
 *  Revision 1.18  2001/03/22 22:45:20  s_a_white
 *  Re-ordered initialisations to match defintions.
 *
 *  Revision 1.17  2001/03/21 22:32:34  s_a_white
 *  Filter redefinition support.  VIC & NMI support added.  Moved fake interrupts
 *  to sid6510 class.
 *
 *  Revision 1.16  2001/03/09 22:26:36  s_a_white
 *  Support for updated C64 player.
 *
 *  Revision 1.15  2001/03/08 22:46:42  s_a_white
 *  playAddr = 0xffff now better supported.
 *
 *  Revision 1.14  2001/03/01 23:46:37  s_a_white
 *  Support for sample mode to be selected at runtime.
 *
 *  Revision 1.13  2001/02/28 18:55:27  s_a_white
 *  Removed initBank* related stuff.  IRQ terminating ROM jumps at 0xea31,
 *  0xea7e and 0xea81 now handled.
 *
 *  Revision 1.12  2001/02/21 21:43:10  s_a_white
 *  Now use VSID code and this handles interrupts much better!  The whole
 *  initialise sequence has been modified to support this.
 *
 *  Revision 1.11  2001/02/13 21:01:14  s_a_white
 *  Support for real interrupts.  C64 Initialisation routine now run from Player::play
 *  instead of Player::initialise.  Prevents lockups if init routine does not return.
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

const double Player::CLOCK_FREQ_NTSC = 1022727.14;
const double Player::CLOCK_FREQ_PAL  = 985248.4;
const double Player::VIC_FREQ_PAL    = 50.0;
const double Player::VIC_FREQ_NTSC   = 60.0;

// These texts are used to override the sidtune settings.
const char  *Player::TXT_PAL_VBI        = "50 Hz VBI (PAL)";
const char  *Player::TXT_PAL_VBI_FIXED  = "60 Hz VBI (PAL FIXED)";
const char  *Player::TXT_PAL_CIA        = "CIA (PAL)";
const char  *Player::TXT_PAL_UNKNOWN    = "UNKNOWN (PAL)";
const char  *Player::TXT_NTSC_VBI       = "60 Hz VBI (NTSC)";
const char  *Player::TXT_NTSC_VBI_FIXED = "50 Hz VBI (NTSC FIXED)";
const char  *Player::TXT_NTSC_CIA       = "CIA (NTSC)";
const char  *Player::TXT_NTSC_UNKNOWN   = "UNKNOWN (NTSC)";
const char  *Player::TXT_NA             = "NA";

// Error Strings
const char  *Player::ERR_CONF_WHILST_ACTIVE    = "SIDPLAYER ERROR: Trying to configure player whilst active.";
const char  *Player::ERR_UNSUPPORTED_FREQ      = "SIDPLAYER ERROR: Unsupported sampling frequency.";
const char  *Player::ERR_UNSUPPORTED_PRECISION = "SIDPLAYER ERROR: Unsupported sample precision.";
const char  *Player::ERR_MEM_ALLOC             = "SIDPLAYER ERROR: Memory Allocation Failure.";
const char  *Player::ERR_UNSUPPORTED_MODE      = "SIDPLAYER ERROR: Unsupported Environment Mode (Coming Soon).";
const char  *Player::ERR_FILTER_DEFINITION     = "SIDPLAYER ERROR: Filter definition is not valid (see docs).";

const char  *Player::credit[];


// Set the ICs environment variable to point to
// this player
Player::Player (void)
// Set default settings for system
:c64env("SID Music Player"),
 sid6510 (&eventContext),
 mos6510 (&eventContext),
 cpu   (&sid6510),
 mos6581_1 (this),
 mos6581_2 (this),
 xsid  (this, &mos6581_1),
 cia   (this),
 cia2  (this),
 vic   (this),
 m_builder (NULL),
 mixerEvent(this),
 rtc   (&eventContext),
 _tune (NULL),
 ram   (NULL),
 rom   (NULL),
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
    sid6510.setEnvironment (this);
    mos6510.setEnvironment (this);

    //----------------------------------------------
    // SID Initialise
    sid  = &xsid;
    sid2 = &mos6581_2;

    // These are optional
    // Emulation type selectable
    filter     (true);
    extFilter  (_samplingFreq / 2);
    // Emulation type selectable
    sidModel   (SID2_MODEL_CORRECT);
    sidSamples (true);
    //----------------------------------------------

    // Rev 2.0.4 (saw) - Added
    configure (sid2_mono, SID2_DEFAULT_SAMPLING_FREQ,
               SID2_DEFAULT_PRECISION, false);

    // Get component credits
    credit[0] = PACKAGE " V" VERSION " Engine:\0\tCopyright (C) 2000 Simon White <sidplay2@email.com>\0";
    credit[1] = mos6581_1.credits ();
    credit[2] = xsid.credits ();
    credit[3] = "*MOS6510 (CPU) Emulation:\0\tCopyright (C) 2000 Simon White <sidplay2@email.com>\0";
    credit[4] = cia.credits ();
    credit[5] = vic.credits ();
    credit[6] = NULL;
}

int Player::clockSpeed (sid2_clock_t clock, bool forced)
{
    if (playerState == _playing)
        return -1;

    _clockSpeed = clock;
    _forced     = forced;
    // Other paremters set later
    if (!_tune)
        return 0;

    // Refresh Information
    _tune->getInfo (tuneInfo);

    // Mirror a real C64
    if (tuneInfo.playAddr == 0xffff)
        forced = true;

    // Detect the Correct Song Speed
    if (clock == SID2_CLOCK_CORRECT)
    {
        clock = SID2_CLOCK_PAL;
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
            clock = SID2_CLOCK_NTSC;
    }
    // If forced change song to be the requested speed
    else if (forced)
    {
        tuneInfo.clockSpeed = SIDTUNE_CLOCK_PAL;
        if (clock == SID2_CLOCK_NTSC)
            tuneInfo.clockSpeed = SIDTUNE_CLOCK_NTSC;
    }

    if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
        vic.chip (MOS6569);
    else // if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
        vic.chip (MOS6567R8);

    if (clock == SID2_CLOCK_PAL)
    {
        _cpuFreq = CLOCK_FREQ_PAL;
        tuneInfo.speedString = TXT_PAL_VBI;
        if (tuneInfo.songSpeed == SIDTUNE_SPEED_CIA_1A)
            tuneInfo.speedString = TXT_PAL_CIA;
        else if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
            tuneInfo.speedString = TXT_PAL_VBI_FIXED;
    }
    else if (clock == SID2_CLOCK_NTSC)
    {
        _cpuFreq = CLOCK_FREQ_NTSC;
        tuneInfo.speedString = TXT_NTSC_VBI;
        if (tuneInfo.songSpeed == SIDTUNE_SPEED_CIA_1A)
            tuneInfo.speedString = TXT_NTSC_CIA;
        else if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
            tuneInfo.speedString = TXT_NTSC_VBI_FIXED;
    }

    // Check for real C64 environment
    if (tuneInfo.playAddr == 0xffff)
    {
        xsid.mute (true);
        tuneInfo.songSpeed   = SIDTUNE_SPEED_CIA_1A;
        tuneInfo.speedString = TXT_PAL_UNKNOWN;
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_NTSC)
            tuneInfo.speedString = TXT_NTSC_UNKNOWN;;
    }

    // Clock speed changes due to loading a new song
    m_samplePeriod = _cpuFreq / (float64_t) _samplingFreq;
    return 0;
}

int Player::configure (sid2_playback_t playback, uint_least32_t samplingFreq, int precision, bool forceDualSids)
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
    m_samplePeriod  = _cpuFreq / (float64_t) samplingFreq;
    _leftVolume     = _userLeftVolume;
    _rightVolume    = _userRightVolume;

    _sidAddress[0]  = 0xd400;
    _sidAddress[1]  = tuneInfo.sidChipBase2;

    // Setup the external filter to avoid aliasing
    extFilter (_samplingFreq / 2);
    
    // Only force dual sids if second wasn't detected
    if (!_sidAddress[1] && _forceDualSids)
        _sidAddress[1] = 0xd500; // Assumed

    if (_playback != sid2_mono)
    {   // Try Spliting channels across 2 sids
        if (!_sidAddress[1])
        {
            _sidAddress[1] = _sidAddress[0];

            // Mute Voices
            sid->voice  (1, 0, true);
            sid2->voice (0, 0, true);
            sid2->voice (2, 0, true);
            // 2 Voices scaled to unity from 4 (was !SID_VOL)
            //    _leftVolume  *= 2;
            //    _rightVolume *= 2;
            // 2 Voices scaled to unity from 3 (was SID_VOL)
            //        _leftVolume  *= 3;
            //        _leftVolume  /= 2;
            //    _rightVolume *= 3;
            //    _rightVolume /= 2;
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
                output = &Player::stereoOut8MonoIn;
            else
                output = &Player::monoOut8MonoIn;
        }
        else
        {
            switch (_playback)
            {
            case sid2_stereo: // Stereo Hardware
                output = &Player::stereoOut8StereoIn;
            break;

            case sid2_right: // Mono Hardware,
                output = &Player::monoOut8StereoRIn;
            break;

            case sid2_left:
                output = &Player::monoOut8MonoIn;
            break;

            case sid2_mono:
                output = &Player::monoOut8StereoIn;
            break;
            }
        }
    break;
            
    case 16:
        if (!_sidAddress[1])
        {
            if (_playback == sid2_stereo)
                output = &Player::stereoOut16MonoIn;
            else
                output = &Player::monoOut16MonoIn;
        }
        else
        {
            switch (_playback)
            {
            case sid2_stereo: // Stereo Hardware
                output = &Player::stereoOut16StereoIn;
            break;

            case sid2_right: // Mono Hardware,
                output = &Player::monoOut16StereoRIn;
            break;

            case sid2_left:
                output = &Player::monoOut16MonoIn;
            break;

            case sid2_mono:
                output = &Player::monoOut16StereoIn;
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

// Set SID emulation
void Player::emulation (sidbuilder *builder)
{
    if (m_builder)
        m_builder->remove ();

    m_builder = builder;
    if (!builder)
    {   // Restore internal sid emulations
        sid  = &mos6581_1;
        sid2 = &mos6581_2;
    } else {
        sid  = builder->create (this);
        sid2 = builder->create (this);
    }
    xsid.emulation (sid);
}

int Player::environment (sid2_env_t env)
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
        m_readMemByte     = &Player::readMemByte_player;
        m_writeMemByte    = &Player::writeMemByte_playsid;
        m_readMemDataByte = &Player::readMemByte_playsid;
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
            m_readMemByte     = &Player::readMemByte_player;
            m_writeMemByte    = &Player::writeMemByte_sidplay;
            m_readMemDataByte = &Player::readMemByte_sidplaytp;
        break;

        case sid2_envBS:
            m_readMemByte     = &Player::readMemByte_player;
            m_writeMemByte    = &Player::writeMemByte_sidplay;
            m_readMemDataByte = &Player::readMemByte_sidplaybs;
        break;

        case sid2_envR:
        default: // <-- Just to please compiler
            m_readMemByte     = &Player::readMemByte_player;
            m_writeMemByte    = &Player::writeMemByte_sidplay;
            m_readMemDataByte = &Player::readMemByte_sidplaybs;
        break;
        }
    }

    // Have to reload the song into memory as
    // everything has changed
    if (_tune)
        return initialise ();

    return 0;
}

int Player::fastForward (uint_least8_t percent)
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

void Player::getInfo (sid2_playerInfo_t *info)
{
    info->name        = PACKAGE;
    info->version     = VERSION;
    info->filter      = _filter;
    info->tuneInfo    = tuneInfo;
    info->environment = _environment;
    info->sidModel    = _sidModel;
    
    if (_sidModel == SID2_MODEL_CORRECT)
    {
        info->sidModel = SID2_MOS6581;
        if (tuneInfo.sidRevision == SIDTUNE_SID_8580)
            info->sidModel = SID2_MOS8580;
    }
}

int Player::initialise ()
{
    playerState = _stopped;
    m_running   = false;

    // Fix the mileage counter if just finished another song.
    mileageCorrect ();
    _mileage += time ();

    envReset ();

    // Code must not be bigger than zero page (256 bytes)
    uint8_t psid_driver[] = {
#       include "player.bin"
    };

    /* Install PSID driver code. */
    memcpy (&ram[endian_little16 (psid_driver)], psid_driver + 2,
        sizeof (psid_driver) - 2);

    /* Install interrupt vectors in both ROM and RAM. */
    memcpy (&ram[0xfffa], &ram[endian_little16 (psid_driver)], 6);
    memcpy (&rom[0xfffa], &ram[endian_little16 (psid_driver)], 6);

    // Setup the Initial entry point
    uint_least16_t playAddr = tuneInfo.playAddr;

    // Check to make sure the play address is legal
    if (playAddr == 0xffff)
        playAddr  = 0;

    // Tell C64 about song, 1st 2 locations reserved for
    // bank switching.
    endian_little16 (&ram[0x04], tuneInfo.initAddr);
    endian_little16 (&ram[0x06], playAddr);
    ram[0x02] = (uint8_t) tuneInfo.currentSong;

    if (tuneInfo.songSpeed == SIDTUNE_SPEED_VBI)
        ram[0x03] = 0;
    else // SIDTUNE_SPEED_CIA_1A
        ram[0x03] = 1;

    // The Basic ROM sets these values on loading a file.
    {   // Program start address
        uint_least16_t addr = tuneInfo.loadAddr;
        endian_little16 (&ram[0x2b], addr);
        // Program end address + 1
        addr += tuneInfo.c64dataLen;
        endian_little16 (&ram[0x2d], addr);
    }

    if (!_tune->placeSidTuneInC64mem (ram))
    {   // Rev 1.6 (saw) - Allow loop through errors
        _errorString = tuneInfo.statusString;
        return -1;
    }

    cpu->reset ();
    rtc.clock  (_cpuFreq);
    mixerReset ();
    xsid.suppress (true);
    return 0;
}

int Player::loadFilter (const sid_fc_t *cutoffs, uint_least16_t points)
{
    if (!mos6581_1.filter (cutoffs, points))
        goto player_loadFilter_error;
    if (!mos6581_2.filter (cutoffs, points))
        goto player_loadFilter_error;
return 0;

player_loadFilter_error:
    _errorString = ERR_FILTER_DEFINITION;
    return -1;
}


int Player::loadSong (SidTune *tune)
{
    _tune = tune;
    _tune->getInfo(tuneInfo);

    // Un-mute all voices
    xsid.mute (false);

    uint_least8_t i = 3;
    while (i--)
    {
        sid->voice  (i, 0, false);
        sid2->voice (i, 0, false);
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

void Player::pause (void)
{
    if (playerState != _stopped)
    {
        playerState  = _paused;
        m_running    = false;
    }
}

uint_least32_t Player::play (void *buffer, uint_least32_t length)
{
    // Make sure a _tune is loaded
    if (!_tune)
        return 0;

    // Setup Sample Information
    m_sampleIndex  = 0;
    m_sampleCount  = length;
    m_sampleBuffer = (char *) buffer;

    // Start the player loop
    playerState = _playing;
    m_running   = true;

    while (m_running)
    {
        cpu->clock ();
        eventContext.clock ();
    }

    if (playerState == _stopped)
        initialise ();
    return m_sampleIndex;
}

void Player::stop (void)
{   // Re-start song
    if (!m_running)
        initialise ();
    else
    {
        playerState = _stopped;
        m_running   = false;
    }
}

void Player::sidSamples (bool enable)
{
    // @FIXME@ Extend this when digi scan added.
    _sidSamples  =   enable;
    _digiChannel =  (enable == false);
    xsid.sidSamples (enable);

    // Now balance voices
    if (enable)
    {
        mos6581_1.gain (0);
        mos6581_2.gain (0);
        xsid.gain (-100);
    } else {
        mos6581_1.gain (-25);
        mos6581_2.gain (-25);
        xsid.gain (-75);
    }
}


//-------------------------------------------------------------------------
// Temporary hack till real bank switching code added

/*
//  Input: A 16-bit effective address
// Output: A default bank-select value for $01.
void Player::initBankSelect (uint_least16_t addr)
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

void Player::evalBankSelect (uint8_t data)
{   // Determine new memory configuration.
    isBasic  = ((data & 3) == 3);
    isIO     = ((data & 7) >  4);
    isKernal = ((data & 2) != 0);
    _bankReg = data;
}

uint8_t Player::readMemByte_player (uint_least16_t addr)
{
    if (_environment == sid2_envR)
        return readMemByte_sidplaybs (addr);
    return readMemByte_plain (addr);
}

uint8_t Player::readMemByte_plain (uint_least16_t addr)
{   // Bank Select Register Value DOES NOT get to ram
    if (addr == 0x0001)
        return _bankReg;
    return ram[addr];
}

uint8_t Player::readMemByte_playsid (uint_least16_t addr)
{
    uint_least16_t tempAddr = (addr & 0xfc1f);

    // Not SID ?
    if (( tempAddr & 0xff00 ) != 0xd400 )
    {
        switch (endian_16hi8 (addr))
        {
        case 0:
            return readMemByte_plain (addr);
        case 0xdc:
            return cia.read (addr&0x0f);
        case 0xdd:
            return cia2.read (addr&0x0f);
        case 0xd0:
            return vic.read (addr&0x3f);
        default:
            return rom[addr];
        }
    }

    // Read real sid for these
    if ((addr & 0xff00) == _sidAddress[1])
        return sid2->read ((uint8_t) addr);
    return sid->read ((uint8_t) tempAddr);
}

uint8_t Player::readMemByte_sidplaytp(uint_least16_t addr)
{
    if (addr < 0xD000)
        return readMemByte_plain (addr);
    else
    {
        // Get high-nibble of address.
        switch (addr >> 12)
        {
        case 0xd:
            if (isIO)
                return readMemByte_playsid (addr);
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
        
uint8_t Player::readMemByte_sidplaybs (uint_least16_t addr)
{
    if (addr < 0xA000)
        return readMemByte_plain (addr);
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
                return readMemByte_playsid (addr);
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

void Player::writeMemByte_plain (uint_least16_t addr, uint8_t data)
{
    if (addr == 0x0001)
    {   // Determine new memory configuration.
        evalBankSelect (data);
        return;
    }
    ram[addr] = data;
}

void Player::writeMemByte_playsid (uint_least16_t addr, uint8_t data)
{
    uint_least16_t tempAddr = (addr & 0xfc1f);

    // Not SID ?
    if (( tempAddr & 0xff00 ) != 0xd400 )
    {
        switch (endian_16hi8 (addr))
        {
        case 0:
            writeMemByte_plain (addr, data);
        return;
        case 0xdc:
            cia.write (addr&0x0f, data);
        return;
        case 0xdd:
            cia2.write (addr&0x0f, data);
        return;
        case 0xd0:
            vic.write (addr&0x3f, data);
        return;
        default:
            rom[addr] = data;
        return;
        }
    }

    // $D41D/1E/1F, $D43D/3E/3F, ...
    // Map to real address to support PlaySID
    // Extended SID Chip Registers.
    if (( tempAddr & 0x00ff ) >= 0x001d )
        xsid.write16 (addr & 0x01ff, data);
    else // Mirrored SID.
    {   // SID.
        // Convert address to that acceptable by resid
        // Support dual sid
        if ((addr & 0xff00) == _sidAddress[1])
        {
            sid2->write (addr & 0xff, data);
            // Prevent sid write accessing other sid
            // if not doing mono to stereo conversion.
            if (_sidAddress[1] != _sidAddress[0])
                return;
        }
        sid->write (tempAddr & 0xff, data);
    }
}

void Player::writeMemByte_sidplay (uint_least16_t addr, uint8_t data)
{
    if (addr < 0xA000)
        writeMemByte_plain (addr, data);
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
                writeMemByte_playsid (addr, data);
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
void Player::envReset (void)
{
    // Select Sidplay1 compatible CPU or real thing
    cpu = &sid6510;
    if (tuneInfo.playAddr == 0xffff)
        cpu = &mos6510;

    eventContext.reset ();
    cpu->reset  ();
    sid->reset  ();
    sid2->reset ();
    cia.reset   ();
    cia2.reset  ();
    vic.reset   ();

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

    // software vectors
    endian_little16 (&ram[0x0314], 0xEA31); // IRQ
    endian_little16 (&ram[0x0316], 0xFE66); // BRK
    endian_little16 (&ram[0x0318], 0xFE47); // NMI

    // hardware vectors
    endian_little16 (&rom[0xfffa],  0xFE43); // NMI
    endian_little16 (&rom[0xfffc],  0xFCE2); // RESET
    endian_little16 (&rom[0xfffe],  0xFF48); // IRQ

    // This provides greater Sidplay1 compatibility at the cost
    // of being totally incompatible with a real C64.
    //if ((tuneInfo.playAddr == 0x0000) ||
    //    (tuneInfo.playAddr == 0xffff))
    {   // Install some basic rom functionality
        /* EA31 IRQ return: jmp($0310). */
        rom[0xea31] = JMPw;
        rom[0xea32] = 0x7e;
        rom[0xea33] = 0xea;

        rom[0xea7e] = NOPn;
        rom[0xea7f] = NOPn;
        rom[0xea80] = NOPn;
        
        // NMI entry
        rom[0xFE43] = SEIn;
        rom[0xFE44] = JMPi;
        rom[0xFE45] = 0x18;
        rom[0xFE46] = 0x03;
    }

    {   // (ms) IRQ ($FFFE) comes here and we do JMP ($0314)
        uint8_t prg[] = {PHAn,  TXAn, PHAn, TYAn, PHAn, TSXn,
                         LDAax, 0x04, 0x01, ANDb, 0x10, BEQr,
                         0x03,  JMPi, 0x16, 0x03, JMPi, 0x14,
                         0x03};
        memcpy (&rom[0xff48], prg, sizeof (prg));
    }

    {   // (ms) IRQ ($FFFE) comes here and we do JMP ($0314)
        uint8_t prg[] = {LDAa, 0x0d, 0xdc, PLAn, TAYn, PLAn,
                         TAXn, PLAn, RTIn};
        memcpy (&rom[0xea81], prg, sizeof (prg));
    }

    // Will get done later if can't now
    if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
        ram[0x02a6] = 1;
    else // SIDTUNE_CLOCK_NTSC
        ram[0x02a6] = 0;

    // Set SID emulation
    if (_sidModel == SID2_MODEL_CORRECT)
    {
        if (tuneInfo.sidRevision == SIDTUNE_SID_6581)
            sidModel (SID2_MOS6581);
        else if (tuneInfo.sidRevision == SIDTUNE_SID_8580)
            sidModel (SID2_MOS8580);
        _sidModel = SID2_MODEL_CORRECT;
    }

    // Set master volume to fix some bad songs
    if (_sidEnabled[1])
        sid->write  (0x18, 0x0f);
    if (_sidEnabled[1])
        sid2->write (0x18, 0x0f);
}

uint8_t Player::envReadMemByte (uint_least16_t addr)
{   // Read from plain only to prevent execution of rom code
    return (this->*(m_readMemByte)) (addr);
}

void Player::envWriteMemByte (uint_least16_t addr, uint8_t data)
{   // Writes must be passed to env version.
    (this->*(m_writeMemByte)) (addr, data);
}

uint8_t Player::envReadMemDataByte (uint_least16_t addr)
{   // Read from plain only to prevent execution of rom code
    return (this->*(m_readMemDataByte)) (addr);
}

bool Player::envCheckBankJump (uint_least16_t addr)
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

