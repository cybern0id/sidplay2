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
    sid.enable_filter(true);
    sid.enable_external_filter(true);
    // Emulation type selectable
    sid.set_chip_model(MOS6581);

    //----------------------------------------------
    // SID Initialise
    // These are optional
    sid2.enable_filter(true);
    sid2.enable_external_filter(true);
    // Emulation type selectable
    sid2.set_chip_model(MOS6581);

    // Set default settings for system
    myTune = tune  = NULL;
    ram    = (rom  = NULL);
    _environment   = _requiredEnv = sidplaybs;
    _playback      = mono;
    _samplingFreq  = SIDPLAYER_DEFAULT_SAMPLING_FREQ;
    playerState    = _stopped;
    // Added Rev 2.0.3
    _forceDualSids = false;

    // Temp @TODO@
    _leftVolume    = 255;
    _rightVolume   = 255;
    _optimiseLevel = 1;
}

sidplayer_pr::~sidplayer_pr ()
{   // Remove the loaded song
    if (myTune != NULL)
        delete myTune;
}

void sidplayer_pr::configure (playback_sidt playback, udword_sidt samplingFreq, bool forceDualSids)
{
    _playback      = playback;
    _samplingFreq  = samplingFreq;
    // Added Rev 2.0.3
    _forceDualSids = forceDualSids;
}

// Stops the emulation routine
void sidplayer_pr::stop (void)
{
    (void) initialise ();
}

void sidplayer_pr::paused (void)
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
    udword_sidt count     = 0;
    uword_sidt  clock     = 0;
    double samplingCount  = 0; // Move this and have it reset on initialise
    double samplingPeriod = _cpuFreq / (double) _samplingFreq;

    // Make sure a tune is loaded
    if (!tune)
        return 0;

    // Start the player loop
    playerState = _playing;
    while (playerState == _playing)
    {   // For sidplay compatibility the cpu must be idle
        // when the play routine exists.  The cpu will stay
        // idle until an interrupt occurs
        while (!cpu.SPWrapped)
        {
            cpu.clock ();
            if (_optimiseLevel < 3)
                break;
        }

        if (_optimiseLevel == 1)
        {   // Sids currently have largest cpu overhead, so have been moved
            // and are now only clocked when an output it required
            // Only clock second sid if we want to hear right channel or
            // stereo.  However having this results in better playback
            if (_playback <= stereo)
                sid.clock ();
            if (_playback > mono)
                sid2.clock ();
            xsid.clock ();
        }

        cia.clock  ();
        clock++;

        // Check to see if we need a new sample from reSID
        samplingCount++;
        if (samplingCount > samplingPeriod)
        {   // Rev 2.0.3 Changed - Using new mixer routines
            (this->*output) (clock, buffer, count);
            // Check to see if the buffer is full and if so return
            // so the samples can be played
            if (count >= length)
                return length;
            samplingCount -= samplingPeriod;
            clock = 0;
        }
    }

    return count;
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
    if (!myTune)    return -1;
    // Make sure the tune loaded correctly
    if (!(*myTune)) return -1;
    myTune->selectSong(songNumber);
    return loadSong (myTune);
};

int sidplayer_pr::loadSong (SidTune *requiredTune)
{
    tune   = requiredTune;
    tune->getInfo(tuneInfo);

    // Setup the audio side, depending on the audio hardware
    // and the information returned by sidtune
    switch (_playback)
    {
    case stereo:
        if (tuneInfo.sidChipBase2 || _forceDualSids)
            output = &sidplayer_pr::stereoOut8StereoIn;
        else
            output = &sidplayer_pr::stereoOut8MonoIn;
    break;

    case right:
        if (tuneInfo.sidChipBase2 || _forceDualSids)
            output = &sidplayer_pr::rightOut8StereoIn;
        else
            output = &sidplayer_pr::monoOut8MonoIn;
    break;

    case left:
        if (tuneInfo.sidChipBase2 || _forceDualSids)
            output = &sidplayer_pr::leftOut8StereoIn;
        else
            output = &sidplayer_pr::monoOut8MonoIn;
    break;

    case mono:
    default:
        if (tuneInfo.sidChipBase2 || _forceDualSids)
            output = &sidplayer_pr::monoOut8StereoIn;
        else
            output = &sidplayer_pr::monoOut8MonoIn;
    break;
    }

    // Disable second sid from read/writes in memory
    // accesses.  The help preventing breaking of songs
    // which deliberately use SID mirroring.
    if (tuneInfo.sidChipBase2 || _forceDualSids)
        _sid2Enabled = true;
    else
        _sid2Enabled = false;

    // Check if environment has not initialised or
    // the user has asked to a different one.
    // This call we initalise the player
    if ((_environment == _requiredEnv) || !ram)
        return setEnvironment (_requiredEnv);

    // Initialise the player
    return initialise ();
}

void sidplayer_pr::getInfo (playerInfo_sidt *info)
{
    info->name        = PACKAGE;
    info->version     = VERSION;
    memcpy (&(info->tuneInfo), &tuneInfo, sizeof (SidTuneInfo));
    info->environment = _environment;
    info->sidFilter   = true;
}

int sidplayer_pr::initialise ()
{   // Now read the sub tune into memory
    ubyte_sidt AC = tuneInfo.currentSong - 1;
    envReset ();
    if (!tune->placeSidTuneInC64mem (ram))
        return -1;

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
    if ((playAddr == 0xffff) || (playAddr == initAddr))
        tuneInfo.playAddr = 0;
    initBankSelect (playAddr);
    _initBankReg = _bankReg;

    // Get the next sequence of notes
    nextSequence ();
    playerState = _stopped;
    return 0;
}

void sidplayer_pr::environment (env_sidt env)
{
    _requiredEnv = env;
}

int sidplayer_pr::setEnvironment (env_sidt env)
{   // Not supported yet
    if (env == real)
        return -1;

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
    if (_environment == playsid)
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
        case sidplaytp:
            readMemByte     = &sidplayer_pr::readMemByte_plain;
            writeMemByte    = &sidplayer_pr::writeMemByte_sidplay;
            readMemDataByte = &sidplayer_pr::readMemByte_sidplaytp;
        break;

        case sidplaybs:
            readMemByte     = &sidplayer_pr::readMemByte_plain;
            writeMemByte    = &sidplayer_pr::writeMemByte_sidplay;
            readMemDataByte = &sidplayer_pr::readMemByte_sidplaybs;
        break;

        case real:
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
    if (_environment == playsid)
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
        switch (addr)
        {
        case 0xdc04:
        case 0xdc05:
            return cia.read ((ubyte_sidt) addr);
        break;

        default:
            return rom[addr];
        }
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
            if (_sid2Enabled)
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
        if (_sid2Enabled)
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
    if (_environment != playsid)
        memset (rom + 0xA000, RTSn, 0x2000);

    ram[0] = 0x2F;
    // defaults: Basic-ROM on, Kernal-ROM on, I/O on
    evalBankSelect(0x07);
    // fake VBI-interrupts that do $D019, BMI ...
    rom[0x0d019] = 0xff;

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
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_PAL;
            cia.reset ((uword_sidt) (_cpuFreq / SIDPLAYER_VIC_FREQ_NTSC + 0.5));
        }

        cia.write (0x0e, 0x01); // Start the timer
        cia.locked = true;
        ram[0x02a6] = 1; // PAL
    }
    else // SIDTUNE_SPEED_CIA_1A
    {
        if (tuneInfo.clockSpeed == SIDTUNE_CLOCK_PAL)
        {
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_PAL;
        }
        else // SIDTUNE_CLOCK_NTSC
        {
            _cpuFreq = SIDPLAYER_CLOCK_FREQ_PAL;
        }

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
	
    // Why? - (Copied from libsidplay)
    if (_environment == playsid)
    {
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
    case sidplaybs:
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

    case sidplaytp:
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

void sidplayer::configure (playback_sidt mode, udword_sidt samplingFreq, bool forceDualSid)
{   player->configure (mode, samplingFreq, forceDualSid); }

void sidplayer::stop (void)
{   player->stop (); }

void sidplayer::paused (void)
{   player->stop (); }

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

void sidplayer::optimisation (ubyte_sidt level)
{   player->optimisation (level); }
