/*
 * /home/ms/files/source/libsidtune/RCS/PSID.cpp,v
 *
 * PlaySID one-file format support.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <fstream.h>
#include <string.h>

#include "config.h"
#include "SidTune.h"
#include "sidendian.h"

struct psidHeader           // all values big-endian
{
    char id[4];             // 'PSID' (ASCII)
    uint8_t version[2];     // 0x0001 or 0x0002
    uint8_t data[2];        // 16-bit offset to binary data in file
    uint8_t load[2];        // 16-bit C64 address to load file to
    uint8_t init[2];        // 16-bit C64 address of init subroutine
    uint8_t play[2];        // 16-bit C64 address of play subroutine
    uint8_t songs[2];       // number of songs
    uint8_t start[2];       // start song out of [1..256]
    uint8_t speed[4];       // 32-bit speed info
                            // bit: 0=50 Hz, 1=CIA 1 Timer A (default: 60 Hz)
    char name[32];          // ASCII strings, 31 characters long and
    char author[32];        // terminated by a trailing zero
    char copyright[32];     //
    uint8_t flags[2];       // only version 0x0002
    uint8_t relocStart;     // only version 0x0002B
    uint8_t relocPages;     // only version 0x0002B
    uint8_t reserved[2];    // only version 0x0002
};

enum
{
    PSID_MUS       = 1 << 0,
    PSID_SAMPLES   = 1 << 1,
    PSID_CLOCK     = 3 << 2,
    PSID_SIDMODEL  = 3 << 4
};

enum
{
    PSID_CLOCK_UNKNOWN = 0,
    PSID_CLOCK_PAL     = 1 << 2,
    PSID_CLOCK_NTSC    = 1 << 3,
    PSID_CLOCK_ANY     = PSID_CLOCK_PAL | PSID_CLOCK_NTSC
};

enum
{
    PSID_SIDMODEL_UNKNOWN = 0,
    PSID_SIDMODEL_6581    = 1 << 2,
    PSID_SIDMODEL_8580    = 1 << 3,
    PSID_SIDMODEL_ANY     = PSID_SIDMODEL_6581 | PSID_SIDMODEL_8580
};

static const char _sidtune_format[] = "PlaySID one-file format (PSID)";
static const char _sidtune_unknown[] = "Unsupported file format";
static const char _sidtune_truncated[] = "ERROR: File is most likely truncated";

const int _sidtune_psid_maxStrLen = 31;


bool SidTune::PSID_fileSupport(const void* buffer, const uint_least32_t bufLen)
{
    int clock;
#ifdef SIDTUNE_PSID2NG
    clock = SIDTUNE_CLOCK_UNKNOWN;
#else
    clock = info.clockSpeed;
#endif

    // Remove any format description or format error string.
    info.formatString = 0;

    // Require minimum size to allow access to the first few bytes.
    // Require a valid ID and version number.
    const psidHeader* pHeader = (const psidHeader*)buffer;
    if ( (bufLen<6) ||
         (endian_big32((const uint_least8_t*)pHeader->id)!=0x50534944) ||
         (endian_big16(pHeader->version)>=3) )
    {
        info.formatString = _sidtune_unknown;
        return false;
    }
    // Due to security concerns, input must be at least as long as version 1
    // header plus 16-bit C64 load address. That is the area which will be
    // accessed.
    if ( bufLen < (sizeof(psidHeader)+2) )
    {
        info.formatString = _sidtune_truncated;
        return false;
    }

    fileOffset        = endian_big16(pHeader->data);
    info.loadAddr     = endian_big16(pHeader->load);
    info.initAddr     = endian_big16(pHeader->init);
    info.playAddr     = endian_big16(pHeader->play);
    info.songs        = endian_big16(pHeader->songs);
    info.startSong    = endian_big16(pHeader->start);
    info.sidChipBase1 = 0xd400;
    info.sidChipBase2 = 0;

    if (info.songs > SIDTUNE_MAX_SONGS)
    {
        info.songs = SIDTUNE_MAX_SONGS;
    }

    info.musPlayer      = false;
    info.sidModel       = SIDTUNE_SIDMODEL_UNKNOWN;
    info.samples        = false;
    info.relocPages     = 0;
    info.relocStartPage = 0;
    if ( endian_big16(pHeader->version) >= 2 )
    {
        uint_least16_t flags = endian_big16(pHeader->flags);
        if (flags & PSID_MUS)
        {   // MUS tunes run at any speed
            clock = SIDTUNE_CLOCK_ANY;
            info.musPlayer = true;
        }

#ifdef SIDTUNE_PSID2NG
        if (flags & PSID_SAMPLES)
            info.samples = true;

        if (flags & PSID_CLOCK_PAL)
            clock |= SIDTUNE_CLOCK_PAL;
        if (flags & PSID_CLOCK_NTSC)
            clock |= SIDTUNE_CLOCK_NTSC;

        info.sidModel = SIDTUNE_SIDMODEL_UNKNOWN;
        if (flags & PSID_SIDMODEL_6581)
            info.sidModel |= SIDTUNE_SIDMODEL_6581;
        if (flags & PSID_SIDMODEL_8580)
            info.sidModel |= SIDTUNE_SIDMODEL_8580;

        info.relocStartPage = pHeader->relocStart;
        info.relocPages     = pHeader->relocPages;
#endif // SIDTUNE_PSID2NG
    }

    // Create the speed/clock setting table.
    convertOldStyleSpeedToTables(endian_big32(pHeader->speed), clock);

    if ( info.loadAddr == 0 )
    {
        uint_least8_t* pData = (uint_least8_t*)buffer + fileOffset;
        info.loadAddr = endian_16( *(pData+1), *pData );
        fileOffset += 2;
    }
    if ( info.initAddr == 0 )
    {
        info.initAddr = info.loadAddr;
    }
    
    // Copy info strings, so they will not get lost.
    info.numberOfInfoStrings = 3;
    // Name
    strncpy(&infoString[0][0],pHeader->name,_sidtune_psid_maxStrLen);
    info.infoString[0] = &infoString[0][0];
    // Author
    strncpy(&infoString[1][0],pHeader->author,_sidtune_psid_maxStrLen);
    info.infoString[1] = &infoString[1][0];
    // Copyright
    strncpy(&infoString[2][0],pHeader->copyright,_sidtune_psid_maxStrLen);
    info.infoString[2] = &infoString[2][0];
    
    info.formatString = _sidtune_format;
    return true;
}


bool SidTune::PSID_fileSupportSave(ofstream& fMyOut, const uint_least8_t* dataBuffer)
{
    psidHeader myHeader;
    endian_big32((uint_least8_t*)myHeader.id,0x50534944);  // 'PSID'
    endian_big16(myHeader.version,2);
    endian_big16(myHeader.data,sizeof(psidHeader));
    endian_big16(myHeader.load,0);
    endian_big16(myHeader.init,info.initAddr);
    endian_big16(myHeader.play,info.playAddr);
    endian_big16(myHeader.songs,info.songs);
    endian_big16(myHeader.start,info.startSong);

    uint_least32_t speed = 0;
    uint_least32_t maxBugSongs = ((info.songs <= 32) ? info.songs : 32);
    for (uint_least32_t s = 0; s < maxBugSongs; s++)
    {
        if (songSpeed[s] == SIDTUNE_SPEED_CIA_1A)
            speed |= (1<<s);
    }
    endian_big32(myHeader.speed,speed);

    uint_least16_t tmpFlags = 0;
    if ( info.musPlayer )
        tmpFlags |= PSID_MUS;

    if ( info.samples )
        tmpFlags |= PSID_SAMPLES;

    if (info.clockSpeed & SIDTUNE_CLOCK_PAL)
        tmpFlags |= PSID_CLOCK_PAL;
    if (info.clockSpeed & SIDTUNE_CLOCK_NTSC)
        tmpFlags |= PSID_CLOCK_NTSC;

    if (info.sidModel & SIDTUNE_SIDMODEL_6581)
        tmpFlags |= PSID_SIDMODEL_6581;
    if (info.sidModel & SIDTUNE_SIDMODEL_8580)
        tmpFlags |= PSID_SIDMODEL_8580;

    endian_big16(myHeader.flags,tmpFlags);
    endian_big16(myHeader.reserved,0);
    myHeader.relocStart = info.relocStartPage;
    myHeader.relocPages = info.relocPages;
    for ( uint i = 0; i < 32; i++ )
    {
        myHeader.name[i] = 0;
        myHeader.author[i] = 0;
        myHeader.copyright[i] = 0;
    }
    strncpy( myHeader.name, info.infoString[0], _sidtune_psid_maxStrLen);
    strncpy( myHeader.author, info.infoString[1], _sidtune_psid_maxStrLen);
    strncpy( myHeader.copyright, info.infoString[2], _sidtune_psid_maxStrLen);
    fMyOut.write( (char*)&myHeader, sizeof(psidHeader) );
    
    // Save C64 lo/hi load address (little-endian).
    uint_least8_t saveAddr[2];
    saveAddr[0] = info.loadAddr & 255;
    saveAddr[1] = info.loadAddr >> 8;
    fMyOut.write( (char*)saveAddr, 2 );  // !cast!

    // Data starts at: bufferaddr + fileoffset
    // Data length: datafilelen - fileoffset
    fMyOut.write( (const char*)dataBuffer + fileOffset, info.dataFileLen - fileOffset );  // !cast!
    if ( !fMyOut )
        return false;
    else
        return true;
}
