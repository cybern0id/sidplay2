/*
 * /home/ms/files/source/libsidtune/RCS/PSID.cpp,v
 *
 * PlaySID one-file format support.
 * Copyright (C) Michael Schwendt <mschwendt@yahoo.com>
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

#include <fstream.h>
#include <string.h>

#include "SidTune.h"

struct psidHeader			// all values big-endian
{
	char id[4];				// 'PSID' (ASCII)
	ubyte_sidt version[2];	// 0x0001 or 0x0002
	ubyte_sidt data[2];		// 16-bit offset to binary data in file
	ubyte_sidt load[2];		// 16-bit C64 address to load file to
	ubyte_sidt init[2];		// 16-bit C64 address of init subroutine
	ubyte_sidt play[2];		// 16-bit C64 address of play subroutine
	ubyte_sidt songs[2];	// number of songs
	ubyte_sidt start[2];	// start song out of [1..256]
	ubyte_sidt speed[4];	// 32-bit speed info
							// bit: 0=50 Hz, 1=CIA 1 Timer A (default: 60 Hz)
	char name[32];			// ASCII strings, 31 characters long and
	char author[32];		// terminated by a trailing zero
	char copyright[32];		//
	ubyte_sidt flags[2];	// only version 0x0002
	ubyte_sidt reserved[4];	// only version 0x0002
};

const char _sidtune_format[] = "PlaySID one-file format (PSID)";
const char _sidtune_unknown[] = "Unsupported file format";
const char _sidtune_truncated[] = "ERROR: File is most likely truncated";

const int _sidtune_psid_maxStrLen = 31;


bool SidTune::PSID_fileSupport(const void* buffer, const udword_sidt bufLen)
{
	// Remove any format description or format error string.
	info.formatString = 0;

	// Require minimum size to allow access to the first few bytes.
	// Require a valid ID and version number.
	const psidHeader* pHeader = (const psidHeader*)buffer;
	if ( (bufLen<6) ||
		 (readBEdword((const ubyte_sidt*)pHeader->id)!=0x50534944) ||
		 (readBEword(pHeader->version)>=3) )
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

	fileOffset = readBEword(pHeader->data);
	info.loadAddr = readBEword(pHeader->load);
	info.initAddr = readBEword(pHeader->init);
	info.playAddr = readBEword(pHeader->play);
	info.songs = readBEword(pHeader->songs);
	info.startSong = readBEword(pHeader->start);
	info.sidChipBase1 = 0xd400;
	info.sidChipBase2 = 0;

	if (info.songs > SIDTUNE_MAX_SONGS)
	{
		info.songs = SIDTUNE_MAX_SONGS;
	}
	
	// Create the speed/clock setting table.
	convertOldStyleSpeedToTables(readBEdword(pHeader->speed));
	
	info.musPlayer = false;
	if ( readBEword(pHeader->version) >= 2 )
	{
		if (( readBEword(pHeader->flags) & 1 ) == 1 )
		{
			info.musPlayer = true;
		}
	}
  
	if ( info.loadAddr == 0 )
	{
		ubyte_sidt* pData = (ubyte_sidt*)buffer + fileOffset;
		info.loadAddr = readEndian( *(pData+1), *pData );
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


bool SidTune::PSID_fileSupportSave(ofstream& fMyOut, const ubyte_sidt* dataBuffer)
{
	psidHeader myHeader;
	writeBEdword((ubyte_sidt*)myHeader.id,0x50534944);  // 'PSID'
	writeBEword(myHeader.version,2);
	writeBEword(myHeader.data,sizeof(psidHeader));
	writeBEword(myHeader.load,0);
	writeBEword(myHeader.init,info.initAddr);
	writeBEword(myHeader.play,info.playAddr);
	writeBEword(myHeader.songs,info.songs);
	writeBEword(myHeader.start,info.startSong);

	udword_sidt speed = 0;
	int maxBugSongs = ((info.songs <= 32) ? info.songs : 32);
	for (int s = 0; s < maxBugSongs; s++)
	{
		if (songSpeed[s] == SIDTUNE_SPEED_CIA_1A)
		{
			speed |= (1<<s);
		}
	}
	writeBEdword(myHeader.speed,speed);

	uword_sidt tmpFlags = 0;
	if ( info.musPlayer )
	{
		tmpFlags |= 1;
	}
	writeBEword(myHeader.flags,tmpFlags);
	writeBEdword(myHeader.reserved,0);
	for ( int i = 0; i < 32; i++ )
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
	ubyte_sidt saveAddr[2];
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
