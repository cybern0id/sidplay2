//
// /home/ms/source/sidplay/libsidplay/fformat/RCS/psid_.cpp,v
//

#include "psid_.h"

const char text_format[] = "PlaySID one-file format (PSID)";
const char text_psidTruncated[] = "ERROR: PSID file is most likely truncated";

const int maxStringLength = 31;

// Denotes the first version of HVSC that was fully v2NG compatible (i.e. no
// garbage in the 32-bit 'reserved' field).
static const HVSCVER HVSCVersion_v2NGCompatible = MAKE_HVSCVER(3,9);

bool sidTune::PSID_fileSupport(const void* buffer, udword bufLen)
{
	// Remove any format description or format error string.
	info.formatString = 0;

	// Require a first minimum size.
	if (bufLen < 6)
	{
		return false;
	}
	// Now it is safe to access the first bytes.
	// Require a valid ID and version number.
	const psidHeader* pHeader = (const psidHeader*)buffer;
	if ( (readBEdword((const ubyte*)pHeader->id) != 0x50534944)  // "PSID" ?
		|| (readBEword(pHeader->version) >= 3) )
	{
		return false;
	}
	// Due to security concerns, input must be at least as long as version 1
	// plus C64 load address data. That is the area which will be accessed.
	if ( bufLen < (sizeof(psidHeader)+2) )
	{
		info.formatString = text_psidTruncated;
		return false;
	}

	fileOffset = readBEword(pHeader->data);
	info.loadAddr = readBEword(pHeader->load);
	info.initAddr = readBEword(pHeader->init);
	info.playAddr = readBEword(pHeader->play);
	info.songs = readBEword(pHeader->songs);
	info.startSong = readBEword(pHeader->start);

	if (info.songs > classMaxSongs)
	{
		info.songs = classMaxSongs;
	}

	// Create the speed/clock setting table.
	convertOldStyleSpeedToTables(readBEdword(pHeader->speed));

	info.musPlayer = false;
	info.psidSpecific = false;
	info.clock = SIDTUNE_CLOCK_UNKNOWN;
	info.sidModel = SIDTUNE_SIDMODEL_UNKNOWN;
	info.relocStartPage = 0;
	info.relocPages = 0;

	if ( readBEword(pHeader->version) >= 2 )
	{
		if (( readBEword(pHeader->flags) & 0x01 ) == 1 )
		{
			info.musPlayer = true;
		}

		if (( readBEword(pHeader->flags) & 0x02 ) != 0 )
		{
			info.psidSpecific = true;
		}

		info.clock = ( readBEword(pHeader->flags) & 0x0C ) >> 2;
		info.sidModel = ( readBEword(pHeader->flags) & 0x30 ) >> 4;

		info.relocStartPage = pHeader->relocStartPage[0];
		info.relocPages = pHeader->relocPages[0];
	}

	if ( info.loadAddr == 0 )
	{
		ubyte* pData = (ubyte*)buffer + fileOffset;
		info.loadAddr = readEndian( *(pData+1), *pData );
		fileOffset += 2;
	}
	if ( info.initAddr == 0 )
	{
		info.initAddr = info.loadAddr;
	}

	// Copy info strings, so they will not get lost.
	strncpy(&infoString[0][0],pHeader->name,maxStringLength);
	info.nameString = &infoString[0][0];
	info.infoString[0] = &infoString[0][0];
	strncpy(&infoString[1][0],pHeader->author,maxStringLength);
	info.authorString = &infoString[1][0];
	info.infoString[1] = &infoString[1][0];
	strncpy(&infoString[2][0],pHeader->copyright,maxStringLength);
	info.copyrightString = &infoString[2][0];
	info.infoString[2] = &infoString[2][0];
	info.numberOfInfoStrings = 3;

	info.formatString = text_format;
	return true;
}


bool sidTune::PSID_fileSupportSave(ofstream& fMyOut, const ubyte* dataBuffer)
{
	psidHeader myHeader;
	writeBEdword((ubyte*)myHeader.id,0x50534944);  // 'PSID'
	writeBEword(myHeader.version,2);
	writeBEword(myHeader.data,0x7C);
	writeBEword(myHeader.load,0);
	writeBEword(myHeader.init,info.initAddr);
	writeBEword(myHeader.play,info.playAddr);
	writeBEword(myHeader.songs,info.songs);
	writeBEword(myHeader.start,info.startSong);

	udword speed = 0;
	int maxBugSongs = ((info.songs <= 32) ? info.songs : 32);
	for (int s = 0; s < maxBugSongs; s++)
	{
		if (songSpeed[s] == SIDTUNE_SPEED_CIA_1A)
		{
			speed |= (1<<s);
		}
	}
	writeBEdword(myHeader.speed,speed);

	uword tmpFlags = 0;
	if ( info.musPlayer )
	{
		tmpFlags |= 1;
	}

	// These fields exist in v2NG only.
	if (hvscvercmp(HVSCversion_found,HVSCVersion_v2NGCompatible)>=0) {
		if ( info.psidSpecific )
		{
			tmpFlags |= 2;
		}

		tmpFlags |= (info.clock << 2);
		tmpFlags |= (info.sidModel << 4);
	}

	writeBEword(myHeader.flags,tmpFlags);

	// If the version of HVSC we are updating is one that is not
	// fully v2NG compatible (i.e. 32-bit v2 'reserved' field may
	// contain garbage), then the below two fields are zeroed out.

	if (hvscvercmp(HVSCversion_found,HVSCVersion_v2NGCompatible)<0) {
		myHeader.relocStartPage[0] = 0;
		myHeader.relocPages[0] = 0;
	}
	else {
		myHeader.relocStartPage[0] = info.relocStartPage;
		myHeader.relocPages[0] = info.relocPages;
	}

	writeBEword(myHeader.reserved,0);
	for ( int i = 0; i < 32; i++ )
	{
		myHeader.name[i] = 0;
		myHeader.author[i] = 0;
		myHeader.copyright[i] = 0;
	}
	strncpy( myHeader.name, info.nameString, 31 );
	strncpy( myHeader.author, info.authorString, 31 );
	strncpy( myHeader.copyright, info.copyrightString, 31 );
	fMyOut.write( (char*)&myHeader, sizeof(psidHeader) );

	// Save C64 lo/hi load address (little-endian).
	ubyte saveAddr[2];
	saveAddr[0] = info.loadAddr & 255;
	saveAddr[1] = info.loadAddr >> 8;
	fMyOut.write( (char*)saveAddr, 2 );
	// Data starts at: bufferaddr + fileoffset
	// Data length: datafilelen - fileoffset
	fMyOut.write( (char*)dataBuffer + fileOffset, info.dataFileLen - fileOffset );
	if ( !fMyOut )
	{
		return false;
	}
	else
	{
		return true;
	}
}
