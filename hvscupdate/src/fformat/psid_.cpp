//
// /home/ms/source/sidplay/libsidplay/fformat/RCS/psid_.cpp,v
//

#include "psid_.h"

#define PSID_ID 0x50534944
#define RSID_ID 0x52534944
#define SIDTUNE_PSID2NG

static const char _sidtune_format_psid[] = "PlaySID one-file format (PSID)";
static const char _sidtune_format_rsid[] = "Real C64 one-file format (RSID)";
static const char _sidtune_unknown[] = "Unsupported file format";
static const char _sidtune_unknown_psid[] = "Unsupported PSID version";
static const char _sidtune_unknown_rsid[] = "Unsupported RSID version";
static const char _sidtune_truncated[] = "ERROR: File is most likely truncated";
static const char _sidtune_invalid[] = "ERROR: File contains invalid data";
static const char _sidtune_reloc[] = "ERROR: File contains bad reloc data";

const int _sidtune_psid_maxStrLen = 31;

// Denotes the first version of HVSC that was fully v2NG compatible (i.e. no
// garbage in the 32-bit 'reserved' field).
static const HVSCVER HVSCVersion_v2NGCompatible = MAKE_HVSCVER(3,9);

bool sidTune::PSID_fileSupport(const void* buffer, udword bufLen)
{
	int clock, compatibility;
	udword speed;
#ifdef SIDTUNE_PSID2NG
	clock = SIDTUNE_CLOCK_UNKNOWN;
#else
	clock = info.clockSpeed;
#endif
	compatibility = SIDTUNE_COMPATIBILITY_C64;

	// Require minimum size to allow access to the first few bytes.
	// Require a valid ID and version number.
	const psidHeader* pHeader = (const psidHeader*)buffer;

	// Remove any format description or format error string.
	info.formatString = 0;

	// File format check
	if (bufLen<6)
		return false;
	if (readBEdword((const ubyte*)pHeader->id)==PSID_ID)
	{
	   if (readBEword(pHeader->version) >= 3)
	   {
		   info.formatString = _sidtune_unknown_psid;
		   return false;
	   }
	   info.formatString = _sidtune_format_psid;
	}
	else if (readBEdword((const ubyte*)pHeader->id)==RSID_ID)
	{
	   if (readBEword(pHeader->version) != 2)
	   {
		   info.formatString = _sidtune_unknown_rsid;
		   return false;
	   }
	   info.formatString = _sidtune_format_rsid;
	   compatibility = SIDTUNE_COMPATIBILITY_R64;
	}
	else
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

	fileOffset         = readBEword(pHeader->data);
	info.loadAddr      = readBEword(pHeader->load);
	info.initAddr      = readBEword(pHeader->init);
	info.playAddr      = readBEword(pHeader->play);
	info.songs         = readBEword(pHeader->songs);
	info.startSong     = readBEword(pHeader->start);
	info.compatibility = compatibility;
	speed              = readBEdword(pHeader->speed);

	if (info.songs > classMaxSongs)
	{
		info.songs = classMaxSongs;
	}

	info.musPlayer      = false;
	info.sidModel       = SIDTUNE_SIDMODEL_UNKNOWN;
	info.relocPages     = 0;
	info.relocStartPage = 0;
	if ( readBEword(pHeader->version) >= 2 )
	{
		uword flags = readBEword(pHeader->flags);
		if (flags & PSID_MUS)
		{	// MUS tunes run at any speed
			clock = SIDTUNE_CLOCK_ANY;
			info.musPlayer = true;
		}

#ifdef SIDTUNE_PSID2NG
		if (flags & PSID_SPECIFIC)
			info.compatibility = SIDTUNE_COMPATIBILITY_PSID;

		if (flags & PSID_CLOCK_PAL)
			clock |= SIDTUNE_CLOCK_PAL;
		if (flags & PSID_CLOCK_NTSC)
			clock |= SIDTUNE_CLOCK_NTSC;
		info.clockSpeed = clock;

		info.sidModel = SIDTUNE_SIDMODEL_UNKNOWN;
		if (flags & PSID_SIDMODEL_6581)
			info.sidModel |= SIDTUNE_SIDMODEL_6581;
		if (flags & PSID_SIDMODEL_8580)
			info.sidModel |= SIDTUNE_SIDMODEL_8580;

		info.relocStartPage = pHeader->relocStartPage[0];
		info.relocPages	 = pHeader->relocPages[0];
#endif // SIDTUNE_PSID2NG
	}

	// Check reserved fields to force real c64 compliance
	if (compatibility == SIDTUNE_COMPATIBILITY_R64)
	{
		if (checkRealC64Info (speed) == false)
		{
			info.formatString = _sidtune_invalid;
			return false;
		}
		// Real C64 tunes appear as CIA
		speed = ~0;
	}
	// Create the speed/clock setting table.
	convertOldStyleSpeedToTables(speed, clock);

	if ( info.loadAddr == 0 )
	{
		ubyte* pData  = (ubyte*)buffer + fileOffset;
		info.loadAddr = readEndian( *(pData+1), *pData );
		fileOffset += 2;
	}

	// Obtain C64 data length now file header is fully
	// extracted
	info.c64dataLen = bufLen - fileOffset;

	if ( info.compatibility == SIDTUNE_COMPATIBILITY_R64 )
	{
		// Check tune is loadable on a real C64
		if ( info.loadAddr < 0x07e8 )
		{
			info.formatString = _sidtune_invalid;
			return false;
		}

		if (checkRealC64Init() == false)
		{
			info.formatString = _sidtune_invalid;
			return false;
		}
	}
	else if ( info.initAddr == 0 )
		info.initAddr = info.loadAddr;

	if ( checkRelocInfo() == false )
	{
		info.formatString = _sidtune_reloc;
		return false;
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
	return true;
}


bool sidTune::PSID_fileSupportSave(ofstream& fMyOut, const ubyte* dataBuffer)
{
	psidHeader myHeader;
	writeBEdword((ubyte*)myHeader.id,PSID_ID);
	writeBEword(myHeader.version,2);
	writeBEword(myHeader.data,sizeof(psidHeader));
	writeBEword(myHeader.load,0);
	writeBEword(myHeader.init,info.initAddr);
	writeBEword(myHeader.play,info.playAddr);
	writeBEword(myHeader.songs,info.songs);
	writeBEword(myHeader.start,info.startSong);

	udword speed = 0;
	udword maxBugSongs = ((info.songs <= 32) ? info.songs : 32);
	for (int s = 0; s < maxBugSongs; s++)
	{
		if (songSpeed[s] == SIDTUNE_SPEED_CIA_1A)
			speed |= (1<<s);
	}
	writeBEdword(myHeader.speed,speed);

	uword tmpFlags = 0;
	if ( info.musPlayer )
		tmpFlags |= PSID_MUS;

	// These fields exist in v2NG only.
	if (hvscvercmp(HVSCversion_found,HVSCVersion_v2NGCompatible)>=0)
	{
		if (info.compatibility == SIDTUNE_COMPATIBILITY_PSID)
			tmpFlags |= PSID_SPECIFIC;

		tmpFlags |= (info.clockSpeed << 2);
		tmpFlags |= (info.sidModel << 4);
	}

	writeBEword(myHeader.flags,tmpFlags);

	// If the version of HVSC we are updating is one that is not
	// fully v2NG compatible (i.e. 32-bit v2 'reserved' field may
	// contain garbage), then the below two fields are zeroed out.

	if (hvscvercmp(HVSCversion_found,HVSCVersion_v2NGCompatible)<0)
	{
		myHeader.relocStartPage[0] = 0;
		myHeader.relocPages[0] = 0;
	}
	else
	{
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
	strncpy( myHeader.name, info.infoString[0], _sidtune_psid_maxStrLen);
	strncpy( myHeader.author, info.infoString[1], _sidtune_psid_maxStrLen);
	strncpy( myHeader.copyright, info.infoString[2], _sidtune_psid_maxStrLen);

	if (info.compatibility == SIDTUNE_COMPATIBILITY_R64)
	{
		writeBEdword((ubyte*)myHeader.id,RSID_ID);
		writeBEword(myHeader.play,0);
		writeBEdword(myHeader.speed,0);
	}

	fMyOut.write( (char*)&myHeader, sizeof(psidHeader) );
	
	// Save C64 lo/hi load address (little-endian).
	ubyte saveAddr[2];
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
