/*
 * /home/ms/files/source/libsidtune/RCS/InfoFile.cpp,v
 *
 * SIDPLAY INFOFILE format support.
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

#include "config.h"
#include "SidTune.h"
#include "SidTuneTools.h"
#include "sidendian.h"

#ifdef HAVE_EXCEPTIONS
#include <new>
#endif
#include <fstream.h>
#include <iostream.h>
#include <iomanip.h>
#if defined(HAVE_STRSTREA_H)
  #include <strstrea.h>
#else
  #include <strstream.h>
#endif
#include <ctype.h>
#include <string.h>

const char text_format[] = "Raw plus SIDPLAY ASCII text file (SID)";

const char text_truncatedError[] = "ERROR: SID file is truncated";
const char text_noMemError[] = "ERROR: Not enough free memory";

const char keyword_id[] = "SIDPLAY INFOFILE";

const char keyword_name[] = "NAME=";            // No white-space characters 
const char keyword_author[] = "AUTHOR=";        // in these keywords, because
const char keyword_copyright[] = "COPYRIGHT=";  // we want to use a white-space
const char keyword_address[] = "ADDRESS=";      // eating string stream to
const char keyword_songs[] = "SONGS=";          // parse most of the header.
const char keyword_speed[] = "SPEED=";
const char keyword_musPlayer[] = "SIDSONG=YES";

const uint_least16_t sidMinFileSize = 1+sizeof(keyword_id);  // Just to avoid a first segm.fault.
const uint_least16_t parseChunkLen = 80;                     // Enough for all keywords incl. their values.


bool SidTune::SID_fileSupport(const void* dataBuffer, uint_least32_t dataBufLen,
                              const void* sidBuffer, uint_least32_t sidBufLen)
{
	// Remove any format description or error string.
	info.formatString = 0;
	
	// Make sure SID buffer pointer is not zero.
	// Check for a minimum file size. If it is smaller, we will not proceed.
	if ((sidBuffer==0) || (sidBufLen<sidMinFileSize))
	{
		return false;
	}

	const char* pParseBuf = (const char*)sidBuffer;
	// First line has to contain the exact identification string.
	if ( SidTuneTools::myStrNcaseCmp( pParseBuf, keyword_id ) != 0 )
	{
		return false;
	}
	else
	{
		// At least the ID was found, so set a default error message.
		info.formatString = text_truncatedError;
		
		// Defaults.
		fileOffset = 0;                // no header in separate data file
		info.sidChipBase1 = 0xd400;
		info.sidChipBase2 = 0;
		info.musPlayer = false;
		info.numberOfInfoStrings = 0;
		uint_least32_t oldStyleSpeed = 0;

		// Flags for required entries.
		bool hasAddress = false,
		    hasName = false,
		    hasAuthor = false,
		    hasCopyright = false,
		    hasSongs = false,
		    hasSpeed = false;
	
		// Using a temporary instance of an input string chunk.
#ifdef HAVE_EXCEPTIONS
		char* pParseChunk = new(nothrow) char[parseChunkLen+1];
#else
		char* pParseChunk = new char[parseChunkLen+1];
#endif
		if ( pParseChunk == 0 )
		{
			info.formatString = text_noMemError;
			return false;
		}
		
		// Parse as long we have not collected all ``required'' entries.
		while ( !hasAddress || !hasName || !hasAuthor || !hasCopyright
			    || !hasSongs || !hasSpeed )
		{
			// Skip to next line. Leave loop, if none.
			if (( pParseBuf = SidTuneTools::returnNextLine( pParseBuf )) == 0 )
			{
				break;
			}
			// And get a second pointer to the following line.
			const char* pNextLine = SidTuneTools::returnNextLine( pParseBuf );
			uint_least32_t restLen;
			if ( pNextLine != 0 )
			{
				// Calculate number of chars between current pos and next line.
				restLen = (uint_least32_t)(pNextLine - pParseBuf);
			}
			else
			{
				// Calculate number of chars between current pos and end of buf.
				restLen = sidBufLen - (uint_least32_t)(pParseBuf - (char*)sidBuffer);
			}
			// Create whitespace eating (!) input string stream.
			istrstream parseStream((char *) pParseBuf, restLen );
			// A second one just for copying.
			istrstream parseCopyStream((char *) pParseBuf, restLen );
			if ( !parseStream || !parseCopyStream )
			{
				break;
			}
			// Now copy the next X characters except white-spaces.
			for ( uint_least16_t i = 0; i < parseChunkLen; i++ )
			{
				char c;
				parseCopyStream >> c;
				pParseChunk[i] = c;
			}
			pParseChunk[parseChunkLen]=0;
			// Now check for the possible keywords.
			// ADDRESS
			if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_address ) == 0 )
			{
				SidTuneTools::skipToEqu( parseStream );
				info.loadAddr = (uint_least16_t)SidTuneTools::readHex( parseStream );
				if ( !parseStream )
				    break;
				info.initAddr = (uint_least16_t)SidTuneTools::readHex( parseStream );
				if ( !parseStream )
				    break;
				info.playAddr = (uint_least16_t)SidTuneTools::readHex( parseStream );
				hasAddress = true;
			}
			// NAME
			else if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_name ) == 0 )
			{
				SidTuneTools::copyStringValueToEOL(pParseBuf,&infoString[0][0],SIDTUNE_MAX_CREDIT_STRLEN);
				info.infoString[0] = &infoString[0][0];
				hasName = true;
			}
			// AUTHOR
			else if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_author ) == 0 )
			{
				SidTuneTools::copyStringValueToEOL(pParseBuf,&infoString[1][0],SIDTUNE_MAX_CREDIT_STRLEN);
				info.infoString[1] = &infoString[1][0];
				hasAuthor = true;
			}
			// COPYRIGHT
			else if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_copyright ) == 0 )
			{
				SidTuneTools::copyStringValueToEOL(pParseBuf,&infoString[2][0],SIDTUNE_MAX_CREDIT_STRLEN);
				info.infoString[2] = &infoString[2][0];
				hasCopyright = true;
			}
			// SONGS
			else if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_songs ) == 0 )
			{
				SidTuneTools::skipToEqu( parseStream );
				info.songs = (uint_least16_t)SidTuneTools::readDec( parseStream );
				info.startSong = (uint_least16_t)SidTuneTools::readDec( parseStream );
				hasSongs = true;
			}
			// SPEED
			else if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_speed ) == 0 )
			{
				SidTuneTools::skipToEqu( parseStream );
				oldStyleSpeed = SidTuneTools::readHex(parseStream);
				hasSpeed = true;
			}
			// SIDSONG
			else if ( SidTuneTools::myStrNcaseCmp( pParseChunk, keyword_musPlayer ) == 0 )
			{
				info.musPlayer = true;
			}
        };
		
        delete[] pParseChunk;
		
		// Again check for the ``required'' values.
		if ( hasAddress || hasName || hasAuthor || hasCopyright || hasSongs || hasSpeed )
		{
			// Create the speed/clock setting table.
			convertOldStyleSpeedToTables(oldStyleSpeed);
			// loadAddr = 0 means, the address is stored in front of the C64 data.
			// We cannot verify whether the dataBuffer contains valid data.
		    // All we want to know is whether the SID buffer is valid.
			// If data is present, we access it (here to get the C64 load address).
			if (info.loadAddr==0 && (dataBufLen>=(fileOffset+2)) && dataBuffer!=0)
			{
				const uint8_t* pDataBufCp = (const uint8_t*)dataBuffer + fileOffset;
				info.loadAddr = endian_16( *(pDataBufCp + 1), *pDataBufCp );
				fileOffset += 2;  // begin of data
			}
			// Keep compatibility to PSID/SID.
			if ( info.initAddr == 0 )
			{
				info.initAddr = info.loadAddr;
			}
			info.numberOfInfoStrings = 3;
			// We finally accept the input data.
			info.formatString = text_format;
			return true;
		}
		else
		{
			// Something is missing (or damaged ?).
			// Error string set above.
			return false;
		}
	}
}


bool SidTune::SID_fileSupportSave( ofstream& fMyOut )
{
	fMyOut << keyword_id << endl
		<< keyword_address << hex << setw(4) << setfill('0') << 0 << ','
		<< hex << setw(4) << info.initAddr << ","
		<< hex << setw(4) << info.playAddr << endl
		<< keyword_songs << dec << (int)info.songs << "," << (int)info.startSong << endl;

	uint_least32_t oldStyleSpeed = 0;
	int maxBugSongs = ((info.songs <= 32) ? info.songs : 32);
	for (int s = 0; s < maxBugSongs; s++)
	{
		if (songSpeed[s] == SIDTUNE_SPEED_CIA_1A)
		{
			oldStyleSpeed |= (1<<s);
		}
	}

	fMyOut
		<< keyword_speed << hex << setw(8) << oldStyleSpeed << endl
		<< keyword_name << info.infoString[0] << endl
		<< keyword_author << info.infoString[1] << endl
		<< keyword_copyright << info.infoString[2] << endl;
	if ( info.musPlayer )
	{
		fMyOut << keyword_musPlayer << endl;
	}
	if ( !fMyOut )
	{
		return false;
	}
	else
	{
		return true;
	}
}
