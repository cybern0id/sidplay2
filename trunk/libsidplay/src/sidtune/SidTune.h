/*
 * /home/ms/files/source/libsidtune/RCS/SidTune.h,v
 *
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

#ifndef SIDTUNE_H
#define SIDTUNE_H

#include "SidTuneTypes.h"
#include "SidTuneEndian.h"
#include "Buffer.h"
#include "SmartPtr.h"

class ofstream;  // <fstream.h>

const uword_sidt SIDTUNE_MAX_SONGS = 256;
// Also PSID file format limit.

const uword_sidt SIDTUNE_MAX_CREDIT_STRINGS = 10;
const uword_sidt SIDTUNE_MAX_CREDIT_STRLEN = 80+1;
// 80 characters plus terminating zero.

const udword_sidt SIDTUNE_MAX_MEMORY = 65536;
const udword_sidt SIDTUNE_MAX_FILELEN = 65536+2+0x7C;
// C64KB+LOAD+PSID

const int SIDTUNE_SPEED_VBI = 0;		// Vertical-Blanking-Interrupt
const int SIDTUNE_SPEED_CIA_1A = 60;	// CIA 1 Timer A

const int SIDTUNE_CLOCK_PAL = 0;		// These are also used in the
const int SIDTUNE_CLOCK_NTSC = 1;		// emulator engine!

struct SidTuneInfo
{
	// An instance of this structure is used to transport values to
	// and from SidTune objects.

	// You must read (i.e. activate) sub-song specific information
	// via:
	// 		const SidTuneInfo& tuneInfo = SidTune[songNumber];
	//		const SidTuneInfo& tuneInfo = SidTune.getInfo();
	//		void SidTune.getInfo(tuneInfo&);
	
	// Consider the following fields as read-only, because the SidTune class
	// does not provide an implementation of: bool setInfo(const SidTuneInfo&).
	// Currently, the only way to get the class to accept values which
	// are written to these fields is by creating a derived class.

	const char* formatString;   // the name of the identified file format
	const char* statusString;   // error/status message of last operation
	
	const char* speedString;	// describing the speed a song is running at
	
	uword_sidt loadAddr;
	uword_sidt initAddr;
	uword_sidt playAddr;
	
	uword_sidt songs;
	uword_sidt startSong;
	
	// The SID chip base address(es) used by the sidtune.
	uword_sidt sidChipBase1;	// 0xD400 (normal, 1st SID)
	uword_sidt sidChipBase2;	// 0xD?00 (2nd SID) or 0 (no 2nd SID)

	// Available after song initialization.
	//
	uword_sidt irqAddr;			// if (playAddr == 0), interrupt handler has been
								// installed and starts calling the C64 player
								// at this address
	uword_sidt currentSong;		// the one that has been initialized
	ubyte_sidt songSpeed;		// intended speed, see top
	ubyte_sidt clockSpeed;		// -"-
	bool musPlayer;				// whether Sidplayer routine has been installed
	bool fixLoad;				// whether load address might be duplicate
	uword_sidt songLength;		// --- not yet supported ---
	//
	// Song title, credits, ...
	// 0 = Title, 1 = Author, 2 = Copyright/Publisher
	//
	ubyte_sidt numberOfInfoStrings;  // the number of available text info lines
	char* infoString[SIDTUNE_MAX_CREDIT_STRINGS];
	//
	uword_sidt numberOfCommentStrings;	// --- not yet supported ---
	char ** commentString;				// --- not yet supported ---
	//
	udword_sidt dataFileLen;	// length of single-file sidtune file
	udword_sidt c64dataLen;		// length of raw C64 data without load address
	char* path;					// path to sidtune files; "", if cwd
	char* dataFileName;			// a first file: e.g. "foo.c64"; "", if none
	char* infoFileName;			// a second file: e.g. "foo.sid"; "", if none
	//
};


class SidTune
{
	
 public:  // ----------------------------------------------------------------

	// If your opendir() and readdir()->d_name return path names
	// that contain the forward slash (/) as file separator, but
	// your operating system uses a different character, there are
	// extra functions that can deal with this special case. Set
	// separatorIsSlash to true if you like path names to be split
	// correctly.
	// You do not need these extra functions if your systems file
	// separator is the forward slash.
	//
	// Load a sidtune from a file.
	//
	// To retrieve data from standard input pass in filename "-".
	// If you want to override the default filename extensions use this
	// contructor. Please note, that if the specified ``sidTuneFileName''
	// does exist and the loader is able to determine its file format,
	// this function does not try to append any file name extension.
	// See ``sidtune.cpp'' for the default list of file name extensions.
	// You can specific ``sidTuneFileName = 0'', if you do not want to
	// load a sidtune. You can later load one with open().
	SidTune(const char* fileName, const char **fileNameExt = 0,
			const bool separatorIsSlash = false);

	// Load a single-file sidtune from a memory buffer.
	// Currently supported: PSID format
	SidTune(const ubyte_sidt* oneFileFormatSidtune, const udword_sidt sidtuneLength);

	virtual ~SidTune();

	// The sidTune class does not copy the list of file name extensions,
	// so make sure you keep it. If the provided pointer is 0, the
	// default list will be activated. This is a static list which
	// is used by all SidTune objects.
	void setFileNameExtensions(const char **fileNameExt);
	
	// Load a sidtune into an existing object.
	// From a file.
	bool load(const char* sidTuneFileName, const bool separatorIsSlash = false);
	
	// From a buffer.
	bool read(const ubyte_sidt* sourceBuffer, const udword_sidt bufferLen);

	// Select sub-song (0 = default starting song)
	// and retrieve active song information.
	const SidTuneInfo& operator[](const uword_sidt songNum);

	// Select sub-song (0 = default starting song)
	// and return active song number out of [1,2,..,SIDTUNE_MAX_SONGS].
	uword_sidt selectSong(const uword_sidt songNum);
	
	// Retrieve sub-song specific information.
	// Beware! Still member-wise copy!
	const SidTuneInfo& getInfo();
	
	// Get a copy of sub-song specific information.
	// Beware! Still member-wise copy!
	void getInfo(SidTuneInfo&);

	// Determine current state of object (true = okay, false = error).
	// Upon error condition use ``getInfo'' to get a descriptive
	// text string in ``SidTuneInfo.statusString''.
	operator bool()  { return status; }
	bool getStatus()  { return status; }

	// Whether sidtune uses two SID chips.
	bool isStereo()
	{
		return (info.sidChipBase1!=0 && info.sidChipBase2!=0);
	}
	
	// Copy sidtune into C64 memory (64 KB).
	bool placeSidTuneInC64mem(ubyte_sidt* c64buf);

	// --- file save & format conversion ---

	// These functions work for any successfully created object.
	// overWriteFlag: true  = Overwrite existing file.
	//				  false = Default, return error when file already
	//						  exists.
	// One could imagine an "Are you sure ?"-checkbox before overwriting
	// any file.
	// returns: true = Successful, false = Error condition.
	bool saveC64dataFile( const char* destFileName, const bool overWriteFlag = false );
	bool saveSIDfile( const char* destFileName, const bool overWriteFlag = false );
	bool savePSIDfile( const char* destFileName, const bool overWriteFlag = false );

	// This function can be used to remove a duplicate C64 load address in
	// the C64 data (example: FE 0F 00 10 4C ...). A duplicate load address
	// of offset 0x02 is indicated by the ``fixLoad'' flag in the SidTuneInfo
	// structure.
	//
	// The ``force'' flag here can be used to remove the first load address
	// and set new INIT/PLAY addresses regardless of whether a duplicate
	// load address has been detected and indicated by ``fixLoad''.
	// For instance, some position independent sidtunes contain a load address
	// of 0xE000, but are loaded to 0x0FFE and call the player code at 0x1000.
	//
	// Do not forget to save the sidtune file.
	void fixLoadAddress(const bool force = false, uword_sidt initAddr = 0,
						uword_sidt playAddr = 0);

	// Does not affect status of object, and therefore can be used
	// to load files. Error string is put into info.statusString, though.
	bool loadFile(const char* fileName, Buffer_sidtt<const ubyte_sidt>& bufferRef);
	
	bool saveToOpenFile( ofstream& toFile, const ubyte_sidt* buffer, udword_sidt bufLen );
	
 protected:  // -------------------------------------------------------------

	SidTuneInfo info;
	bool status;

	ubyte_sidt songSpeed[SIDTUNE_MAX_SONGS];
	ubyte_sidt clockSpeed[SIDTUNE_MAX_SONGS];
	uword_sidt songLength[SIDTUNE_MAX_SONGS];

	// holds text info from the format headers etc.
	char infoString[SIDTUNE_MAX_CREDIT_STRINGS][SIDTUNE_MAX_CREDIT_STRLEN];

	// See instructions at top.
	bool isSlashedFileName;

	// For files with header: offset to real data
	udword_sidt fileOffset;

	// Needed for MUS/STR player installation.
	uword_sidt musDataLen;
	
	Buffer_sidtt<const ubyte_sidt> cache;

	// Filename extensions to append for various file types.
	static const char** fileNameExtensions;

	// --- protected member functions ---

	// Convert 32-bit PSID-style speed word to internal tables.
	void convertOldStyleSpeedToTables(udword_sidt oldStyleSpeed);

	// Support for various file formats.

	virtual bool PSID_fileSupport(const void* buffer, const udword_sidt bufLen);
	virtual bool PSID_fileSupportSave(ofstream& toFile, const ubyte_sidt* dataBuffer);

	virtual bool SID_fileSupport(const void* dataBuffer, udword_sidt dataBufLen,
								 const void* sidBuffer, udword_sidt sidBufLen);
	virtual bool SID_fileSupportSave(ofstream& toFile);

	virtual bool MUS_fileSupport(Buffer_sidtt<const ubyte_sidt>& musBufRef,
								 Buffer_sidtt<const ubyte_sidt>& strBufRef);
	virtual bool MUS_detect(const void* buffer, const udword_sidt bufLen,
							udword_sidt& voice3Index);
	virtual bool MUS_mergeParts(Buffer_sidtt<const ubyte_sidt>& musBufRef,
								Buffer_sidtt<const ubyte_sidt>& strBufRef);
	virtual void MUS_setPlayerAddress();
	virtual void MUS_installPlayer(ubyte_sidt *c64buf);
	virtual int MUS_decodePetLine(SmartPtr_sidtt<const ubyte_sidt>&, char*);

	virtual bool INFO_fileSupport(const void* dataBuffer, udword_sidt dataBufLen,
								  const void* infoBuffer, udword_sidt infoBufLen);

    // Error and status message strings.
    static const char* txt_songNumberExceed;
    static const char* txt_empty;
    static const char* txt_unrecognizedFormat;
    static const char* txt_noDataFile;
    static const char* txt_notEnoughMemory;
    static const char* txt_cantLoadFile;
    static const char* txt_cantOpenFile;
    static const char* txt_fileTooLong;
    static const char* txt_dataTooLong;
    static const char* txt_cantCreateFile;
    static const char* txt_fileIoError;
    static const char* txt_PAL_VBI;
    static const char* txt_PAL_CIA;
    static const char* txt_NTSC_VBI;
    static const char* txt_NTSC_CIA;
    static const char* txt_noErrors;
    static const char* txt_na;

 private:  // ---------------------------------------------------------------
	
	void init();
	void cleanup();
#if !defined(SIDTUNE_NO_STDIN_LOADER)
	void getFromStdIn();
#endif
	void getFromFiles(const char* name);
	
	void deleteFileNameCopies();
	
	// Try to retrieve single-file sidtune from specified buffer.
	void getFromBuffer(const ubyte_sidt* const buffer, const udword_sidt bufferLen);
	
	// Cache the data of a single-file or two-file sidtune and its
	// corresponding file names.
	bool acceptSidTune(const char* dataFileName, const char* infoFileName,
					   Buffer_sidtt<const ubyte_sidt>& buf);

	bool createNewFileName(Buffer_sidtt<char>& destString,
						   const char* sourceName, const char* sourceExt);

 private:	// prevent copying
	SidTune(const SidTune&);
	SidTune& operator=(SidTune&);  

};
	
#endif  /* SIDTUNE_H */
