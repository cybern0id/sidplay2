/*
 * A basic WAV output file type - Implementation.
 * Initial implementation by Michael Schwendt <mschwendt@yahoo.com>
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 *  Revision 1.4  2000/12/11 19:07:42  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

#include <stdlib.h>
#include <sidplay/sidendian.h>
#include "WavFile.h"
#include "WavFileDefs.h"

#ifdef WAV_HAVE_EXCEPTIONS
#include <new>
#endif


const wavHeader WavFile::defaultWavHdr = {
    // ASCII keywords are hex-ified.
    {0x52,0x49,0x46,0x46}, {0,0,0,0}, {0x57,0x41,0x56,0x45},
    {0x66,0x6d,0x74,0x20}, {16,0,0,0},
    {1,0}, {0,0}, {0,0,0,0}, {0,0,0,0}, {0,0}, {0,0},
    {0x64,0x61,0x74,0x61}, {0,0,0,0}
};

WavFile::WavFile()
: wavHdr(defaultWavHdr)
{
    isOpen = headerWritten = false;
}

void* WavFile::open(AudioConfig &cfg, const char* name,
                    const bool overWrite)
{
    if (isOpen && !file.fail())
        close();

    unsigned long  int freq;
    unsigned short int channels, bits;
    unsigned short int blockAlign;

    byteCount    = 0;
    channels     = cfg.channels;
    freq         = cfg.frequency;
    bits         = cfg.precision;
    blockAlign   = (bits>>3)*channels;
    // Rev 1.3 (saw) - Buffersize changed to 1 second
    bufSize      = cfg.frequency * blockAlign;

    // We need to make a buffer for the user
#if defined(WAV_HAVE_EXCEPTIONS)
    _sampleBuffer = new(nothrow) uint_least8_t[bufSize];
#else
    _sampleBuffer = new uint_least8_t[bufSize];
#endif
    if (!_sampleBuffer)
        return 0;

    // Fill in header with parameters and expected file size.
    endian_little32(wavHdr.length,sizeof(wavHeader)-8);
    endian_little16(wavHdr.channels,channels);
    endian_little32(wavHdr.sampleFreq,freq);
    endian_little32(wavHdr.bytesPerSec,freq*blockAlign);
    endian_little16(wavHdr.blockAlign,blockAlign);
    endian_little16(wavHdr.bitsPerSample,bits);
    endian_little32(wavHdr.dataChunkLen,0);

       if (overWrite)
    {
#if defined(WAV_HAVE_IOS_BIN)
        file.open( name, ios::out|ios::bin|ios::trunc );
#else
        file.open( name, ios::out|ios::binary|ios::trunc );
#endif
    }
    else
    {
#if defined(WAV_HAVE_IOS_BIN)
        file.open( name, ios::out|ios::bin|ios::noreplace );
#else
        file.open( name, ios::out|ios::binary|ios::noreplace );
#endif
    }
    
    isOpen = !file.fail();  // good->true, bad->false
    cfg.bufSize = bufSize;
    _settings = cfg;
    return _sampleBuffer;
}

void* WavFile::write(unsigned long int bytes)
{
    if (isOpen && !file.fail())
    {
        if (!headerWritten)
        {
            file.write((char*)&wavHdr,sizeof(wavHeader));
            headerWritten = true;
        }
        
        // This should never happen
        if (bytes > bufSize)
            bytes = bufSize;

        byteCount += bytes;
#if defined(WAV_WORDS_BIGENDIAN)
        if (bitsPerSample == 16)
        {
            _sampleBuffer = bytes;
                    endian_32swap (_sampleBuffer);
        }
#endif
        file.write((char*)_sampleBuffer,bytes);
#if defined(WAV_WORDS_BIGENDIAN) && defined(WAV_REVERT_BUFFER_CHANGES)
        if (bitsPerSample == 16)
        {
            _sampleBuffer = bytes;
                    endian_32swap (_sampleBuffer);
        }
#endif
    }
    return _sampleBuffer;
}

void WavFile::close()
{
    if (isOpen && !file.fail())
    {
        endian_little32(wavHdr.length,byteCount+sizeof(wavHeader)-8);
        endian_little32(wavHdr.dataChunkLen,byteCount);
        file.seekp(0,ios::beg);
        file.write((char*)&wavHdr,sizeof(wavHeader));
        file.close();
        isOpen = false;
        delete [] (int_least8_t *) _sampleBuffer;
    }
}
