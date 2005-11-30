/*
 * A basic iRAW output file type - Implementation.
 * Initial implementation by Simon White <sidplay2@yahoo.com>
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
 ***************************************************************************/

#include <stdlib.h>
#include <sidplay/sidendian.h>
#include "RawFile.h"

#ifdef WAV_HAVE_EXCEPTIONS
#   include <new>
#endif

#if defined(HAVE_IOS_OPENMODE)
    typedef std::ios::openmode openmode;
#else
    typedef int openmode;
#endif


RawFile::RawFile()
{
    isOpen = false;
}

void* RawFile::open(AudioConfig &cfg, const char* name,
                    const bool overWrite)
{
    unsigned long  int freq;
    unsigned short int channels, bits;
    unsigned short int blockAlign;
    unsigned long  int bufSize;

    bits        = cfg.precision;
    channels    = cfg.channels;
    freq        = cfg.frequency;
    blockAlign  = (bits>>3)*channels;
    bufSize     = freq * blockAlign;
    cfg.bufSize = bufSize;

    // Setup Encoding
    cfg.encoding = AUDIO_SIGNED_PCM;
    if (bits == 8)
        cfg.encoding = AUDIO_UNSIGNED_PCM;

    if (name == NULL)
        return NULL;

    if (isOpen && !file.fail())
        close();
   
    byteCount = 0;

    // We need to make a buffer for the user
#if defined(WAV_HAVE_EXCEPTIONS)
    _sampleBuffer = new(std::nothrow) uint_least8_t[bufSize];
#else
    _sampleBuffer = new uint_least8_t[bufSize];
#endif
    if (!_sampleBuffer)
        return NULL;

    openmode createAttr = std::ios::out;
#if defined(WAV_HAVE_IOS_BIN)
    createAttr |= std::ios::bin;
#else
    createAttr |= std::ios::binary;
#endif

    if (overWrite)
        file.open( name, createAttr|std::ios::trunc );
    else
        file.open( name, createAttr|std::ios::app );

    isOpen = !(file.fail() || file.tellp());
    _settings = cfg;
    return _sampleBuffer;
}

void* RawFile::write()
{
    if (isOpen && !file.fail())
    {
        unsigned long int bytes = _settings.bufSize;
        byteCount += bytes;
        file.write((char*)_sampleBuffer,bytes);
    }
    return _sampleBuffer;
}

void RawFile::close()
{
    if (isOpen && !file.fail())
    {
        file.close();
        isOpen = false;
        delete [] (int_least8_t *) _sampleBuffer;
    }
}
