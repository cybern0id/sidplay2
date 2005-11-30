/*
 * A basic RAW output file type - Interface.
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
 *
 ***************************************************************************/

#ifndef RAW_FILE_HEADER_H
#define RAW_FILE_HEADER_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include "config.h"
#include "../AudioBase.h"

class RawFile: public AudioBase
{
private:
    unsigned long int byteCount;

    std::fstream file;
    bool isOpen;         // whether file has been opened

public:

    RawFile();
    
    // Only unsigned 8-bit, and signed 16-bit, samples are supported.
    // Endian-ess is adjusted if necessary.
    //
    // If number of sample bytes is given, this can speed up the
    // process of closing a huge file on slow storage media.

    void *open(AudioConfig &cfg, const char *name)
    { return open (cfg, name, true); }
    void *open(AudioConfig &cfg, const char *name,
               const bool overWrite);
    
    // After write call old buffer is invalid and you should
    // use the new buffer provided instead.
    void *write();
    void  close();
    void  pause() {;}
    const char *extension () const { return ""; }
    ~RawFile() { close(); }
    
    // Rev 1.3 (saw) - Changed, see AudioBase.h
    void *reset ()
    {
        if (isOpen)
            return _sampleBuffer;
        return NULL;
    }

    // Stream state.
    bool fail() const { return (file.fail() != 0); }
    bool bad()  const { return (file.bad()  != 0); }

    operator bool()  const { return (file.good() != 0); }
    bool operator!() const { return (file.fail() != 0); }
};

#endif /* WAVE_FILE_HEADER_H */
