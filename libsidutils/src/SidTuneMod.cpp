/*
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
#if defined(HAVE_MSWINDOWS) || defined(DLL_EXPORT)
// Support for DLLs
#   define SID_EXPORT __declspec(dllexport)
#endif

#include <sidplay/sidendian.h>
#include "SidTuneMod.h"
#include "MD5/MD5.h"

void SidTuneMod::createMD5(MD5& myMD5)
{
    if (status)
    {
        // Include C64 data.
        uint_least16_t currentSong = info.currentSong;
        md5_byte_t tmp[2];

        myMD5.append(cache.get()+fileOffset,info.c64dataLen);
        // Include INIT and PLAY address.
        endian_little16 (tmp,info.initAddr);
        myMD5.append    (tmp,sizeof(tmp));
        endian_little16 (tmp,info.playAddr);
        myMD5.append    (tmp,sizeof(tmp));
        // Include number of songs.
        endian_little16 (tmp,info.songs);
        myMD5.append    (tmp,sizeof(tmp));
        // Include song speed for each song.
        for (uint_least16_t s = 1; s <= info.songs; s++)
        {
            selectSong (s);
            myMD5.append (&info.songSpeed,sizeof(info.songSpeed));
        }
        // Restore old song
        selectSong (currentSong);
    }
}

