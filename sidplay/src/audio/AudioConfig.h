/***************************************************************************
                          AudioConfig.h  -  description
                             -------------------
    begin                : Sat Jul 8 2000
    copyright            : (C) 2000 by Simon White
    email                : s_a_white@email.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.3  2000/12/11 19:07:14  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

#ifndef _AudioConfig_h_
#define _AudioConfig_h_

#include <sidplay/sidtypes.h>

// Configuration constants.
enum
{
    AUDIO_SIGNED_PCM   = 0x7f,
    AUDIO_UNSIGNED_PCM = 0x80
};


class AudioConfig
{
public:
    uint_least32_t frequency;
    int            precision;
    int            channels;
    int            encoding;
    uint_least32_t bufSize;       // sample buffer size
    
    AudioConfig()
    {
        frequency = 22050;
        precision = 8;
        channels  = 1;
        encoding  = AUDIO_UNSIGNED_PCM;
        bufSize   = 0;
    }
};

#endif  // _AudioConfig_h_
