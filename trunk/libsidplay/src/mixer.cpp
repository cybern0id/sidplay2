/***************************************************************************
                          mixer.cpp  -  Sids Mixer Routines
                             -------------------
    begin                : Sun Jul 9 2000
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
 ***************************************************************************/

#include "player.h"
#include "sidendian.h"
#include <stdio.h>

const int_least32_t VOLUME_MAX = 255;
int_least32_t min        = 0x7FFFFFFFL;
int_least32_t max        = 0x80000000L;

//-------------------------------------------------------------------------
// Generic sound output generation routines
//-------------------------------------------------------------------------
//inline
int_least32_t player::monoOutGenericMonoIn (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits)
{
    int_least32_t sample;
    count++;

    if (_optimiseLevel)
    {
        sid.clock  (clock);
        xsid.clock (clock);
    }
    sample  = sid.output (bits);

#ifndef XSID_USE_SID_VOLUME
    sample *= 3;
    sample += xsid.output (bits);
    sample /= 4;
#endif

    // Apply volume
    sample *= _leftVolume;
    sample /= VOLUME_MAX;
    return sample;
}

//inline
int_least32_t player::monoOutGenericStereoIn (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits)
{
    int_least32_t sampleL, sampleR;
    count++;

    if (_optimiseLevel)
    {
        sid.clock  (clock);
        sid2.clock (clock);
        xsid.clock (clock);
    }
    sampleL = sid.output  (bits);
    sampleR = sid2.output (bits);

#ifndef XSID_USE_SID_VOLUME
    int_least32_t sampleX;
    sampleL *= 3;
    sampleR *= 3;
    sampleX  = xsid.output (bits);
    sampleL  = (sampleL + sampleX) / 4;
    sampleR  = (sampleR + sampleX) / 4;
#endif

    // Apply volume
    sampleL *= _leftVolume;
    sampleL /= VOLUME_MAX;
    sampleR *= _rightVolume;
    sampleR /= VOLUME_MAX;
    // Convert to mono
    return (sampleL + sampleR) / 2;
}

//inline
int_least32_t player::monoOutGenericStereoRIn (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits)
{
    int_least32_t sample;
    count++;
	
    if (_optimiseLevel)
    {
        sid2.clock (clock);
        xsid.clock (clock);
    }
    sample  = sid2.output (bits);

#ifndef XSID_USE_SID_VOLUME
    sample *= 3;
    sample += xsid.output (bits);
    sample /= 4;
#endif

    // Apply volume
    sample *= _rightVolume;
    sample /= VOLUME_MAX;
    return sample;
}

//inline
int_least32_t player::stereoOutGenericMonoIn (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits)
{
    int_least32_t sample;
    count += 2;

    if (_optimiseLevel)
    {
        sid.clock  (clock);
        xsid.clock (clock);
    }
    sample  = sid.output (bits);

#ifndef XSID_USE_SID_VOLUME
    sample *= 3;
    sample += xsid.output (bits);
    sample /= 4;
#endif

    // Apply volume
    sample *= _leftVolume;
    sample /= VOLUME_MAX;
    return sample;
}

//inline
int_least32_t player::stereoOutGenericStereoIn (uint_least16_t clock, uint_least32_t &count, uint_least8_t bits,
                                                int_least32_t &sampleR)
{
    int_least32_t sampleL;
    count += 2;

    if (_optimiseLevel)
    {
        sid.clock  (clock);
        sid2.clock (clock);
        xsid.clock (clock);
    }
    sampleL = sid.output  (bits);
    sampleR = sid2.output (bits);

#ifndef XSID_USE_SID_VOLUME
    int_least32_t sampleX;
    sampleL *= 3;
    sampleR *= 3;
    sampleX  = xsid.output (bits);
    sampleL /= 4;
    sampleR  = (sampleR + sampleX) / 4;
#endif

    // Apply volume
    sampleL *= _leftVolume;
    sampleL /= VOLUME_MAX;
    sampleR *= _rightVolume;
    sampleR /= VOLUME_MAX;
    return sampleL;  // sampleR is reference
}

//-------------------------------------------------------------------------
// 8 bit sound output generation routines
//-------------------------------------------------------------------------
void player::monoOut8MonoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    int_least8_t *buf   = (int_least8_t *) buffer + count;
    int_least8_t sample = (int_least8_t) monoOutGenericMonoIn (clock, count, 8);
    *buf = sample ^ '\x80';
}

void player::monoOut8StereoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    int_least8_t *buf   = (int_least8_t *) buffer + count;
    int_least8_t sample = (int_least8_t) monoOutGenericStereoIn (clock, count, 8);
    *buf = sample ^ '\x80';
}

void player::monoOut8StereoRIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    int_least8_t *buf   = (int_least8_t *) buffer + count;
    int_least8_t sample = (int_least8_t) monoOutGenericStereoRIn (clock, count, 8);
    *buf = sample ^ '\x80';
}

void player::stereoOut8MonoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    int_least8_t *buf   = (int_least8_t *) buffer + count;
    int_least8_t sample = (int_least8_t) stereoOutGenericMonoIn (clock, count, 8);
    sample ^= '\x80';
    *buf++  = sample; 
    *buf    = sample; 
}

void player::stereoOut8StereoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    int_least32_t sampleR; // Need to get direct. Only sampleL is returned.
    int_least8_t *buf     = (int_least8_t *) buffer + count;
    int_least8_t  sampleL = (int_least8_t) stereoOutGenericStereoIn (clock, count, 8, sampleR);
    *buf++ = sampleL ^ '\x80';
    *buf   = (int_least8_t) sampleR ^ '\x80';
}

//-------------------------------------------------------------------------
// 16 bit sound output generation routines
//-------------------------------------------------------------------------
void player::monoOut16MonoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    uint_least8_t *buf    = (uint_least8_t *) buffer + (count << 1);
    endian_16 (buf, (uint_least16_t) monoOutGenericMonoIn (clock, count, 16));
}

void player::monoOut16StereoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    uint_least8_t *buf    = (uint_least8_t *) buffer + (count << 1);
    endian_16 (buf, (uint_least16_t) monoOutGenericStereoIn (clock, count, 16));
}

void player::monoOut16StereoRIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    uint_least8_t *buf    = (uint_least8_t *) buffer + (count << 1);
    endian_16 (buf, (uint_least16_t) monoOutGenericStereoRIn (clock, count, 16));
}

void player::stereoOut16MonoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    uint_least8_t *buf    = (uint_least8_t *) buffer + (count << 1);
    uint_least16_t sample = (uint_least16_t)  stereoOutGenericMonoIn (clock, count, 16);
    endian_16 (buf, sample);
    buf += 2;
    endian_16 (buf, sample);
}

void player::stereoOut16StereoIn (uint_least16_t clock, void *buffer, uint_least32_t &count)
{
    int_least32_t  sampleR; // Need to get direct. Only sampleL is returned.
    uint_least8_t *buf     = (uint_least8_t *) buffer + (count << 1);
    uint_least16_t sampleL = (uint_least16_t)  stereoOutGenericStereoIn (clock, count, 16, sampleR);
    endian_16 (buf, sampleL);
    buf += 2;
    endian_16 (buf, (uint_least16_t) sampleR);
}

