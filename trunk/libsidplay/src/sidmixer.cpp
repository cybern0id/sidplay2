/***************************************************************************
                          sidmixer.cpp  -  Sids Mixer Routines
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
#include "sidplayer_pr.h"

const ubyte_sidt VOLUME_MAX = 255;

//-------------------------------------------------------------------------
// Generic sound output generation routines
//-------------------------------------------------------------------------
//inline
sdword_sidt sidplayer_pr::monoOutGenericMonoIn (uword_sidt clock, udword_sidt &count, ubyte_sidt bits)
{
    sdword_sidt sample;
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
sdword_sidt sidplayer_pr::monoOutGenericStereoIn (uword_sidt clock, udword_sidt &count, ubyte_sidt bits)
{
    sdword_sidt sampleL, sampleR;
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
    sword_sidt sampleX;
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
sdword_sidt sidplayer_pr::monoOutGenericStereoRIn (uword_sidt clock, udword_sidt &count, ubyte_sidt bits)
{
    sdword_sidt sample;
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
sdword_sidt sidplayer_pr::stereoOutGenericMonoIn (uword_sidt clock, udword_sidt &count, ubyte_sidt bits)
{
    sdword_sidt sample;
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
sdword_sidt sidplayer_pr::stereoOutGenericStereoIn (uword_sidt clock, udword_sidt &count, ubyte_sidt bits, sdword_sidt &sampleR)
{
    sdword_sidt sampleL;
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
    sdword_sidt sampleX;
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
    return sampleL;  // sampleR is reference
}

//-------------------------------------------------------------------------
// 8 bit sound output generation routines
//-------------------------------------------------------------------------
void sidplayer_pr::monoOut8MonoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sbyte_sidt *buf = (sbyte_sidt *) buffer + count;
    sbyte_sidt sample = (sbyte_sidt) monoOutGenericMonoIn (clock, count, 8);
    *buf = sample ^ '\x80';
}

void sidplayer_pr::monoOut8StereoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sbyte_sidt *buf = (sbyte_sidt *) buffer + count;
    sbyte_sidt sample = (sbyte_sidt) monoOutGenericStereoIn (clock, count, 8);
    *buf = sample ^ '\x80';
}

void sidplayer_pr::monoOut8StereoRIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sbyte_sidt *buf = (sbyte_sidt *) buffer + count;
    sbyte_sidt sample = (sbyte_sidt) monoOutGenericStereoRIn (clock, count, 8);
    *buf = sample ^ '\x80';
}

void sidplayer_pr::stereoOut8MonoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sbyte_sidt *buf = (sbyte_sidt *) buffer + count;
    sbyte_sidt sample = (sbyte_sidt) stereoOutGenericMonoIn (clock, count, 8);
    sample ^= '\x80';
    *buf++ = sample; 
    *buf = sample; 
}

void sidplayer_pr::stereoOut8StereoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{
    sbyte_sidt *buf = (sbyte_sidt *) buffer + count;
    sdword_sidt sampleR; // Need to get direct. Only sampleL is returned.
    sbyte_sidt sample = (sbyte_sidt) stereoOutGenericStereoIn (clock, count, 8, sampleR);
    *buf++ = sample ^ '\x80';
    *buf = (sbyte_sidt) sampleR ^ '\x80';
}

//-------------------------------------------------------------------------
// 16 bit sound output generation routines
//-------------------------------------------------------------------------
void sidplayer_pr::monoOut16MonoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sword_sidt *buf = (sword_sidt *) buffer + count;
    *buf = (sword_sidt) monoOutGenericMonoIn (clock, count, 16);
}

void sidplayer_pr::monoOut16StereoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sword_sidt *buf = (sword_sidt *) buffer + count;
    *buf = (sword_sidt) monoOutGenericStereoIn (clock, count, 16);
}

void sidplayer_pr::monoOut16StereoRIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sword_sidt *buf = (sword_sidt *) buffer + count;
    *buf = (sword_sidt) monoOutGenericStereoRIn (clock, count, 16);
}

void sidplayer_pr::stereoOut16MonoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{   // Get sample(s)
    sword_sidt *buf = (sword_sidt *) buffer + count;
	sword_sidt sample = (sword_sidt) stereoOutGenericMonoIn (clock, count, 16);
	*buf++ = sample;
    *buf   = sample;
}

void sidplayer_pr::stereoOut16StereoIn (uword_sidt clock, void *buffer, udword_sidt &count)
{
    sword_sidt *buf = (sword_sidt *) buffer + count;
    sdword_sidt sampleR; // Need to get direct. Only sampleL is returned.
    *buf++ = (sword_sidt) stereoOutGenericStereoIn (clock, count, 16, sampleR);
    *buf   = (sword_sidt) sampleR;
}

