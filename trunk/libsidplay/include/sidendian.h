/***************************************************************************
                          sidendian.h  -  Improtant endian functions
                             -------------------
    begin                : Mon Jul 3 2000
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

#ifndef _sidendian_h_
#define _sidendian_h_

#include "sidtypes.h"

#if defined(SID_WORDS_BIGENDIAN)
/* byte-order: HIHI..3210..LO */
#elif defined(SID_WORDS_LITTLEENDIAN)
/* byte-order: LO..0123..HIHI */
#else
  #error Please check source code configuration!
#endif

/*
Labeling:
0 - LO
1 - HI
2 - HILO
3 - HIHI
*/

///////////////////////////////////////////////////////////////////
// WORD FUNCTIONS
///////////////////////////////////////////////////////////////////
// Set the lo byte (8 bit) in a word (16 bit)
inline void sidlobyte (uword_sidt &word, ubyte_sidt byte)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	((ubyte_sidt *) &word)[0] = byte;
#else
	((ubyte_sidt *) &word)[1] = byte;
#endif
}

// Set the hi byte (8 bit) in a word (16 bit)
inline void sidhibyte (uword_sidt &word, ubyte_sidt byte)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	((ubyte_sidt *) &word)[1] = byte;
#else
	((ubyte_sidt *) &word)[0] = byte;
#endif
}

// Get the lo byte (8 bit) in a word (16 bit)
inline ubyte_sidt sidlobyte (uword_sidt &word)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	return ((ubyte_sidt *) &word)[0];
#else
	return ((ubyte_sidt *) &word)[1];
#endif
}

// Set the hi byte (8 bit) in a word (16 bit)
inline ubyte_sidt sidhibyte (uword_sidt &word)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	return ((ubyte_sidt *) &word)[1];
#else
	return ((ubyte_sidt *) &word)[0];
#endif
}


///////////////////////////////////////////////////////////////////
// DWORD FUNCTIONS
///////////////////////////////////////////////////////////////////
// Set the lo word (16bit) in a dword (32 bit)
inline void sidloword (udword_sidt &dword, uword_sidt word)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	((uword_sidt *) &dword)[0] = word;
#else
	((uword_sidt *) &dword)[1] = word;
#endif
}

// Get the lo word (16bit) in a dword (32 bit)
inline uword_sidt sidloword (udword_sidt &dword)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	return ((uword_sidt *) &dword)[0];
#else
	return ((uword_sidt *) &dword)[1];
#endif
}

// Get the hi word (16bit) in a dword (32 bit)
inline uword_sidt sidhiword (udword_sidt &dword)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	return ((uword_sidt *) &dword)[1];
#else
	return ((uword_sidt *) &dword)[0];
#endif
}

// Set the lo byte (8 bit) in a dword (32 bit)
inline void sidlobyte (udword_sidt &dword, ubyte_sidt byte)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	((ubyte_sidt *) &dword)[0] = byte;
#else
	((ubyte_sidt *) &dword)[3] = byte;
#endif
}

// Set the hi byte (8 bit) in a dword (32 bit)
inline void sidhibyte (udword_sidt &dword, ubyte_sidt byte)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	((ubyte_sidt *) &dword)[1] = byte;
#else
	((ubyte_sidt *) &dword)[2] = byte;
#endif
}

// Get the lo byte (8 bit) in a dword (32 bit)
inline ubyte_sidt sidlobyte (udword_sidt &dword)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	return ((ubyte_sidt *) &dword)[0];
#else
	return ((ubyte_sidt *) &dword)[3];
#endif
}

// Get the hi byte (8 bit) in a dword (32 bit)
inline ubyte_sidt sidhibyte (udword_sidt &dword)
{
#if defined(SID_WORDS_LITTLEENDIAN)
	return ((ubyte_sidt *) &dword)[1];
#else
	return ((ubyte_sidt *) &dword)[2];
#endif
}

#endif // _sidendian_h_
