/***************************************************************************
                          sidplayer.cpp  -  Wrapper to hide private
                                            header files (see below)
                             -------------------
    begin                : Fri Jun 9 2000
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
 *  Revision 1.4  2001/02/13 21:32:35  s_a_white
 *  Windows DLL export fix.
 *
 *  Revision 1.3  2001/02/07 20:57:08  s_a_white
 *  New SID_EXPORT define.  Supports SidTune now.
 *
 *  Revision 1.2  2001/01/23 21:26:28  s_a_white
 *  Only way to load a tune now is by passing in a sidtune object.  This is
 *  required for songlength database support.
 *
 *  Revision 1.1  2000/12/12 19:14:44  s_a_white
 *  Library wrapper.
 *
 ***************************************************************************/

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
// Redirection to private version of sidplayer (This method is called Cheshire Cat)
// [ms: which is J. Carolan's name for a degenerate 'bridge']
// This interface can be directly replaced with a libsidplay1 or C interface wrapper.
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

#include "config.h"
#if defined(HAVE_MSWINDOWS) || defined(DLL_EXPORT)
// Support for DLLs
#   define SID_EXPORT __declspec(dllexport)
#endif

#include "player.h"

#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

sidplay2::sidplay2 ()
#ifdef HAVE_EXCEPTIONS
: sidplayer (*(new(nothrow) player))
#else
: sidplayer (*(new player))
#endif
{
}

sidplay2::~sidplay2 ()
{   if (&sidplayer) delete &sidplayer; }

void sidplay2::configure (sid2_playback_t mode, uint_least32_t samplingFreq, uint_least8_t precision, bool forceDualSid)
{   sidplayer.configure (mode, samplingFreq, precision, forceDualSid); }

void sidplay2::stop (void)
{   sidplayer.stop (); }

void sidplay2::pause (void)
{   sidplayer.pause (); }

uint_least32_t sidplay2::play (void *buffer, uint_least32_t length)
{   return sidplayer.play (buffer, length); }

int sidplay2::loadSong (SidTune *tune)
{   return sidplayer.loadSong (tune); }

int sidplay2::environment (sid2_env_t env)
{   return sidplayer.environment (env); }

void sidplay2::getInfo (sid2_playerInfo_t *info)
{   sidplayer.getInfo (info); }

void sidplay2::optimisation (uint_least8_t level)
{   sidplayer.optimisation (level); }

uint_least32_t sidplay2::time (void)
{   return sidplayer.time (); }

uint_least32_t sidplay2::mileage (void)
{   return sidplayer.mileage (); }

void sidplay2::filter (bool enabled)
{   sidplayer.filter (enabled); }

void sidplay2::extFilter (bool enabled)
{   sidplayer.extFilter (enabled); }

void sidplay2::sidModel (sid2_model_t model)
{   sidplayer.sidModel (model); }

void sidplay2::clockSpeed (sid2_clock_t clock, bool forced)
{   sidplayer.clockSpeed (clock, forced); }

const char *sidplay2::getErrorString (void)
{   return sidplayer.getErrorString (); }

int  sidplay2::fastForward  (uint_least8_t percent)
{   return sidplayer.fastForward (percent); }
