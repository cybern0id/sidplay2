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
{   return sidplayer._errorString; }

int  sidplay2::fastForward  (uint_least8_t percent)
{   return sidplayer.fastForward (percent); }
