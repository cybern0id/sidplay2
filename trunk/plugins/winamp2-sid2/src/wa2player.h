/*
   in_sid2 - a .sid file winamp input plugin based on libsidplay2
   Copyright (c) 2003 Michael Fink

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   $Id: wa2player.h,v 1.1 2004-02-27 16:30:29 vividos Exp $

*/
/* \file wa2player.h

   \brief Winamp2 player class

   The class wa2_player is a wrapper class to use in the Winamp2 input module
   functions in in_sid2.cpp. It accesses the libsidplay2 classes to play back
   sid tunes.

*/

// include guard
#ifndef wa2player_hpp_
#define wa2player_hpp_

// needed includes
#include <string>


// forward references

namespace SIDPLAY2_NAMESPACE {
class Player;
}
using SIDPLAY2_NAMESPACE::Player;
class SidTune;
class ReSIDBuilder;
struct sid2_info_t;


// structs

struct wa2_player_cfg
{
   // sample rate
   unsigned int samplerate;

   // sid model to use; 0: default, 1: old sid, 2: new sid
   unsigned int sid_model;

   // indicates if filters should be used
   bool use_filters;

   // indicates if stereo mode should be used
   bool stereo;

   // optimization level, either 0 or 1
   unsigned int optlevel;

   // title format string
   std::string title_fmt;
};


// classes

// Winamp2 player class
class wa2_player
{
public:
   // ctor
   wa2_player();

   // inits player
   void init();

   // reinits player, e.g. after config changes
   void reinit();

   // loads a tune
   bool load(const char* filename);

   //! starts a song
   void start_song(unsigned int songnum=0);

   // cleans up player
   void done();

   // gets samples to play back; returns number of samples returned
   unsigned int play(signed short* samples, unsigned int numsamples);

   // pause/unpause playback
   void pause();

   // stops playback and unloads tune
   void stop();

   // get functions

   // returns samplerate
   unsigned int get_samplerate();

   // returns number of channels
   unsigned int get_channels();

   // returns tune length in seconds
   unsigned int get_total_time();

   // returns current playing time
   unsigned int get_current_time();

   // returns number of songs
   unsigned int get_num_songs();

   // returns current song number
   unsigned int get_current_song();

   // returns file info about the given file
   void get_file_info(const char* filename, char* title, unsigned int& length);

   // returns player info struct
   const sid2_info_t& get_player_info() const;

   // returns config data
   wa2_player_cfg& get_cfg();

protected:
   // loads config
   void load_cfg();

   // saves config
   void save_cfg();

protected:
   // sid tune to play
   SidTune* tune;

   // player class
   Player* player;

   // resid builder class
   ReSIDBuilder* rs;

   // player configuration
   wa2_player_cfg cfg;

   // last played filename
   std::string last_filename;
};

#endif
