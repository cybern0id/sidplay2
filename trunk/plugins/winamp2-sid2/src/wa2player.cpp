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

   $Id: wa2player.cpp,v 1.1 2004-02-27 16:30:29 vividos Exp $

*/
/* \file wa2player.cpp

   \brief Winamp2 player class implementation

   This file contains the implementation of the wa2_player class. Most methods
   just call the appropriate functions in libsidplay2 classes.

*/

// needed includes
#include "sidconfig.h"
#include "wa2player.h"
#include <cstdio>

// libsndfile2 includes
#include "player.h"
#include "sidtune.h"
#include "resid.h"


// wa2_player methods

wa2_player::wa2_player()
:tune(NULL),player(NULL),rs(NULL)
{
}

void wa2_player::init()
{
   // load config
   load_cfg();

   // create player
   player = new SIDPLAY2_NAMESPACE::Player;
   tune = NULL;

   reinit();
}

void wa2_player::reinit()
{
   // set player config
   sid2_config_t pcfg = player->config();
   pcfg.frequency = cfg.samplerate;
   pcfg.sampleFormat = SID2_LITTLE_SIGNED;
   pcfg.optimisation = cfg.optlevel;
   pcfg.precision = 16;
   pcfg.playback = cfg.stereo ? sid2_stereo : sid2_mono;
   pcfg.forceDualSids = cfg.stereo;
   pcfg.sidModel =
      cfg.sid_model==0 ? SID2_MODEL_CORRECT :
      cfg.sid_model==1 ? SID2_MOS6581 : SID2_MOS8580;

   // add ReSID sidbuilder
   if (pcfg.sidEmulation == NULL)
   {
      rs = new ReSIDBuilder("ReSID");
      rs->create(2);

      pcfg.sidEmulation = rs;
   }

   rs->filter(cfg.use_filters);
   rs->sampling(cfg.samplerate);

   // TODO set more config options

   if (tune != NULL)
      player->stop();
   player->config(pcfg);
}

bool wa2_player::load(const char* filename)
{
   delete tune;
   tune = NULL;

   // load tune
   tune = new SidTune(NULL);
   if (!tune->load(filename))
      return false;

   tune->selectSong(0);

   // load tune into player
   if (player->load(tune)!=0)
   {
      delete tune;
      tune = NULL;
      return false;
   }

   last_filename = filename;

   return true;
}


void wa2_player::start_song(unsigned int songnum)
{
   tune->selectSong(songnum);
   player->load(tune);
}

void wa2_player::done()
{
   // save config
   save_cfg();

   player->stop();
   player->load(NULL);

   // clean up all objects
   delete player;
   player = NULL;

   delete tune;
   tune = NULL;

   rs->remove();
   delete rs;
   rs = NULL;
}

unsigned int wa2_player::play(signed short* samples, unsigned int numsamples)
{
   // try to get some samples
   unsigned int read = player == NULL ? 0 :
      player->play(samples,numsamples*sizeof(signed short));

   return read/sizeof(signed short);
}

void wa2_player::pause()
{
   if (player != NULL) player->pause();
}

void wa2_player::stop()
{
   player->stop();
   player->load(NULL);

   delete tune;
   tune = NULL;
}

unsigned int wa2_player::get_samplerate()
{
   return cfg.samplerate;
}

unsigned int wa2_player::get_channels()
{
   return cfg.stereo ? 2 : 1;
}

unsigned int wa2_player::get_total_time()
{
   return tune==NULL ? 0 : tune->getInfo().songLength*1000;
}

unsigned int wa2_player::get_current_time()
{
   return 0;
}

unsigned int wa2_player::get_num_songs()
{
   return tune->getInfo().songs;
}

unsigned int wa2_player::get_current_song()
{
   return tune->getInfo().currentSong;
}

void wa2_player::get_file_info(const char* filename, char* title, unsigned int& length)
{
   SidTune tune2(NULL);
   SidTune* thetune = NULL;
   bool cursong = false;

   if (filename == NULL || *filename == 0)
   {
      // currently playing file
      thetune = tune;
      filename = last_filename.c_str();
      cursong = true;
   }
   else
   {
      // some other file
      tune2.load(filename);
      thetune = &tune2;
   }

   // get tune info
   const SidTuneInfo& info = thetune->getInfo();

   length = info.songLength;

   if (title!=NULL)
   {
      // do title string template replacement
      std::string titlestr(cfg.title_fmt),repl_str;
      std::string::size_type pos = 0;
      while( std::string::npos != (pos = titlestr.find("%",pos) ) )
      {
         bool do_replace = true;

         // check which string to replace
         switch(titlestr.at(pos+1))
         {
         case 't': repl_str = info.infoString[0]; break; // title
         case 'a': repl_str = info.infoString[1]; break; // artist
         case 'y': repl_str = info.infoString[2]; break; // artist
         case 'c': // current song
         case 'n': // number of songs
            {
               unsigned int songnr = titlestr.at(pos+1)=='c' ?
                  (cursong ? info.currentSong : info.startSong) : info.songs;

               char buffer[16];
               _snprintf(buffer,sizeof(buffer),"%u",songnr);
               repl_str = buffer;
            }
            break; 

         case '%': repl_str = "%"; break;
         default:
            do_replace = false;
            break;
         }

         if (do_replace)
         {
            titlestr.replace(pos,2,repl_str);
            pos += repl_str.size();
         }
         else
            pos++;
      }

      strcpy(title,titlestr.c_str());
   }
}

const sid2_info_t& wa2_player::get_player_info() const
{
   return player->info();
}

wa2_player_cfg& wa2_player::get_cfg()
{
   return cfg;
}

#include <windows.h>

const char* reg_root = "Software\\Winamp\\in_sid2 Plugin";

void wa2_player::load_cfg()
{
   // some presets, in case no registry keys exist yet
   cfg.samplerate = 44100;
   cfg.use_filters = true;
   cfg.sid_model = 0;
   cfg.stereo = false;
   cfg.optlevel = 0;
   cfg.title_fmt.assign("%t [%a] %c/%n (%y)");

   HKEY key;

   if (ERROR_SUCCESS == ::RegOpenKeyEx(HKEY_CURRENT_USER,reg_root,0,KEY_READ,&key))
   {
      DWORD value,type,size;

      type = REG_DWORD;
      size = sizeof(value);

      ::RegQueryValueEx(key,"Samplerate",NULL,&type,(BYTE*)&value,&size);
      cfg.samplerate = value;

      ::RegQueryValueEx(key,"UseFilters",0,&type,(BYTE*)&value,&size);
      cfg.use_filters = value != 0;

      ::RegQueryValueEx(key,"SidModel",0,&type,(BYTE*)&value,&size);
      cfg.sid_model = value > 2 ? 0 : value;

      ::RegQueryValueEx(key,"Stereo",0,&type,(BYTE*)&value,&size);
      cfg.stereo = value != 0;

      ::RegQueryValueEx(key,"OptLevel",0,&type,(BYTE*)&value,&size);
      cfg.optlevel = value > 1 ? 0 : value;

      char buffer[256];
      type = REG_SZ;
      size = sizeof(buffer);
      if (ERROR_SUCCESS == ::RegQueryValueEx(key,"TitleFormat",0,&type,(BYTE*)&buffer,&size))
         cfg.title_fmt.assign(buffer);

      ::RegCloseKey(key);
   }

}

void wa2_player::save_cfg()
{
   HKEY key;

   if (ERROR_SUCCESS == ::RegCreateKeyEx(HKEY_CURRENT_USER,reg_root,0,NULL,
      REG_OPTION_NON_VOLATILE,KEY_WRITE,NULL,&key,NULL))
   {
      DWORD value = cfg.samplerate;
      ::RegSetValueEx(key,"Samplerate",0,REG_DWORD,(CONST BYTE*)&value,sizeof(value));

      value = cfg.use_filters ? 1 : 0;
      ::RegSetValueEx(key,"UseFilters",0,REG_DWORD,(CONST BYTE*)&value,sizeof(value));

      value = cfg.sid_model;
      ::RegSetValueEx(key,"SidModel",0,REG_DWORD,(CONST BYTE*)&value,sizeof(value));

      value = cfg.stereo ? 1 : 0;
      ::RegSetValueEx(key,"Stereo",0,REG_DWORD,(CONST BYTE*)&value,sizeof(value));

      value = cfg.optlevel;
      ::RegSetValueEx(key,"OptLevel",0,REG_DWORD,(CONST BYTE*)&value,sizeof(value));

      ::RegSetValueEx(key,"TitleFormat",0,REG_SZ,(CONST BYTE*)cfg.title_fmt.c_str(),cfg.title_fmt.size()+1);

      ::RegCloseKey(key);
   }
}
