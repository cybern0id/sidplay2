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

   $Id: in_sid2.cpp,v 1.2 2004-06-01 20:13:27 vividos Exp $

*/
/* \file in_sid2.cpp

   \brief Winamp 2.x input file module

   This file is an input file module for use in Winamp2 to play back files
   with extensions .sid, .dat and .psid. It uses the libsidplay2 library.

   most of this file is copied from the example "in_minisdk.zip"

*/

// needed includes
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// winamp plugin includes
#include "in2.h"
#include "wa2player.h"


// from wa_ipc.h:

#define WM_WA_IPC WM_USER

#define IPC_UPDTITLE 243
/* (requires Winamp 2.2+)
** SendMessage(hwnd_winamp,WM_WA_IPC,0,IPC_UPDTITLE);
** IPC_UPDTITLE will ask Winamp to update the informations about the current title.
*/


// forward references

bool wa2_config_dlg(HINSTANCE inst, HWND wnd);  // config dialog
void wa2_about_dlg(HINSTANCE inst, HWND wnd);   // about dialog
void wa2_info_dlg(const char* filename, HINSTANCE inst, HWND wnd); // info dialog

extern In_Module mod;  // input module (declared near the bottom of this file)


// global variables

wa2_player* player; // player class

int paused;         // are we paused?

int killDecodeThread=0;                       // the kill switch for the decode thread
HANDLE thread_handle=INVALID_HANDLE_VALUE;    // the handle to the decode thread
DWORD WINAPI __stdcall DecodeThread(void *b); // the decode thread procedure

// post this to the main window at end of file (after playback has stopped)
#define WM_WA_MPEG_EOF WM_USER+2


// functions

void config(HWND hwndParent)
{
   if (!wa2_config_dlg(mod.hDllInstance,hwndParent))
      return;

   if (mod.outMod != NULL)
   {
      mod.outMod->Flush(0);

      // reopen device
      mod.outMod->Close();

      unsigned int samplerate = player->get_samplerate();
      unsigned int channels = player->get_channels();

      mod.outMod->Open(samplerate,channels,16,-1,-1);
      mod.SetInfo((samplerate*16*channels)/1000,samplerate/1000,channels,1);
   }

   // reinit player
   player->reinit();
}

void about(HWND hwndParent)
{
   wa2_about_dlg(mod.hDllInstance,hwndParent);
}

void init()
{
   player = new wa2_player;
   player->init();
}

void quit()
{
   player->done();
   delete player;
   player = NULL;
}

int isourfile(char *fn)
{
   return 0;
}

int play(char* fn)
{
   unsigned int samplerate = player->get_samplerate();
   unsigned int channels = player->get_channels();

   if (player != NULL)
      player->load(fn);

   paused=0;

   int maxlatency = mod.outMod->Open(samplerate,channels,16, -1,-1);
   if (maxlatency < 0) // error opening device
   {
      return 1;
   }

   // dividing by 1000 for the first parameter of setinfo makes it
   // display 'H'... for hundred.. i.e. 14H Kbps.
   mod.SetInfo((samplerate*16*channels)/1000,samplerate/1000,channels,1);

   // initialize vis stuff
   mod.SAVSAInit(maxlatency,samplerate);
   mod.VSASetInfo(samplerate,channels);

   mod.outMod->SetVolume(-666); // set the output plug-ins default volume

   // create playing thread
   killDecodeThread=0;

   unsigned long thread_id;
   thread_handle = (HANDLE) CreateThread(NULL,0,
      (LPTHREAD_START_ROUTINE) DecodeThread,(void*) &killDecodeThread,0,&thread_id);

   return 0;
}

void pause()
{
   if (player != NULL)
      player->pause();

   paused=1;
   mod.outMod->Pause(1);
}
void unpause()
{
   if (player != NULL)
      player->pause();

   paused=0;
   mod.outMod->Pause(0);
}

int ispaused()
{
   return paused;
}

void stop()
{
   if (thread_handle != INVALID_HANDLE_VALUE)
   {
      // waiting for thread termination
      killDecodeThread=1;
      if (WaitForSingleObject(thread_handle,INFINITE) == WAIT_TIMEOUT)
      {
         MessageBox(mod.hMainWindow,"error asking thread to die!\n","error killing decode thread",0);
         TerminateThread(thread_handle,0);
      }

      CloseHandle(thread_handle);
      thread_handle = INVALID_HANDLE_VALUE;
   }

   // stop player
   if (player != NULL)
      player->stop();

   mod.outMod->Close();
   mod.SAVSADeInit();
}

int getlength()
{
#ifdef HAVE_TIME_SUPPORT
   return player==NULL ? 0 : player->get_total_time()*1000;
#else
   return player==NULL ? 0 : player->get_num_songs()*1000;
#endif
}

int getoutputtime()
{
#ifdef HAVE_TIME_SUPPORT
   return player==NULL ? 0 : player->get_current_time()*1000;
#else
   return player==NULL ? 0 : (player->get_current_song())*1000;
#endif
}

void setoutputtime(int time_in_ms)
{
#ifdef HAVE_TIME_SUPPORT
   // TODO seek to position
#else
   // start new song
   mod.outMod->Flush(0);
   //player->start_song((time_in_ms+1000)/1000-1);
   player->start_song(time_in_ms/1000);

   // update winamp title
   SendMessage(mod.hMainWindow,WM_WA_IPC,0,IPC_UPDTITLE);
#endif
}

void setvolume(int volume)
{
   mod.outMod->SetVolume(volume);
}

void setpan(int pan)
{
   mod.outMod->SetPan(pan);
}

int infodlg(char* fn, HWND hwnd)
{
   wa2_info_dlg(fn,mod.hDllInstance,hwnd);
   return 0;
}

void getfileinfo(char* filename, char* title, int* length_in_ms)
{
   unsigned int length;
   player->get_file_info(filename,title,length);

   if (length_in_ms != NULL)
      *length_in_ms = length*1000;
}

void eq_set(int on, char data[10], int preamp)
{
}


// thread function

DWORD WINAPI __stdcall DecodeThread(void* b)
{
   unsigned __int64 samplenum=0;
   int done=0;
   while (! *((int *)b) )
   {
      if (done)
      {
         mod.outMod->CanWrite();
         if (!mod.outMod->IsPlaying())
         {
            PostMessage(mod.hMainWindow,WM_WA_MPEG_EOF,0,0);
            return 0;
         }
         Sleep(10);
      }
      else if (mod.outMod->CanWrite() >= 256*2)
      {
         signed short buffer[256];
         unsigned int read = player->play(buffer,256);
         mod.outMod->Write(reinterpret_cast<char*>(&buffer[0]),read*sizeof(signed short));

         // do vis stuff
         {
            static char vis_buffer[576 * 2*2];
            // &buffer[0]

            //mod.SAAddPCMData(&buffer[0], player->get_channels(), 16, samplenum*2);
            //mod.VSAAddPCMData(vis_buffer, player->get_channels(), 16, samplenum*2);
         }

         samplenum += 256;

         Sleep(0);
      }
      else Sleep(20);
   }
   return 0;
}


#include "config.h"

// Winamp2 input module struct

In_Module mod =
{
   IN_VER,
   "Winamp Sid File Player (based on "PACKAGE" "VERSION")",

   0,  // hMainWindow
   0,  // hDllInstance
   "SID\0Sid File (*.sid)\0",
   1, // is_seekable
   1, // uses output

   config,
   about,
   init,
   quit,
   getfileinfo,
   infodlg,
   isourfile,
   play,
   pause,
   unpause,
   ispaused,
   stop,

   getlength,
   getoutputtime,
   setoutputtime,

   setvolume,
   setpan,

   0,0,0,0,0,0,0,0,0, // vis stuff

   0,0, // dsp

   eq_set,

   NULL,    // setinfo

   0 // out_mod
};


// exported functions

// returns handle to module
extern "C"
__declspec(dllexport)
In_Module* winampGetInModule2()
{
   return &mod;
}
