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

   $Id: infodlg.cpp,v 1.1 2004-02-27 16:30:29 vividos Exp $

*/
/* \file infodlg.cpp

   \brief info dialog for a Sid File

   This file contains the info dialog for a selected sid file in the playlist.
   It shows most informations from the SidTuneInfo struct to the user.

*/

// needed includes
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "resource.h"

#include <string>

// libsidplay2 stuff
#include "sidtune.h"


// functions

// dialog proc function for info dialog
int CALLBACK
InfoDlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
   switch(uMsg)
   {
   case WM_COMMAND:
      {
         int wmId    = LOWORD(wParam);
         int wmEvent = HIWORD(wParam);

         switch (wmId)
         {
         case IDOK:
         case IDCANCEL:
            EndDialog(hWnd,wmId);
            break;

         default:
            return FALSE;
         }
      }
      break;

   case WM_INITDIALOG:
      {
         const SidTuneInfo& info = *reinterpret_cast<SidTuneInfo*>(lParam);
         std::string infostr;

         // set info string
         {
            for(unsigned int i=0; i<info.numberOfInfoStrings; i++)
            {
               infostr.append(info.infoString[i]);
               if (i<info.numberOfInfoStrings-1)
                  infostr.append("\r\n");
            }

            ::SetDlgItemText(hWnd,IDC_EDIT_INFOSTR,infostr.c_str());
         }

         // set details string
         {
            char buffer[512];

            _snprintf(buffer,sizeof(buffer),
               "Filename: %s\r\n"
               "Format: %s\r\n"
               "Songs: %u\r\n"
               "Load Address: $%04x\r\n"
               "Init/%s Address: $%04x/$%04x\r\n"
               "Length: $%04x"
               ,
               info.dataFileName,
               info.formatString,
               info.songs,
               info.loadAddr,
               "Play",//info.playAddr == 0 ? "IRQ" : "Play",
               info.initAddr,
               info.playAddr,//info.playAddr == 0 ? info.irqAddr : info.playAddr,
               info.c64dataLen
            );

            ::SetDlgItemText(hWnd,IDC_EDIT_DETAILS,buffer);
         }
      }
      break;

   case WM_DESTROY:
      break;

   default:
      return FALSE;
   }

   return TRUE;
}

void wa2_info_dlg(const char* filename, HINSTANCE inst, HWND wnd)
{
   // get tune info
   SidTune tune(filename);
   const SidTuneInfo& info = tune.getInfo();

   // show dialog box
   ::DialogBoxParam(inst,MAKEINTRESOURCE(IDD_DIALOG_INFO),wnd,
      &InfoDlgProc,(LPARAM)(void*)&info);

   // reset focus on Winamp window
   SetFocus(wnd);
}
