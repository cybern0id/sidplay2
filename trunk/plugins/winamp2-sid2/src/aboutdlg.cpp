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

   $Id: aboutdlg.cpp,v 1.1 2004-02-27 16:30:29 vividos Exp $

*/
/* \file aboutdlg.cpp

   \brief about dialog for the in_sid2 Winamp plugin

   This file contains the about dialog for the plugin. It shows some copyright
   and library version infos, along with libsidplay2 credit strings.

*/

// needed includes
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "resource.h"

#include "wa2player.h"
#include "sid2types.h"


// extern references

extern wa2_player* player;
extern "C" const char* resid_version_string;


// functions

// dialog proc function for info dialog
int CALLBACK
AboutDlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
   switch(uMsg)
   {
   case WM_COMMAND:
      {
         int wmId = LOWORD(wParam);
         if (wmId==IDOK)
            EndDialog(hWnd,wmId);
         else
            return FALSE;
      }
      break;

   case WM_INITDIALOG:
      {
         const sid2_info_t& info = player->get_player_info();

         // libsidplay2/resid version
         {
            char buffer[512];

            _snprintf(buffer,sizeof(buffer),
               "in_sid2 uses %s %s and %s",
               info.name,info.version,resid_version_string);

            ::SetDlgItemText(hWnd,IDC_STATIC_VERSIONS,buffer);
         }

         // libsidplay credits
         {
            // collect all credits strings
            std::string sid_credits;

            unsigned int i=0;
            const char* credits;
            while((credits = info.credits[i++]) != NULL)
            {
               if (sid_credits.size()!=0)
                  sid_credits.append("\r\n");

               // nasty: \0's in strings
               while(*credits != 0)
               {
                  sid_credits.append(credits);
                  credits += strlen(credits)+1;
               }
            }

            // replace \t's with spaces
            std::string::size_type pos = 0;
            while( std::string::npos != (pos = sid_credits.find('\t',pos)) )
               sid_credits.replace(pos,1," ");

            ::SetDlgItemText(hWnd,IDC_EDIT_SIDCREDITS,sid_credits.c_str());
         }
      }
      break;

   default:
      return FALSE;
   }

   return TRUE;
}

void wa2_about_dlg(HINSTANCE inst, HWND wnd)
{
   // show dialog box
   ::DialogBoxParam(inst,MAKEINTRESOURCE(IDD_DIALOG_ABOUT),wnd,
      &AboutDlgProc,(LPARAM)NULL);

   // reset focus on Winamp window
   SetFocus(wnd);
}
