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

   $Id: configdlg.cpp,v 1.1 2004-02-27 16:30:29 vividos Exp $

*/
/* \file configdlg.cpp

   \brief config dialog for a Sid File

   This file contains the config dialog for the in_sid2 Winamp plugin.
   It lets the user configure certain aspects of the plugin.

*/

// needed includes
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "resource.h"

#include "wa2player.h"


// extern references

extern wa2_player* player;
extern "C" int _snprintf( char *buffer, size_t count, const char *format, ... );


// global variables
wa2_player_cfg* curcfg;


// functions

// dialog proc function for info dialog
int CALLBACK
ConfigDlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
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
         // set config options
         char buffer[32];
         _snprintf(buffer,sizeof(buffer),"%5d",curcfg->samplerate);

         // "Samplerate" combobox
         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_ADDSTRING, 0, (LPARAM)"44100");
         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_ADDSTRING, 0, (LPARAM)"22050");
         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_ADDSTRING, 0, (LPARAM)"11025");
         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_SETITEMDATA, 0, 44100);
         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_SETITEMDATA, 1, 22050);
         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_SETITEMDATA, 2, 11025);

         SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_SELECTSTRING,
            (WPARAM)-1, (LPARAM)buffer);

         // "SID Model" combobox
         SendDlgItemMessage(hWnd, IDC_COMBO_SIDMODEL, CB_ADDSTRING, 0, (LPARAM)"best");
         SendDlgItemMessage(hWnd, IDC_COMBO_SIDMODEL, CB_ADDSTRING, 0, (LPARAM)"SID 6581 (old model)");
         SendDlgItemMessage(hWnd, IDC_COMBO_SIDMODEL, CB_ADDSTRING, 0, (LPARAM)"SID 8580 (new model)");

         SendDlgItemMessage(hWnd, IDC_COMBO_SIDMODEL, CB_SETCURSEL,
            curcfg->sid_model, 0);

         // "use SID filter" checkbox
         CheckDlgButton(hWnd, IDC_CHECK_SIDFILTER, curcfg->use_filters ? BST_CHECKED : BST_UNCHECKED);

         // "Stereo" checkbox
         CheckDlgButton(hWnd, IDC_CHECK_STEREO, curcfg->stereo ? BST_CHECKED : BST_UNCHECKED);

         // "Optimization" radio buttons
         CheckDlgButton(hWnd, IDC_RADIO_OPT_LEVEL1+curcfg->optlevel, BST_CHECKED);

         // "Title Format" editbox
         SetDlgItemText(hWnd,IDC_EDIT_TITLE_FMT,curcfg->title_fmt.c_str());
      }
      break;

   case WM_DESTROY:
      {
         // get config options

         // "Samplerate" combobox
         int sel = SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_GETCURSEL, 0, 0);
         curcfg->samplerate = SendDlgItemMessage(hWnd, IDC_COMBO_SAMPLERATE, CB_GETITEMDATA, sel, 0);

         // "SID Model" combobox
         curcfg->sid_model = SendDlgItemMessage(hWnd, IDC_COMBO_SIDMODEL, CB_GETCURSEL, 0, 0);

         // "use SID filter" checkbox
         curcfg->use_filters = BST_CHECKED == IsDlgButtonChecked(hWnd, IDC_CHECK_SIDFILTER);

         // "Stereo" checkbox
         curcfg->stereo = BST_CHECKED == IsDlgButtonChecked(hWnd, IDC_CHECK_STEREO);

         // "Optimization" radio buttons
         sel = IsDlgButtonChecked(hWnd, IDC_RADIO_OPT_LEVEL1);
         curcfg->optlevel = sel == BST_CHECKED ? 0 : 1;

         // "Title Format" editbox
         char buffer[64];
         GetDlgItemText(hWnd,IDC_EDIT_TITLE_FMT,buffer,sizeof(buffer));
         curcfg->title_fmt.assign(buffer);
      }
      break;

   default:
      return FALSE;
   }

   return TRUE;
}

bool wa2_config_dlg(HINSTANCE inst, HWND wnd)
{
   curcfg = &player->get_cfg();

   // show dialog box
   int ret = ::DialogBoxParam(inst,MAKEINTRESOURCE(IDD_DIALOG_CONFIG),wnd,
      &ConfigDlgProc,(LPARAM)NULL);

   curcfg = NULL;

   // reset focus on Winamp window
   SetFocus(wnd);

   return ret == IDOK;
}
