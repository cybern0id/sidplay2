/***************************************************************************
                          audiodrv.h  -  ``DirectSound for Windows''
                                         specific audio driver interface.
                             -------------------
    begin                : Mon Jul 31 2000
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
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 ***************************************************************************/

#ifndef audio_directx_h_
#define audio_directx_h_

#include "config.h"
#ifdef   HAVE_DIRECTX
#   ifndef AudioDriver
#   define AudioDriver Audio_DirectX
#   endif
#endif

#if DIRECTSOUND_VERSION < 0x0500
#   undef  DIRECTSOUND_VERSION
#   define DIRECTSOUND_VERSION 0x0500       /* version 5.0 */ 
#endif

#include <DSound.h>
#include <mmsystem.h>

#include "../AudioBase.h"
#define AUDIO_DIRECTX_BUFFERS 2

class Audio_DirectX: public AudioBase
{	
private:  // ------------------------------------------------------- private
    HWND   hwnd;

    // DirectSound Support
    LPDIRECTSOUND       lpds;
    LPDIRECTSOUNDBUFFER lpDsb;
    LPDIRECTSOUNDNOTIFY lpdsNotify;
    void               *lpvData;
    // DirectSound Notify
    HANDLE rghEvent[AUDIO_DIRECTX_BUFFERS];
    DWORD  bufSize;

    bool isOpen;
    bool isPlaying;

private:
    HWND GetConsoleHwnd ();

public:  // --------------------------------------------------------- public
    Audio_DirectX();
    ~Audio_DirectX();

    // This first one assumes progrm is built as a
    // console application
    void *open  (AudioConfig &cfg);
    void *open  (AudioConfig &cfg, HWND hwnd);
    void  close ();	
    // Rev 1.3 (saw) - Changed
    void *reset ();
    void *write ();
};

#endif // HAVE_DIRECTX
#endif // audio_directx_h_
