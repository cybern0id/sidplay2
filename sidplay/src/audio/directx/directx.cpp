/***************************************************************************
                          audiodrv.cpp  -  ``DirectSound for Windows''
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
 ***************************************************************************/

#include "directx.h"
#ifdef   HAVE_DIRECTX

#include <stdio.h>
#ifdef HAVE_EXCEPTIONS
#   include <new>
#endif

#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }

Audio_DirectX::Audio_DirectX ()
{
    isOpen     = false;
    lpdsNotify = 0;
    lpDsb      = 0;
    lpds       = 0;
}

Audio_DirectX::~Audio_DirectX()
{
    close();
}

// Need this to setup DirectX
HWND Audio_DirectX::GetConsoleHwnd ()
{   // Taken from Microsoft Knowledge Base
    // Article ID: Q124103
    #define MY_bufSize 1024 // buffer size for console window totles
    HWND hwndFound;         // this is whta is returned to the caller
    char pszNewWindowTitle[MY_bufSize]; // contains fabricated WindowTitle
    char pszOldWindowTitle[MY_bufSize]; // contains original WindowTitle

    // fetch curent window title
    GetConsoleTitle (pszOldWindowTitle, MY_bufSize);

    // format a "unique" NewWindowTitle
    wsprintf (pszNewWindowTitle, "%d/%d", GetTickCount (),
        GetCurrentProcessId ());

    // change the window title
    SetConsoleTitle (pszNewWindowTitle);

    // ensure window title has been updated
    Sleep (40);

    // look for NewWindowTitle
    hwndFound = FindWindow (NULL, pszNewWindowTitle);

    // restore original window title
    SetConsoleTitle (pszOldWindowTitle);
    return (hwndFound);
}

void *Audio_DirectX::open (AudioConfig &cfg)
{
    HWND hwnd;
    // Assume we have a console.  Use other other
    // if we have a non console Window
    hwnd = GetConsoleHwnd ();
    return open (cfg, hwnd);
}

void *Audio_DirectX::open (AudioConfig &cfg, HWND hwnd)
{ 
    DSBUFFERDESC        dsbdesc; 
    LPDIRECTSOUNDBUFFER lpDsbPrimary;
    WAVEFORMATEX        wfm;
    DWORD               dwBytes;
    int i;

    if (isOpen)
    {
        _errorString = "DIRECTX ERROR: Audio device already open.";
        return NULL;
    }
    isOpen = true;

    for (i = 0; i < AUDIO_DIRECTX_BUFFERS; i++) 
        rghEvent[i] = CreateEvent(NULL, FALSE, FALSE, NULL);

    if (FAILED (DirectSoundCreate (NULL, &lpds, NULL)))
    {
        _errorString = "DIRECTX ERROR: Could not open audio device.";
        return NULL;
    }
    if (FAILED (lpds->SetCooperativeLevel (hwnd, DSSCL_PRIORITY)))
    {
        _errorString = "DIRECTX ERROR: Could not set cooperative level.";
        return NULL;
    }

    // Primary Buffer Setup
    memset (&dsbdesc, 0, sizeof(DSBUFFERDESC));
    dsbdesc.dwSize        = sizeof(DSBUFFERDESC);
    dsbdesc.dwFlags       = DSBCAPS_PRIMARYBUFFER; 
    dsbdesc.dwBufferBytes = 0;
    dsbdesc.lpwfxFormat   = NULL;

    // Format
    memset (&wfm, 0, sizeof(WAVEFORMATEX));
    wfm.wFormatTag      = WAVE_FORMAT_PCM;
    wfm.nChannels       = cfg.channels;
    wfm.nSamplesPerSec  = cfg.frequency;
    wfm.wBitsPerSample  = cfg.precision;
    wfm.nBlockAlign     = wfm.wBitsPerSample / 8 * wfm.nChannels;
    wfm.nAvgBytesPerSec = wfm.nSamplesPerSec * wfm.nBlockAlign;

    if (FAILED (lpds->CreateSoundBuffer(&dsbdesc, &lpDsbPrimary, NULL)))
    {
        _errorString = "DIRECTX ERROR: Unable to create sound buffer.";
	  return NULL;
    }
    if (FAILED (lpDsbPrimary->SetFormat(&wfm)))
    {
        _errorString = "DIRECTX ERROR: Unable to setup required sampling format.";
        lpDsbPrimary->Release ();
	  return NULL;
    }
    lpDsbPrimary->Release ();

    // Rev 2.0.4 (saw) - Need about a secs worth to work well
    // Rev 1.5 (saw) - Buffer size reduced to 2 blocks of 250ms
    bufSize = wfm.nAvgBytesPerSec / 4;
    if (wfm.nAvgBytesPerSec & 0x3)
        bufSize++;

    // Allocate secondary buffers
    memset (&dsbdesc, 0, sizeof(DSBUFFERDESC));
    dsbdesc.dwSize  = sizeof(DSBUFFERDESC);
    dsbdesc.dwFlags = DSBCAPS_CTRLPOSITIONNOTIFY |
        DSBCAPS_GETCURRENTPOSITION2 | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLPAN;
    dsbdesc.dwBufferBytes = bufSize * AUDIO_DIRECTX_BUFFERS;
    dsbdesc.lpwfxFormat   = &wfm;

    if (FAILED (lpds->CreateSoundBuffer(&dsbdesc, &lpDsb, NULL)))
    {
        _errorString = "DIRECTX ERROR: Could not create sound buffer.";
        return false;
    }
    lpDsb->Stop();

    // Apparently this is used for timing ------------------------
    DSBPOSITIONNOTIFY rgdscbpn[AUDIO_DIRECTX_BUFFERS];
    // Buffer Start Notification
    // Rev 2.0.4 (saw) - On starting to play a buffer
    for (i = 0; i < AUDIO_DIRECTX_BUFFERS; i++) 
    {   // Track one buffer ahead
        rgdscbpn[i].dwOffset     = bufSize * ((i + 1) % AUDIO_DIRECTX_BUFFERS);
        rgdscbpn[i].hEventNotify = rghEvent[i];
    }

    if (FAILED (lpDsb->QueryInterface (IID_IDirectSoundNotify, (VOID **) &lpdsNotify)))
    {
        _errorString = "DIRECTX ERROR: Sound interface query failed.";
        return false;
    }
    if (FAILED (lpdsNotify->SetNotificationPositions(AUDIO_DIRECTX_BUFFERS, rgdscbpn)))
    {
        _errorString = "DIRECTX ERROR: Unable to set up sound notification positions.";
        return false;
    }
    // -----------------------------------------------------------

    lpDsb->Stop ();
    if (FAILED (lpDsb->Lock (0, bufSize, &lpvData, &dwBytes, NULL, NULL, 0)))
    {
        _errorString = "DIRECTX ERROR: Unable to lock sound buffer.";
        return NULL;
    }

    // Update the users settings
    cfg.bufSize  = bufSize;
	cfg.encoding = AUDIO_UNSIGNED_PCM; // IS this write?
    _settings    = cfg;

    isPlaying = false;
    return lpvData;
}

void *Audio_DirectX::write ()
{
    DWORD dwEvt; 
    DWORD dwBytes;

    if (!isOpen)
    {
        _errorString = "DIRECTX ERROR: Device not open.";
		return NULL;
    }
	// Unlock the current buffer for playing
    lpDsb->Unlock (lpvData, bufSize, NULL, 0);

    // Check to see of the buffer is playing
    // and if not start it off
    if (!isPlaying)
    {
        isPlaying = true;
        // Rev 1.7 (saw) - Set the play position back to the begining
        if (FAILED (lpDsb->SetCurrentPosition(0)))
		{
            _errorString = "DIRECTX ERROR: Unable to set play position to start of buffer.";
		    return NULL;
	    }

        if (FAILED (lpDsb->Play (0,0,DSBPLAY_LOOPING)))
		{
            _errorString = "DIRECTX ERROR: Unable to start playback.";
		    return NULL;
	    }
    }

	// Check the incoming event to make sure it's one of our event messages and
	// not something else
	do
	{
	    dwEvt  = MsgWaitForMultipleObjects (AUDIO_DIRECTX_BUFFERS, rghEvent, FALSE, INFINITE, QS_ALLINPUT);
	    dwEvt -= WAIT_OBJECT_0; 
	} while (dwEvt >= AUDIO_DIRECTX_BUFFERS);

    // Lock the next buffer for filling
    if (FAILED (lpDsb->Lock (bufSize * dwEvt, bufSize, &lpvData, &dwBytes, NULL, NULL, 0)))
    {
        _errorString = "DIRECTX ERROR: Unable to lock sound buffer.";
		return NULL;
    }
    return lpvData;
}

void *Audio_DirectX::reset (void)
{
    DWORD dwBytes;
    if (!isOpen)
         return NULL;

    // Stop play and kill the current music.
    // Start new music data being added at the begining of
    // the first buffer
    lpDsb->Stop ();
	// Rev 1.7 (saw) - Prevents output going silent after reset
    isPlaying = false;
    lpDsb->Unlock (lpvData, bufSize, NULL, 0);

    // Rev 1.4 (saw) - Added as lock can fail.
    if (FAILED (lpDsb->Lock (0, bufSize, &lpvData, &dwBytes, NULL, NULL, 0)))
    {
        _errorString = "DIRECTX ERROR: Unable to lock sound buffer.";
        return NULL;
    }		
    return lpvData;
}

// Rev 1.8 (saw) - Alias fix
void Audio_DirectX::close (void)
{
    if (!isOpen)
        return;
    isOpen = false;

    if (lpDsb)
    {
		lpDsb->Stop();
        isPlaying = false;
		// Rev 1.4 (iv) - Unlock before we release buffer.
        lpDsb->Unlock (lpvData, bufSize, NULL, 0);      
    }

    SAFE_RELEASE (lpdsNotify);
    SAFE_RELEASE (lpDsb);
    SAFE_RELEASE (lpds);

    // Rev 1.3 (Ingve Vormestrand) - Changed "<=" to "<"
    // as closing invalid handle.
    for (int i=0;i < AUDIO_DIRECTX_BUFFERS; i++) 
        CloseHandle (rghEvent[i]);
}

#endif // HAVE_DIRECTX
