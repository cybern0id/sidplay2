/***************************************************************************
                          sid2_plugin.cpp  -  Main plugin interface
                             -------------------
    begin                : Sat Nov 25 2000
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sidplay/sidplay2.h>

//#define DEBUG 1

extern "C" {
#include <xmms/plugin.h> // XMMS hooking stuff
#include <xmms/util.h>   // usleep

static void sid_init         (void);
static int  sid_our_file     (char *filename);
static void sid_play         (char *filename);
static void sid_stop         (void);
static void sid_subtune      (int tune);
static void sid_pause        (short paused);
static void sid_song_info    (char *filename, char **title, int *length);
static int  sid_time         (void);
}

static void sid_create_title (const SidTuneInfo &info, char **title);

#ifdef DEBUG
    static class loadCheck_t
    {
    public:
        loadCheck_t ()
        {
	    printf ("Loaded\n");
        }
    } loadCheck;
#endif
        

// ==================================================================
// InputPlugin structure
// ==================================================================

InputPlugin sid_ip =
{
    NULL,                              // handle (internal)
    NULL,                              // filename (internal)
    NULL,                              // description
    sid_init,                          // init 
    NULL,                              // about 
    NULL,                              // configure 
    sid_our_file,                      // is_our_file 
    NULL,                              // scan_dir 
    sid_play,                          // play_file 
    sid_stop,                          // stop 
    sid_pause,                         // pause 
    sid_subtune,                       // seek
    NULL,                              // seek_eq 
    sid_time,                          // get_time 
    NULL,                              // get_volume 
    NULL,                              // set_volume 
    NULL,                              // add_vis (obsolete) 
    NULL,                              // get_vis_type (obsolete) 
    NULL,                              // add_vis_pcm -- this can be done 
    NULL,                              // set_info 
    NULL,                              // set_info_text 
    sid_song_info,                     // get_song_info 
    NULL,                              // file_info_box -- this can be done 
    NULL                               // output (internal) 
};

// ==================================================================
// Other variables
// ==================================================================

static pthread_t       play_thread;
static pthread_mutex_t lib_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile bool   running   = false;

static struct sidplayer_t
{
    sidplay2       lib;
    bool           error;
    uint_least8_t  selectedSong;
    uint_least8_t *buffer;
    uint_least32_t bufferSize;
    char          *filename;
} sidplayer;


// ==================================================================
// The things that make the world go 'round (functions)
// ==================================================================

extern "C" InputPlugin *get_iplugin_info(void)
{
    char buffer[200] = "SIDPlay2 Player";
#ifdef DEBUG
    printf ("get_iplugin_info\n");
#endif

    sidplayer.error = false;
    if (!sidplayer.lib)
    {
        sidplayer.error = true;
        sprintf (buffer, "%s (Failied Init)", buffer);
    }
    // No way to reclaim this!
    sid_ip.description = strdup (buffer);
    return &sid_ip;
}

void sid_init (void)
{
#ifdef DEBUG
    printf ("sid_init\n");
#endif

    pthread_mutex_lock   (&lib_mutex);
    // Run with les cpu overhead
    sidplayer.lib.optimisation (1);
    sidplayer.lib.configure    (sid2_stereo, SID2_DEFAULT_SAMPLING_FREQ, SID2_DEFAULT_PRECISION, false);
    pthread_mutex_unlock (&lib_mutex);
}

int sid_our_file (char *filename)
{
    int ret;

#ifdef DEBUG
    printf ("sid_our_file (%s) ", filename);
#endif
    // If an errors occured do not claim the file
    if (sidplayer.error)
        return 0;

    {
        SidTune tune(filename);
        if (!tune)
	{
#ifdef DEBUG
            printf ("[No]\n");
#endif
            return 0;
        }
    }
#ifdef DEBUG
    printf ("[Yes]\n");
#endif
    return 1;
}

static void *playloop (void *arg)
{
    uint_least8_t *buffer = sidplayer.buffer;
    int            length = sidplayer.bufferSize;
    
    while (running)
    {
        pthread_mutex_lock   (&lib_mutex);
        (void) sidplayer.lib.play (buffer, length);
        pthread_mutex_unlock (&lib_mutex);
	sid_ip.add_vis_pcm   (sid_ip.output->written_time(), FMT_S16_LE, 2, length, buffer);
        while ((sid_ip.output->buffer_free() < length) && running)
            xmms_usleep(10000);
        sid_ip.output->write_audio (buffer, length);
    }
}

// Assume file to have been loaded by 
void sid_play (char *filename)
{
    int ret;

#ifdef DEBUG
    printf ("sid_play\n");
#endif

    sid_ip.filename        = filename;
    sidplayer.selectedSong = 0;
    pthread_mutex_lock   (&lib_mutex);
    ret = sidplayer.lib.loadSong (filename, sidplayer.selectedSong);

    if (!(ret < 0))
    {   // Get tune number we ended up on
        sid2_playerInfo_t playerInfo;
        sidplayer.lib.getInfo (&playerInfo);
        sidplayer.selectedSong = playerInfo.tuneInfo.currentSong;
    }
    pthread_mutex_unlock (&lib_mutex);
    if (ret < 0)
        return;

    // Allocate buffer (250 msec)
    sidplayer.bufferSize = 512 * (SID2_DEFAULT_PRECISION / 8) * 2;
    sidplayer.buffer     = (uint_least8_t *) malloc (sizeof (uint_least8_t) * sidplayer.bufferSize);
    if (!sidplayer.buffer)
        return;

    // OPEN AUDIO.
    if (sid_ip.output->open_audio(FMT_S16_LE,
	                          SID2_DEFAULT_SAMPLING_FREQ,
                                  2) /* Stereo */ == 0)
    {
	return;
    }

    {   // Bits per second
        int  rate = SID2_DEFAULT_SAMPLING_FREQ * SID2_DEFAULT_PRECISION * 2;
        int  length;
        char *title;
        sid_song_info   (filename, &title, &length);
        sid_ip.set_info (title, length, rate, SID2_DEFAULT_SAMPLING_FREQ, 2);
    }

    running = true;
    pthread_create (&play_thread, NULL, playloop, NULL);
}

void sid_stop (void)
{
#ifdef DEBUG
    printf ("sid_stop\n");
#endif
    running = false;
    pthread_join (play_thread, NULL);
    sid_ip.output->close_audio();
    pthread_mutex_lock   (&lib_mutex);
    sidplayer.lib.stop   ();
    pthread_mutex_unlock (&lib_mutex);
    free (sidplayer.buffer);
}


void sid_subtune (int tune)
{
    bool was_running = false;
    int  ret;

#ifdef DEBUG
    printf ("sid_subtune\n");
#endif
    if(running)
    {   // Basically copied from above.
        was_running = true;
        running     = false;
        pthread_join (play_thread, NULL);
    }

    pthread_mutex_lock   (&lib_mutex);
    {   // Get tune number we ended up on
        sid2_playerInfo_t playerInfo;
        sidplayer.lib.getInfo (&playerInfo);

        if (tune > 0)
            sidplayer.selectedSong++;
        else
            sidplayer.selectedSong = 1;
        if (sidplayer.selectedSong > playerInfo.tuneInfo.songs)
            sidplayer.selectedSong = 1;
    }
    (void) sidplayer.lib.loadSong (sidplayer.selectedSong);

    {   // Get tune number we ended up on
        sid2_playerInfo_t playerInfo;
        sidplayer.lib.getInfo (&playerInfo);
        sidplayer.selectedSong = playerInfo.tuneInfo.currentSong;
    }
    pthread_mutex_unlock (&lib_mutex);
    sid_ip.output->flush (0);

    {   // Redefine song name
        char *title;
        sid2_playerInfo_t playerInfo;
        sidplayer.lib.getInfo (&playerInfo);
        sid_create_title (playerInfo.tuneInfo, &title);
        if (title)
            sid_ip.set_info_text (title);
    }

    if (was_running)
    {
        running = true;
        pthread_create (&play_thread, NULL, playloop, NULL);
    }
}


void sid_pause(short paused)
{
#ifdef DEBUG
    printf ("sid_pause\n");
#endif
    pthread_mutex_lock (&lib_mutex);
    if (paused)
        sidplayer.lib.pause ();
    pthread_mutex_unlock (&lib_mutex);
    sid_ip.output->pause (paused);
}


void sid_song_info (char *filename, char **title, int *length)
{ 
#ifdef DEBUG
    printf ("sid_song_info\n");
#endif
    SidTune tune(filename);
    if (!tune)
        return;
    tune.selectSong (0);
    *length = 1000 * 10 * 60;
    sid_create_title (tune.getInfo (), title);

    if (!*title)
    {   // Best Guess Title
        char *name;
        name = strrchr (filename, '/');
        if (!name)
	    name = filename;
        else
            name++;
        name   = strdup (name);
        *title = name;

        if (name)
        {   // Memory alloc can fail
            name = strrchr (name, '.');
            if (name)
                *name = '\0';
        }
    }

#ifdef DEBUG
    printf ("Title: %s\n", *title); 
#endif
}


void sid_create_title (const SidTuneInfo &info, char **title)
{
    *title = NULL;
    if (info.numberOfInfoStrings == 3)
    {
        char buffer[200];
        sprintf (buffer, "[%s] %s %u/%u (%s)", info.infoString[1], info.infoString[0],
                 info.currentSong, info.songs, info.infoString[2]);
        *title = strdup (buffer);
    }
}


int sid_time (void)
{
    int msecs;
    //    pthread_mutex_lock   (&lib_mutex);
    // Convert seconds to milli-seconds for xmms
    // msecs = sidplayer.lib.time ();
    //    pthread_mutex_unlock (&lib_mutex);
    msecs = sid_ip.output->output_time();
#if DEBUG > 1
    printf ("sid_time [%lu]\n", msecs);
#endif
    return msecs;
}
