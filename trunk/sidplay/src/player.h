/***************************************************************************
                          player.h  -  Frontend Player
                             -------------------
    begin                : Sun Oct 7 2001
    copyright            : (C) 2001 by Simon White
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

#ifndef _player_h_
#define _player_h_

#include "config.h"

#ifdef WITH_SIDPLAY
#   include <sidplay/sidplay.h>
#   define  sidplay2 sidplay
#else
#   include <sidplay/sidplay2.h>
#endif

#include <sidplay/utils/SidDatabase.h>
#include "audio/AudioDrv.h"
#include "IniConfig.h"

typedef enum {black, red, green, yellow, blue, magenta, cyan, white}
    player_colour_t;
typedef enum {tableStart, tableMiddle, tableSeperator, tableEnd}
    player_table_t;
typedef enum {playerError, playerRunning, playerPaused,
              playerRestart, playerStopped, playerExit} player_state_t;

typedef enum {/* Same as EMU_DEFAULT except no soundcard.
                 Still allows wav generation */
              EMU_NONE = 0,
              /* The following require a soundcard */
              EMU_DEFAULT, EMU_RESID, EMU_SIDPLAY1,
              /* The following should disable the soundcard */
              EMU_HARDSID, EMU_SIDSTATION, EMU_COMMODORE,
              EMU_SIDSYN, EMU_END} SIDEMUS;

typedef enum {/* Define possible output sources */
              OUT_NULL = 0,
              /* Hardware */
              OUT_SOUNDCARD,
              /* File creation support */
              OUT_WAV, OUT_AU, OUT_END} OUTPUTS;

// Error and status message numbers.
enum
{
    ERR_SYNTAX = 0,
    ERR_NOT_ENOUGH_MEMORY,
    ERR_SIGHANDLER,
    ERR_FILE_OPEN
};

void displayError (const char *arg0, uint num);


// Grouped global variables
class Player: public Event
{
private:
    static const char  RESID_ID[];
    static const char  HARDSID_ID[];

    const char* const  m_name; 
    sidplay2           m_engine;
    sid2_config_t      m_engCfg;
    SidTuneMod         m_tune;
    player_state_t     m_state;
    const char*        m_outfile;
    EventContext      *m_context;

    IniConfig          m_iniCfg;
    SidDatabase        m_database;

    // Display parameters
    uint_least8_t      m_quietLevel;
    uint_least8_t      m_verboseLevel;

    struct m_filter_t
    {
        SidFilter      definition;
	    bool           enabled;
    } m_filter;

    struct m_driver_t
    {
        OUTPUTS        output;   // Selected output type
        SIDEMUS        sid;      // Sid emulation
        bool           file;     // File based driver
        AudioConfig    cfg;
        AudioBase*     selected; // Selected Output Driver
        AudioBase*     device;   // HW/File Driver
        Audio_Null     null;     // Used for everything
    } m_driver;

    struct m_timer_t
    {   // secs
        uint_least32_t start;
        uint_least32_t current;
        uint_least32_t stop;
        uint_least32_t length;
        bool           valid;
    } m_timer;

    struct m_track_t
    {
        uint_least16_t first;
        uint_least16_t selected;
        uint_least16_t songs;
        bool           loop;
        bool           single;
    } m_track;

    struct m_speed_t
    {
        uint_least8_t current;
        uint_least8_t max;
    } m_speed;

private:
    // Console
    void consoleColour  (player_colour_t colour, bool bold);
    void consoleTable   (player_table_t table);
    void consoleRestore (void);

    // Command line args
    bool parseTime      (char *str, uint_least32_t &time);
    void displayArgs    ();

    bool createOutput   (OUTPUTS driver, const SidTuneInfo *tuneInfo);
    bool createSidEmu   (SIDEMUS emu);
    void displayError   (const char *error);
    void displayError   (uint num) { ::displayError (m_name, num); }
    void decodeKeys     (void);
    void event          (void);
    void emuflush       (void);
    void menu           (void);

public:
    Player (const char * const name);

    bool args    (int argc, char *argv[]);
    bool open    (void);
    void close   (void);
    bool play    (void);
    void stop    (void);
    bool restart (void) { return m_state == playerRestart; }
};

#endif // _player_h_
