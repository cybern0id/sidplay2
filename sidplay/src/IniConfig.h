#ifndef _IniConfig_h_
#define _IniConfig_h_

#include <sidplay/sidtypes.h>
#include <sidplay/utils/libini.h>
#include <sidplay/utils/SidFilter.h>

class IniConfig
{
public:
    struct console_section
    {   // INI Section - [Console]
        bool ansi;
        char topLeft;
        char topRight;
        char bottomLeft;
        char bottomRight;
        char vertical;
        char horizontal;
        char junctionLeft;
        char junctionRight;
    };

    struct audio_section
    {   // INI Section - [Audio]
        long frequency;
        sid2_playback_t playback;
        int  precision;
    };

    struct emulation_section
    {   // INI Section - [Emulation]
        sid2_clock_t  clockSpeed;
        bool          clockForced;
        sid2_model_t  sidModel; 
        bool          filter;
        bool          extFilter;
        char         *filter6581;
        char         *filter8580;
        uint_least8_t optimiseLevel;
        bool          sidSamples;
    };

protected:
    static const char *DIR_NAME;
    static const char *FILE_NAME;

    bool      status;
    char     *database;
    struct    console_section   console_s;
    struct    audio_section     audio_s;
    struct    emulation_section emulation_s;
	SidFilter filter6581;
	SidFilter filter8580;

protected:
    void  clear ();

    bool  readInt    (ini_fd_t ini, char *key, int &value);
    bool  readString (ini_fd_t ini, char *key, char *&str);
    bool  readBool   (ini_fd_t ini, char *key, bool &boolean);
    bool  readChar   (ini_fd_t ini, char *key, char &ch);

    bool  readConsole   (ini_fd_t ini);
    bool  readAudio     (ini_fd_t ini);
    bool  readEmulation (ini_fd_t ini);

public:
    IniConfig  ();
    ~IniConfig ();

    void read ();
    operator bool () { return status; }

    // Sidplay2 Specific Section
    const char*              songLengthDB () { return database; }
    const console_section&   console      () { return console_s; }
    const audio_section&     audio        () { return audio_s; }
    const emulation_section& emulation    () { return emulation_s; }
	const sid_filter_t*      filter       (sid2_model_t model);
};

#endif // _IniConfig_h_
