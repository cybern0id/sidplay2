#include <stdlib.h>
#include <string.h>
#include <sidplay/sidplay2.h>
#include "IniConfig.h"

#define SAFE_FREE(p) { if(p) { free (p); (p)=NULL; } }
const char *IniConfig::DIR_NAME  = ".sidplay";
const char *IniConfig::FILE_NAME = "sidplay2.ini";

IniConfig::IniConfig ()
:status(true),
 database(NULL)
{   // Initialise everything else
    emulation_s.filter6581 = NULL;
    emulation_s.filter8580 = NULL;
    clear ();
}

IniConfig::~IniConfig ()
{
    clear ();
}

void IniConfig::clear ()
{
    SAFE_FREE (database);

    console_s.ansi          = false;
    console_s.topLeft       = '+';
    console_s.topRight      = '+';
    console_s.bottomLeft    = '+';
    console_s.bottomRight   = '+';
    console_s.vertical      = '|';
    console_s.horizontal    = '-';
    console_s.junctionLeft  = '+';
    console_s.junctionRight = '+';

    audio_s.frequency = SID2_DEFAULT_SAMPLING_FREQ;
    audio_s.playback  = sid2_mono;
    audio_s.precision = SID2_DEFAULT_PRECISION;

    emulation_s.clockSpeed    = SID2_CLOCK_CORRECT;
    emulation_s.clockForced   = false;
    emulation_s.sidModel      = SID2_MOS6581;
    emulation_s.filter        = true;
    emulation_s.extFilter     = true;
    emulation_s.optimiseLevel = SID2_DEFAULT_OPTIMISATION;
    emulation_s.sidSamples    = true;

    SAFE_FREE (emulation_s.filter6581);
    SAFE_FREE (emulation_s.filter8580);
}


bool IniConfig::readInt (ini_fd_t ini, char *key, int &value)
{
    int i = value;
    if (ini_locateKey (ini, key) < 0)
    {   // Dosen't exist, add it
        (void) ini_writeString (ini, "");
    }
    if (ini_readInt (ini, &i) < 0)
        return false;
    value = i;
    return true;
}

bool IniConfig::readString (ini_fd_t ini, char *key, char *&str)
{
    char  *ret;
    size_t length;

    if (ini_locateKey (ini, key) < 0)
    {   // Dosen't exist, add it
        (void) ini_writeString (ini, "");
    }

    length = (size_t) ini_dataLength (ini);
    if (!length)
        return 0;

    ret = (char *) malloc (++length);
    if (!ret)
        return false;
    
    if (ini_readString (ini, ret, (uint) length) < 0)
        goto IniCofig_readString_error;

    str = ret;
return true;

IniCofig_readString_error:
    if (str)
        free (str);
    return false;
}


bool IniConfig::readBool (ini_fd_t ini, char *key, bool &boolean)
{
    int  i   = -1;
    bool ret = readInt (ini, key, i);
    if (!ret)
        return false;

    // Check with boolean limits
    if (i < 0 || i > 1)
        return false;
    boolean = (i != 0);
    return true;
}


bool IniConfig::readChar (ini_fd_t ini, char *key, char &ch)
{
    char *str;
    bool  ret = readString (ini, key, str);
    if (!ret)
        return false;

    ret = true;
    // Check if we have an actual chanracter
    if (str[0] == '\'')
    {
        if (str[2] != '\'')
	    ret = false;
        else
            ch = str[1];
    } // Nope is number
    else
      ch = (char) atoi (str);

    free (str);
    return ret;
}


bool IniConfig::readConsole (ini_fd_t ini)
{
    bool ret = true;
    (void) ini_locateHeading (ini, "Console");
    status &= readBool (ini, "Ansi",                console_s.ansi);
    status &= readChar (ini, "Char Top Left",       console_s.topLeft);
    status &= readChar (ini, "Char Top Right",      console_s.topRight);
    status &= readChar (ini, "Char Bottom Left",    console_s.bottomLeft);
    status &= readChar (ini, "Char Bottom Right",   console_s.bottomRight);
    status &= readChar (ini, "Char Vertical",       console_s.vertical);
    status &= readChar (ini, "Char Horizontal",     console_s.horizontal);
    status &= readChar (ini, "Char Junction Left",  console_s.junctionLeft);
    status &= readChar (ini, "Char Junction Right", console_s.junctionRight);
    return ret;
}


bool IniConfig::readAudio (ini_fd_t ini)
{
    bool ret = true;
    (void) ini_locateHeading (ini, "Audio");

	{
		int frequency = (int) audio_s.frequency;
        status &= readInt (ini, "Frequency", frequency);
        audio_s.frequency = (unsigned long) frequency;
	}

	{
		int channels = 0;
        status &= readInt (ini, "Channels",  channels);
		if (channels)
		{
			audio_s.playback = sid2_mono;
			if (channels != 1)
				audio_s.playback = sid2_stereo;
		}
	}

    status &= readInt (ini, "BitsPerSample", audio_s.precision);
    return ret;
}


bool IniConfig::readEmulation (ini_fd_t ini)
{
    bool ret = true;
    (void) ini_locateHeading (ini, "Emulation");

    {
        int clockSpeed = -1;
        status &= readInt (ini, "ClockSpeed", clockSpeed);
        if (clockSpeed != -1)
        {
            emulation_s.clockSpeed = SID2_CLOCK_PAL;
            if (clockSpeed)
                emulation_s.clockSpeed = SID2_CLOCK_NTSC;
        }
    }

    status &= readBool (ini, "ForceSongSpeed", emulation_s.clockForced);

    {
        bool mos8580 = false;
        status &= readBool (ini, "MOS8580", mos8580);
        if (mos8580)
            emulation_s.sidModel = SID2_MOS8580;
    }

    status &= readBool (ini, "UseFilter",    emulation_s.filter);
    status &= readBool (ini, "UseExtFilter", emulation_s.extFilter);

    {
        int optimiseLevel = -1;
        status &= readInt  (ini, "OptimiseLevel", optimiseLevel);
        if (optimiseLevel != -1)
            emulation_s.optimiseLevel = optimiseLevel;
    }

    status &= readString (ini, "Filter6581", emulation_s.filter6581);
    status &= readString (ini, "Filter8580", emulation_s.filter8580);
    status &= readBool   (ini, "SidSamples", emulation_s.sidSamples);

    // These next two change the ini section!
	if (emulation_s.filter6581)
	{   // Try to load the filter
        filter6581.read (ini, emulation_s.filter6581);
		if (!filter6581)
		{
            filter6581.read (emulation_s.filter6581);
			if (!filter6581)
				status = false;
		}
    }

	if (emulation_s.filter8580)
	{   // Try to load the filter
        filter8580.read (ini, emulation_s.filter8580);
		if (!filter8580)
		{
            filter8580.read (emulation_s.filter8580);
			if (!filter8580)
				status = false;
		}
    }

    return ret;
}


void IniConfig::read ()
{
    char   *path = (char *) getenv ("HOME");
    ini_fd_t ini  = 0;
    char   *configPath;
    size_t  length;

    if (!path)
        path = (char *) getenv ("windir");

    if (!path)
        path = "";

    length     = strlen (path) + strlen (DIR_NAME) + strlen (FILE_NAME) + 3;
    configPath = (char *) malloc (length);
    if (!configPath)
        goto IniConfig_read_error;

    {   // Format path from system
        char *s = path;
		while (*s != '\0')
		{
			if (*s == '\\')
				*s = '/';
			s++;
		}
	}

#ifdef HAVE_UNIX
    sprintf (configPath, "%s/%s/%s", path, DIR_NAME, FILE_NAME);
#else
    sprintf (configPath, "%s/%s", path, FILE_NAME);
#endif

    // Opens an existing file or creates a new one
    ini = ini_new (configPath);

    // Unable to open file?
    if (!ini)
        goto IniConfig_read_error;

    clear ();

    // This may not exist here...
    (void) ini_locateHeading (ini, "SIDPlay2");
    status &= readString    (ini, "Songlength Database", database);
    status &= readConsole   (ini);
    status &= readAudio     (ini);
    status &= readEmulation (ini);
    ini_close (ini);
return;

IniConfig_read_error:
    if (ini)
        ini_close (ini);
    clear ();
    status = false;
}

const sid_filter_t* IniConfig::filter (sid2_model_t model)
{
	if (model == SID2_MOS8580)
	    return filter8580.definition ();
	return filter6581.definition ();
}
