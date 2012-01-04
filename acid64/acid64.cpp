/***************************************************************************
                          acid64.cpp  -  Emulation of acid64.dll
                             -------------------
    begin                : Sat Dec 24 2011
    copyright            : (C) 2011 by Simon White
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

#include <windows.h>
#include <algorithm>
#include <vector>
#include <sidplay/sidplay2.h>
#include <sidplay/sidlazyiptr.h>
#include <sidplay/utils/SidDatabase.h>
#include <sidplay/utils/SidTuneMod.h>
#include <stil.h>

#include "acid64-cmd.h"
#include "acid64-builder.h"

typedef void* handle_t;

typedef enum command_t
{
  ACID64_CMD_IDLE      = 0,
  ACID64_CMD_DELAY     = 1,
  ACID64_CMD_WRITE     = 2,
  ACID64_CMD_READ      = 3,
  ACID64_CMD_NEXT_PART = 4
} command_t;

static SidDatabase g_database;
static STIL        g_stil;

struct Acid64: public Acid64Cmd
             , public Event
{
    Acid64        * const me;
    Acid64Builder         builder;
    float64_t             cpuClock;
    long                  cycleCorrection;
    SidLazyIPtr<sidplay2> engine;
    LPVOID                engineFiber;
    LPVOID                mainFiber;
    char                 *stack;
    SidTuneMod            tune;
    SidTuneInfo           tuneInfo;
    const char           *tuneMD5;
    std::string           tunePath;

    // Event
    virtual void event ();

    // Acid64 Command interface
    void    delay (event_clock_t cycles);
    void    write (event_clock_t cycles, uint_least8_t addr, uint8_t data);
    uint8_t read  (event_clock_t cycles, uint_least8_t addr);

    // Acid64 Command data
    long command;
    WORD cmdCycles;
    BYTE cmdAddr;
    BYTE cmdData;

    Acid64 ()
    :Event("ACID64 First SID Access")
    ,me(_self())
    ,builder("ACID64", *_self())
    ,tune(0)
    { 
        ;
    }

private:
    Acid64 *_self () { return this; }
};

class SafeThreadToFibre
{
public:
    SafeThreadToFibre (LPVOID &fiber)
        :m_converted(false)
        ,m_fiber(fiber)
    {
        fiber = GetCurrentFiber ();
        if ((fiber == 0) || (fiber == (LPVOID)0x1E00/*see boost*/))
        {
            fiber = ConvertThreadToFiber (NULL);
            if (!fiber)
                throw 0;
            m_converted = true;
        }
    }

    ~SafeThreadToFibre ()
    {
        if (m_converted)
            ConvertFiberToThread ();
        m_fiber = 0;
    }

private:
    bool    m_converted;
    LPVOID &m_fiber;
};
        
static void __stdcall start (LPVOID lp)
{
    Acid64 &inst = *reinterpret_cast<Acid64*>(lp);
    char buffer[8000];

    {   // Configure emulation
        sid2_config_t cfg = inst.engine->config ();
        cfg.sidEmulation  = inst.builder.iunknown ();
        cfg.precision     = 8;
        cfg.frequency     = sizeof(buffer);
        inst.engine->config (cfg);
    }

    for (;;)
    {
        SwitchToFiber (inst.mainFiber);

        do
        {
            inst.engine->play (buffer, sizeof(buffer)); // buffer, length
        } while (inst.engine->state() == sid2_playing);
    }
}

static void stop (Acid64 &inst)
{
    SafeThreadToFibre convert(inst.mainFiber);
    while (inst.engine->state() != sid2_stopped)
    {
        SwitchToFiber (inst.mainFiber);
        // Never get here
    }
    inst.cycleCorrection = 0;
}

static std::string stilEntry (const char *filepath, uint_least16_t tune)
{
    for (;;)
    {
        std::string entry;
        const char *s = g_stil.getAbsGlobalComment (filepath);
        if (!s)
        {
            s = g_stil.getGlobalComment (filepath);
            if (!s)
                break;
            entry += s;
            s = g_stil.getEntry (filepath, tune, STIL::all);
            if (!s)
                break;
            entry += s;
            s = g_stil.getBug (filepath, tune);
            if (!s)
                break;
            entry += s;
        }
        else
        {
            entry += s;
            s = g_stil.getAbsEntry (filepath, tune, STIL::all);
            if (!s)
                break;
            entry += s;
            s = g_stil.getAbsBug (filepath, tune);
            if (!s)
                break;
            entry += s;
        }
    }
    return std::string();
}

void Acid64::delay (event_clock_t cycles)
{
    cpuClock += cycles;
    if (!pending())
    {
        cycleCorrection += (long)cycles;
        while (cycleCorrection > 0xffff)
        {   // Issue delay operation
            command          = ACID64_CMD_DELAY;
            cmdCycles        = 0xffff;
            cycleCorrection -= 0xffff;
            SwitchToFiber (mainFiber);
        }
    }
}

void Acid64::event ()
{
    ;
}

void Acid64::write (event_clock_t cycles, uint_least8_t addr, uint8_t data)
{
    delay (cycles);

    // Copy with not enough cycles between SID accesses
    if (cycleCorrection < 0)
        cmdCycles = (WORD)-cycleCorrection;
    else
    {
        cmdCycles = (WORD)cycleCorrection;
        cycleCorrection = 0;
    }
    cycleCorrection -= 4; // minimum access period

    command = ACID64_CMD_WRITE;
    cmdAddr = addr;
    cmdData = data;
    SwitchToFiber (mainFiber);
}

uint8_t Acid64::read  (event_clock_t cycles, uint_least8_t addr)
{
    delay (cycles);

    // Copy with not enough cycles between SID accesses
    if (cycleCorrection < 0)
        cmdCycles = (WORD)-cycleCorrection;
    else
    {
        cmdCycles = (WORD)cycleCorrection;
        cycleCorrection  = 0;
    }
    cycleCorrection -= 4; // minimum access period

    command = ACID64_CMD_READ;
    cmdAddr = addr;
    SwitchToFiber (mainFiber);
    return cmdData;
}

extern "C"
{

#define BEGIN(pInst, handle) \
    try \
    { \
        Acid64 &pInst = *reinterpret_cast<Acid64*>(handle); \
        if (handle == 0) \
            throw 0; \
        if (&pInst != pInst.me) \
            throw 0;

#define END \
    } \
    catch (...) \
    { \
    }

static DWORD stacksize = 0x100000 * sizeof(int);

__declspec(dllexport) int __stdcall getVersion()
{
    return 0x107; // 1.07
}

__declspec(dllexport) handle_t __stdcall createC64 ()
{
    handle_t handle = 0;

    try
    {
        std::auto_ptr<Acid64> inst(new Acid64);
        inst->engine = ISidplay2::create ();
        if (!inst->engine)
            throw 0;
        inst->cycleCorrection = 0;
        inst->cpuClock = 0.0;
        inst->builder.create (inst->engine->info().maxsids);

        // Create the fiber
        inst->engineFiber = CreateFiber (0, &start, inst.get());
        if (!inst->engineFiber)
            throw 0;

        try
        {
            SafeThreadToFibre convert(inst->mainFiber);
            SwitchToFiber (inst->engineFiber);
        }
        catch (...)
        {
            DeleteFiber (inst->engineFiber);
            throw;
        }

        handle = (handle_t)inst.get ();
        inst.release ();
    }
    catch (...)
    {
        ;
    }

    return handle;
}

__declspec(dllexport) BOOL __stdcall closeC64 (handle_t handle)
{
    BEGIN (inst, handle)
    stop  (*inst.me);
    DeleteFiber (inst.engineFiber);
    delete &inst;
    return TRUE;
    END
    return FALSE;
}

__declspec(dllexport) BOOL __stdcall loadFile (handle_t handle, const char *filename)
{
    BEGIN (inst, handle)
    std::string path = filename; 
    if (inst.tune.load (filename))
    {   // Default tune
        inst.tune.selectSong (0);
        inst.tuneInfo = inst.tune.getInfo   ();
        inst.tuneMD5  = inst.tune.createMD5 ();
        inst.tunePath.swap (path);
        return TRUE;
    }
    END
    return FALSE;
}

__declspec(dllexport) int __stdcall getNumberOfSongs (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.songs;
    END
    return 0;
}

__declspec(dllexport) int __stdcall getDefaultSong (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.startSong;
    END
    return 0;
}

__declspec(dllexport) void __stdcall setSongToPlay (handle_t handle, int songToPlay)
{
    BEGIN (inst, handle)
    stop  (*inst.me);
    inst.tune.selectSong (songToPlay);
    inst.tuneInfo = inst.tune.getInfo ();
    END
}

__declspec(dllexport) void __stdcall run (handle_t handle)
{
    BEGIN (inst, handle)
    inst.command = ACID64_CMD_IDLE;
    SafeThreadToFibre convert(inst.mainFiber);
    if (inst.engine->state() == sid2_stopped)
    {
        inst.cpuClock = 0.0;
        if (inst.engine->load(&inst.tune) < 0)
            throw 0;
        inst.engine->state();
    }
    SwitchToFiber (inst.engineFiber);
    END
}

__declspec(dllexport) void __stdcall skipSilence (handle_t handle, BOOL skip)
{
    BEGIN (inst, handle)
    sid2_config_t cfg = inst.engine->config ();
    cfg.sidFirstAccess = skip ? inst.me : 0;
    inst.engine->config (cfg);
    END
}

__declspec(dllexport) void __stdcall enableVolumeFix (handle_t handle, BOOL)
{
    BEGIN (inst, handle)
    // Not sure this is needed
    END
}

__declspec(dllexport) void __stdcall pressButtons (handle_t handle)
{
    BEGIN (inst, handle)
    // @FIXME@ - what does this actually do?
    END
}

__declspec(dllexport) void __stdcall enableFixedStartup (handle_t handle)
{
    BEGIN (inst, handle)
    // @FIXME@ - how do disable?
    sid2_config_t cfg = inst.engine->config ();
    cfg.powerOnDelay = 0;
    inst.engine->config (cfg);
    END
}

//-------------------------------------------------------------------------------------
// SidTune
__declspec(dllexport) const char * __stdcall getTitle (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
    {
        if (inst.tuneInfo.numberOfInfoStrings == 3)
            return inst.tuneInfo.infoString[0];
    }
    END
    return "UNKNOWN";
}

__declspec(dllexport) const char *__stdcall getAuthor (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
    {
        if (inst.tuneInfo.numberOfInfoStrings == 3)
            return inst.tuneInfo.infoString[1];
    }
    END
    return "UNKNOWN";
}

__declspec(dllexport) const char *__stdcall getReleased (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
    {
        if (inst.tuneInfo.numberOfInfoStrings == 3)
            return inst.tuneInfo.infoString[2];
    }
    END
    return "UNKNOWN";
}

__declspec(dllexport) int __stdcall getLoadAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.loadAddr;
    END
    return 0;
}

__declspec(dllexport) int __stdcall getLoadEndAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.loadAddr + inst.tuneInfo.c64dataLen; //- 1;
    END
    return 0;
}

__declspec(dllexport) int __stdcall getInitAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.initAddr;
    END
    return 0;
}

__declspec(dllexport) int __stdcall getPlayAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.playAddr;
    END
    return 0;
}

__declspec(dllexport) int __stdcall getSIDModel (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.sidModel1;
    END
    return SIDTUNE_SIDMODEL_UNKNOWN;
}

__declspec(dllexport) int __stdcall getC64Version (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneInfo.clockSpeed;
    END
    return SIDTUNE_CLOCK_UNKNOWN;
}

__declspec(dllexport) void __stdcall setC64Version (handle_t handle, int model)
{
    BEGIN (inst, handle)
    stop  (*inst.me);
    sid2_clock_t m = SID2_CLOCK_CORRECT;
    if (model == SIDTUNE_CLOCK_PAL)
        m = SID2_CLOCK_PAL;
    else if (model == SIDTUNE_CLOCK_NTSC)
        m = SID2_CLOCK_NTSC;
    sid2_config_t cfg = inst.engine->config ();
    cfg.clockForced = true;
    cfg.clockSpeed  = m;
    inst.engine->config (cfg);
    END
}

__declspec(dllexport) int __stdcall getCommand (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.command == ACID64_CMD_READ)
        inst.cmdData = 0; // @FIXME@ - workaround for no reads
    return inst.command;
    END
    return ACID64_CMD_IDLE;
}

__declspec(dllexport) WORD __stdcall getCycles (handle_t handle)
{
    BEGIN (inst, handle)
    return inst.cmdCycles;
    END
    return 0;
}

__declspec(dllexport) BYTE __stdcall getRegister (handle_t handle)
{
    BEGIN (inst, handle)
    return inst.cmdAddr;
    END
    return 0;
}

__declspec(dllexport) BYTE __stdcall getData (handle_t handle)
{
    BEGIN (inst, handle)
    return inst.cmdData;
    END
    return 0;
}


//-------------------------------------------------------------------------------------
// MD5 Database
__declspec(dllexport) BOOL __stdcall checkSLDB (const char *filename)
{
    SidDatabase database;
    if (database.open(filename) >= 0)
        return TRUE;
    return FALSE;
}

__declspec(dllexport) BOOL __stdcall checkSLDBFromBuffer (const char *buffer, int size)
{
    // @FIXME@ - what is check for vs load?
    return FALSE;
}

__declspec(dllexport) BOOL __stdcall loadSLDB (const char *filename)
{
    if (g_database.open(filename) >= 0)
        return TRUE;
    return FALSE;
}

__declspec(dllexport) BOOL __stdcall loadSLDBFromBuffer(const char *buffer, int size)
{
    // @FIXME@
    // Is this really supported - presumably you must store off the memory somewhere disk?
    return FALSE;
}

__declspec(dllexport) const char * __stdcall getMD5Hash (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return inst.tuneMD5;
    END
    return "";
}

__declspec(dllexport) const char * __stdcall getFileName (const char *md5hash)
{
    return ""; // @FIXME@ - The database dosen't support reverse lookup, so what does this do?
}

__declspec(dllexport) int __stdcall getSongLength (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
        return (int)g_database.length (inst.tuneMD5, inst.tuneInfo.currentSong);
    END
    return -1;
}
  
__declspec(dllexport) DWORD __stdcall getTime (handle_t handle)
{
    BEGIN (inst, handle)
    // Accuracy needs to be ms.  This is not supported by the default clock
    // which has an accuracy of 1/10th of a second.
    // SidIPtr<ISidTimer> timer(inst.engine);
    // return (DWORD)(timer->time() * (1000 / timer->timebase()));
    // Instead we count the cycles that have passed to the SID and from that we can
    // deduce the cycle accuracy to that inuse by the emulation to drive the CPU.
    if (!inst.pending())
        return (DWORD)(inst.cpuClock * 1000.0 / inst.engine->info().cpuFrequency + 0.5);
    END
    return 0;
}

//-------------------------------------------------------------------------------------
// STIL
__declspec(dllexport) BOOL __stdcall loadSTIL (const char *directory)
{
    if (g_stil.setBaseDir (directory))
        return TRUE;
    return FALSE;
}

__declspec(dllexport) BOOL __stdcall loadSTILFromBuffer (const char *buffer, int size)
{
    return FALSE;
}

__declspec(dllexport) const char *__stdcall getSTILEntry (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst.tune)
    {
        static std::string entry = stilEntry (inst.tunePath.c_str(), inst.tuneInfo.currentSong);
        return entry.c_str ();
    }
    END
    return "";
}

//-------------------------------------------------------------------------------------
// Debug
__declspec(dllexport) void  __stdcall getMemoryUsageRam (handle_t handle, char *buffer, int size)
{
    memset (buffer, 0, size);
}

__declspec(dllexport) void  __stdcall getMemoryUsageRom (handle_t handle, char *buffer, int size)
{
    memset (buffer, 0, size);
}

__declspec(dllexport) void  __stdcall clearMemUsageOnFirstSIDAccess (handle_t handle, BOOL clear)
{
}

__declspec(dllexport) void  __stdcall getMemory (handle_t handle, char *buffer, int size)
{
    memset (buffer, 0, size);
}

} // extern "C"
