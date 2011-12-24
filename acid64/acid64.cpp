#include <algorithm>
#include <sidplay/sidplay2.h>
#include <sidplay/sidlazyiptr.h>
#include <sidplay/utils/SidDatabase.h>
#include <sidplay/utils/SidTuneMod.h>

struct Acid64
{
    Acid64 *me;
    SidLazyIPtr<sidplay2> engine;
    SidTuneMod            tune;
    SidTuneInfo           tuneInfo;
    const char           *tuneMD5;

    Acid64 () : tune(0) { ; }
};

static SidDatabase g_database;

extern "C"
{

#define BEGIN(pInst, handle) \
    try \
    { \
        Acid64 *pInst; \
        if (handle == 0) \
            throw 0; \
        pInst = reinterpret_cast<Acid64*>(handle); \
        if (pInst != pInst->me) \
            throw 0;

#define END \
    } \
    catch (...) \
    { \
    }

typedef void* handle_t;

typedef enum BOOL
{
    TRUE  = -1,
    FALSE = 0
} BOOL;

typedef long sidmodel_t;
typedef long c64model_t;

__declspec(dllexport) int __stdcall getVersion()
{
    return 1; // @FIXME@ - any specific number and in what format?
}

__declspec(dllexport) handle_t __stdcall createC64 ()
{
    try
    {
        std::auto_ptr<Acid64> inst(new Acid64);
        inst->me     = inst.get ();
        inst->engine = ISidplay2::create ();

        handle_t handle = (handle_t)inst.get ();
        inst.release ();
        return handle;
    }
    catch (...)
    {
        ;
    }
    return 0;
}

__declspec(dllexport) BOOL __stdcall closeC64 (handle_t handle)
{
    BEGIN (inst, handle)
    delete inst;
    END
    return TRUE;
}

__declspec(dllexport) BOOL __stdcall loadFile (handle_t handle, const char *filename)
{
    BEGIN (inst, handle)
    if (inst->tune.load (filename))
    {   // Default tune
        inst->tune.selectSong (0);
        inst->tuneInfo = inst->tune.getInfo   ();
        inst->tuneMD5  = inst->tune.createMD5 ();
        return TRUE;
    }
    END
    return FALSE;
}

__declspec(dllexport) int __stdcall getNumberOfSongs (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
        return inst->tuneInfo.songs;
    END
    return 0;
}

__declspec(dllexport) int __stdcall getDefaultSong (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
        return inst->tuneInfo.startSong;
    END
    return -1; // @FIXME@ - error condition
}

__declspec(dllexport) void __stdcall setSongToPlay (handle_t handle, int songToPlay)
{
    BEGIN (inst, handle)
    inst->tune.selectSong (songToPlay);
    if (inst->engine->load(&inst->tune) >= 0)
        inst->tuneInfo = *inst->engine->info().tuneInfo;
    END
}

__declspec(dllexport) void __stdcall run (handle_t handle)
{
    BEGIN (inst, handle)
    // @FIXME@ - blocking, non blocking.  Who aborts?
    END
}

__declspec(dllexport) void __stdcall skipSilence (handle_t handle, BOOL skip)
{
    BEGIN (inst, handle)
    // @FIXME@
    END
}

__declspec(dllexport) void __stdcall enableVolumeFix (handle_t handle, BOOL fix)
{
    BEGIN (inst, handle)
    // @FIXME@
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
    // @FIXME@ - what does this actually do?
    END
}

//-------------------------------------------------------------------------------------
// SidTune
__declspec(dllexport) const char * __stdcall getTitle (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
    {
        if (inst->tuneInfo.numberOfInfoStrings == 3)
            return inst->tuneInfo.infoString[0];
    }
    END
    return "";
}

__declspec(dllexport) const char *__stdcall getAuthor (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
    {
        if (inst->tuneInfo.numberOfInfoStrings == 3)
            return inst->tuneInfo.infoString[1];
    }
    END
    return "";
}

__declspec(dllexport) const char *__stdcall getReleased (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
    {
        if (inst->tuneInfo.numberOfInfoStrings == 3)
            return inst->tuneInfo.infoString[2];
    }
    END
    return "";
}

__declspec(dllexport) int __stdcall getLoadAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
        return inst->tuneInfo.loadAddr;
    END
    return -1; // @FIXME@ - what to return for error condition
}

__declspec(dllexport) int __stdcall getLoadEndAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
        return inst->tuneInfo.loadAddr + inst->tuneInfo.c64dataLen - 1;
    END
    return -1; // @FIXME@ - what to return for error condition
}

__declspec(dllexport) int __stdcall getInitAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
        return inst->tuneInfo.initAddr;
    END
    return -1; // @FIXME@ - what to return for error condition
}

__declspec(dllexport) int __stdcall getPlayAddress (handle_t handle)
{
    BEGIN (inst, handle)
    if (inst->tune)
        return inst->tuneInfo.playAddr;
    END
    return -1; // @FIXME@ - what to return for error condition
}

__declspec(dllexport) sidmodel_t __stdcall getSIDModel (handle_t handle)
{
    BEGIN (inst, handle)
    // @FIXME@ - what are the enums here
    END
    return -1; // @FIXME@ - what to return for error condition
}

__declspec(dllexport) c64model_t __stdcall getC64Version (handle_t handle)
{
    BEGIN (inst, handle)
    // @FIXME@ - what are the enums here
    END
    return -1; // @FIXME@ - what to return for error condition
}

__declspec(dllexport) void __stdcall setC64Version (handle_t handle, c64model_t model)
{
    BEGIN (inst, handle)
    // @FIXME@ - what are the enums here
    END
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
    if (inst->tune)
        return inst->tuneMD5;
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
    g_database.length (inst->tuneMD5, inst->tuneInfo.currentSong);
    END
    return 0;
}
  
//-------------------------------------------------------------------------------------
// STIL
__declspec(dllexport) BOOL           __stdcall loadSTIL (const char *directory);
__declspec(dllexport) BOOL           __stdcall loadSTILFromBuffer (const char *buffer, int size);
__declspec(dllexport) const char *   __stdcall getSTILEntry (handle_t handle);

//-------------------------------------------------------------------------------------
// Debug
__declspec(dllexport) int            __stdcall getCommand(handle_t handle); // Opcode that is currently executing?
__declspec(dllexport) unsigned short __stdcall getCycles(handle_t handle); // From sid start or for opcode?
__declspec(dllexport) unsigned char  __stdcall getRegister(handle_t handle); // which one?
__declspec(dllexport) unsigned char  __stdcall getData(handle_t handle); // For what?
__declspec(dllexport) unsigned long  __stdcall getTime(handle_t handle); // For what?
__declspec(dllexport) void           __stdcall getMemoryUsageRam (handle_t handle, const char *buffer, int size); // What usage info?
__declspec(dllexport) void           __stdcall getMemoryUsageRom (handle_t handle, const char *buffer, int size); // What usage info?
__declspec(dllexport) void           __stdcall clearMemUsageOnFirstSIDAccess (handle_t handle, BOOL clear);
__declspec(dllexport) void           __stdcall getMemory (handle_t handle, char *buffer, int size); // All c64 memory?

} // extern "C"
