#include "sid6510c.h"


SID6510::SID6510 ()
{
    uword_sidt i;
    sbyte_sidt n;
    sbyte_sidt maxCycle = -1;

    // Added V1.04 (saw) - Support of sidplays break functionality
    // Prevent break from working correctly and locking the player
    // @FIXME@: Memory has not been released for cycle 2 and above
    instrTable[BRKn].lastCycle = 1;
    // Rev 1.2 (saw) - Changed nasty union to reinterpret_cast
    instrTable[BRKn].cycle[1]  = reinterpret_cast <void (MOS6510::*)()>
        (&SID6510::sid_brk);

    // Ok start all the hacks for sidplay.  This prevents
    // execution of code in roms.  For real c64 emulation
    // create object from base class!  Also stops code
    // rom execution when bad code switches roms in over
    // itself.
    for (i = 0; i < OPCODE_MAX; i++)
    {
        if (instrTable[i].cycle == NULL) continue;
        maxCycle  = instrTable[i].lastCycle + 1;
        procCycle = instrTable[i].cycle;

        for (n = 0; n < maxCycle; n++)
        {
            if (instrTable[i].cycle[n] == &MOS6510::FetchEffAddrDataByte)
            {   // Rev 1.2 (saw) - Changed nasty union to reinterpret_cast
                instrTable[i].cycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_FetchEffAddrDataByte);
            }
            else if (instrTable[i].cycle[n] == &MOS6510::illegal_instr)
            {   // Rev 1.2 (saw) - Changed nasty union to reinterpret_cast
                instrTable[i].cycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_suppressError);
                // Need to make instruction appear in debug output
                // By default it is -1, which is debug off.
                instrTable[i].lastAddrCycle = 0;
            }
        }
    }

    {   // Stop jumps into rom code
        void (MOS6510::*p) ();
        // Rev 1.2 (saw) - Changed nasty union to reinterpret_cast
        p = reinterpret_cast <void (MOS6510::*)()> (&SID6510::sid_jmp);
        instrTable[JSRw].cycle[4] = p;
        instrTable[JMPw].cycle[2] = p;
        instrTable[JMPi].cycle[4] = p;
    }
}
    
void SID6510::reset (ubyte_sidt a, ubyte_sidt x, ubyte_sidt y)
{
    // Registers not touched by a reset
    Register_Accumulator = a;
    Register_X           = x;
    Register_Y           = y;

    // Reset the processor
    reset ();
}

void SID6510::reset ()
{
    SPWrapped = (PCWrapped = false);
    // Call inherited reset
    MOS6510::reset ();
}

void SID6510::clock (void)
{   // Call inherited emulate
    MOS6510::clock ();

    // Sidplay requires that we check to see if
    // the stack has overflowed.  This then returns
    // control back to sidplay so music can be played
    if (sidhibyte (Register_StackPointer)   != SP_PAGE) SPWrapped = true;
    if (sidhiword (Register_ProgramCounter) != 0)       PCWrapped = true;
}

//**************************************************************************************
// For sidplay compatibility the memory handle should always be non-banked!  To be able
// to use the switched in I/O, etc, different versions of fetch/put effective address
// data have been created.  These versions should call a non flat memory handle where
// the data is aquired depending on the actual enviroment.
//**************************************************************************************
void SID6510::sid_FetchEffAddrDataByte (void)
{   // For sidplay compatibility, inherited from environment
    Cycle_Data = envReadMemDataByte (Cycle_EffectiveAddress);
}

// Sidplay Suppresses Illegal Instructions
void SID6510::sid_suppressError (void)
{
    Register_ProgramCounter++;
}


//**************************************************************************************
// For sidplay compatibility implement those instructions which don't behave properly.
//**************************************************************************************
void SID6510::sid_brk (void)
{
    setFlagB(true);
    setFlagI(true);
#if !defined(NO_RTS_UPON_BRK)
    sid_rts ();
#endif
}

void SID6510::sid_jmp (void)
{   // For sidplay compatibility, inherited from environment
    if (envCheckBankJump (Cycle_EffectiveAddress))
        jmp_instr ();
    else
        sid_rts   ();
}

// Will do a full rts in 1 cycle, to
// destroy current function and quit
void SID6510::sid_rts (void)
{
    PopLowPC();
    PopHighPC();
    rts_instr();
}
