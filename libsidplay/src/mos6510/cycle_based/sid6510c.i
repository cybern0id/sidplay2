/***************************************************************************
                          sid6510c.h  -  Sidplay Specific 6510 emulation
                             -------------------
    begin                : Thu May 11 2000
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
 *  Revision 1.5  2001/03/09 22:28:03  s_a_white
 *  Speed optimisation update.
 *
 *  Revision 1.4  2001/02/13 21:02:16  s_a_white
 *  Small tidy up and possibly a small performace increase.
 *
 *  Revision 1.3  2000/12/11 19:04:32  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

#include "sid6510c.h"


SID6510::SID6510 ()
{
    uint i;
    int8_t n;
    int8_t maxCycle = -1;

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
    
void SID6510::reset (uint8_t a, uint8_t x, uint8_t y)
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
    sleeping = false;
    // Call inherited reset
    MOS6510::reset ();
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


//**************************************************************************************
// Sidplay compatibility interrupts.  Basically wakes CPU if it is sleeping
//**************************************************************************************
void SID6510::triggerRST (void)
{
    MOS6510::triggerRST ();
	if (sleeping)
	{
        FetchOpcode ();
        sleeping = false;
	}
}

void SID6510::triggerNMI (void)
{
    MOS6510::triggerNMI ();
	if (sleeping)
    {
        FetchOpcode ();
        sleeping = false;
	}
}

void SID6510::triggerIRQ (void)
{
    MOS6510::triggerIRQ ();
	if (sleeping)
	{
        FetchOpcode ();
        sleeping = false;
	}
}
