/***************************************************************************
                          sid6510c.i  -  Sidplay Specific 6510 emulation
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
 *  Revision 1.18  2002/02/07 18:02:10  s_a_white
 *  Real C64 compatibility fixes. Debug of BRK works again. Fixed illegal
 *  instructions to work like sidplay1.
 *
 *  Revision 1.17  2002/02/06 17:49:12  s_a_white
 *  Fixed sign comparison warning.
 *
 *  Revision 1.16  2002/02/04 23:53:23  s_a_white
 *  Improved compatibilty of older sidplay1 modes. Fixed BRK to work like sidplay1
 *  only when stack is 0xff in real mode for better compatibility with C64.
 *
 *  Revision 1.15  2002/01/28 19:32:16  s_a_white
 *  PSID sample improvements.
 *
 *  Revision 1.14  2001/10/02 18:00:37  s_a_white
 *  Removed un-necessary cli.
 *
 *  Revision 1.13  2001/09/18 07:51:39  jpaana
 *  Small fix to rti-processing.
 *
 *  Revision 1.12  2001/09/03 22:23:06  s_a_white
 *  Fixed faked IRQ trigger on BRK for sidplay1 environment modes.
 *
 *  Revision 1.11  2001/09/01 11:08:06  s_a_white
 *  Fixes for sidplay1 environment modes.
 *
 *  Revision 1.10  2001/08/05 15:46:02  s_a_white
 *  No longer need to check on which cycle an instruction ends or when to print
 *  debug information.
 *
 *  Revision 1.9  2001/07/14 13:17:40  s_a_white
 *  Sidplay1 optimisations moved to here.  Stack & PC invalid tests now only
 *  performed on a BRK.
 *
 *  Revision 1.8  2001/03/24 18:09:17  s_a_white
 *  On entry to interrupt routine the first instruction in the handler is now always
 *  executed before pending interrupts are re-checked.
 *
 *  Revision 1.7  2001/03/22 22:40:07  s_a_white
 *  Replaced tabs characters.
 *
 *  Revision 1.6  2001/03/21 22:26:24  s_a_white
 *  Fake interrupts now been moved into here from player.cpp.  At anytime it's
 *  now possible to ditch this compatibility class and use the real thing.
 *
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


SID6510::SID6510 (EventContext *context)
:MOS6510(context),
 m_mode(sid2_envR)
{   // Ok start all the hacks for sidplay.  This prevents
    // execution of code in roms.  For real c64 emulation
    // create object from base class!  Also stops code
    // rom execution when bad code switches roms in over
    // itself.
    for (uint i = 0; i < OPCODE_MAX; i++)
    {
        procCycle = instrTable[i].cycle;
        if (procCycle == NULL) continue;

        for (uint n = 0; n < instrTable[i].cycles; n++)
        {
            if (procCycle[n] == &MOS6510::illegal_instr)
            {   // Rev 1.2 (saw) - Changed nasty union to reinterpret_cast
                procCycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_illegal);
            }
            else if (procCycle[n] == &MOS6510::jmp_instr)
            {   // Stop jumps into rom code
                procCycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_jmp);
            }
            else if (procCycle[n] == &MOS6510::cli_instr)
            {   // No overlapping IRQs allowed
                procCycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_cli);
            }
        }
    }

    {   // Since no real IRQs, all RTIs mapped to RTS
        // Required for fix bad tunes in old modes
        uint n;
        procCycle = instrTable[RTIn].cycle;
        for (n = 0; n < instrTable[RTIn].cycles; n++)
        {
            if (procCycle[n] == &MOS6510::PopSR)
            {
                procCycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_rti);
                break;
            }
        }

        procCycle = interruptTable[oIRQ].cycle;
        for (n = 0; n < interruptTable[oIRQ].cycles; n++)
        {
            if (procCycle[n] == &MOS6510::IRQRequest)
            {
                procCycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_irq);
                break;
            }
        }
    }

    {   // Support of sidplays BRK functionality
        procCycle = instrTable[BRKn].cycle;
        for (uint n = 0; n < instrTable[BRKn].cycles; n++)
        {
            if (procCycle[n] == &MOS6510::PushHighPC)
            {
                procCycle[n] = reinterpret_cast <void (MOS6510::*)()>
                    (&SID6510::sid_brk);
                break;
            }
        }
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
    m_sleeping = false;
    // Call inherited reset
    MOS6510::reset ();
}


//**************************************************************************************
// For sidplay compatibility implement those instructions which don't behave properly.
//**************************************************************************************
void SID6510::sid_brk (void)
{
    if ( (m_mode == sid2_envR) &&
         ((Register_StackPointer & 0xFF) != 0xFF) )
    {
        MOS6510::PushHighPC ();
        return;
    }

    sei_instr ();
#if !defined(NO_RTS_UPON_BRK)
    sid_rts ();
#endif

    // Sid tunes end by wrapping the stack.  For compatibilty it
    // has to be handled.
    m_sleeping |= (endian_16hi8  (Register_StackPointer)   != SP_PAGE);
    m_sleeping |= (endian_32hi16 (Register_ProgramCounter) != 0);

    if (!m_sleeping)
    {
        MOS6510::FetchOpcode ();
        return;
    }

    // The CPU is about to sleep.  It can only be woken by a
    // reset or interrupt.
    envSleep ();
    Initialise ();

    // When we return from interrupt we will do a break
    // which will sleep the CPU.
    {
        uint8_t prg[] = {BRKn, TXSn, 0xFF, LDXb};
        for (size_t i = 0; i < sizeof (prg); i++)
        {
            Register_Accumulator = prg[i];
            pha_instr ();
        }
    }
    Register_ProgramCounter = Register_StackPointer;
    if (m_mode == sid2_envR)
    {   // in sidplay1 RTI behaves like RTS which mean
        // the return address is already +1. A real
        // RTI dosen't do this.
        Register_ProgramCounter++;
    }

    // Check for outstanding interrupts
    interrupts.delay = 0;
    if (interrupts.pending)
    {   // Start processing the interrupt
        if (interrupts.irqs)
        {
            interrupts.irqs--;
            triggerIRQ ();
        }
        else
        {
            MOS6510::clock ();
            m_sleeping = false;
        }
    }
}

void SID6510::sid_jmp (void)
{   // For sidplay compatibility, inherited from environment
    if (m_mode == sid2_envR)
    {
        jmp_instr ();
        return;
    }

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

void SID6510::sid_cli (void)
{
    if (m_mode == sid2_envR)
        cli_instr ();
}

void SID6510::sid_rti (void)
{
    if (m_mode == sid2_envR)
    {
        PopSR ();
        return;
    }
    
    // Fake RTS
    sid_rts ();
    FetchOpcode ();
}

void SID6510::sid_irq (void)
{
    MOS6510::IRQRequest ();
    if (m_mode != sid2_envR)
    {   // RTI behaves like RTI in sidplay1 modes
        Register_StackPointer++;
    }
}

// Sidplay Suppresses Illegal Instructions
void SID6510::sid_illegal (void)
{
    if (m_mode == sid2_envR)
    {
        MOS6510::illegal_instr ();
        return;
    }
#ifdef MOS6510_DEBUG
    DumpState ();
#endif
}


//**************************************************************************************
// Sidplay compatibility interrupts.  Basically wakes CPU if it is m_sleeping
//**************************************************************************************
void SID6510::triggerRST (void)
{   // All modes
    MOS6510::triggerRST ();
    if (m_sleeping)
    {
        MOS6510::clock ();
        m_sleeping = false;
    }
}

void SID6510::triggerNMI (void)
{   // Only in Real C64 mode
    if (m_mode == sid2_envR)
    {
        MOS6510::triggerNMI ();
        if (m_sleeping)
        {
            MOS6510::clock ();
            m_sleeping = false;
        }
    }
}

void SID6510::triggerIRQ (void)
{
    switch (m_mode)
    {
    default:
        if (interrupts.irqs)
            return;
        // Deliberate run on
    case sid2_envR:
        MOS6510::triggerIRQ ();
        if (m_sleeping)
        {
            MOS6510::clock ();
            // Can't support overlapped IRQs in older
            // environment modes and RTIs are RTSs.
            if (m_mode != sid2_envR)
                clearIRQ ();
            m_sleeping = false;
        }
    }
}
