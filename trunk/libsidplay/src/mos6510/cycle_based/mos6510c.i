/***************************************************************************
                          mos6510.i  -  Cycle Accurate 6510 emulation
                             -------------------
    begin                : Thu May 11 06:22:40 BST 2000
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
 *  Revision 1.8  2001/03/09 22:28:51  s_a_white
 *  Speed optimisation update and fix for interrupt flag in PushSR call.
 *
 *  Revision 1.7  2001/02/22 08:28:57  s_a_white
 *  Interrupt masking fixed.
 *
 *  Revision 1.6  2001/02/13 23:01:44  s_a_white
 *  envReadMemDataByte now used for some memory accesses.
 *
 *  Revision 1.5  2000/12/24 00:45:38  s_a_white
 *  HAVE_EXCEPTIONS update
 *
 *  Revision 1.4  2000/12/14 23:55:07  s_a_white
 *  PushSR optimisation and PopSR code cleanup.
 *
 ***************************************************************************/

#include "config.h"

// Microsoft Visual C++ Version Number to work around compiler bug
// Currently both Visual C++ Versions 5, 6 are broken.
#define _MSC_VER_BAD_NEW 1200 /* Defines VC6 and below */

//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Status Register Routines                                                //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Use macros to access flags.  Allows compatiblity with other versions
// of this emulation
// Set N and Z flags according to byte
#define setFlagsNZ(x) (Register_z_Flag = (Register_n_Flag = (int_least8_t) (x)))
#define setFlagN(x)   (Register_n_Flag = (int_least8_t) (x))
#define setFlagV(x)   (Register_v_Flag = (int_least8_t) (x))
#define setFlagB(x)   (Register_Status = (Register_Status & ~(1 << SR_BREAK)) \
                                       | (((x) != 0) << SR_BREAK))
#define setFlagD(x)   (Register_Status = (Register_Status & ~(1 << SR_DECIMAL)) \
                                       | (((x) != 0) << SR_DECIMAL))
#define setFlagI(x)   (Register_Status = (Register_Status & ~(1 << SR_INTERRUPT)) \
                                       | (((x) != 0) << SR_INTERRUPT))
#define setFlagZ(x)   (Register_z_Flag = (int_least8_t) (x))
#define setFlagC(x)   (Register_c_Flag = (int_least8_t) (x))


#define getFlagN()    ((Register_n_Flag &  (1 << SR_NEGATIVE))  != 0)
#define getFlagV()    (Register_v_Flag != 0)
#define getFlagD()    ((Register_Status  & (1 << SR_DECIMAL))   != 0)
#define getFlagI()    ((Register_Status  & (1 << SR_INTERRUPT)) != 0)
#define getFlagZ()    (Register_z_Flag == 0)
#define getFlagC()    (Register_c_Flag != 0)

// Push P on stack, decrement S
void MOS6510::PushSR (void)
{
    uint_least16_t addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    /* Rev 1.04 - Corrected flag mask */
    Register_Status &= ((1 << SR_NOTUSED) | (1 << SR_INTERRUPT)
                     |  (1 << SR_DECIMAL) | (1 << SR_BREAK));
    Register_Status |= (getFlagN () << SR_NEGATIVE);
    Register_Status |= (getFlagV () << SR_OVERFLOW);
    Register_Status |= (getFlagZ () << SR_ZERO);
    Register_Status |= (getFlagC () << SR_CARRY);
    envWriteMemByte (addr, Register_Status);
    Register_StackPointer--;
}

// increment S, Pop P off stack
void MOS6510::PopSR (void)
{
    Register_StackPointer++;
    {
        uint_least16_t addr = Register_StackPointer;
        endian_16hi8 (addr, SP_PAGE);
        Register_Status     = envReadMemByte (addr);
    }
    setFlagB (false);
    setFlagN (Register_Status);
    setFlagV (Register_Status   & (1 << SR_OVERFLOW));
    setFlagZ (!(Register_Status & (1 << SR_ZERO)));
    setFlagC (Register_Status   & (1 << SR_CARRY));
}


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Interrupt Routines                                                      //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
#define iIRQSMAX 3
enum
{
    oNONE = -1,
    oRST,
    oNMI,
    oIRQ
};

enum
{
    iNONE = 0,
    iRST  = (1 << oRST),
    iNMI  = (1 << oNMI),
    iIRQ  = (1 << oIRQ)
};

void MOS6510::triggerRST (void)
{
    interrupts.pending |= iRST;
}

void MOS6510::triggerNMI (void)
{
    interrupts.pending |= iNMI;
}

// Level triggered interrupt
void MOS6510::triggerIRQ (void)
{   // IRQ Suppressed
    if (!getFlagI ())
        interrupts.pending |= iIRQ;
    interrupts.irqs++;
    if (interrupts.irqs > iIRQSMAX)
    {
        printf ("MOS6510 Error: An external component is not clearing down it's IRQs.\n");
        printf ("               Aborting...\n\n");
        exit (-1);
    }
}

void MOS6510::clearIRQ (void)
{
    if (interrupts.irqs > 0)
    {   
        if (!(--interrupts.irqs))
        {   // Clear off the interrupts
            interrupts.pending  &= (~iIRQ);
        }
    }
}

void MOS6510::interruptPending (void)
{
    int_least8_t       offset;
    const int_least8_t offTable[] = {oNONE, oRST, oNMI, oRST,
                                     oIRQ,  oRST, oNMI, oRST};

    // Service the highest priority interrupt
    offset = offTable[interrupts.pending];
    switch (offset)
    {
    case oNONE:
    default:
        return;

    case oIRQ:
    case oRST:
        break;

    case oNMI:
        // Simulate Edge Triggering
        interrupts.pending &= (~iNMI);
    break;
    }

#ifdef MOS6510_DEBUG
    printf ("****************************************************\n");
    switch (offset)
    {
    case oIRQ:
        printf (" IRQ Routine\n");
    break;
    case oNMI:
        printf (" NMI Routine\n");
    break;
    case oRST:
        printf (" RST Routine\n");
    break;
    }
    printf ("****************************************************\n");
#endif

    // Start the interrupt
    instrCurrent = &interruptTable[offset];
}

void MOS6510::RSTRequest (void)
{
    envReset ();
}

void MOS6510::NMIRequest (void)
{
    endian_16lo8 (Cycle_EffectiveAddress, envReadMemDataByte (0xFFFA));
}

void MOS6510::NMI1Request (void)
{
    endian_16hi8  (Cycle_EffectiveAddress, envReadMemDataByte (0xFFFB));
    endian_32lo16 (Register_ProgramCounter, Cycle_EffectiveAddress);
}

void MOS6510::IRQRequest (void)
{
    PushSR ();
    setFlagI (true);
    // No need to keep this pending, disable them
    // and re-enable them later
    interrupts.pending &= (~iIRQ);
}

void MOS6510::IRQ1Request (void)
{
    endian_16lo8 (Cycle_EffectiveAddress, envReadMemDataByte (0xFFFE));
}

void MOS6510::IRQ2Request (void)
{
    endian_16hi8  (Cycle_EffectiveAddress, envReadMemDataByte (0xFFFF));
    endian_32lo16 (Register_ProgramCounter, Cycle_EffectiveAddress);
}


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Common Instruction Addressing Routines                                  //
// Addressing operations as described in 64doc by John West and            //
// Marko Makela                                                            //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

// Fetch opcode, increment PC
// Addressing Modes:    All
void MOS6510::FetchOpcode (void)
{
    instrCurrent = NULL;

    interruptPending ();
    if (!instrCurrent)
    {
        instrStartPC = endian_32lo16  (Register_ProgramCounter++);
        instrOpcode  = envReadMemByte (instrStartPC);
        // Convert opcode to pointer in instruction table
        instrCurrent = &instrTable[instrOpcode];
        Instr_Operand = 0;
    }

    procCycle  = instrCurrent->cycle;
    lastCycle  = instrCurrent->lastCycle;
    cycleCount = 0;
}

// Fetch value, increment PC
/* Addressing Modes:    Immediate
                        Relative
*/
void MOS6510::FetchDataByte (void)
{   // Get data byte from memory
    Cycle_Data = envReadMemByte (endian_32lo16 (Register_ProgramCounter));
    Register_ProgramCounter++;

    // Nextline used for Debug
    Instr_Operand = (uint_least16_t) Cycle_Data;
}

// Fetch low address byte, increment PC
/* Addressing Modes:    Stack Manipulation
                        Absolute
                        Zero Page
                        Zerp Page Indexed
                        Absolute Indexed
                        Absolute Indirect
*/                      
void MOS6510::FetchLowAddr (void)
{
    Cycle_EffectiveAddress = envReadMemByte (endian_32lo16 (Register_ProgramCounter));
    Register_ProgramCounter++;

    // Nextline used for Debug
    Instr_Operand = Cycle_EffectiveAddress;
}

// Read from address, add index register X to it
// Addressing Modes:    Zero Page Indexed
void MOS6510::FetchLowAddrX (void)
{
    uint8_t page;
    FetchLowAddr ();
    // Page boundary crossing is not handled
    page = endian_16hi8 (Cycle_EffectiveAddress);
    endian_16lo8 (Cycle_EffectiveAddress, (uint8_t) Cycle_EffectiveAddress + Register_X);

#ifdef MOS6510_ACCURATE_CYCLES
    // If page boundary crossing were handled
//    if (endian_16lo8 (Cycle_EffectiveAddress) == page)
        cycleCount++;
#endif
}

// Read from address, add index register Y to it
// Addressing Modes:    Zero Page Indexed
void MOS6510::FetchLowAddrY (void)
{
    uint8_t page;
    FetchLowAddr ();
    // Page boundary crossing is not handled
    page = endian_16hi8 (Cycle_EffectiveAddress);
    endian_16lo8 (Cycle_EffectiveAddress, (uint8_t) Cycle_EffectiveAddress + Register_Y);

#ifdef MOS6510_ACCURATE_CYCLES
    // If page boundary crossing were handled
//    if (endian_16lo8 (Cycle_EffectiveAddress) == page)
        cycleCount++;
#endif
}

// Fetch high address byte, increment PC (Absoulte Addressing)
// Low byte must have been obtained first!
// Addressing Modes:    Absolute
void MOS6510::FetchHighAddr (void)
{   // Get the high byte of an address from memory
    endian_16hi8 (Cycle_EffectiveAddress, envReadMemByte (endian_32lo16 (Register_ProgramCounter)));
    Register_ProgramCounter++;

    // Nextline used for Debug
    endian_16hi8 (Instr_Operand, endian_16hi8 (Cycle_EffectiveAddress));
}

// Fetch high byte of address, add index register X to low address byte,
// increment PC
// Addressing Modes:    Absolute Indexed
void MOS6510::FetchHighAddrX (void)
{
    uint8_t page;
    // Rev 1.05 (saw) - Call base Function
    FetchHighAddr ();
    page = endian_16hi8 (Cycle_EffectiveAddress);
    Cycle_EffectiveAddress += Register_X;

#ifdef MOS6510_ACCURATE_CYCLES
    // Handle page boundary crossing
    if (endian_16hi8 (Cycle_EffectiveAddress) == page)
        cycleCount++;
#endif
}

// Fetch high byte of address, add index register Y to low address byte,
// increment PC
// Addressing Modes:    Absolute Indexed
void MOS6510::FetchHighAddrY (void)
{
    uint8_t page;
    // Rev 1.05 (saw) - Call base Function
    FetchHighAddr ();
    page = endian_16hi8 (Cycle_EffectiveAddress);
    Cycle_EffectiveAddress += Register_Y;

#ifdef MOS6510_ACCURATE_CYCLES
    // Handle page boundary crossing
    if (endian_16hi8 (Cycle_EffectiveAddress) == page)
        cycleCount++;
#endif
}

// Fetch pointer address low, increment PC
/* Addressing Modes:    Absolute Indirect
                        Indirect indexed (post Y)
*/
void MOS6510::FetchLowPointer (void)
{
    Cycle_Pointer = envReadMemByte (endian_32lo16 (Register_ProgramCounter));
    Register_ProgramCounter++;
    // Nextline used for Debug
    Instr_Operand = Cycle_Pointer;
}

// Read pointer from the address and add X to it
// Addressing Modes:    Indexed Indirect (pre X)
void MOS6510::FetchLowPointerX (void)
{
    uint8_t page;
    // Rev 1.05 (saw) - Call base Function
    FetchLowPointer ();
    // Page boundary crossing is not handled
    page = endian_16hi8 (Cycle_Pointer);
    endian_16lo8 (Cycle_Pointer, (uint8_t) Cycle_Pointer + Register_X);

#ifdef MOS6510_ACCURATE_CYCLES
    // If page boundary crossing were handled
//    if (sidhobyte (Cycle_Pointer) == page)
        cycleCount++;
#endif
}

// Fetch pointer address high, increment PC
// Addressing Modes:    Absolute Indirect
void MOS6510::FetchHighPointer (void)
{
    endian_16hi8 (Cycle_Pointer, envReadMemByte (endian_32lo16 (Register_ProgramCounter)));
    Register_ProgramCounter++;

    // Nextline used for Debug
    endian_16hi8 (Instr_Operand, endian_16hi8 (Cycle_Pointer));
}

// Fetch effective address low
/* Addressing Modes:    Indirect
                        Indexed Indirect (pre X)
                        Indirect indexed (post Y)
*/
void MOS6510::FetchLowEffAddr (void)
{
    Cycle_EffectiveAddress = envReadMemDataByte (Cycle_Pointer);
}

// Fetch effective address high
/* Addressing Modes:    Indirect
                        Indexed Indirect (pre X)
*/
void MOS6510::FetchHighEffAddr (void)
{   // Rev 1.03 (Mike) - Extra +1 removed
    endian_16lo8 (Cycle_Pointer, (uint8_t) ++Cycle_Pointer);
    endian_16hi8 (Cycle_EffectiveAddress, envReadMemDataByte (Cycle_Pointer));
}

// Fetch effective address high, add Y to low byte of effective address
// Addressing Modes:    Indirect indexed (post Y)
void MOS6510::FetchHighEffAddrY (void)
{
    uint8_t page;
    // Rev 1.05 (saw) - Call base Function
    FetchHighEffAddr ();
    page = endian_16hi8 (Cycle_EffectiveAddress);
    Cycle_EffectiveAddress += Register_Y;

#ifdef MOS6510_ACCURATE_CYCLES
    // Handle page boundary crossing
    if (endian_16hi8 (Cycle_EffectiveAddress) == page)
        cycleCount++;
#endif
}


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Common Data Accessing Routines                                          //
// Data Accessing operations as described in 64doc by John West and        //
// Marko Makela                                                            //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

void MOS6510::FetchEffAddrDataByte (void)
{
    Cycle_Data = envReadMemByte (Cycle_EffectiveAddress);
}

void MOS6510::PutEffAddrDataByte (void)
{
    envWriteMemByte (Cycle_EffectiveAddress, Cycle_Data);
}

// Push Program Counter Low Byte on stack, decrement S
void MOS6510::PushLowPC (void)
{
    uint_least16_t addr;
    addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    envWriteMemByte (addr, endian_32lo8 (Register_ProgramCounter));
    Register_StackPointer--;
}

// Push Program Counter High Byte on stack, decrement S
void MOS6510::PushHighPC (void)
{
    uint_least16_t addr;
    addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    envWriteMemByte (addr, endian_32hi8 (Register_ProgramCounter));
    Register_StackPointer--;
}

// Increment stack and pull program counter low byte from stack,
void MOS6510::PopLowPC (void)
{
    uint_least16_t addr;
    Register_StackPointer++;
    addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    endian_16lo8 (Cycle_EffectiveAddress, envReadMemByte (addr));
}

// Increment stack and pull program counter high byte from stack,
void MOS6510::PopHighPC (void)
{
    uint_least16_t addr;
    Register_StackPointer++;
    addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    endian_16hi8 (Cycle_EffectiveAddress, envReadMemByte (addr));
}

void MOS6510::WasteCycle (void)
{
}


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Common Instruction Opcodes                                              //
// See and 6510 Assembly Book for more information on these instructions   //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

void MOS6510::brk_instr (void)
{
    setFlagB   (true);
    PushHighPC ();
}

void MOS6510::brk1_instr (void)
{
    PushSR ();
    setFlagI (true);
}

void MOS6510::brk2_instr (void)
{
    endian_16lo8 (Cycle_EffectiveAddress, envReadMemByte (0xFFFE));
}

void MOS6510::brk3_instr (void)
{
    endian_16hi8  (Cycle_EffectiveAddress, envReadMemByte (0xFFFF));
    endian_32lo16 (Register_ProgramCounter, Cycle_EffectiveAddress);
}

void MOS6510::cld_instr (void)
{
    setFlagD (false);
}

void MOS6510::cli_instr (void)
{
    setFlagI (false);
    if (interrupts.irqs)
        interrupts.pending |= iIRQ;
}

void MOS6510::jmp_instr (void)
{
    endian_32lo16 (Register_ProgramCounter, Cycle_EffectiveAddress);
}

void MOS6510::jsr_instr (void)
{
    // Added V1.02 (saw) - Needed as JSR is not absoulute addressed instruction
    Register_ProgramCounter--;
    PushHighPC ();
}

void MOS6510::pha_instr (void)
{
    uint_least16_t addr;
    addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    envWriteMemByte (addr, Register_Accumulator);
    Register_StackPointer--;
}

void MOS6510::plp_instr (void)
{
    PopSR ();
    // Check to see if interrupts got re-enabled
    if (!getFlagI ())
    {   // Yep, now see if any Women (erm IRQs) require servicing
        if (interrupts.irqs)
            interrupts.pending |= iIRQ;
    }
}

void MOS6510::rti_instr (void)
{
#ifdef MOS6510_DEBUG
    printf ("****************************************************\n\n");
#endif

    endian_32lo16 (Register_ProgramCounter, Cycle_EffectiveAddress);
    // Check to see if interrupts got re-enabled
    if (!getFlagI ())
    {   // Yep, ok check if they need servicing
        if (interrupts.irqs)
            interrupts.pending |= iIRQ;
    }
}

void MOS6510::rts_instr (void)
{
    endian_32lo16 (Register_ProgramCounter, Cycle_EffectiveAddress);
    Register_ProgramCounter++;
}

void MOS6510::sed_instr (void)
{
    setFlagD (true);
}

void MOS6510::sei_instr (void)
{
    setFlagI (true);
    // No need to keep this pending, disable them
    // and re-enable them later
    interrupts.pending &= (~iIRQ);
}

void MOS6510::sta_instr (void)
{
    Cycle_Data = Register_Accumulator;
}

void MOS6510::stx_instr (void)
{
    Cycle_Data = Register_X;
}

void MOS6510::sty_instr (void)
{
    Cycle_Data = Register_Y;
}



//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Common Instruction Undocumented Opcodes                                 //
// See documented 6502-nmo.opc by Adam Vardy for more details              //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

// Undocumented - This opcode stores the result of A AND X AND the high
// byte of the target address of the operand +1 in memory.
void MOS6510::axa_instr (void)
{
    Cycle_Data = Register_X & Register_Accumulator & (endian_16hi8 (Cycle_EffectiveAddress) + 1);
}

// Undocumented - AXS ANDs the contents of the A and X registers (without changing the
// contents of either register) and stores the result in memory.
// AXS does not affect any flags in the processor status register.
void MOS6510::axs_instr (void)
{
    Cycle_Data = Register_Accumulator & Register_X;

}

/* Not required - Operation performed By another method
// Undocumented - HLT crashes the microprocessor.  When this opcode is executed, program
// execution ceases.  No hardware interrupts will execute either.  The author
// has characterized this instruction as a halt instruction since this is the
// most straightforward explanation for this opcode's behaviour.  Only a reset
// will restart execution.  This opcode leaves no trace of any operation
// performed!  No registers affected.
void MOS6510::hlt_instr (void)
{
}
*/

/* Not required - Operation performed By another method
void MOS6510::nop_instr (void)
{
}
*/

/*
// Not required - Operation performed By another method
void MOS6510::php_instr (void)
{
}
*/

// Undocumented - This opcode ANDs the contents of the Y register with <ab+1> and stores the
// result in memory.
void MOS6510::say_instr (void)
{
    Cycle_Data = Register_Y & (endian_16hi8 (Cycle_EffectiveAddress) + 1);
}

/* Not required - Operation performed By another method
// Undocumented - skip next byte.
void MOS6510::skb_instr (void)
{
    Register_ProgramCounter++;
}
*/

/* Not required - Operation performed By another method
// Undocumented - skip next word.
void MOS6510::skw_instr (void)
{
    Register_ProgramCounter += 2;
}
*/

// Undocumented - This opcode ANDs the contents of the X register with <ab+1> and stores the
// result in memory.
void MOS6510::xas_instr (void)
{
    Cycle_Data = Register_X & (endian_16hi8 (Cycle_EffectiveAddress) + 1);
}


#ifdef X86
#include "MOS6510\CYCLE_~1\X86.CPP"
//#include "MOS6510\CYCLE_BASED\X86.CPP"
#else

//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Generic Binary Coded Decimal Correction                                 //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

void MOS6510::Perform_ADC (void)
{
    uint_least16_t regAC2;
    uint8_t        flagCF;
    flagCF = getFlagC ();
    regAC2 = (uint_least16_t)(uint8_t) Register_Accumulator + (uint8_t) Cycle_Data + flagCF;

    if (getFlagD ())
    {   // Decimal mode
        setFlagZ (regAC2);

        // BCD fixup for lower nybble
        if ((regAC2 & 0x0f) > 0x09) regAC2 += 0x06;

        // Set flags
        setFlagV (((((uint8_t) Register_Accumulator ^ (uint8_t) Cycle_Data ^
                     (uint8_t) regAC2) & 0x80) != 0) ^ flagCF);
        setFlagN (regAC2);

        // BCD fixup for upper nybble
        if ((regAC2 & 0xf0) > 0x90) regAC2 += 0x60;

        setFlagC (regAC2 > 0xff);
        // Compose result
        Register_Accumulator = (int8_t) regAC2;
    }
    else
    {   // Binary mode
        setFlagC   (regAC2 > 0xff);
        flagCF =   getFlagC ();
        setFlagV   (((((uint8_t) Register_Accumulator ^ (uint8_t) Cycle_Data ^
                       (uint8_t) regAC2) & 0x80) != 0) ^ flagCF);
        setFlagsNZ (Register_Accumulator = (int8_t) regAC2);
    }
}

void MOS6510::Perform_SBC (void)
{
    Cycle_Data = ~Cycle_Data;
    Perform_ADC ();
    // This one probably not necessary, but usefull for debugging
    Cycle_Data = ~Cycle_Data;
}



//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Generic Instruction Addressing Routines                                 //
//-------------------------------------------------------------------------/


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Generic Instruction Opcodes                                             //
// See and 6510 Assembly Book for more information on these instructions   //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

void MOS6510::adc_instr (void)
{
    Perform_ADC ();
}

void MOS6510::and_instr (void)
{
    setFlagsNZ (Register_Accumulator &= Cycle_Data);
}

void MOS6510::ane_instr (void)
{
    setFlagsNZ (Register_Accumulator = (Register_Accumulator & Cycle_Data & 0x11) | (Cycle_Data & 0xee));
}

void MOS6510::asl_instr (void)
{
    setFlagC   (Cycle_Data & 0x80);
    setFlagsNZ (Cycle_Data <<= 1);
}

void MOS6510::asla_instr (void)
{
    setFlagC   (Register_Accumulator & 0x80);
    setFlagsNZ (Register_Accumulator <<= 1);
}

void MOS6510::bcc_instr (void)
{
    if (!getFlagC ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::bcs_instr (void)
{
    if (getFlagC ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::beq_instr (void)
{
    if (getFlagZ ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::bit_instr (void)
{
    setFlagZ (Register_Accumulator & Cycle_Data);
    setFlagN (Cycle_Data);
    setFlagV (Cycle_Data & 0x40);
}

void MOS6510::bmi_instr (void)
{
    if (getFlagN ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::bne_instr (void)
{
    if (!getFlagZ ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::bpl_instr(void)
{
    if (!getFlagN ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::bvc_instr (void)
{
    if (!getFlagV ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::bvs_instr (void)
{
    if (getFlagV ())
#ifdef MOS6510_ACCURATE_CYCLES
    {
        uint8_t page;
        page = endian_16hi8 (Cycle_EffectiveAddress);
        Register_ProgramCounter += Cycle_Data;

        // Handle page boundary crossing
        if (endian_32hi8 (Register_ProgramCounter) == page)
           cycleCount++;
    }
    else
    {
        cycleCount += 2;
    }
#else
        Register_ProgramCounter += Cycle_Data;
#endif
}

void MOS6510::clc_instr (void)
{
    setFlagC (false);
}

void MOS6510::clv_instr (void)
{
    setFlagV (false);
}

void MOS6510::cmp_instr (void)
{
    uint_least16_t tmp = (uint_least16_t)(uint8_t) Register_Accumulator - (uint8_t) Cycle_Data;
    setFlagsNZ (tmp);
    setFlagC   (tmp < 0x100);
}

void MOS6510::cpx_instr (void)
{
    uint_least16_t tmp = (uint_least16_t)(uint8_t) Register_X - (uint8_t) Cycle_Data;
    setFlagsNZ (tmp);
    setFlagC   (tmp < 0x100);
}

void MOS6510::cpy_instr (void)
{
    uint_least16_t tmp = (uint_least16_t)(uint8_t) Register_Y - (uint8_t) Cycle_Data;
    setFlagsNZ (tmp);
    setFlagC   (tmp < 0x100);
}

void MOS6510::dec_instr (void)
{
    setFlagsNZ (--Cycle_Data);
}

void MOS6510::dex_instr (void)
{
    setFlagsNZ (--Register_X);
}

void MOS6510::dey_instr (void)
{
    setFlagsNZ (--Register_Y);
}

void MOS6510::eor_instr (void)
{
    setFlagsNZ (Register_Accumulator^= Cycle_Data);
}

void MOS6510::inc_instr (void)
{
    setFlagsNZ (++Cycle_Data);
}

void MOS6510::inx_instr (void)
{
    setFlagsNZ (++Register_X);
}

void MOS6510::iny_instr (void)
{
    setFlagsNZ (++Register_Y);
}

void MOS6510::lda_instr (void)
{
    setFlagsNZ (Register_Accumulator = Cycle_Data);
}

void MOS6510::ldx_instr (void)
{
    setFlagsNZ (Register_X = Cycle_Data);
}

void MOS6510::ldy_instr (void)
{
    setFlagsNZ (Register_Y = Cycle_Data);
}

void MOS6510::lsr_instr (void)
{
    setFlagC   (Cycle_Data & 0x01);
    setFlagsNZ (Cycle_Data = (int8_t) ((uint8_t) Cycle_Data >> 1));
}

void MOS6510::lsra_instr (void)
{
    setFlagC   (Register_Accumulator & 0x01);
    setFlagsNZ (Register_Accumulator = (int8_t) ((uint8_t) Register_Accumulator >> 1));
}

void MOS6510::ora_instr (void)
{
    setFlagsNZ (Register_Accumulator |= Cycle_Data);
}

void MOS6510::pla_instr (void)
{
    uint_least16_t addr;
    Register_StackPointer++;
    addr = Register_StackPointer;
    endian_16hi8 (addr, SP_PAGE);
    setFlagsNZ (Register_Accumulator = envReadMemByte (addr));
}

void MOS6510::rol_instr (void)
{
    uint8_t tmp = Cycle_Data & 0x80;
    Cycle_Data   <<= 1;
    if (getFlagC ()) Cycle_Data |= 0x01;
    setFlagsNZ (Cycle_Data);
    setFlagC   (tmp);
}

void MOS6510::rola_instr (void)
{
    uint8_t tmp = Register_Accumulator & 0x80;
    Register_Accumulator <<= 1;
    if (getFlagC ()) Register_Accumulator |= 0x01;
    setFlagsNZ (Register_Accumulator);
    setFlagC   (tmp);
}

void MOS6510::ror_instr (void)
{
    uint8_t tmp = Cycle_Data & 0x01;
    Cycle_Data  = (int8_t) ((uint8_t) Cycle_Data >> 1);
    if (getFlagC ()) Cycle_Data |= 0x80;
    setFlagsNZ (Cycle_Data);
    setFlagC   (tmp);
}

void MOS6510::rora_instr (void)
{
    uint8_t tmp = Register_Accumulator & 0x01;
    Register_Accumulator = (int8_t) ((uint8_t) Register_Accumulator >> 1);
    if (getFlagC ()) Register_Accumulator |= 0x80;
    setFlagsNZ (Register_Accumulator);
    setFlagC   (tmp);
}

void MOS6510::sbx_instr (void)
{
    uint_least16_t tmp = (Register_Accumulator & Register_X) - Cycle_Data;
    setFlagsNZ (tmp);
    Register_X   = Cycle_Data = (int8_t) (tmp & 0x00ff);
    setFlagC   (tmp >> 8);
}

void MOS6510::sbc_instr (void)
{
    Perform_SBC ();
}

void MOS6510::sec_instr (void)
{
    setFlagC (true);
}

void MOS6510::shs_instr (void)
{
    endian_16lo8 (Register_StackPointer, (Register_Accumulator & Register_X));
    uint_least16_t tmp = (Cycle_EffectiveAddress + 1) & Register_StackPointer;
    setFlagsNZ (endian_16lo8 (tmp));
}

void MOS6510::tax_instr (void)
{
    setFlagsNZ (Register_X = Register_Accumulator);
}

void MOS6510::tay_instr (void)
{
    setFlagsNZ (Register_Y = Register_Accumulator);
}

void MOS6510::tsx_instr (void)
{   // Rev 1.03 (saw) - Got these tsx and txs reversed
    setFlagsNZ (Register_X = endian_16lo8 (Register_StackPointer));
}

void MOS6510::txa_instr (void)
{
    setFlagsNZ (Register_Accumulator = Register_X);
}

void MOS6510::txs_instr (void)
{   // Rev 1.03 (saw) - Got these tsx and txs reversed
    endian_16lo8 (Register_StackPointer, Register_X);
    setFlagsNZ   (Register_X);
}

void MOS6510::tya_instr (void)
{
    setFlagsNZ (Register_Accumulator = Register_Y);
}

void MOS6510::illegal_instr (void)
{
    printf ("ILLEGAL INSTRUCTION, resetting emulation. **************\n");
    DumpState ();
    printf ("********************************************************\n");
    // Perform Environment Reset
    envReset ();
}


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// Generic Instruction Undocuemented Opcodes                               //
// See documented 6502-nmo.opc by Adam Vardy for more details              //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

// Undocumented - This opcode ANDs the contents of the A register with an immediate value and
// then LSRs the result.
void MOS6510::alr_instr (void)
{
    Register_Accumulator &= Cycle_Data;
    setFlagC   (Register_Accumulator & 0x01);
    setFlagsNZ (Register_Accumulator = (int8_t) ((uint8_t) Register_Accumulator >> 1));
}

// Undcouemented - ANC ANDs the contents of the A register with an immediate value and then
// moves bit 7 of A into the Carry flag.  This opcode works basically
// identically to AND #immed. except that the Carry flag is set to the same
// state that the Negative flag is set to.
void MOS6510::anc_instr (void)
{
    setFlagsNZ (Register_Accumulator &= Cycle_Data);
    setFlagC   (getFlagN ());
}

// Undocumented - This opcode ANDs the contents of the A register with an immediate value and
// then RORs the result (Implementation based on that of Frodo C64 Emulator)
void MOS6510::arr_instr (void)
{
    uint8_t data = (uint8_t) (Cycle_Data & Register_Accumulator);
    data >>= 1;
    Register_Accumulator = (int8_t) data;
    if (getFlagC ()) Register_Accumulator |= 0x80;

    if (!getFlagD ())
    {
        setFlagsNZ (Register_Accumulator);
        setFlagC   (Register_Accumulator & 0x40);
        setFlagV  ((Register_Accumulator & 0x40) ^ ((Register_Accumulator & 0x20) << 1));
    }
    else
    {
        setFlagN (0);
        if (getFlagC ()) setFlagN (1 << SR_NEGATIVE);
        setFlagZ (Register_Accumulator);
        setFlagV ((Cycle_Data ^ Register_Accumulator) & 0x40);

        if ((data & 0x0f) + (data & 0x01) > 5)
            Register_Accumulator  = Register_Accumulator & 0xf0 | (Register_Accumulator + 6) & 0x0f;
        setFlagC (((data + (data & 0x10)) & 0x1f0) > 0x50);
        if (getFlagC ())
            Register_Accumulator += 0x60;
    }
}

// Undocumented - This opcode ASLs the contents of a memory location and then ORs the result
// with the accumulator.
void MOS6510::aso_instr (void)
{
    setFlagC   (Cycle_Data & 0x80);
    Cycle_Data <<= 1;
    setFlagsNZ (Register_Accumulator |= Cycle_Data);
}

// Undocumented - This opcode DECs the contents of a memory location and then CMPs the result
// with the A register.
void MOS6510::dcm_instr (void)
{
    uint_least16_t tmp;
    Cycle_Data--;
    tmp = (uint_least16_t)(uint8_t) Register_Accumulator - (uint8_t) Cycle_Data;
    setFlagsNZ (tmp);
    setFlagC   (tmp < 0x100);
}

// Undocumented - This opcode INCs the contents of a memory location and then SBCs the result
// from the A register.
void MOS6510::ins_instr (void)
{
    Cycle_Data++;
    Perform_SBC ();
}

// Undocumented - This opcode ANDs the contents of a memory location with the contents of the
// stack pointer register and stores the result in the accumulator, the X
// register, and the stack pointer.  Affected flags: N Z.
void MOS6510::las_instr (void)
{
    setFlagsNZ (Cycle_Data &= endian_16lo8 (Register_StackPointer));
    Register_Accumulator  = Cycle_Data;
    Register_X            = Cycle_Data;
    Register_StackPointer = Cycle_Data;
}

// Undocumented - This opcode loads both the accumulator and the X register with the contents
// of a memory location.
void MOS6510::lax_instr (void)
{
    setFlagsNZ (Register_Accumulator = Register_X = Cycle_Data);
}

// Undocumented - LSE LSRs the contents of a memory location and then EORs the result with
// the accumulator.
void MOS6510::lse_instr (void)
{
    setFlagC     (Cycle_Data & 0x01);
    Cycle_Data = (int8_t) ((uint8_t) Cycle_Data >> 1);
    setFlagsNZ   (Register_Accumulator ^= Cycle_Data);
}

// Undocumented - This opcode ORs the A register with #xx, ANDs the result with an immediate
// value, and then stores the result in both A and X.
// xx may be EE,EF,FE, OR FF, but most emulators seem to use EE
void MOS6510::oal_instr (void)
{
    setFlagsNZ (Register_X = (Register_Accumulator = (Cycle_Data & (Register_Accumulator | 0xee))));
}

// Undocumented - RLA ROLs the contents of a memory location and then ANDs the result with
// the accumulator.
void MOS6510::rla_instr (void)
{
    uint8_t tmp = Cycle_Data & 0x80;
    Cycle_Data  = Cycle_Data << 1;
    if (getFlagC ()) Cycle_Data |= 0x01;
    setFlagC   (tmp);
    setFlagsNZ (Register_Accumulator &= Cycle_Data);
}

// Undocumented - RRA RORs the contents of a memory location and then ADCs the result with
// the accumulator.
void MOS6510::rra_instr (void)
{
    uint8_t tmp = Cycle_Data & 0x01;
    Cycle_Data  = (int8_t) ((uint8_t) Cycle_Data >> 1);
    if (getFlagC ()) Cycle_Data |= 0x80;
    setFlagC (tmp);
    Perform_ADC ();
}

// Undocumented - This opcode ANDs the contents of the A and X registers (without changing
// the contents of either register) and transfers the result to the stack
// pointer.  It then ANDs that result with the contents of the high byte of
// the target address of the operand +1 and stores that final result in
// memory.
void MOS6510::tas_instr (void)
{
    endian_16lo8  (Register_StackPointer, Register_Accumulator & Register_X);
    uint_least16_t tmp = Register_StackPointer & (Cycle_EffectiveAddress + 1);
    Cycle_Data         = (signed) endian_16lo8 (tmp);
}

#endif // X86


//-------------------------------------------------------------------------//
// Initialise and create CPU Chip                                          //

//MOS6510::MOS6510 (model_t _model, const char *id)
MOS6510::MOS6510 ()
{
    struct ProcessorOperations *instr;
    uint8_t legalMode  = true;
    uint8_t legalInstr = true;
    uint    i, pass;

    //----------------------------------------------------------------------
    // Build up the processor instruction table
    for (i = 0; i < 0x100; i++)
    {
#if MOS6510_DEBUG > 1
        printf ("Building Command %d[%02x]..", i, (uint8_t) i);
#endif

        // Pass 1 allocates the memory, Pass 2 builds the instruction
        instr                = &instrTable[i];
        instr->cycle         = NULL;
        instr->overlap       = true;

        for (pass = 0; pass < 2; pass++)
        {
            cycleCount = -1;
            legalMode  = true;
            legalInstr = true;
            if (pass) procCycle = instr->cycle;

            switch (i)
            {
            // Accumulator or Implied addressing
            case ASLn: case CLCn: case CLDn: case CLIn: case CLVn:  case DEXn:
            case DEYn: case INXn: case INYn: case LSRn: case NOPn_: case PHAn:
            case PHPn: case PLAn: case PLPn: case ROLn: case RORn:  case RTIn:
            case RTSn: case SECn: case SEDn: case SEIn: case TAXn:  case TAYn:
            case TSXn: case TXAn: case TXSn: case TYAn:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            // Zero Page Addressing Mode Handler
            case ADCz: case ANDz: case ASLz: case BITz: case CMPz: case CPXz:
            case CPYz: case DCPz: case DECz: case EORz: case INCz: case ISBz:
            case LAXz: case LDAz: case LDXz: case LDYz: case LSRz: case NOPz_:
            case ORAz: case ROLz: case RORz: case SAXz: case SBCz: case SREz:
            case STAz: case STXz: case STYz: case SLOz: case RLAz: case RRAz:
            // ASOz AXSz DCMz INSz LSEz - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowAddr;
            break;

            // Zero Page with X Offset Addressing Mode Handler
            case ADCzx:  case ANDzx: case ASLzx: case CMPzx: case DCPzx: case DECzx:
            case EORzx:  case INCzx: case ISBzx: case LDAzx: case LDYzx: case LSRzx:
            case NOPzx_: case ORAzx: case RLAzx: case ROLzx: case RORzx: case RRAzx:
            case SBCzx:  case SLOzx: case SREzx: case STAzx: case STYzx:
            // ASOzx DCMzx INSzx LSEzx - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowAddrX;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            // Zero Page with Y Offset Addressing Mode Handler
            case LDXzy: case STXzy: case SAXzy: case LAXzy:
            // AXSzx - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowAddrY;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            // Absolute Addressing Mode Handler
            case ADCa: case ANDa: case ASLa: case BITa: case CMPa: case CPXa:
            case CPYa: case DCPa: case DECa: case EORa: case INCa: case ISBa:
            case JMPw: case JSRw: case LAXa: case LDAa: case LDXa: case LDYa:
            case LSRa: case NOPa: case ORAa: case ROLa: case RORa: case SAXa:
            case SBCa: case SLOa: case SREa: case STAa: case STXa: case STYa:
            case RLAa: case RRAa:
            // ASOa AXSa DCMa INSa LSEa - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowAddr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighAddr;
            break;

            // Absolute With X Offset Addressing Mode Handler
            case ADCax:  case ANDax: case ASLax: case CMPax: case DCPax: case DECax:
            case EORax:  case INCax: case ISBax: case LDAax: case LDYax: case LSRax:
            case NOPax_: case ORAax: case RLAax: case ROLax: case RORax: case RRAax:
            case SBCax:  case SHYax: case SLOax: case SREax: case STAax:
            // ASOax DCMax INSax LSEax SAYax - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowAddr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighAddrX;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            // Absolute With Y Offset Addresing Mode Handler
            case ADCay: case ANDay: case CMPay: case DCPay: case EORay: case ISBay:
            case LASay: case LAXay: case LDAay: case LDXay: case ORAay: case RLAay:
            case RRAay: case SBCay: case SHAay: case SHSay: case SHXay: case SLOay:
            case SREay: case STAay:
            // ASOay AXAay DCMay INSax LSEay TASay XASay - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowAddr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighAddrY;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            // Immediate and Relative Addressing Mode Handler
            case ADCb: case ANDb:  case ANCb_: case ANEb: case ASRb: case ARRb:
            case BCCr: case BCSr:  case BEQr:  case BMIr: case BNEr: case BPLr:
            case BRKn: case BVCr:  case BVSr:  case CMPb: case CPXb: case CPYb:
            case EORb: case LDAb:  case LDXb:  case LDYb: case LXAb: case NOPb_:
            case ORAb: case SBCb_: case SBXb:
            // OALb ALRb XAAb - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchDataByte;
            break;

            // Absolute Indirect Addressing Mode Handler
            case JMPi:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowPointer;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighPointer;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowEffAddr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighEffAddr;
            break;

            // Indexed with X Preinc Addressing Mode Handler
            case ADCix: case ANDix: case CMPix: case DCPix: case EORix: case ISBix:
            case LAXix: case LDAix: case ORAix: case SAXix: case SBCix: case SLOix:
            case SREix: case STAix: case RLAix: case RRAix:
            // ASOix AXSix DCMix INSix LSEix - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowPointerX;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowEffAddr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighEffAddr;
            break;

            // Indexed with Y Postinc Addressing Mode Handler
            case ADCiy: case ANDiy: case CMPiy: case DCPiy: case EORiy: case ISBiy:
            case LAXiy: case LDAiy: case ORAiy: case RLAiy: case RRAiy: case SBCiy:
            case SHAiy: case SLOiy: case SREiy: case STAiy:
            // AXAiy ASOiy LSEiy DCMiy INSiy - Optional Opcode Names
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowPointer;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchLowEffAddr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchHighEffAddrY;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            default:
                legalMode = false;
            break;
            }

            // By default the addressing mode ends after the above cycles
            instr->lastAddrCycle = cycleCount;
            // But some instructions still do a few extra ones that need
            // to be catered for e.g those doing read modify operations

            //---------------------------------------------------------------------------------------
            // Addressing Modes Finished, other cycles are instruction dependent
            switch(i)
            {
            case ADCz: case ADCzx: case ADCa: case ADCax: case ADCay: case ADCix:
            case ADCiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case ADCb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::adc_instr;
            break;

            case ANCb_:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::anc_instr;
            break;

            case ANDz: case ANDzx: case ANDa: case ANDax: case ANDay: case ANDix:
            case ANDiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case ANDb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::and_instr;
            break;

            case ANEb: // Also known as XAA
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::ane_instr;
            break;

            case ARRb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::arr_instr;
            break;

            case ASLn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::asla_instr;
            break;

            case ASLz: case ASLzx: case ASLa: case ASLax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::asl_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case ASRb: // Also known as ALR
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::alr_instr;
            break;

            case BCCr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bcc_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BCSr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bcs_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BEQr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::beq_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BITz: case BITa:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bit_instr;
            break;

            case BMIr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bmi_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BNEr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bne_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BPLr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bpl_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BRKn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::brk_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushLowPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::brk1_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::brk2_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::brk3_instr;
                instr->overlap = false;
            break;

            case BVCr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bvc_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case BVSr:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::bvs_instr;
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
            break;

            case CLCn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::clc_instr;
            break;

            case CLDn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::cld_instr;
            break;

            case CLIn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::cli_instr;
            break;

            case CLVn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::clv_instr;
            break;

            case CMPz: case CMPzx: case CMPa: case CMPax: case CMPay: case CMPix:
            case CMPiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case CMPb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::cmp_instr;
            break;

            case CPXz: case CPXa:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case CPXb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::cpx_instr;
            break;

            case CPYz: case CPYa:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case CPYb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::cpy_instr;
            break;

            case DCPz: case DCPzx: case DCPa: case DCPax: case DCPay: case DCPix:
            case DCPiy: // Also known as DCM
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::dcm_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case DECz: case DECzx: case DECa: case DECax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::dec_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case DEXn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::dex_instr;
            break;

            case DEYn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::dey_instr;
            break;

            case EORz: case EORzx: case EORa: case EORax: case EORay: case EORix:
            case EORiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case EORb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::eor_instr;
            break;

/* HLT // Also known as JAM
            case 0x02: case 0x12: case 0x22: case 0x32: case 0x42: case 0x52:
            case 0x62: case 0x72: case 0x92: case 0xb2: case 0xd2: case 0xf2:
            case 0x02: case 0x12: case 0x22: case 0x32: case 0x42: case 0x52:
            case 0x62: case 0x72: case 0x92: case 0xb2: case 0xd2: case 0xf2:
                cycleCount++; if (pass) procCycle[cycleCount] = hlt_instr;
                instr->overlap = false;
            break;
*/

            case INCz: case INCzx: case INCa: case INCax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::inc_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case INXn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::inx_instr;
            break;

            case INYn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::iny_instr;
            break;

            case ISBz: case ISBzx: case ISBa: case ISBax: case ISBay: case ISBix:
            case ISBiy: // Also known as INS
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::ins_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case JMPw: case JMPi:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::jmp_instr;
            break;

            case JSRw:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::jsr_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushLowPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::jmp_instr;
                instr->overlap = false;
            break;

            case LASay:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::las_instr;
            break;

            case LAXz: case LAXzy: case LAXa: case LAXay: case LAXix: case LAXiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::lax_instr;
            break;

            case LDAz: case LDAzx: case LDAa: case LDAax: case LDAay: case LDAix:
            case LDAiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case LDAb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::lda_instr;
            break;

            case LDXz: case LDXzy: case LDXa: case LDXay:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case LDXb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::ldx_instr;
            break;

            case LDYz: case LDYzx: case LDYa: case LDYax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case LDYb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::ldy_instr;
            break;

            case LSRn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::lsra_instr;
            break;

            case LSRz: case LSRzx: case LSRa: case LSRax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::lsr_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case NOPn_: case NOPb_:
                // Should not be required!
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
            break;

            case NOPz_: case NOPzx_: case NOPa: case NOPax_:
            // NOPb NOPz NOPzx - Also known as SKBn
            // NOPa NOPax      - Also known as SKWn
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
#endif
                instr->lastAddrCycle = cycleCount;

                // Should not be required!
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
            break;

            case LXAb: // Also known as OAL
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::oal_instr;
            break;

            case ORAz: case ORAzx: case ORAa: case ORAax: case ORAay: case ORAix:
            case ORAiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case ORAb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::ora_instr;
            break;

            case PHAn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::pha_instr;
                instr->overlap = false;
            break;

            case PHPn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushSR;
                instr->overlap = false;
            break;

            case PLAn:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::pla_instr;
                instr->overlap = false;
            break;

            case PLPn:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::plp_instr;
                instr->overlap = false;
            break;

            case RLAz: case RLAzx: case RLAix: case RLAa: case RLAax: case RLAay:
            case RLAiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rla_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case ROLn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rola_instr;
            break;

            case ROLz: case ROLzx: case ROLa: case ROLax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rol_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case RORn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rora_instr;
            break;

            case RORz: case RORzx: case RORa: case RORax:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::ror_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case RRAa: case RRAax: case RRAay: case RRAz: case RRAzx: case RRAix:
            case RRAiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rra_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case RTIn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PopSR;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PopLowPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PopHighPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rti_instr;
                instr->overlap = false;
            break;

            case RTSn:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PopLowPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PopHighPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::rts_instr;
                instr->overlap = false;
            break;

            case SAXz: case SAXzy: case SAXa: case SAXix: // Also known as AXS
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::axs_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
            break;

            case SBCz: case SBCzx: case SBCa: case SBCax: case SBCay: case SBCix:
            case SBCiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
            case SBCb_:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sbc_instr;
            break;

            case SBXb:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sbx_instr;
            break;

            case SECn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sec_instr;
            break;

            case SEDn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sed_instr;
            break;

            case SEIn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sei_instr;
            break;

            case SHAay: case SHAiy: // Also known as AXA
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::axa_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case SHSay: // Also known as TAS
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::shs_instr;
            break;

            case SHXay: // Also known as XAS
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::xas_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
            break;

            case SHYax: // Also known as SAY
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::say_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
            break;

            case SLOz: case SLOzx: case SLOa: case SLOax: case SLOay: case SLOix:
            case SLOiy: // Also known as ASO
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::aso_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case SREz: case SREzx: case SREa: case SREax: case SREay: case SREix:
            case SREiy: // Also known as LSE
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::FetchEffAddrDataByte;
                instr->lastAddrCycle = cycleCount;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::lse_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case STAz: case STAzx: case STAa: case STAax: case STAay: case STAix:
            case STAiy:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sta_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case STXz: case STXzy: case STXa:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::stx_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case STYz: case STYzx: case STYa:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::sty_instr;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PutEffAddrDataByte;
                instr->overlap = false;
            break;

            case TAXn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::tax_instr;
            break;

            case TAYn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::tay_instr;
            break;

            case TSXn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::tsx_instr;
            break;

            case TXAn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::txa_instr;
            break;

            case TXSn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::txs_instr;
            break;

            case TYAn:
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::tya_instr;
            break;

            default:
                legalInstr = false;
            break;
            }

            if (!legalMode && !legalInstr)
            {
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::illegal_instr;
                // -1 is debug off, however we must set to -2 to count the
                // +1 at end of loop!
                instr->lastAddrCycle = -2;
                instr->overlap = false;
            }
            else if (!legalMode || !legalInstr)
            {
                printf ("\nInstruction 0x%x: Not built correctly.\n\n", i);
                exit(1);
            }

            cycleCount++;
            if (!pass)
            {   // Pass 1 - Allocate Memory
                if (cycleCount)
                {
#if defined(_MSC_VER) && (_MSC_VER <= _MSC_VER_BAD_NEW)
                    typedef void (MOS6510::*ptr2cycle) (void);
                    instr->cycle = (ptr2cycle*) new char[sizeof (ptr2cycle) *cycleCount];
#else
#   ifdef HAVE_EXCEPTIONS
                    instr->cycle = new(nothrow) (void (MOS6510::*[cycleCount]) (void));
#   else
                    instr->cycle = new (void (MOS6510::*[cycleCount]) (void));
#   endif
#endif // _MSC_VER
                    if (!instr->cycle)
                        goto MOS6510_MemAllocFailed;
                }
            }
            else
                instr->opcode = i;

#if MOS6510_DEBUG > 1
            printf (".");
#endif
        }

        instr->lastAddrCycle++;
        instr->lastCycle = cycleCount - 1;
#if MOS6510_DEBUG > 1
        printf ("Done [%d Cycles]\n", cycleCount);
#endif
    }

    //----------------------------------------------------------------------
    // Build interrupts
    for (i = 0; i < 3; i++)
    {
#if MOS6510_DEBUG > 1
        printf ("Building Interrupt %d[%02x]..", i, (uint8_t) i);
#endif

        // Pass 1 allocates the memory, Pass 2 builds the interrupt
        instr                = &interruptTable[i];
        instr->cycle         = NULL;
        instr->overlap       = false;
        instr->lastAddrCycle = -1;
        instr->opcode        = 0;

        for (int pass = 0; pass < 2; pass++)
        {
            cycleCount = -1;
            if (pass) procCycle = instr->cycle;

            switch (i)
            {
            case oRST:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::RSTRequest;
            break;

            case oNMI:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushHighPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushLowPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushSR;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::NMIRequest;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::NMI1Request;
            break;

            case oIRQ:
#ifdef MOS6510_ACCURATE_CYCLES
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::WasteCycle;
#endif
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushHighPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::PushLowPC;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::IRQRequest;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::IRQ1Request;
                cycleCount++; if (pass) procCycle[cycleCount] = &MOS6510::IRQ2Request;
            break;
            }

            cycleCount++;
            if (!pass)
            {   // Pass 1 - Allocate Memory
                if (cycleCount)
                {
#if defined(_MSC_VER) && (_MSC_VER <= _MSC_VER_BAD_NEW)
                    typedef void (MOS6510::*ptr2cycle) (void);
                    instr->cycle = (ptr2cycle*) new char[sizeof (ptr2cycle) *cycleCount];
#else
#   ifdef HAVE_EXCEPTIONS
                    instr->cycle = new(nothrow) (void (MOS6510::*[cycleCount]) (void));
#   else
                    instr->cycle = new (void (MOS6510::*[cycleCount]) (void));
#   endif
#endif // _MSC_VER
                    if (!instr->cycle)
                        goto MOS6510_MemAllocFailed;
                }
            }

#if MOS6510_DEBUG > 1
            printf (".");
#endif
        }

        instr->lastCycle = cycleCount - 1;
#if MOS6510_DEBUG > 1
        printf ("Done [%d Cycles]\n", cycleCount);
#endif
    }

    // Intialise Processor Registers
    Register_Accumulator   = 0;
    Register_X             = 0;
    Register_Y             = 0;

    Cycle_EffectiveAddress = 0;
    Cycle_Data             = 0;
    fetchCycle[0]          = &MOS6510::FetchOpcode;

    Initialise ();
return;

MOS6510_MemAllocFailed:
    printf ("Unable to allocate enough memory.\n\n");
exit (-1);
}

MOS6510::~MOS6510 ()
{
    struct ProcessorOperations *instr;
    uint i;

    // Remove Opcodes
    for (i = 0; i < 0x100; i++)
    {
        instr = &instrTable[i];
        if (instr->cycle != NULL) delete [] instr->cycle;
    }

    // Remove Interrupts
    for (i = 0; i < 3; i++)
    {
        instr = &interruptTable[i];
        if (instr->cycle != NULL) delete [] instr->cycle;
    }
}


//-------------------------------------------------------------------------//
// Initialise CPU Emulation (Registers)                                    //
void MOS6510::Initialise (void)
{
    // Reset Cycle Count
    cycleCount = 0;
    procCycle  = fetchCycle;

    // Reset Status Register
    Register_Status = (1 << SR_NOTUSED);
    // FLAGS are set from data directly and do not require
    // being calculated first before setting.  E.g. if you used
    // SetFlags (0), N flag would = 0, and Z flag would = 1.
    setFlagsNZ (1);
    setFlagC   (false);
    setFlagV   (false);

    // Set PC to some value
    Register_ProgramCounter = 0;
}

//-------------------------------------------------------------------------//
// Reset CPU Emulation                                                     //
void MOS6510::reset (void)
{
    // Internal Stuff
    Initialise ();

    // Reset stack
    Register_StackPointer = endian_16 (SP_PAGE, 0xFF);

    // Reset Interrupts
    interrupts.pending = false;
    interrupts.irqs    = 0;

    // Requires External Bits
    // Read from reset vector for program entry point
    endian_16lo8 (Cycle_EffectiveAddress, envReadMemDataByte (0xFFFC));
    endian_16hi8 (Cycle_EffectiveAddress, envReadMemDataByte (0xFFFD));
    Register_ProgramCounter = Cycle_EffectiveAddress;
}

//-------------------------------------------------------------------------//
// Module Credits                                                          //
void MOS6510::credits (char *sbuffer)
{   // Copy credits to buffer
    sprintf (sbuffer, "%sModule     : MOS6510 Cycle Exact Emulation\n", sbuffer);
    sprintf (sbuffer, "%sWritten By : %s\n", sbuffer, MOS6510_AUTHOR);
    sprintf (sbuffer, "%sVersion    : %s\n", sbuffer, MOS6510_VERSION);
    sprintf (sbuffer, "%sReleased   : %s\n", sbuffer, MOS6510_DATE);
    sprintf (sbuffer, "%sEmail      : %s\n", sbuffer, MOS6510_EMAIL);
}
