#ifndef _sidusage_h_
#define _sidusage_h_

#include "sidtypes.h"

enum
{
    SID_READ        = 1 << 0,
    SID_WRITE       = 1 << 1,
    SID_EXECUTE     = 1 << 2,
    SID_STACK       = 1 << 3,
    SID_SAMPLE      = 1 << 4,
    SID_BAD_READ    = 1 << 5,
    SID_BAD_EXECUTE = 1 << 6,

    // Next byte provides extended information
    SID_EXTENSION   = 1 << 7,

    // Once SID_EXTENSION has been tested for, you
    // are recommened to throw that bit away and
    // assign it the following meaning where
    // appropriate.  This bit is not saved to file.
    SID_LOAD_IMAGE  = 1 << 7
};

enum
{
    SID_IRQ         = 1 << 0,  // Play == 0
    SID_IRQ_RTI     = 1 << 1,  // Switch out kernal with no I flag
    SID_IRQ_IFLAG   = 1 << 2,  // Bad I flag
    SID_IRQ_DISABLE = 1 << 3,
    SID_IRQ_NOACK   = 1 << 4,
    SID_NMI         = 1 << 5,  // Enables NMI IRQs
    SID_EXECUTE_IO  = 1 << 6,  // Execution in IO (under)
    SID_EXECUTE_ROM = 1 << 7,  // Execution in ROM (under)
    SID_IRQ_OTHER   = 1 << 8,  // Enabled alternative IRQ
    SID_BAD_STACK   = 1 << 9,  // Tune corrupts stack contents
    SID_INSTR_EX    = 1 << 10, // Extended 6510 instructions
    SID_INSTR_ILL   = 1 << 11, // Illegal 6510 instructions
    //SID_PSID_RANDOM = 1 << 12, // PSID Random extension
    SID_INSTR_BRK   = 1 << 13  // 6510 BRK, indications of a bad tune...
};

// All unused have a reserved meaning and must contain
// a zero value
typedef struct sid_usage_t
{
    uint_least32_t flags;
    // The next value may change to 16 bits
    uint_least8_t  memory[0x10000];
    char           md5[33]; // MD5 key
    uint_least16_t length; // usage scan length
} sid_usage_t;

#endif /* _sidusage_h_ */
