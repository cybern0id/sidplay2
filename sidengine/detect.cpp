/***************************************************************************
                          detect.cpp  -  Take multiple sid tunes and detect
                                         common sequences
                             -------------------
    begin                : Sun Jan 5 2003
    copyright            : (C) 2003 by Simon White
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

#include <new>
#include <stdio.h>
#include <string.h>

#ifdef HAVE_MSWINDOWS
#   include <sys/stat.h>
#   include "win32-dirent.h"
    typedef unsigned int uint32_t;
#else
#   include <sys/fcntl.h>
#   include <dirent.h>
#endif

#include <sidplay/utils/libini.h>
#include <sidplay/utils/SidTuneMod.h>
#include <sidplay/utils/SidUsage.h>

#define SDEBUG(x)
#define MDEBUG(x) printf x
#define MAX_PATH_LEN 1024
#define NLENGTH(dirent) ((int)strlen((dirent)->d_name))

#define RULE_MATCH 30      /* Minimum number continuous match positions */
#define RULE_MAX_REPEAT 1  /* Maximum number of repeated values */
#define RULE_MIN_MATCH 100 /* Minimum number before assuming we are good */
#define RULE_MAX_ERROR 2   /* Maximum repeated errors before throwing a rule */

// Table represents addresses bye operands only!
// Perhaps later use
static int opcode[0x100] =
{
  0, // BRKn (0x00)
  1, // ORAix
  0, // HLT
  1, // SLOix
  1, // NOPz
  1, // ORAz
  1, // ASLz
  1, // SLOz
  0, // PHPn
  0, // ORAb
  0, // ASLn
  0, // ANCb
  2, // NOPa
  2, // ORAa
  2, // ASLa
  2, // SLOa
  0, // BPLr (0x10)
  1, // ORAiy
  0, // HLT
  1, // SLOiy
  1, // NOPzx
  1, // ORAzx
  1, // ASLzx
  1, // SLOzx
  0, // CLCn
  2, // ORAay
  0, // NOPn
  2, // SLOay
  2, // NOPax
  2, // ORAax
  2, // ASLax
  2, // SLOax
  2, // JSRw (0x20)
  1, // ANDix
  0, // HLT
  1, // RLAix
  1, // BITz
  1, // ANDz
  1, // ROLz
  1, // RLAz
  0, // PLPn
  0, // ANDb
  0, // ROLn
  0, // ANCb
  2, // BITa
  2, // ANDa
  2, // ROLa
  2, // RLAa
  0, // BMIr (0x30)
  1, // ANDiy
  0, // HLT
  1, // RLAiy
  1, // NOPzx
  1, // ANDzx
  1, // ROLzx
  1, // RLAzx
  0, // SECn
  2, // ANDay
  0, // NOPn
  2, // RLAay
  2, // NOPax
  2, // ANDax
  2, // ROLax
  2, // RLAax
  0, // RTIn (0x40)
  1, // ANDix
  0, // HLT
  1, // SREix
  1, // NOPz
  1, // EORz
  1, // LSRz
  1, // SREz
  0, // PHAn
  0, // EORb
  0, // LSRn
  0, // ASRb
  2, // JMPw
  2, // EORa
  2, // LSRa
  2, // SREa
  0, // BVCr (0x50)
  1, // EORiy
  0, // HLT
  1, // SREiy
  1, // NOPzx
  1, // EORzx
  1, // LSRzx
  1, // SREzx
  0, // CLIn
  2, // EORay
  0, // NOPn
  2, // SREay
  2, // NOPax
  2, // EORax
  2, // LSRax
  2, // SREax
  0, // RTSn (0x60)
  1, // ADCix
  0, // HLT
  1, // RRAix
  1, // NOPz
  1, // ADCz
  1, // RORz
  1, // RRAz
  0, // PLAn
  0, // EORb
  0, // RORn
  0, // ARRb
  2, // JMPi
  2, // ADCa
  2, // RORa
  2, // RRAa
  0, // BVSr (0x70)
  1, // ADCiy
  0, // HLT
  1, // RRAiy
  1, // NOPzx
  1, // ADCzx
  1, // RORzx
  1, // RRAzx
  0, // SEIn
  2, // ADCay
  0, // NOPn
  2, // RRAay
  2, // NOPax
  2, // ADCax
  2, // RORax
  2, // RRAax
  0, // NOPb (0x80)
  1, // STAix
  0, // NOPb
  1, // SAXix
  1, // STYz
  1, // STAz
  1, // STXz
  1, // SAXz
  0, // DEYn
  0, // NOPb
  0, // TAXn
  0, // ANEb
  2, // STYa
  2, // STAa
  2, // STXa
  2, // SAXa
  0, // BCCr (0x90)
  1, // STAiy
  0, // HLT
  1, // SHAiy
  1, // STYzx
  1, // STAzx
  1, // STXzy
  1, // SAXzy
  0, // TYAn
  2, // STAay
  0, // TXSn
  2, // SHSay
  2, // SHYax
  2, // STAax
  2, // SHXay
  2, // SHAay
  0, // LDYb (0xA0)
  1, // LDAix
  0, // LDXb
  1, // LAXix
  1, // LDYz
  1, // LDAz
  1, // LDXz
  1, // LAXz
  0, // TAYn
  0, // LDAb
  0, // TAXn
  0, // LXAb
  2, // LDYa
  2, // LDAa
  2, // LDXa
  2, // LAXa
  0, // BCSr (0xB0)
  1, // LDAiy
  0, // HLT
  1, // LAXiy
  1, // LDYzx
  1, // LDAzx
  1, // LDXzy
  1, // SAXzy
  0, // TYAn
  2, // STAay
  0, // TXSn
  2, // SHSay
  2, // SHYax
  2, // STAax
  2, // SHXay
  2, // LAXay
  0, // CPYb (0xC0)
  1, // CMPix
  0, // NOPb
  1, // DCPix
  1, // CPYz
  1, // CMPz
  1, // DECz
  1, // DCPz
  0, // INYn
  0, // CMPb
  0, // DEXn
  0, // SBXb
  2, // CPYa
  2, // CMPa
  2, // DECa
  2, // DCPa
  0, // BNEr (0xD0)
  1, // CMPiy
  0, // HLT
  1, // DCPiy
  1, // NOPzx
  1, // CMPzx
  1, // DECzx
  1, // DCPzx
  0, // CLDn
  2, // CMPay
  0, // NOPn
  2, // DCPay
  2, // NOPax
  2, // CMPax
  2, // DECax
  2, // DCPax
  0, // CPXb (0xE0)
  1, // SBCix
  0, // NOPb
  1, // ISBix
  1, // CPXz
  1, // SBCz
  1, // INCz
  1, // ISBz
  0, // INXn
  0, // SBCb
  0, // NOPn
  0, // SBCb
  2, // CPXa
  2, // SBCa
  2, // INCa
  2, // ISBa
  0, // BEQr (0xF0)
  1, // SBCiy
  0, // HLT
  1, // ISBiy
  1, // NOPzx
  1, // SBCzx
  1, // INCzx
  1, // ISBzx
  0, // SEDn
  2, // SBCay
  0, // NOPn
  2, // ISBay
  2, // NOPax
  2, // SBCax
  2, // INCax
  2  // ISBax
};


static uint32_t analyse (FILE *fmatched, const char *file);
static uint32_t recurse (FILE *fmatched, char *dir);


struct md5_store
{
    char md5[SIDTUNE_MD5_LENGTH + 1];
    int  player_id;
    md5_store *next;
};


// Stores md5s of sids matching the rule
static md5_store *md5s[0x10000] = {0};


class Rule
{
protected:
    typedef enum {
        IGNORED   = 0,
        REL_LO    = 1,
        REL_HI    = 2,
        REL_MASK  = 3,
        UNKNOWN   = 4,
        CONFIRMED = 8,
        CONSTANT  = 12
    };

    unsigned char m_memory[0x10000];
    int        m_flags[0x10000];
    int        m_addr[0x10000];

    ini_fd_t m_inifd;
    int  m_base;
    int  m_length;
    int  m_ids;
    bool m_status;
    char m_filename[MAX_PATH_LEN];
    char m_md5[SIDTUNE_MD5_LENGTH + 1];
    int  m_player_id;
    int  m_matches;

public: // These are debug only
    int  addr    (int id) { return m_addr[id]; }
    int  base    () const { return m_base; }
    int  length  () const { return m_length; }
    void player_id (int player_id) { m_player_id = player_id; }
    int  player_id () { return m_player_id; }

    bool md5_used ();
    void md5_add  (const char *md5, int player_id);
    void md5_add  () { md5_add (m_md5, m_player_id); }
    void md5_del  ();
    const char *md5_get() { return m_md5; }

protected:
    int  _end   () const { return m_base + m_length; }

    void _image (int base, int length, int repeats);
    void _usage (const char *filename, const char *md5);
    void _eval  ();
    void _compress ();
    int  _match (int addr, int taddr, const Rule& test);

public:
    Rule (): m_base(0), m_length (0), m_ids(0), m_status(false), m_player_id(0), m_matches(0)
    { m_filename[0] = '\0'; m_md5[0] = '\0'; m_inifd = ini_open ("ids.ini", "w", ";"); }
    ~Rule () { ini_close (m_inifd); }

    int  ids     () const { return m_ids; }
    bool load    (const char *filename, int repeats, bool usage);
    int  find    (const Rule &rule, int id, int length) const;
    bool combine (const Rule &rule, int id, int myid, int threshold);
    const char *filename () const { return m_filename; }
    // Returns an id value or -1 on error
    int  operator[](int id) const { return m_memory[m_addr[id]]; }
    operator bool () const { return m_status; }
    bool store   ();
    bool recall  (int player_id);
};

// Setup default rule image
void Rule::_image (int base, int length, int repeats)
{
    int end    = base + length;
    int count  = 0;
    int lval   = -1;
    int repeat = 0;

    if (repeats < 0)
    {   // Allow all repeats
        for (int i = base; i < end; i++)
        {   // Add ID to default rule
            m_flags[i] = UNKNOWN;
            m_addr[count++] = i;
        }
    }
    else
    {   // Remove continuous repeats
        for (int i = base; i < end; i++)
        {
            int val = m_memory[i];
            if (val == lval)
            {
                if (repeat >= repeats)
                    continue;
                repeat++;
            }
            else
		    {
                repeat = 0;
                lval   = val;
		    }

            // Add ID to default rule
            m_flags[i] = UNKNOWN;
            m_addr[count++] = i;
        }
    }
    m_base   = base;
    m_length = length;
    m_ids    = count;
}

// Load sid file and setup default rule ids
bool Rule::load (const char *filename, int repeats, bool usage)
{
    SidTuneMod tune(0);
    if (!tune.load(filename))
    {
        m_status = false;
        return false;
    }

    strcpy (m_md5, tune.createMD5 ());
    if (md5_used ())
	    return false;

    for (int i = 0; i < 0x10000; i++)
        m_flags[i] = IGNORED;
    tune.placeSidTuneInC64mem (m_memory);
    { // Setup load image
        const SidTuneInfo &info = tune.getInfo ();
        _image (info.loadAddr, info.c64dataLen, repeats);
    }
    if (usage)
	    _usage (filename, m_md5);

    _eval  ();
    m_status   = true;
    strcpy (m_filename, filename);
    return true;
}

// Remove all ignored values fomr the rule
void Rule::_compress ()
{
    int addr;
    int start = m_ids, end = 0;

    // trim start
    addr = this->_end ();
    for (int id = 0; id < m_ids; id++)
    {
        addr = m_addr[id];
        if (m_flags[addr] != IGNORED)
	    {
            start  = id;
		    break;
	    }
    }
    m_base = addr;

    // trim end
    addr = m_base - 1;
    for (int id = m_ids; id-- > start;)
    {
        addr = m_addr[id];
        if (m_flags[addr] != IGNORED)
	    {
            end = id + 1;
		    break;
	    }
    }
    m_length = addr - m_base + 1;

    // determine available id matches
    int count = 0;
    for (int id = 0; id < m_ids; id++)
    {
        addr = m_addr[id];
        if (m_flags[addr] != IGNORED)
        {
            m_addr[count] = addr;
            count++;
        }
    }
    m_ids = count;
}

// Remove any id groups that contain less than the minimum number of ids
// and clean up the rule
void Rule::_eval ()
{
    int start, last, count;

    start = 0;
    last  = m_addr[start];
    count = 0;
    for (int id = 0; id < m_ids; id++)
    {
	    int addr = m_addr[id];
        // If address if ingored then skip it
	    if (m_flags[addr] == IGNORED)
            continue;

	    // Address distance must not be greater the the rule allows
	    // else we have moved into another group
	    if (addr <= (last + RULE_MAX_ERROR + 1))
	    {
		    last = addr;
            count++;
		    continue;
	    }

	    // End of a match group, check if it meets requirements
	    if (count < RULE_MIN_MATCH)
	    {
		    // Remove this group as is too short
		    while (count > 0)
            {
                int a = m_addr[start++];
                if (m_flags[a] != IGNORED)
                {
                    m_flags[a] = IGNORED;
                    count--;
                }
            }
	    }

	    last  = addr;
	    start = id;
        count = 1;
  }

  // End of a match group, check if it meets requirements
    if (count < RULE_MIN_MATCH)
    {
        // Remove this group as is too short
        while (count > 0)
        {
            int a = m_addr[start++];
            if (m_flags[a] != IGNORED)
            {
                m_flags[a] = IGNORED;
                count--;
            }
        }
    }

    _compress ();
}

// Load tunes usage information to better identify usefull
// memory areas to use for the default rule
void Rule::_usage (const char *filename, const char *md5)
{
    sid2_usage_t usage;

    {
        SidUsage loader;
        size_t len = strlen(filename);
        char *name = new(std::nothrow) char[len + 3];
        strcpy (name, filename);

        // Search for extension
        size_t ext = len;
        while (ext > 0)
        {
            if (name[--ext] == '.')
                break;
        }

        // No extension found so add new one to end
        if (!ext)
            ext = len;
        strcpy (&name[ext], ".mm");
        loader.read (name, usage);
        if (!loader)
        {
            MDEBUG (("\tFile load failed %s\n", name));
            delete [] name;
            return;
        }
        delete [] name;
    }

    // If there is an md5 make sure they match so we know the
    // usage file is actually the correct one/uptodate
    if (*usage.md5 != '\0')
    {
        if (strcmp (usage.md5, md5))
        {
            MDEBUG (("\tUsage is not for file %s, or is not upto date\n", filename));
            return;
        }
    }

#if 0
    { // Get a better idea of the data content
        for (int i = usage.start; i <= usage.end; i++)
        {
	        sid_usage_t::memflags_t flags = usage.memory[i];
            if ((flags & (SID_LOAD_IMAGE | SID_BAD_EXECUTE)) == (SID_LOAD_IMAGE | SID_BAD_EXECUTE))
            {
                _abs (i, CONFIRMED);
                if (flags & SID_OPCODE)
                {   // Determine if opcode provides a 16 bit address
                    // and if so indicate it
                    if (opcode (m_memory[i]) == 2)
                        i += _rel16_lo (i+1, UNKNOWN);
                }
            }
        }
    }
#endif
}

// Given a test rule and offset into that rule search ourself
// to see if we have a length match, and return the position
// of that match within ourselves
int Rule::find (const Rule &rule, int id, int length) const
{
    // Check the rules to make sure they are within range
    int end = ids () - length;
    if (end < 0)
        return -1;
    if ((id < 0) || (id >= (rule.ids () - length)))
        return -1;

    int base = rule.m_addr[id];
    for (int myid = 0; myid < end; myid++)
    {
        int i, addr = m_addr[myid] - 1;
		int error = 0;

        for (i = 0; i < length; i++)
        {   // Search region must be continous
            int flags = m_flags[++addr];
            if ((flags == IGNORED) || (rule.m_flags[base+i] == IGNORED) ||
                (m_memory[addr] != rule.m_memory[base+i]))
            {
			    if (++error > RULE_MAX_ERROR)
				    break;
				length++;
            }
			else
			    error = 0;
		}

        if (i == length)
            return myid;
    }
    return -1;
}

// Combine our rule with the supplied rule
bool Rule::combine (const Rule &rule, int id, int myid, int threshold)
{
    // Check the rules to make sure they are within range
    if ((myid < 0) || (myid >= m_ids))
        return false;
    if ((id < 0) || (id >= rule.ids ()))
        return false;

	/*
    int base = rule.m_addr[0];
    {   // Calculate base addresses
        int addr     = m_addr[myid];
        int myoffset = addr - m_base;
        int offset   = rule.m_addr[id] - base;
        myid = 0; // Calculate real starting id
        while (offset < myoffset)
        {
            m_memory[m_base] = IGNORED;
            m_base   = m_addr[++myid];
            myoffset = addr - m_base;
        }
        base = rule.m_addr[id] - myoffset;
    }
	*/

    int base = rule.m_addr[id];
    for (int i = 0; i < myid; i++)
        m_memory[m_addr[i]] = IGNORED;
    m_base = m_addr[myid];

    int end = m_addr[m_ids-1];
    {   // Calculate end address
        int mylen = end - m_base;
        int len   = rule.m_addr[rule.ids()-1] - base;
        if (len   < mylen)
            end   = m_base + len;
    }
    

    // Combine rule
    int count = 0;
    for (; myid < m_ids; myid++)
    {
        int addr = m_addr[myid];
        if (addr > end)
            break;
        int flags = m_flags[addr];
        int i = addr - m_base;
        if (m_memory[addr] != rule.m_memory[base+i])
		{
            m_flags[addr] = IGNORED;
            continue;
        }
        m_addr[count++] = addr;
    }

    // Trim off any remaining end bits
    while (myid < m_ids)
        m_flags[m_addr[myid++]] = IGNORED;
    m_ids = count;

    // This filters the rule of spurious data
    _eval ();
    //md5_add ();
    m_matches++;
    return true;
}


bool Rule::md5_used ()
{
    int index;
    if (m_md5[0] == '\0')
	    return false;
    sscanf (m_md5, "%04x", &index);
    md5_store *p = md5s[index];
    while (p)
	{
	    if (!strcmp (p->md5, m_md5))
		    return true;
		p = p->next;
	}
	return false;
}


void Rule::md5_add (const char *md5, int player_id)
{
    int index;
    if (md5[0] == '\0')
	    return;
    sscanf (md5, "%04x", &index);
    md5_store *n = new(std::nothrow) md5_store;
    if (!n)
        return;
    strcpy (n->md5, md5);
    n->player_id = player_id;
    n->next = md5s[index];
    md5s[index] = n;
}


void Rule::md5_del ()
{
    for (int i = 0; i < 0x10000; i++)
	{
	    md5_store *p = md5s[i];
        while (p)
		{
		    md5_store *n = p->next;
            delete n;
			p = n;
		}
		md5s[i] = 0;
	}
}


bool Rule::store ()
{
    char buff[10];
    char data[0x40006]; // Cater for worst possible case
    if (!m_matches)
        return false;
    sprintf (buff, "%d", m_player_id);
    ini_locateHeading (m_inifd, buff);

    {
        int   base  = m_addr[0] & 0xff00;
	    int   laddr = m_addr[0];
        int   group = 1;
        char *p     = data;

        sprintf (buff, "ID%d", group++);
        ini_locateKey (m_inifd, buff);
        p += sprintf (p, "$%04X", laddr - base);
        laddr--;
        for (int i = 0; i < m_ids; i++)
        {
            int addr = m_addr[i];
            if ((addr - 1) != laddr)
            {
                if ((addr - laddr) > (RULE_MAX_ERROR + 1))
                {
                    if (ini_writeString (m_inifd, data) < 0)
                        return false; // Error storing
                    p  = data;
                    // Move to next group
                    sprintf (buff, "ID%d", group++);
                    ini_locateKey (m_inifd, buff);
                    p  = data;
                    p += sprintf (p, "$%04X", addr - base);
                }
                else
                {
                    while (++laddr < addr)
                        p += sprintf (p, ":-");
                }
            }
            p += sprintf (p, ":$%02X", m_memory[addr]);
            laddr = addr;
        }
        if (ini_writeString (m_inifd, data) < 0)
            return false; // Error storing
    }
    return true;
}
   

bool Rule::recall (int player_id)
{
    char buff[10];
    char data[0x40006]; // Cater for worst possible case
    sprintf (buff, "%d", m_player_id);
    if (ini_locateHeading (m_inifd, buff) < 0)
        return false;

    {
        int   addr  = 0;
        int   count = 0;
        int   group = 1;
        char *p     = data;

        memset (m_memory, 0, sizeof (m_memory));

        for (;;)
        {
            int ret, val;
            sprintf (buff, "ID%d", group++);
            if (ini_locateKey (m_inifd, buff) < 0)
            {
                if (group == 1)
                    return false;
                break;
            }
            if (ini_readString (m_inifd, data, sizeof (data)) < 0)
                return false;

            p   = data;
            ret = 0;
            sscanf (p, "$%x%n", &val, &ret);
            if (ret != 5)
                return false;
            addr = val;
            p   += ret;
            while (*p != '\0')
            {
                int val;

                // Seperator
                if (*p++ != ':')
                    return false;

                // Place holder
                if (*p == '-')
                {
                    addr++;
                    p++;
                    continue;
                }

                // value
                ret = 0;
                sscanf (p, "$%x%n", &val, &ret);
                if (ret != 3)
                    return false;
                p += ret;
                m_memory[addr]  = val;
                m_flags[addr]   = UNKNOWN;
                m_addr[count++] = addr++;
            }
        }
        m_ids = count;
    }

    // Reset
    m_base        = m_addr[0];
    m_status      = true;
    m_filename[0] = '\0';
    m_matches     = 0;
    m_player_id   = player_id;

    // Double check input values are ok
    _eval ();
    return true;
}

// Return -1 no match
// Returns 0 meaning match
// Return +1 meaning match quires skipping of next byte (16 bit address)
int Rule::_match (int addr, int taddr, const Rule &test)
{
    int type  = m_flags[addr] & ~REL_MASK;
    int ttype = test.m_flags[taddr] & ~REL_MASK;
    int rel   = m_flags[addr] & REL_MASK;
    int trel  = test.m_flags[addr] & REL_MASK;

    // If any of the locations are ignored then we cannot match them
    if (type == IGNORED || ttype == IGNORED)
        return -1;

    // If one of the locations is known, then the other must be the same
    if (type == UNKNOWN)
        type =  ttype;

    // Basic absolute check
    if (!(rel | trel))
    {
        if (m_memory[addr] == m_memory[taddr])
            return 0;
    }

    // Throw out relative collisions as when they happen
    // atleast one of the address bytes will be CONST or
    // CONFIRMED thereby forcing the mode of the potentially
    // UNKNOWN byte
    if ((rel | trel) == REL_MASK)
        return -1;

    // Values differ in value or mode such that
    // now only relative combinations can match
    if (type == UNKNOWN)
    {   // Absolute data can be changed to rel, or vice versa
        
    }
    // CONST or CONFIRMED
    else
    {   // No data cannot be change mode
    }

    return -1;
}


static Rule rule;
static Rule test;


int main (int argc, char *argv[])
{
  int player_id = -1;
  FILE *fmatched = fopen ("matched.txt", "a+");

  // Read all current matches
  fseek (fmatched, 0, SEEK_SET);
  {
      static char buff[MAX_PATH_LEN * 2];
      while (fgets (buff, sizeof (buff), fmatched))
	  {
		  char md5[SIDTUNE_MD5_LENGTH+1];
          int  id;
          sscanf (buff, "%s %d", md5, &id);
          if (id > player_id)
              player_id = id + 1;
          rule.md5_add (md5, id);
	  }
  }
  fseek (fmatched, 0, SEEK_END);

  // Look at md5 of input file and determine what player it is from
  // @FIXME@ for now just assume it is a new one

  // Recall matched player
  //if (player_id >= 0)
  //      rule.recall (0);

  uint32_t matches = 0;
  for (int i = 1; i < argc; i++)
  {
      uint32_t ret = recurse (fmatched, argv[i]);
      if (!ret)
          matches += analyse (fmatched, argv[i]);
	  else
          matches += ret;
  }

  rule.store ();
  rule.md5_del ();
  fclose (fmatched);
  return 0;
}


uint32_t analyse (FILE *fmatched, const char *file)
{
      int rule_id = -1, test_id = -1;

      if (!rule)
	  {   // Initial rule
          MDEBUG(("%s\n", file));
          if (!rule.load (file, RULE_MAX_REPEAT, true))
          {
              MDEBUG (("\tFailed to reference tune %s\n", file));
              return 0;
          }
          MDEBUG (("\tInitial rule base $%04x, length %d, matches %d\n", rule.base(), rule.length(), rule.ids()));
		  //rule.md5_add ();
		  return 1;
	  }

      // Now try for a match with tune two. 
      if (!test.load (file, -1, true))
      {
          MDEBUG (("%s\n\tFailed to load, skipping\n", file));
          return 0;
      }

      fprintf (stderr, "%s\n", file);

      int end = test.ids() - RULE_MATCH;
      if (end < 0)
      {
          MDEBUG (("\tInput file not long enough to match against\n", file));
          return 0;
      }

      for (int id = 0; id < end; id++)
      {
          rule_id = rule.find (test, id, RULE_MATCH);
          if (rule_id < 0)
              continue;
          test_id = id;
          break;
      }

      if (rule_id < 0)
      {
          MDEBUG (("\tNo match found\n"));
          return 0;
      }
      else
      {
          MDEBUG (("\tMatch rule id %d ($%04x) with test id %d ($%04x)\n", rule_id, rule.addr(rule_id), test_id, test.addr(test_id)));
      }

      if (!rule.combine (test, test_id, rule_id, RULE_MIN_MATCH))
          MDEBUG (("\tMatches less that minimum threshold, tune ignored\n"));
      else
          MDEBUG (("\tRule base $%04x, length %d, matches %d\n", rule.base(), rule.length(), rule.ids()));
      test.md5_add ();
      fprintf (fmatched, "%s %d %s @ %04x\n", test.md5_get(), rule.player_id(), file, test.addr(test_id));
      return 1;
}


// Taken from tsid
uint32_t recurse(FILE *fmatched, char *dir)
{
    uint32_t matches = 0;
    dirent *dp;
    struct stat st;
    DIR    *dirp;
    char   nbuf[MAX_PATH_LEN];
    size_t len;

    SDEBUG("Recurse into directory " << dir << endl);

    dirp=opendir(dir);

    if (dirp==NULL)
    {
        SDEBUG(dir << " unreadable\n");
        return 0;
    }

    while ((dp=readdir(dirp)) != NULL)
    {
        if (!strcmp(dp->d_name,".") || !strcmp(dp->d_name,".."))
            continue;

        len=strlen(dir);
        if (len+NLENGTH(dp)+1 < MAX_PATH_LEN-1)
        {
            strcpy(nbuf,dir);
            if (len != 0) // dir = "" means current dir on Amiga
                nbuf[len++]='/';
            strcpy(nbuf+len, dp->d_name);

            stat(nbuf, &st);

            if (S_ISDIR(st.st_mode))
                matches += recurse(fmatched, nbuf); // recurse into directory

            if (S_ISREG(st.st_mode))
                matches += analyse(fmatched, nbuf); // add the file
        }
        else
        {
            SDEBUG(dir << "/" << dp->d_name << ": pathname too long\n");
        }
    }
    closedir(dirp);
    return matches;
}
