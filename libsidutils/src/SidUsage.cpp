/***************************************************************************
                          SidUsage.cpp  -  sidusage file support
                             -------------------
    begin                : Tues Nov 19 2002
    copyright            : (C) 2002 by Simon White
    email                : sidplay2@yahoo.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <sidplay/sidendian.h>
#include <sidplay/sidusage.h>
#include <sidplay/SidTune.h>
#include "SidUsage.h"
#include "smm0.h"

static const char *txt_na        = "SID Usage: N/A";
static const char *txt_file      = "SID Usage: Unable to open file";
static const char *txt_corrupt   = "SID Usage: File corrupt";
static const char *txt_invalid   = "SID Usage: Invalid file format";
static const char *txt_missing   = "SID Usage: Mandatory chunks missing";
static const char *txt_supported = "SID Usage: File type not supported";
static const char *txt_reading   = "SID Usage: Error reading file";
static const char *txt_writing   = "SID Usage: Error writing file";


SidUsage::SidUsage ()
:m_status(false)
{
    m_errorString = txt_na;

    // Probably a better way to do this
    // Detup decode table to convert usage flags to text characters
    for (int i = 0; i < SID_LOAD_IMAGE; i++)
    {        
        m_decodeMAP[i][2] = '\0';
        switch (i & (SID_EXECUTE | SID_STACK | SID_SAMPLE))
        {
        case 0:
            switch (i & (SID_READ | SID_WRITE))
            {
            case 0: // Not used
                m_decodeMAP[i][0] = '.';
                m_decodeMAP[SID_LOAD_IMAGE | i][0] = ',';
                break;
            case SID_READ:
                m_decodeMAP[i][0] = 'r';
                m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'R';
                break;
            case SID_WRITE:
                m_decodeMAP[i][0] = 'w';
                m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'W';
                break;
            case SID_WRITE | SID_READ:
                m_decodeMAP[i][0] = 'x';
                m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'X';
                break;
            }
            break;
        case SID_EXECUTE:
            m_decodeMAP[i][0] = 'p';
            if (i & SID_WRITE)
                m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'M';
            else
                m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'P';
            break;
        case SID_STACK:
            m_decodeMAP[i][0] = 's';
            m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'S';
            break;
        case SID_STACK | SID_EXECUTE:
            m_decodeMAP[i][0] = '$';
            m_decodeMAP[SID_LOAD_IMAGE | i][0] = '&';
            break;
        case SID_SAMPLE:
            m_decodeMAP[i][0] = 'd';
            m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'D';
            break;
        case SID_SAMPLE | SID_EXECUTE:
            m_decodeMAP[i][0] = 'e';
            m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'E';
            break;
        case SID_SAMPLE | SID_STACK:
            m_decodeMAP[i][0] = 'z';
            m_decodeMAP[SID_LOAD_IMAGE | i][0] = 'Z';
            break;
        case SID_SAMPLE | SID_STACK | SID_EXECUTE:
            m_decodeMAP[i][0] = '+';
            m_decodeMAP[SID_LOAD_IMAGE | i][0] = '*';
            break;
        }

        switch (i & (SID_BAD_EXECUTE | SID_BAD_READ))
        {
        case SID_BAD_READ:
            m_decodeMAP[i][1] = '?';
            // Fine when is load image
            m_decodeMAP[SID_LOAD_IMAGE | i][1] = ' ';
            break;
        case SID_BAD_EXECUTE:
        case SID_BAD_EXECUTE | SID_BAD_READ:
            m_decodeMAP[i][1] = '!';
            // Fine when is load image
            m_decodeMAP[SID_LOAD_IMAGE | i][1] = ' ';
            break;
        default:
            m_decodeMAP[i][1] = ' ';
            // We wrote first, meaning location need
            // not be in load image.
            m_decodeMAP[SID_LOAD_IMAGE | i][1] = '-';
        }
    }

    // Initialise post filter
    memset (m_filterMAP, ~0, sizeof (m_filterMAP));
    // We are filtering off bad reads on these standard known locations
    filterMAP (0x0000, 0x0001, SID_BAD_READ); /* Bank regs */
    filterMAP (0x00a5, 0x00ac, SID_BAD_READ); /* Bug in tons of SIDs */
    filterMAP (0x00fb, 0x00ff, SID_BAD_READ); /* Bug in tons of SIDs */
    filterMAP (0x02a6, 0x02a6, SID_BAD_READ); /* PAL/NTSC flag */
    filterMAP (0x02a7, 0x02ff, SID_BAD_READ); /* Bug in tons of sids */
    filterMAP (0x0314, 0x0319, SID_BAD_READ); /* Interrupt vectors */
    filterMAP (0x07e8, 0x07f7, SID_BAD_READ); /* Bug in tons of sids */
}

void SidUsage::read (const char *filename, sid_usage_t &usage)
{
    FILE *file;
    size_t i = strlen (filename);
    const char *ext = NULL;

    m_status = false;
    file = fopen (filename, "rb");
    if (file == NULL)
    {
        m_errorString = txt_file;
        return;
    }

    // Find start of extension
    while (i > 0)
    {
        if (filename[--i] == '.')
        {
            ext = &filename[i + 1];
            break;
        }
    }

    // Initialise optional fields
    usage.flags  = 0;
    usage.md5[0] = '\0';
    usage.length = 0;

    if (readSMM (file, usage, ext)) ;
    else if (readMM (file, usage, ext)) ;
    else m_errorString = txt_invalid;
    fclose (file);
}


void SidUsage::write (const char *filename, const sid_usage_t &usage)
{
    FILE *file;
    size_t i = strlen (filename);
    const char *ext = NULL;

    m_status = false;
    file = fopen (filename, "wb");
    if (file == NULL)
    {
        m_errorString = txt_file;
        return;
    }

    // Find start of extension
    while (i > 0)
    {
        if (filename[--i] == '.')
        {
            ext = &filename[i + 1];
            break;
        }
    }

    if (!strcmp (ext, "mm"))
        writeSMM0 (file, usage);
    else if (!strcmp (ext, "map"))
        writeMAP (file, usage);
    else
        m_errorString = txt_invalid;
    fclose (file);
}


bool SidUsage::readMM (FILE *file, sid_usage_t &usage, const char *ext)
{
    // Need to check extension
    if (strcmp (ext, "mm"))
        return false;

    {   // Read header
        char version;
        unsigned short flags;
        fread (&version, sizeof (version), 1, file);
        if (version != 0)
        {
            m_errorString = txt_supported;
            return true;
        }
        fread (&flags, sizeof (flags), 1, file);
        usage.flags = flags;
    }

    {   // Read load image details
        int length;
        fread (&usage.start, sizeof (usage.start), 1, file);
        fread (&usage.end, sizeof (usage.end), 1, file);
        length = (int) usage.start - (int) usage.end + 1;
        if (length < 0)
        {
            m_errorString = txt_corrupt;
            return true;
        }
        memset (&usage.memory[usage.start], SID_LOAD_IMAGE, sizeof (char) * length);
    }

    {   // Read usage
        int ret = fgetc (file);
        while (ret != EOF)
        {   // Read usage
            if (fread (&usage.memory[ret << 8], sizeof (char) * 0x100, 1, file) != 1)
            {
                m_errorString = txt_reading;
                return true;
            }   
            ret = fgetc (file);
        }
    }
    m_status = true;
    return true;
}


bool SidUsage::readSMM (FILE *file, sid_usage_t &usage, const char *)
{
    uint8_t   chunk[4] = {0};
    IffHeader header;
    uint_least32_t length;

    // Read header chunk
    fread (&chunk, sizeof (chunk), 1, file);
    if (endian_big32 (chunk) != FORM_ID)
        return false;
    length = readChunk (file, header);
    if (!length)
        return false;

    // Determine SMM version
    switch (endian_big32 (header.type))
    {
    case SMM0_ID:
        m_status = readSMM0 (file, usage, header);
        break;

    case SMM1_ID:
    case SMM2_ID:
    case SMM3_ID:
    case SMM4_ID:
    case SMM5_ID:
    case SMM6_ID:
    case SMM7_ID:
    case SMM8_ID:
    case SMM9_ID:
        m_errorString = txt_supported;
        break;
    }
    return true;
}


bool SidUsage::readSMM0 (FILE *file, sid_usage_t &usage, const IffHeader &header)
{
    Smm_v0 smm;
    smm.header = header;

    {   // Read file
        long  pos   = ftell (file);
        bool  error = true;

        for(;;)
        {
            size_t  ret;
            uint_least32_t length = 0;
            uint8_t chunk[4];
            // Read a chunk header
            ret = fread (&chunk, sizeof (chunk), 1, file);
            // If no chunk header assume end of file
            if (ret != 1)
                break;

            // Check for a chunk we are interested in
            switch (endian_big32 (chunk))
            {
            case INF0_ID:
                length = readChunk (file, smm.info);
                break;

            case ERR0_ID:
                length = readChunk (file, smm.error);
                usage.flags = endian_big16(smm.error.flags);
                break;

            case MD5_ID:
                length = readChunk (file, smm.md5);
                memcpy (usage.md5, smm.md5.key, sizeof (smm.md5.key));
                break;

            case TIME_ID:
                length = readChunk (file, smm.time);
                usage.length = endian_big16(smm.time.stamp);
                break;

            case BODY_ID:            
                length = readChunk (file, smm.body);
                break;

            default:
                length = skipChunk (file);
            }

            if (!length)
            {
                error = true;
                break;
            }

            // Move past the chunk
            pos += (long) length + (sizeof(uint8_t) * 8);
            fseek (file, pos, SEEK_SET);
            if (ftell (file) != pos)
            {
                error = true;
                break;
            }
            error = false;
        }

        // Check for file reader error
        if (error)
        {
            m_errorString = txt_reading;
            return false;
        }
    }

    // Test that all required checks were found
    if ((smm.info.length == 0) || (smm.body.length == 0))
    {
        m_errorString = txt_missing;
        return false;
    }

    {   // Extract usage information
        int last = 0;
        for (int i = 0; i < 0x100; i++)
        {
            int addr = smm.body.usage[i].page << 8;
            if (addr < last)
                break;
            // @FIXME@ Handled extended information (upto another 3 optional bytes)
            for (int j = 0; j < 0x100; j++)
                 usage.memory[addr++] = smm.body.usage[i].flags[j] & ~SID_EXTENSION;
            last = addr;
        }
    }

    {   // File in the load range
        int length;
        usage.start = endian_big16 (smm.info.startAddr);
        usage.end   = endian_big16 (smm.info.stopAddr);
        length = (int) usage.end - (int) usage.start + 1;

        if (length < 0)
        {
            m_errorString = txt_corrupt;
            return false;
        }

        uint_least8_t *p = &usage.memory[usage.start];
        {for (int i = 0; i < length; i++)
            p[i] |= SID_LOAD_IMAGE;
        }
    }
    return true;
}


void SidUsage::writeSMM0 (FILE *file, const sid_usage_t &usage)
{
    struct Smm_v0  smm0;
    uint_least32_t headings = 2; /* Mandatory */

    endian_big32 (smm0.header.type, SMM0_ID);

    // Optional
    if (usage.flags == 0)
        smm0.error.length = 0;
    else
    {
        endian_big16 (smm0.error.flags, (uint_least16_t) usage.flags);
        headings++;
    }

    if (usage.length == 0)
        smm0.time.length = 0;
    else
    {
        endian_big16 (smm0.time.stamp, (uint_least16_t) usage.length);
        headings++;
    }

    if ( usage.md5[0] == '\0' )
        smm0.md5.length = 0;
    else
    {
        memcpy (smm0.md5.key, usage.md5, sizeof (smm0.md5.key));
        headings++;
    }
    // End Optional

    endian_big16 (smm0.info.startAddr, usage.start);
    endian_big16 (smm0.info.stopAddr,  usage.end);
    
    {
        uint8_t i = 0;
        smm0.body.length = 0;
        {for (int page = 0; page < 0x100; page++)
        {
            for (int j = 0; j < 0x100; j++)
            {
                if (!(usage.memory[(page << 8) | j] & ~SID_LOAD_IMAGE))
                    continue;
           
                int addr = page << 8;
                for (j = 0; j < 0x100; j++)
                     smm0.body.usage[i].flags[j] = usage.memory[addr++] & ~SID_LOAD_IMAGE;
                smm0.body.length += 0x101;
                smm0.body.usage[i].page = (uint8_t) page;
                i++;
            }
        }}
    }

    uint_least32_t filelength = smm0.header.length + smm0.error.length
                              + smm0.info.length   + smm0.md5.length
                              + smm0.time.length   + smm0.body.length
                              + (sizeof (uint8_t) * 8 * headings);

    if ( writeChunk (file, smm0.header, FORM_ID, filelength) == false )
        goto writeSMM0_error;
    if ( writeChunk (file, smm0.error, ERR0_ID) == false )
        goto writeSMM0_error;
    if ( writeChunk (file, smm0.info, INF0_ID)  == false )
        goto writeSMM0_error;
    if ( writeChunk (file, smm0.md5, MD5_ID)    == false )
        goto writeSMM0_error;
    if ( writeChunk (file, smm0.time, TIME_ID)  == false )
        goto writeSMM0_error;
    if ( writeChunk (file, smm0.body, BODY_ID)  == false )
        goto writeSMM0_error;
    m_status = true;
    return;

writeSMM0_error:
    m_errorString = txt_writing;
}


// Add filtering to the specified memory locations
void SidUsage::filterMAP (int from, int to, uint_least8_t mask)
{
    for (int i = from; i <= to; i++)
        m_filterMAP[i] = ~mask;
}


void SidUsage::writeMAP (FILE *file, const sid_usage_t &usage)
{
    bool err = false;
    // Find out end unused regions which can be removed from
    // load image
    uint_least16_t faddr = usage.start;
    uint_least16_t laddr = usage.end;

    // Trim ends unused off load image
    for (; faddr < laddr; faddr++)
    {
//        if (usage.memory[faddr] & (SID_BAD_READ | SID_BAD_EXECUTE))
        if (usage.memory[faddr] & ~SID_LOAD_IMAGE)
            break;
    }

    for (; laddr > faddr; laddr--)
    {
//        if (usage.memory[laddr] & (SID_BAD_READ | SID_BAD_EXECUTE))
        if (usage.memory[laddr] & ~SID_LOAD_IMAGE)
            break;
    }

    for (int page = 0; page < 0x100; page++)
    {
        bool used = false;
        for (int offset = 0; offset < 0x100; offset++)
            used |= (usage.memory[(page << 8) | offset] != 0);
               
        if (used)
        {
            for (int i = 0; i < 4; i++)
            {
                fprintf (file, "%02X%02X=", page, i << 6);
                for (int j = 0; j < 64; j++)
                {
                    int addr = (page << 8) | (i << 6) | j;
                    uint_least8_t u = usage.memory[addr];
                    // The addresses which don't need to be in the load image have now been
                    // trimmed off.  Anything between faddr and laddr needs to be kept
                    if ((addr >= faddr) && (addr <= laddr))
                        u |= (SID_BAD_READ | SID_BAD_EXECUTE);
                    // Apply usage filter for this memory location
                    u &= m_filterMAP[addr];
                    err |= fprintf (file, "%s", m_decodeMAP[u]) < 0;
                    if ((j & 7) == 7)
                        err |= fprintf (file, " ") < 0;
                }
                err |= fprintf (file, "\n") < 0;
            }
        }
    }

    if (err)
        m_errorString = txt_writing;
    else
        m_status = true;
}


uint_least32_t SidUsage::readChunk (FILE *file, Chunk &chunk)
{
    uint8_t tmp[4];
    size_t  ret;
    uint_least32_t l;

    ret = fread (tmp, sizeof(tmp), 1, file);
    if ( ret != 1 )
        return 0;

    l = endian_big32 (tmp);
    if (l < chunk.length)
        chunk.length = l;
    ret = fread ((char *) (&chunk+1), chunk.length, 1, file);
    if ( ret != 1 )
        return 0;
    return l;
}


bool SidUsage::writeChunk (FILE *file, const Chunk &chunk, uint_least32_t type,
                           uint_least32_t length)
{
    uint8_t tmp[4];
    size_t  ret;

    if (chunk.length)
    {
        endian_big32 (tmp, type);
        ret = fwrite (tmp, sizeof(tmp), 1, file);
        if ( ret != 1 )
            return false;
        if (length == 0)
            length = chunk.length;
        endian_big32 (tmp, length);
        ret = fwrite (tmp, sizeof(tmp), 1, file);
        if ( ret != 1 )
            return false;
        ret = fwrite ((const char *) (&chunk+1), chunk.length, 1, file);
        if ( ret != 1 )
            return false;
    }
    return true;
}


uint_least32_t SidUsage::skipChunk (FILE *file)
{
    uint8_t tmp[4];
    uint_least32_t ret;
    ret = fread (tmp, sizeof(tmp), 1, file);
    if ( ret != 1 )
        return 0;
    return endian_big32 (tmp);
}
