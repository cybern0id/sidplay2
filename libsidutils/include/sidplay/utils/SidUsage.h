/***************************************************************************
                          SidUsage.h  -  sidusage file support
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

#ifndef _SidUsage_h_
#define _SidUsage_h_

#include <sidplay/sidusage.h>

struct SidTuneInfo;
struct IffHeader;
struct Chunk;


class SID_EXTERN SidUsage
{
private:
    char m_decodeMAP[0x100][3];
    // Ignore errors
    uint_least8_t m_filterMAP[0x10000];

protected:
    bool  m_status;
    const char *m_errorString;

private:
    // Old obsolete MM file format
    bool           readMM     (FILE *file, sid_usage_t &usage, const char *ext);
    // Sid Memory Map (MM file)
    bool           readSMM    (FILE *file, sid_usage_t &usage, const char *ext);
    bool           readSMM0   (FILE *file, sid_usage_t &usage,
                               const IffHeader &header);
    void           writeSMM0  (FILE *file, const sid_usage_t &usage);
    void           writeMAP   (FILE *file, const sid_usage_t &usage);
    void           filterMAP  (int from, int to, uint_least8_t mask);

protected:
    uint_least32_t readChunk  (FILE *file, Chunk &chunk);
    bool           writeChunk (FILE *file, const Chunk &chunk,
                               uint_least32_t type,
                               uint_least32_t length = 0);
    uint_least32_t skipChunk  (FILE *file);

public:
    SidUsage ();

    // @FIXME@ add ext to these
    void           read       (const char *filename, sid_usage_t &usage);
    void           write      (const char *filename, const sid_usage_t &usage);
    const char *   error      (void) { return m_errorString; }

    operator bool () { return m_status; }
};

#endif // _SidUsage_h_
