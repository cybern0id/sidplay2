/***************************************************************************
                          SidFilter.cpp  -  filter type decoding support
                             -------------------
    begin                : Sun Mar 11 2001
    copyright            : (C) 2001 by Simon White
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

#include <sidplay/sidtypes.h>
#include "libini.h"

// For compatibilty with libsidplay2-0.7.
#ifndef sid_filter_t
typedef int sid_fc_t[2];
typedef struct
{
    sid_fc_t       cutoff[0x800];
    uint_least16_t points;
} sid_filter_t;
#define sid_filter_t sid_filter_t
#endif

class SID_EXTERN SidFilter
{
protected:
    bool  status;
    char *errorString;
    sid_filter_t filter;

protected:
    void readType1 (ini_fd_t ini);
    void readType2 (ini_fd_t ini);
    void clear ();

public:
    SidFilter ();
    ~SidFilter ();

    void  read (char *filename);
    void  read (ini_fd_t ini, char *heading);
	void  calcType2 (double fs, double fm, double ft);
    const sid_filter_t* definition ()
    {
        if (!status)
            return NULL;
        return &filter;
    }

    operator bool () { return status; }

    const char* error (void)
    {   return errorString; }
};
