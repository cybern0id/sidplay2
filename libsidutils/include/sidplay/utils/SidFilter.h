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

typedef struct
{
    sid_fc_t       fc[0x800];
    uint_least16_t points;
} sid_filter_t;

class SID_EXTERN SidFilter
{
protected:
    bool  status;
    char *errorString;
    sid_filter_t filter;

protected:
    void readFilterType1 (ini_fd_t ini);
    void readFilterType2 (ini_fd_t ini);
    void clear ();

public:
    SidFilter ();
    ~SidFilter ();

    void  read (char *filename);
    void  read (ini_fd_t ini, char *heading);
    const sid_filter_t* definition ()
    {
        if (!status)
            return NULL;
        return &filter;
    }

    operator bool () { return status; }

    const char* getErrorString (void)
    {   return errorString; }
};
