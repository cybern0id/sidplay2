/***************************************************************************
                          SidFilter.cpp  -  read filter
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

#include <new>
#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "SidFilter.h"


SidFilter::SidFilter ()
:status(false)
{
    errorString = "SID Filter: No filter loaded";
}

SidFilter::~SidFilter ()
{
    clear ();
}

void SidFilter::clear ()
{
    m_filter.points = 0;
    status      = false;
    errorString = "SID Filter: No filter loaded";
}

void SidFilter::read (char *filename)
{
    ini_fd_t ini = ini_open (filename, "r");

    // Illegal ini fd
    if (!ini)
    {
        status      = false;
        errorString = "SID Filter: Unable to open filter file";
        return;
    }

    read (ini, "Filter");
    ini_close (ini);
}

void SidFilter::read (ini_fd_t ini, char *heading)
{
    int type = 1;

    clear ();
    status = true;

    if (ini_locateHeading (ini, heading) < 0)
    {
        status = false;
        errorString = "SID Filter: Unable to locate filter section in input file";
        return;
    }

    (void) ini_locateKey (ini, "type");
    (void) ini_readInt   (ini, &type);
    switch (type)
    {
    case 1:
        readType1 (ini);
    break;

    case 2:
        readType2 (ini);
    break;

    default:
        status = false;
        errorString = "SID Filter: Invalid filter type";
    break;
    }
}

void SidFilter::readType1 (ini_fd_t ini)
{
    int points;

    // Does Section exist in ini file
    if (ini_locateKey (ini, "points") < 0)
        goto SidFilter_readType1_errorDefinition;
    if (ini_readInt (ini, &points) < 0)
        goto SidFilter_readType1_errorDefinition;

    // Make sure there are enough filter points
    if ((points < 2) || (points > 0x800))
        goto SidFilter_readType1_errorDefinition;
    m_filter.points = (uint_least16_t) points;

    // Set the ini reader up for array access
    if (ini_listDelims (ini, ",") < 0)
        goto SidFilter_readType1_errorMemory;

    {
        char key[12];
        int  reg = -1, fc = -1;
        for (int i = 0; i < m_filter.points; i++)
        {   // First lets read the SID cutoff register value
            sprintf (key, "point%d", i + 1);
            ini_locateKey (ini, key);
            if (ini_readInt (ini, &reg) < 0)
                goto SidFilter_readType1_errorDefinition;
            if (ini_readInt (ini, &fc)  < 0)
                goto SidFilter_readType1_errorDefinition;

            // Got valid reg/fc
            m_filter.cutoff[i][0] = (uint) reg;
            m_filter.cutoff[i][1] = (uint) fc;
        }
    }
return;

SidFilter_readType1_errorDefinition:
    clear ();
    errorString = "SID Filter: Invalid Type 1 filter definition";
    status = false;
return;

SidFilter_readType1_errorMemory:
    errorString = "SID Filter: Out of memory";
    status = false;
}

void SidFilter::readType2 (ini_fd_t ini)
{
    double fs, fm, ft;
    
    // Read filter parameters
    ini_locateKey (ini, "fs");
    if (ini_readDouble (ini, &fs) < 0)
        goto SidFilter_readType2_errorDefinition;
    ini_locateKey (ini, "fm");
    if (ini_readDouble (ini, &fm) < 0)
        goto SidFilter_readType2_errorDefinition;
    ini_locateKey (ini, "ft");
    if (ini_readDouble (ini, &ft) < 0)
        goto SidFilter_readType2_errorDefinition;

    // Calculate the filter
    calcType2 (fs, fm, ft);
return;

SidFilter_readType2_errorDefinition:
    clear ();
    errorString = "SID Filter: Invalid Type 2 filter definition";
    status = false;
}

// Calculate a Type 2 filter (Sidplay 1 Compatible)
void SidFilter::calcType2 (double fs, double fm, double ft)
{
    double fcMax = 1.0;
    double fcMin = 0.01;
    double fc;

    // Definition from reSID
    m_filter.points = 0x100;

    // Create filter
    for (uint i = 0; i < 0x100; i++)
    {
        uint rk = i << 3;
        m_filter.cutoff[i][0] = rk;
        fc = exp ((double) rk / 0x800 * log (fs)) / fm + ft;
        if (fc < fcMin)
            fc = fcMin;
        if (fc > fcMax)
            fc = fcMax;
        m_filter.cutoff[i][1] = (uint) (fc * 4100);
    }
}

// Get filter
const sid_filter_t *SidFilter::provide () const
{
    if (!status)
        return NULL;
    return &m_filter;
}

// Copy filter from another SidFilter class
const SidFilter &SidFilter::operator= (const SidFilter &filter)
{
    const sid_filter_t *f = filter.provide ();
    if (f)
        m_filter = *f;
    return filter;
}

// Copy sidplay2 sid_filter_t object
const sid_filter_t &SidFilter::operator= (const sid_filter_t &filter)
{
    m_filter = filter;
    return filter;
}
