/***************************************************************************
                          c64sid.h  -  ReSid Wrapper for redefining the
                                       filter
                             -------------------
    begin                : Fri Apr 4 2001
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

#include "c64sid.h"

char c64sid::credit[];

c64sid::c64sid (c64env *env)
:m_env(*env),
 m_gain(100)
{
    char *p = credit;
    sprintf (p, "ReSID V%s Engine:", resid_version_string);
    p += strlen (p) + 1;
    strcpy  (p, "\tCopyright (C) 1999 Dag Lem <resid@nimrod.no>");
    p += strlen (p) + 1;
    *p = '\0';
    reset ();
}

bool c64sid::filter (const sid_fc_t * const cutoffs, const uint_least16_t points)
{
    fc_point fc[0x800];
 
    // Make sure there are enough filter points and they are legal
    if ((points < 2) || (points > 0x800))
        return false;

    {
        const sid_fc_t *val, *valp, vals = {-1, 0};
        // Last check, make sure they are list in numerical order
        // for both axis
        val = &vals; // (start)
        for (int i = 0; i < points; i++)
        {
            valp = val;
            val  = &cutoffs[i];
            if ((*valp)[0] >  (*val)[0])
                return 0;
            fc[i][0] = (sound_sample) (*val)[0];
            fc[i][1] = (sound_sample) (*val)[1];
        }
    }

    {   // function from reSID
        uint_least16_t p = points - 1;
        interpolate (fc, fc, fc + p, fc + p, m_sid.fc_plotter (), 1.0);
    }

    return true;
}
