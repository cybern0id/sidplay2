/***************************************************************************
               hardsid.h  -  Hardsid support interface.
	                     Created from Jarno's original
		             Sidplay2 patch
                             -------------------
    begin                : Fri Dec 15 2000
    copyright            : (C) 2000-2002 by Simon White
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
 *
 ***************************************************************************/

#ifndef  _hardsid_h_
#define  _hardsid_h_

#include <vector>
#include "sidplay/sidbuilder.h"
#include "config.h"


class HardSIDBuilder: public sidbuilder
{
private:
    static uint m_instance;
    char   m_errorBuffer[100];
    std::vector<sidemu *> sidobjs;
#ifdef HAVE_MSWINDOWS
    init ();
#endif

public:
    HardSIDBuilder  (const char * const name);
    ~HardSIDBuilder (void);
    // true will give you the number of used devices.
    //    return values: 0 none, positive is used sids
    // false will give you all available sids.
    //    return values: 0 endless, positive is available sids.
    // use bool operator to determine error
    uint        devices (bool used);
    uint        create  (uint sids);
    sidemu     *lock    (c64env *env, sid2_model_t model);
    void        unlock  (sidemu *device);
    void        remove  (void);
    const char *error   (void) const { return m_errorBuffer; }
    const char *credits (void);
    void        flush   (void);
    void        filter  (bool enable);
};

#endif // _hardsid_h_
