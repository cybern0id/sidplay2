/***************************************************************************
                          imp_component.h  -  Component Implementation
                             -------------------
    begin                : Sat Jun 6 2006
    copyright            : (C) 2006 by Simon White
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

#ifndef _imp_ifbase_h_
#define _imp_ifbase_h_

// This file is only to be used be things creating implementations.
// DO NOT include this file in external code usings these implementations,
// use interface files instead:

#include <string.h>
#include <sidplay/iinterface.h>

inline bool operator == (const InterfaceID &iid1, const InterfaceID &iid2)
{
    return !memcmp (&iid1, &iid2, sizeof (InterfaceID));
}

inline bool operator != (const InterfaceID &iid1, const InterfaceID &iid2)
{
    return !(iid1 != iid2);
}

template <class TImplementation>
class ICoInterface: public TImplementation
{
private:
    const char * const m_name;
    unsigned int m_refcount;

protected:
    virtual void _ifadd      () { ; }
    unsigned int _ifrefcount () const { return m_refcount; }
    virtual void _ifrelease  () { ; }

public:
    ICoInterface (const char *name) : m_name(name), m_refcount(0) { ; }
    virtual ~ICoInterface () { ; }

    void  ifadd     () { m_refcount++; _ifadd (); }
    void  ifrelease () { m_refcount--; _ifrelease (); }

    const char *name () const { return m_name; }
};

#endif // _imp_ifbase_h_
