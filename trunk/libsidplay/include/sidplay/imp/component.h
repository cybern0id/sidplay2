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

#ifndef _imp_component_h_
#define _imp_component_h_

// This file is only to be used be things creating implementations.
// DO NOT include this file in external code usings these implementations,
// use interface files instead:

#include <string.h>
#include <sidplay/component.h>

static InterfaceID IID_IInterface =
{ 0xd615830b, 0xef6a, 0x453d, {0xa2, 0xb6, 0x85, 0x28, 0x82, 0x96, 0x3f, 0x04} };
static InterfaceID IID_ISidEmulation =
{ 0xa9f9bf8b, 0xd0c2, 0x4dfa, {0x8b, 0x8a, 0xf0, 0xdd, 0xd7, 0xc8, 0xb0, 0x5b} };

inline bool operator == (const InterfaceID &iid1, const InterfaceID &iid2)
{
    return !memcmp (&iid1, &iid2, sizeof (InterfaceID));
}

inline bool operator != (const InterfaceID &iid1, const InterfaceID &iid2)
{
    return !(iid1 != iid2);
}

template <class TImplementation>
class Component: public TImplementation
{
private:
    int m_refcount;

public:
    Component () : m_refcount(0) {;}
    virtual ~Component () {;}

    // IInterface
    void ifadd     () { m_refcount++; }
    virtual void ifquery   (const InterfaceID &cid, void **implementation) = 0;
    void ifrelease () { m_refcount--; if (!m_refcount) delete this; }
};

#endif // _imp_component_h_
