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

#include <sidplay/component.h>

class Component : virtual public IComponent
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
