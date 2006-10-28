/***************************************************************************
                          ifptr.h  -  Interface helper class.
                             -------------------
    begin                : Fri Oct 27 2006
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

#ifndef _ifptr_h_
#define _ifptr_h_

#include "iinterface.h"

// An interface ptr class to automatically
// handle querying and releasing of the interface.
template<class T>
class IfPtr
{
private:
    T * m_interface;

public:
    IfPtr ()
    :m_interface(0)
    {
        ;
    }

    IfPtr (IInterface &unknown)
    :m_interface(if_cast<T>(&unknown))
    {
        ;
    }

    IfPtr (IInterface *unknown)
    :m_interface(if_cast<T>(unknown))
    {
        ;
    }

    ~IfPtr ()
    {
        if (m_interface)
            m_interface->ifrelease ();
    }

    T *operator -> () { assert (m_interface); return m_interface; }
    operator T *&  () { return m_interface; }
    operator T **  () { return &m_interface; }
    operator bool  () { return m_interface != 0; }
    T &operator *  () { assert (m_interface); return *m_interface; }

    T  *operator = (T *_interface) { return (m_interface = _interface); }
};

#endif // _ifptr_h_
