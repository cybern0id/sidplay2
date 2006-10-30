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
public:
    T *p;

    IfPtr ()
    :p(0)
    {
        ;
    }

    IfPtr (IInterface &unknown)
    :p(if_cast<T>(&unknown))
    {
        ;
    }

    IfPtr (IInterface *unknown)
    :p(if_cast<T>(unknown))
    {
        ;
    }

    ~IfPtr ()
    {
        if (p)
            p->ifrelease ();
    }

    T *operator -> () { assert (p); return p; }
    operator bool  () { return p != 0; }
    T &operator *  () { assert (p); return *p; }

    T   *operator =  (T *_interface) { return (p =  _interface); }
    bool operator == (T *_interface) { return (p == _interface); }
};

#endif // _ifptr_h_
