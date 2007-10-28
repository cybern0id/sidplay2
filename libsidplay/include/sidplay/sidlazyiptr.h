/***************************************************************************
                          sidlazyiptr.h  -  Interface ptr implementation
                             -------------------
    begin                : Sat Oct 27 2007
    copyright            : (C) 2007 by Simon White
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

#ifndef _SIDLAZYIPTR_H_
#define _SIDLAZYIPTR_H_

#include "sidiptr.h"

class SidLazyIPtrAssign
{
protected:
    virtual ~SidLazyIPtrAssign () { ; }
public:
    virtual const SidIUnknown *operator = (const SidIUnknown *unknown) = 0;
    virtual const SidIUnknown &operator = (const SidIUnknown &unknown) = 0;
    virtual operator bool () const = 0;

private:
    SidLazyIPtrAssign &operator = (const SidLazyIPtrAssign &);
};

template<class T>
class SidLazyIPtr: public SidIPtr<T>, public SidLazyIPtrAssign
{
public:
    SidLazyIPtr () { ; }
    SidLazyIPtr (const SidIUnknown &unknown) { *this = unknown; }
    SidLazyIPtr (const SidIUnknown *unknown) { *this = unknown; }
    SidLazyIPtr (const SidLazyIPtr<T> &unknown) { *this = unknown; }

    const SidIUnknown *operator = (const SidIUnknown *unknown)
        { SidIPtr<T>::operator = (unknown); _notify (); return unknown; }
    const SidIUnknown &operator = (const SidIUnknown &unknown)
        { SidIPtr<T>::operator = (unknown); _notify (); return unknown; }
    const SidLazyIPtr<T> &operator = (const SidLazyIPtr<T> &unknown)
        { SidIPtr<T>::operator = (unknown); _notify (); return unknown; }

    operator bool () const
        { return SidIPtr<T>::operator bool (); }
};

#endif // _SIDLAZYIPTR_H_
