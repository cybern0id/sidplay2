/***************************************************************************
                          ifbase.h  -  Interface definition class.
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

#ifndef _ifbase_h_
#define _ifbase_h_

#include "sidtypes.h"

struct InterfaceID
{
    uint_least32_t d1;
    uint_least16_t d2;
    uint_least16_t d3;
    uint_least8_t  d4[8];
};

template<uint_least32_t d1,  uint_least16_t d2,  uint_least16_t d3,
         uint_least8_t d4_1, uint_least8_t d4_2, uint_least8_t d4_3,
         uint_least8_t d4_4, uint_least8_t d4_5, uint_least8_t d4_6,
         uint_least8_t d4_7, uint_least8_t d4_8>
static const InterfaceID &IID ()
{
    static const InterfaceID iid =
        {d1, d2, d3, {d4_1, d4_2, d4_3, d4_4, d4_5, d4_6, d4_7, d4_8} };
    return iid;
};

class IInterface
{
protected:
    virtual ~IInterface () = 0;

public:
    static const InterfaceID &iid () {
        return IID<0xd615830b, 0xef6a, 0x453d, 0xa2, 0xb6, 0x85, 0x28, 0x82, 0x96, 0x3f, 0x04>();
    }

    // IInterface
    virtual void ifadd     () = 0;
    virtual bool ifquery   (const InterfaceID &iid, void **implementation) = 0;
    virtual void ifrelease () = 0;

    virtual const char *name () const = 0;
};

inline IInterface::~IInterface () { ; }

// Query an interface (old method)
#define IF_QUERY(Interface,Implementation) Interface::iid(), \
    reinterpret_cast<void**>(static_cast<Interface**>(Implementation))

// Query an interface (new method)
// The one and only real function.  Do not modify or add new
// non virtual functions.  You will break binary compatibility
template<class T, class U> inline T *if_cast (U unknown)
{
    T *implementation = 0;
    unknown->ifquery (IF_QUERY(T, &implementation));
    return implementation;
}

#endif // _ifbase_h_
