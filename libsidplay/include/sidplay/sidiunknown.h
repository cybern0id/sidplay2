/***************************************************************************
                          sidiunknown.h  -  Unknown interface
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

#ifndef _SIDIUNKNOWN_H_
#define _SIDIUNKNOWN_H_

#include "sidconfig.h"
#include "sidint.h"

template <class T> class SidIPtr;

SIDPLAY2_NAMESPACE_START

struct Iid
{
    uint_least32_t d1;
    uint_least16_t d2;
    uint_least16_t d3;
    uint_least16_t d4;
    uint_least32_t d5;
};

SIDPLAY2_NAMESPACE_END

template<uint_least32_t d1, uint_least16_t d2, uint_least16_t d3,
         uint_least16_t d4, uint_least32_t d5>
inline const SIDPLAY2_NAMESPACE::Iid &SIDIID ()
{
    static const SIDPLAY2_NAMESPACE::Iid iid = {d1, d2, d3, d4, d5};
    return iid;
};


class SidIUnknown
{
#if defined(_MSC_VER) && (_MSC_VER < 1300)
public:
#else
    template<class T> friend class SidIPtr;

protected:
#endif
    typedef SidIid SIDPLAY2_NAMESPACE::Iid;

    virtual ~SidIUnknown () = 0;

    static const SidIid &iid () {
        return SIDIID<0xa595fcc4, 0xa138, 0x449a, 0x9711, 0x4ea5, 0xbb301d2a>();
    }

    virtual void iadd      () = 0;
    virtual bool iquery    (const SidIid &iid, void **implementation) = 0;
    virtual void irelease  () = 0;

public:
    virtual SidIUnknown  &iaggregate ()  = 0;
    virtual const SidIid &iid   () const = 0;
    virtual const char   *iname () const = 0;
};

inline SidIUnknown::~SidIUnknown () { ; }

#endif // _SIDIUNKNOWN_H_
