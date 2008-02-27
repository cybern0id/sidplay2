/***************************************************************************
                          sidunknown.h  -  Unknown interface
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

#ifndef _sidunknown_h_
#define _sidunknown_h_

#include <sidplay/sidconfig.h>
#include <sidplay/sidint.h>

template <class T> class SidIPtr;

SIDPLAY2_NAMESPACE_START

template <class TInterface> class CoAggregate;

struct Iid
{
    uint_least32_t d1;
    uint_least16_t d2;
    uint_least16_t d3;
    uint_least16_t d4;
    uint_least16_t d5;
    uint_least32_t d6;
};

SIDPLAY2_NAMESPACE_STOP

template<uint_least32_t d1, uint_least16_t d2, uint_least16_t d3,
         uint_least16_t d4, uint_least16_t d5, uint_least32_t d6>
inline const SIDPLAY2_NAMESPACE::Iid &SIDIID ()
{
    static const SIDPLAY2_NAMESPACE::Iid iid = {d1, d2, d3, d4, d5, d6};
    return iid;
};


class ISidUnknown
{
protected:
    typedef SIDPLAY2_NAMESPACE::Iid Iid;

public:
    static const Iid &iid () {
        return SIDIID<0xa595fcc4, 0xa138, 0x449a, 0x9711, 0x4ea5, 0xbb301d2a>();
    }

#if defined(_MSC_VER) && (_MSC_VER >= 1300)
private:
    template<class T> friend class SidIPtr;
    template<class TInterface> friend class SIDPLAY2_NAMESPACE::CoAggregate;

#endif

    virtual void       _iadd     () = 0;
    virtual const Iid &_iid      () const = 0;
    virtual void       _irelease () = 0;

protected:
    virtual ~ISidUnknown () = 0;
    virtual bool _iquery (const Iid &iid, void **implementation) = 0;

public:
    virtual ISidUnknown *iaggregate ()  = 0;
    virtual const char  *iname () const = 0;
};

inline ISidUnknown::~ISidUnknown () { ; }

inline bool operator == (const SIDPLAY2_NAMESPACE::Iid &iid1, const SIDPLAY2_NAMESPACE::Iid &iid2)
{
    return (iid1.d1 == iid2.d1) & (iid1.d2 == iid2.d2) & (iid1.d3 == iid2.d3) &
           (iid1.d4 == iid2.d4) & (iid1.d5 == iid2.d5) & (iid1.d6 == iid2.d6);
}

#endif // _sidunknown_h_
