/***************************************************************************
                          sidcounknown.h  -  Unknown companion class
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

#ifndef _sidcounknown_h_
#define _sidcounknown_h_

#include <assert.h>
#include <string.h>
#include <sidplay/sidconfig.h>
#include <sidplay/sidunknown.h>

inline bool operator != (const SIDPLAY2_NAMESPACE::Iid &iid1, const SIDPLAY2_NAMESPACE::Iid &iid2)
{
    return !(iid1 == iid2);
}

inline bool operator < (const SIDPLAY2_NAMESPACE::Iid &iid1, const SIDPLAY2_NAMESPACE::Iid &iid2)
{
    return memcmp (&iid1, &iid2, sizeof (SIDPLAY2_NAMESPACE::Iid)) < 0;
}

SIDPLAY2_NAMESPACE_START

template <class TInterface>
class CoUnknown: public TInterface
{
private:
    const char * const m_name;
    unsigned int       m_refcount;
 
protected:
    virtual void _idestroy  () { ; }
    virtual void _iadd      (unsigned int /*count*/) { ; }
    virtual bool _iquery    (const Iid &iid, void **implementation) = 0;
    virtual void _irelease  (unsigned int /*count*/) { ; }
    unsigned int _irefcount () const { return m_refcount; }

public:
    CoUnknown (const char *name) : m_name(name), m_refcount(0) { ; }
    virtual ~CoUnknown () { _idestroy (); assert (!m_refcount); }

    ISidUnknown *iunknown     () { return this; }
    virtual const char *iname () const { return m_name; }

private:
    const Iid &_iid () const { return TInterface::iid(); }

    void _iadd     () { _iadd (++m_refcount); }
    void _irelease ()
    {
        _irelease (--m_refcount);
        if (!m_refcount)
            _idestroy ();
    }

    bool iquery (const Iid &iid, void **implementation)
    {
        return _iquery (iid, implementation);
    }
    CoUnknown (const CoUnknown &);
    CoUnknown &operator= (const CoUnknown &);
};

SIDPLAY2_NAMESPACE_STOP

#endif // _sidcounknown_h_
