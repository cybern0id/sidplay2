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

#ifndef _SIDCOUNKNOWN_H_
#define _SIDCOUNKNOWN_H_

#include <assert.h>
#include "sidconfig.h"
#include "sidiunknown.h"

SIDPLAY2_NAMESPACE_START

inline bool operator == (const Iid &iid1, const Iid &iid2)
{
    return !memcmp (&iid1, &iid2, sizeof (Iid));
}

inline bool operator != (const Iid &iid1, const Iid &iid2)
{
    return !(iid1 == iid2);
}

inline bool operator < (const Iid &iid1, const Iid &iid2)
{
    return memcmp (&iid1, &iid2, sizeof (Iid)) < 0;
}

template <class TImplementation>
class CoUnknown: public TImplementation
{
private:
    const char * const m_name;
    unsigned int       m_refcount;
 
protected:
    virtual void _idestroy  () { ; }
    virtual void _iadd      (unsigned int /*count*/) { ; }
    virtual void _irelease  (unsigned int /*count*/) { ; }
    unsigned int _irefcount () const { return m_refcount; }

public:
    CoUnknown (const char *name) : m_name(name), m_refcount(0) { ; }
    virtual ~CoUnknown () { _idestroy (); assert (!m_refcount); }

    virtual const Iid &iid () const { return TImplementation::iid(); }

    SidIUnknown *iaggregate   () { return this; }
    virtual const char *iname () const { return m_name; }

private:
    void iadd     () { _iadd (++m_refcount); }
    void irelease ()
    {
        _irelease (--m_refcount);
        if (!m_refcount)
            _idestroy ();
    }
};

SIDPLAY2_NAMESPACE_STOP

#endif // _SIDCOUNKNOWN_H_
