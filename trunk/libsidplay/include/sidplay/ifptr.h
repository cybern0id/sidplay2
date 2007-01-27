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

#include <assert.h>
#include "iinterface.h"

// An interface ptr class to automatically
// handle querying and releasing of the interface.
// NEVER pass one of these across a binary boundary!!
template<class T>
class IfPtr: public IInterface
{
private:
    const InterfaceID &m_ifid;
    T                 *m_if;
    IInterface        *m_unknown;

public:
    IfPtr  (const IfPtr<T> &unknown) // std::list objects are const
        :m_ifid(T::iid ()) { _init (const_cast<IfPtr<T>*>(&unknown)); }
    IfPtr  (IInterface &unknown)
        :m_ifid(T::iid ()) { _init (&unknown); }
    IfPtr  (IInterface *unknown)
        :m_ifid(T::iid ()) { _init (unknown); }
    ~IfPtr () { _release (); }

public:
    const InterfaceID &ifid () const { return m_ifid; }
    const char *name () const { return m_unknown->name (); }

    T *operator ->       () const { assert (m_unknown); return m_if; }
    // This operator is for compatibility only to make it easy to call operators.  Do not
    // pass the pointer around!
    const T &operator *  () const { assert (m_unknown); return *m_if; }
    //afUnknown* operator &() const { return m_unknown; }
    //operator afUnknown  &() const { assert (m_unknown); return *m_unknown; }
    operator bool        () const { return m_unknown != 0; }

    bool operator ==     (const IInterface *unknown) const
    {
        return (m_unknown == unknown);
    }

private:
    IInterface *aggregate () { return m_unknown; }

    // afUnknown
    bool ifquery (const InterfaceID &iid, void **implementation)
    {
        if (iid == m_ifid)
        {
            if (m_unknown)
            {
                *implementation = m_if;
                return true;
            }
            return false;
        }
        return m_unknown->ifquery (iid, implementation);
    }

    unsigned int ifrelease () { assert (0); return 0; }
    unsigned int ifadd     () { assert (0); return 0; }

    void _assign (IInterface *unknown)
    {   // @FIXME@ This is should fail if not found.  Use Lazy instead
        // if expecting an interface to not be present
        if (!unknown)
            return;

        IInterface *u = unknown;
        for (;;)
        {
            m_if = 0;
            if (u->ifquery (m_ifid, reinterpret_cast<void**>(&m_if)))
                break;
            else if (!m_if)
                return;
            u = m_if; // chain
        }
        m_unknown = unknown->aggregate ();
        unsigned int count = m_unknown->ifadd ();
    }

    void _init (IInterface *unknown)
    {
        m_unknown = 0;
        _assign (unknown);
        assert  (m_unknown);
    }

    void _release ()
    {
        if (m_unknown)
        {
            unsigned int count = m_unknown->ifrelease ();
            m_unknown = 0;
            m_if = 0;
        }
    }

protected: // For lazy initialisation
    IfPtr () : m_ifid(T::iid ()), m_unknown(0) { ; }

    const IInterface *operator = (const IInterface *unknown)
    {
        _release ();
        _assign  (const_cast<IInterface*>(unknown));
        return unknown;
    }

    const IInterface &operator = (const IInterface &unknown)
    {
        *this = &unknown;
        return unknown;
    }

    const IfPtr<T>& operator= (const IfPtr<T> &unknown)
    {
         *this = &unknown;
         return unknown;
    }
};

template<class T>
class IfLazyPtr: public IfPtr<T>
{
public:
    IfLazyPtr () { ; }
    IfLazyPtr (IInterface &unknown) { *this = unknown; }
    IfLazyPtr (IInterface *unknown) { *this = unknown; }
    IfLazyPtr (const IfLazyPtr<T> &unknown) { *this = unknown; }

    using IfPtr<T>::operator =;

private:
    IfLazyPtr (const IfPtr<T> &) { assert (0); } // Do not use
};

#endif // _ifptr_h_
