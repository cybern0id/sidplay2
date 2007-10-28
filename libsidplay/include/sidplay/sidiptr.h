/***************************************************************************
                          sidiptr.h  -  Interface ptr implementation
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

#ifndef _SIDIPTR_H_
#define _SIDIPTR_H_

#include <assert.h>

#include <sidconfig.h>
#include <sidiunknown.h>

template<class TImplementation> class SidLazyIPtr;

// An interface ptr class to automatically
// handle querying and releasing of the interface.
// NEVER pass one of these across a binary boundary,
// this class is free to change!!
template<class T>
class SidIPtr: public SidIUnknown
{
private:
    typedef SIDPLAY2_NAMESPACE::Iid SidIid;

    const SidIid &m_iid;
    T            *m_if;
    SidIUnknown  *m_unknown;

public:
    SidIfPtr  (const SidIfPtr<T> &unknown) // std::list objects are const
        : m_iid(T::iid ()) { _init (&unknown); }
    SidIfPtr  (const SidIUnknown &unknown)
        : m_iid(T::iid ()) { _init (&unknown); }
    SidIfPtr  (const SidIUnknown *unknown)
        : m_iid(T::iid ()) { _init (unknown); }
    ~SidIfPtr () { _release (); }

public:
    const SidIid &iid  () const { return m_iid; }
    const char   *name () const { return m_unknown->name (); }

    T *operator -> () const { assert (m_unknown); return m_if; }
    operator bool  () const { return m_unknown != 0; }

    bool operator == (const SidIUnknown *unknown) const
    {
        return (m_unknown == unknown);
    }

    bool operator == (const SidIPtr<T>& other) const
    {
        return m_unknown == other.m_unknown;
    }

    // Some interfaces are hidden within another interface.  This
    // calls allows you to access those hidden ones (i.e. consider
    // a private interface aggregated and passed around with a
    // public one)
    template<class Timplementation>
    SidLazyIPtr<Timplementation> hidden ()
    {
        SidLazyIPtr<Timplementation> hidden = m_if;
        return hidden;
    }

private:
    SidIUnknown *iaggregate () { return m_unknown; }

    // afUnknown
    bool iquery (const afId &afid, void **implementation)
    {
        if (afid == m_afid)
        {
            if (m_unknown)
            {
                *implementation = m_if;
                return true;
            }
            return false;
        }
        if (m_unknown)
            return m_unknown->iquery (afid, implementation);
        return false;
    }

    void iadd      () { assert (0); }
    void irelease  () { assert (0); }

    void _assign (SidIUnknown *unknown)
    {
        if (!unknown)
        {
            _release ();
            return;
        }

        SidIUnknown *u = unknown;
        for (;;)
        {
            m_if = 0;
            if (u->iquery (m_afid, reinterpret_cast<void**>(&m_if)))
                break;
            else if (!m_if)
                return;
            u = m_if->aggregate (); // chain
        }
        m_unknown = unknown->aggregate ();
        m_unknown->iadd ();
    }

    void _init (const SidIUnknown *unknown)
    {
        m_unknown = 0;
        _assign (const_cast<SidIUnknown*>(unknown));
        assert  (m_unknown);
    }

    void _release ()
    {
        if (m_unknown)
        {
            m_unknown->irelease ();
            m_unknown = 0;
            m_if = 0;
        }
    }

protected: // For lazy initialisation
    SidIPtr () : m_iid(T::afid ()), m_unknown(0) { ; }

    const SidIUnknown *operator = (const SidIUnknown *unknown)
    {
        _assign (const_cast<afUnknown*>(unknown));
        return unknown;
    }

    const SidIUnknown &operator = (const SidIUnknown &unknown)
    {
        _assign (const_cast<afUnknown*>(&unknown));
        return unknown;
    }

    const SidIUnknown &operator = (const SidIPtr<T> &unknown)
    {
        _assign (const_cast<SidIUnknown*>(&unknown));
        return unknown;
    }
};

template <class T>
bool operator == (const SidIPtr<T>& first, const SidIPtr<T>& second)
{
    return first.operator==(second);
}

#endif // _SIDIPTR_H_
