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

#ifndef _sidiptr_h_
#define _sidiptr_h_

#include <assert.h>

#include <sidplay/sidconfig.h>
#include <sidplay/sidunknown.h>

template<class TInterface> class SidLazyIPtr;

// An interface ptr class to automatically
// handle querying and releasing of the interface.
// NEVER pass one of these across a binary boundary,
// this class is free to change!!
template<class TInterface>
class SidIPtr: public ISidUnknown
{
private:
    const Iid   &m_iid;
    TInterface  *m_if;
    ISidUnknown *m_unknown;

public:
    SidIPtr (const SidIPtr<TInterface> &unknown) // std::list objects are const
        : m_iid(TInterface::iid ()) { _init (&unknown); }
    SidIPtr (const ISidUnknown &unknown)
        : m_iid(TInterface::iid ()) { _init (&unknown); }
    SidIPtr (const ISidUnknown *unknown)
        : m_iid(TInterface::iid ()) { _init (unknown); }
    ~SidIPtr () { _release (); }

public:
    ISidUnknown *iaggregate () { return m_unknown; }
    const char  *iname      () const { return m_unknown->iname (); }

    TInterface *operator -> () const { assert (m_unknown); return m_if; }
    operator    bool        () const { return m_unknown != 0; }

    bool operator == (const ISidUnknown *unknown) const
    {
        return (m_unknown == unknown);
    }

    bool operator == (const SidIPtr<TInterface>& other) const
    {
        return m_unknown == other.m_unknown;
    }

    // Some interfaces are hidden within another interface.  This
    // calls allows you to access those hidden ones (i.e. consider
    // a private interface aggregated and passed around with a
    // public one)
    template<class THiddenInterface>
    SidLazyIPtr<THiddenInterface> hidden ()
    {
        SidLazyIPtr<THiddenInterface> hidden = m_if;
        return hidden;
    }

private:
    // ISidUnknown
    bool _iquery (const Iid &iid, void **implementation)
    {
        if (iid == m_iid)
        {
            if (m_unknown)
            {
                *implementation = m_if;
                return true;
            }
            return false;
        }
        if (m_unknown)
            return m_unknown->_iquery (iid, implementation);
        return false;
    }

    void  _iadd     () { assert (0); }
    const Iid &_iid () const { return m_iid; }
    void  _irelease () { assert (0); }

    void _assign (ISidUnknown *unknown)
    {
        _release ();

        if (unknown)
        {
            ISidUnknown *u = unknown;
            for (;;)
            {
                m_if = 0;
                if (u->_iquery (m_iid, reinterpret_cast<void**>(&m_if)))
                {
                    if (m_if)
                        break;
                    assert (0); // Error, no interface present
                    return;
                }
                else if (!m_if)
                    return;
                u = m_if->iaggregate (); // chain
            }
            m_unknown = unknown->iaggregate ();
            m_unknown->_iadd ();
        }
    }

    void _init (const ISidUnknown *unknown)
    {
        m_unknown = 0;
        _assign (const_cast<ISidUnknown*>(unknown));
        assert  (m_unknown);
    }

    void _release ()
    {
        if (m_unknown)
        {
            m_unknown->_irelease ();
            m_unknown = 0;
            m_if = 0;
        }
    }

protected: // For lazy initialisation
    SidIPtr () : m_iid(TInterface::iid ()), m_unknown(0) { ; }

    const ISidUnknown *operator = (const ISidUnknown *unknown)
    {
        _assign (const_cast<ISidUnknown*>(unknown));
        return unknown;
    }

    const ISidUnknown &operator = (const ISidUnknown &unknown)
    {
        _assign (const_cast<ISidUnknown*>(&unknown));
        return unknown;
    }

    const ISidUnknown &operator = (const SidIPtr<TInterface> &unknown)
    {
        const ISidUnknown *u = &unknown;
        _assign (const_cast<ISidUnknown*>(u));
        return unknown;
    }
};

template <class TInterface>
bool operator == (const SidIPtr<TInterface>& first, const SidIPtr<TInterface>& second)
{
    return first.operator==(second);
}

#endif // _sidiptr_h_
