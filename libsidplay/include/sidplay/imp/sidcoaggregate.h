/***************************************************************************
                          sidcoaggregate.h  -  Compile time aggregates
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

#ifndef _sidcoaggregate_h_
#define _sidcoaggregate_h_

#include <sidplay/sidiunknown.h>

SIDPLAY2_NAMESPACE_START

template <class TImplementation>
class CoAggregate: public TImplementation
{
private:
    SidIUnknown &m_unknown;

public:
    CoAggregate (SidIUnknown &unknown)
        : m_unknown(unknown) { ; }

private:
    const SidIid &iid () const { return TImplementation::iid (); }

    virtual SidIUnknown &iaggregate () = 0;
    const char *iname () const { return m_unknown.iname (); }

    void iadd     () { return m_unknown.iadd (); }
    void irelease () { return m_unknown.irelease (); }

protected:
    virtual bool iquery (const SidIid &iid, void **implementation)
                        { return m_unknown.ifquery (iid, implementation); }

};

SIDPLAY2_NAMESPACE_END

#endif // _sidcoaggregate_h_
