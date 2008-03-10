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

#include <sidplay/sidunknown.h>

SIDPLAY2_NAMESPACE_START

template <class TInterface>
class CoAggregate: public TInterface
{
private:
    ISidUnknown &m_unknown;

public:
    CoAggregate (ISidUnknown &unknown)
        : m_unknown(unknown) { ; }

    virtual ISidUnknown *iaggregate () = 0;
    const char *iname () const { return m_unknown.iname (); }

private:
    void       _iadd     () { return m_unknown._iadd (); }
    const Iid &_iid      () const { return TInterface::iid (); }
    void       _irelease () { return m_unknown._irelease (); }

protected:
    virtual bool iquery (const Iid &iid, void **implementation)
                        { return m_unknown.iquery (iid, implementation); }

private:
    CoAggregate (const CoAggregate &);
    CoAggregate &operator= (const CoAggregate &);
};

SIDPLAY2_NAMESPACE_STOP

#endif // _sidcoaggregate_h_
