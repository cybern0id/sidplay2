/***************************************************************************
                          imp_sidbuilder.h  -  Sid Builder Implementation
                             -------------------
    begin                : Sat June 17 2006
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

#ifndef _sidcobuilder_h_
#define _sidcobuilder_h_

// This file is only to be used be things creating implementations.
// DO NOT include this file in external code usings these implementations,
// use interface files instead:

#include <sidplay/sidconfig.h>
#include <sidplay/sidbuilder.h>

//-------------------------------------------------------------------------
#include <sidplay/imp/component.h>

SIDPLAY2_NAMESPACE_START

// Inherit this class to create a new SID emulations for libsidplay2.
template <class TInterface, class TBuilder = ISidBuilder>
class CoEmulation: public CoComponent<TInterface>
{
private:
    TBuilder * const m_builder;

private:
    void reset (void) { reset (0); }

public:
    CoEmulation (const char *name, TBuilder *builder)
    :CoComponent<TInterface>(name), m_builder (builder) {;}

    // ISidEmulation
    ISidUnknown         *builder      (void) const { return m_builder; }
    virtual void         clock        (sid2_clock_t clk) {;}
    virtual void         gain         (int_least8_t precent) {;}
    virtual void         optimisation (uint_least8_t level) {;}
    virtual void         reset        (uint_least8_t volume) = 0;
};

template <class TInterface>
class CoBuilder: public CoUnknown<TInterface>
{
protected:
    bool m_status;

public:
    // Determine current state of object (true = okay, false = error).
    CoBuilder(const char * name)
    : CoUnknown<TInterface>(name), m_status (true) {;}

    // ISidUnknown
    void _idestroy () { delete this; }

    // ISidBuilder (implementations only)
    operator bool   () const { return m_status; }
};

SIDPLAY2_NAMESPACE_STOP

#endif // _sidcobuilder_h_
