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

#ifndef _imp_sidbuilder_h_
#define _imp_sidbuilder_h_

// This file is only to be used be things creating implementations.
// DO NOT include this file in external code usings these implementations,
// use interface files instead:

#include <sidplay/sidbuilder.h>

//-------------------------------------------------------------------------
#include <sidplay/imp/component.h>

// Inherit this class to create a new SID emulations for libsidplay2.
class SidBuilder;
class SidEmulation: virtual public ISidEmulation, public Component
{
private:
    ISidBuilder * const m_builder;

public:
    SidEmulation (ISidBuilder *builder)
    :m_builder (builder) {;}

    // IInterface
    virtual void ifquery   (const InterfaceID &cid, void **implementation) = 0;

    // IComponent
    virtual const char *credits (void) = 0;
    virtual const char *error   (void) = 0;
    virtual uint8_t     read    (uint_least8_t addr) = 0;
    void                reset   (void) { reset (0); }
    virtual void        write   (uint_least8_t addr, uint8_t data) = 0;

    // ISidEmulation
    ISidBuilder         *builder      (void) const { return m_builder; }
    virtual void         clock        (sid2_clock_t clk) {;}
    virtual void         gain         (int_least8_t precent) {;}
    virtual void         mute         (uint_least8_t num, bool enable) = 0;
    virtual void         optimisation (uint_least8_t level) {;}
    virtual void         reset        (uint_least8_t volume) = 0;
    virtual void         volume       (uint_least8_t num, uint_least8_t level) = 0;
};


class SidBuilder : virtual public ISidBuilder
{
private:
    const char * const m_name;
    int m_refcount;

protected:
    bool m_status;

public:
    // Determine current state of object (true = okay, false = error).
    SidBuilder(const char * const name)
        : m_name(name), m_refcount(0), m_status (true) {;}

    // IInterface
    virtual void ifadd     () { m_refcount++; }
    virtual void ifquery   (const InterfaceID &cid, void **implementation) = 0;
    virtual void ifrelease () { m_refcount--; if (!m_refcount) delete this; }

    // ISidBuilder (implementations only)
    operator               bool    () const { return m_status; }
    virtual const char    *credits (void) = 0;
    virtual const char    *error   (void) const = 0;
    virtual ISidEmulation *lock    (c64env *env, sid2_model_t model) = 0;
    const char            *name    (void) const { return m_name; }
    virtual void           unlock  (ISidEmulation *device) = 0;
};

#endif // _imp_sidbuilder_h_
