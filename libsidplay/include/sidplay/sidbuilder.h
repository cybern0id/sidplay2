/***************************************************************************
                          sidbuilder.h  -  Sid Builder Classes
                             -------------------
    begin                : Sat May 6 2001
    copyright            : (C) 2000 by Simon White
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

#ifndef _sidbuilder_h_
#define _sidbuilder_h_

#include <vector>
#include "sid2types.h"
#include "component.h"
#include "c64env.h"


// Inherit this class to create a new SID emulations for libsidplay2.
class sidemu: public component
{
public:
    virtual ~sidemu (void) {;}

    // Standard component functions
    virtual void    reset (void) = 0;
    virtual uint8_t read  (const uint_least8_t addr) = 0;
    virtual void    write (const uint_least8_t addr, const uint8_t data) = 0;
    virtual const   char *credits (void) = 0;

    // Standard SID functions
    virtual int_least32_t output (const uint_least8_t bits) = 0;
    virtual void          model  (const sid2_model_t model) = 0;
    virtual void          filter (const bool enable) = 0;
    virtual void          voice  (const uint_least8_t num,
                                  const uint_least8_t vol,
                                  const bool mute) = 0;
    virtual void          gain   (const int_least8_t precent) = 0;
};


class Player;
class sidbuilder
{
private:
    friend Player;
    const char * const m_name;

protected:
    bool m_status;
    std::vector<sidemu *> sidobjs;

    virtual sidemu *create (c64env *env) = 0;
    void    remove ()
    {
        int size = sidobjs.size ();
        for (int i = 0; i < size; i++)
            delete sidobjs[i];
        sidobjs.clear();
    }

public:
    // Determine current state of object (true = okay, false = error).
    sidbuilder(const char * const name)
        : m_status (true), m_name(name) {;}
    const char * const name (void) { return m_name; }
    operator bool()  { return m_status; }
    virtual ~sidbuilder() { remove (); }
    virtual const char *error (void) = 0;
};

#endif // _sidbuilder_h_
