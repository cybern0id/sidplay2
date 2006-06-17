/***************************************************************************
                          sidbuilder.h  -  Sid Builder Interface
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

#include "sid2types.h"
#include "component.h"
#include "c64env.h"

class ISidBuilder;
class ISidEmulation: virtual public IComponent
{
public:
    virtual ISidBuilder *builder      (void) const = 0;
    virtual void         clock        (sid2_clock_t clk) = 0;
    virtual void         gain         (int_least8_t precent) = 0;
    virtual void         mute         (uint_least8_t num, bool enable) = 0;
    virtual void         optimisation (uint_least8_t level) = 0;
    virtual void         reset        (uint_least8_t volume) = 0;
    virtual void         volume       (uint_least8_t num, uint_least8_t level) = 0;

    // @FIXME@ Export via another interface
    virtual int_least32_t output  (uint_least8_t bits) = 0;
};

class ISidBuilder: virtual public IInterface
{
public:
    virtual operator       bool    () const = 0;
    virtual const char    *credits (void) = 0;
    virtual const char    *error   (void) const = 0;
    virtual ISidEmulation *lock    (c64env *env, sid2_model_t model) = 0;
    virtual const char    *name    (void) const = 0;
    virtual void           unlock  (ISidEmulation *device) = 0;
};

#endif // _sidbuilder_h_
