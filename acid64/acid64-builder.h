/***************************************************************************
                          acid64-builder.h  -  Public Interface
                             -------------------
    begin                : Sat Dec 24 2011
    copyright            : (C) 2011 by Simon White
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

#ifndef _acid64_builder_h_
#define _acid64_builder_h_

#include <algorithm>
#include <sidplay/sidlazyiptr.h>
#include "acid64-cmd.h"

class Acid64BuilderPrivate;

class Acid64Builder
{
public:
    Acid64Builder (const char * name, Acid64Cmd &cmd);
    ISidUnknown *iunknown () { return m_iptr->iunknown (); }

    uint create (uint sids);

private:
    Acid64BuilderPrivate    *m_impl;
    SidLazyIPtr<ISidUnknown> m_iptr;
};

#endif // _acid64_builder_h_
