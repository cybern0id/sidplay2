/***************************************************************************
          hardsid-stream.h - Hardsid streams.
                             -------------------
    begin                : Sun Mar 13 2005
    copyright            : (C) 2005 by Simon White
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.1  2005/03/20 22:47:39  s_a_white
 *  Added synchronous stream support for MK4 styles hardware.
 *
 ***************************************************************************/

#ifndef _hardsid_stream_h_
#define _hardsid_stream_h_

#include <vector>
#include "hardsid-emu.h"

// Provide stream interface, currently Linux only but should
// also support Windows at some point
class HardSIDStream
{
private:
    sidbuilder * const m_builder;
    bool               m_status;
    int                m_handle;
    event_clock_t      m_accessClk;
    char               m_errorBuffer[100];
    uint               m_devUsed;
    uint               m_devAvail;

    std::vector<HardSID *> m_sids;

public:
    HardSIDStream  (sidbuilder *builder);
    ~HardSIDStream ();

    HardSID      *lock       (c64env *env, sid2_model_t model = SID2_MODEL_CORRECT);
    void          filter     (bool enable);
    void          flush      (void);
    uint          allocate   (uint sids);
    uint          allocated  (void) { return m_sids.size (); }
    bool          reallocate (HardSID *sid, sid2_model_t model);
    const char   *error      (void) { return m_errorBuffer;}

    operator bool () const { return m_status; }
};

#endif // _hardsid_stream_h_
