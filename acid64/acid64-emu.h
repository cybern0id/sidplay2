/***************************************************************************
                          acid64-emu.h
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

#include "config.h"
#include <sidplay/imp/sidcoaggregate.h>
#include <sidplay/imp/sidcobuilder.h>

class Acid64Builder;
class Acid64Cmd;
using namespace SIDPLAY2_NAMESPACE;

class Acid64Emu: public  CoEmulation<ISidEmulation>,
                 public  CoAggregate<ISidMixer>,
                 private Event
{
private:
    Acid64Cmd    &m_cmd;
    EventContext *m_context;
    event_phase_t m_phase;
    event_clock_t m_accessClk;
    const  char  *m_error;
    bool          m_status;

public:
    static const Iid &iid () {
        SIDIID(0x04F9045E, 0x60B2, 0x4ed7, 0x980B, 0x8608, 0x1E30FF96);
    }

    Acid64Emu (Acid64BuilderPrivate &builder, Acid64Cmd &cmd);

    ISidUnknown *iunknown () { return CoEmulation<ISidEmulation>::iunknown (); }

    // IInterface
    bool _iquery (const Iid &iid, void **implementation);

    // Standard component functions
    const char   *credits (void);
    void          reset   (uint8_t volume);
    uint8_t       read    (uint_least8_t addr);
    void          write   (uint_least8_t addr, uint8_t data);
    const char   *error   (void) {return m_error;}

    // Standard SID functions
    bool          lock    (c64env *env);
    int_least32_t output  (uint_least8_t bits);
    void          volume  (uint_least8_t num, uint_least8_t level);
    void          mute    (uint_least8_t num, bool enable);
    void          gain    (int_least8_t) { ; }

    operator bool () { return m_status; }

private:
    void          event   (void);
};
