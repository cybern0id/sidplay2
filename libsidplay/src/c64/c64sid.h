/***************************************************************************
                          c64sid.h  -  ReSid Wrapper
                             -------------------
    begin                : Fri Apr 4 2001
    copyright            : (C) 2001 by Simon White
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

#ifndef _c64sid_h_
#define _c64sid_h_

/* This file could be a specialisation of a sid implementation.
 * However since the sid emulation is not part of this project
 * we are actually creating a wrapper instead.
 */

#include "c64env.h"
#include "sidbuilder.h"
#include "sidplay2.h"
#include "../mos6581/mos6581.h"

class c64sid: public sidemu
{
private:
    c64env        &m_env;
    SID            m_sid;
    event_clock_t  m_accessClk;
    int_least32_t  m_gain;
    static char    credit[100];

public:
    c64sid (c64env *env);
    const char *credits (void) {return credit;}
    const char *error   (void) {return "";}

    // Standard component options
    void reset (void)
    {
        m_accessClk = 0;
        m_sid.reset ();
    }

    uint8_t read (const uint_least8_t addr)
    {
            event_clock_t cycles = m_env.eventContext.getTime (m_accessClk);
            m_accessClk += cycles;
            if (cycles)
                m_sid.clock (cycles);
//            while (cycles--)
//                m_sid.clock ();
            return m_sid.read (addr);
    }

    void write (const uint_least8_t addr, const uint8_t data)
    {
            event_clock_t cycles = m_env.eventContext.getTime (m_accessClk);
            m_accessClk += cycles;
            if (cycles)
                m_sid.clock (cycles);
//            while (cycles--)
//                m_sid.clock ();
            m_sid.write (addr, data);
    }

    int_least32_t output (const uint_least8_t bits)
    {
            event_clock_t cycles = m_env.eventContext.getTime (m_accessClk);
            m_accessClk += cycles;
            if (cycles)
                m_sid.clock (cycles);
//            while (cycles--)
//                m_sid.clock ();
            return m_sid.output (bits) * m_gain / 100;
    }

    void filter (const bool enable) {m_sid.enable_filter (enable);}
    void model  (const sid2_model_t model)
    {
        if (model == SID2_MOS6581)
            m_sid.set_chip_model (MOS6581);
        else
            m_sid.set_chip_model (MOS8580);
    }
    void voice (const uint_least8_t num, const uint_least8_t volume,
        const bool mute) {m_sid.mute (num, mute);}
    
    void gain  (const int_least8_t percent)
    {
        // 0 to 99 is loss, 101 - 200 is gain
        m_gain  = percent;
        m_gain += 100;
        if (m_gain > 200)
            m_gain = 200;
    }

    // ReSID specific options
    void exfilter (const double cutoff)
    {m_sid.enable_external_filter (true, cutoff);}
    bool filter   (const sid_filter_t *filter);
};

#endif // _c64sid_h_
