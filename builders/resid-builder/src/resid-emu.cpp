/***************************************************************************
                          c64sid.h  -  ReSid Wrapper for redefining the
                                       filter
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

#include "resid.h"
#include "config.h"

#if HAVE_EXCEPTIONS
#   include <new.h>
#endif

// Allow resid to be in more than one location
#ifdef HAVE_LOCAL_RESID
#   include "resid/sid.h"
#else
#   ifdef HAVE_USER_RESID
#       include "sid.h"
#   else
#       include <resid/sid.h>
#   endif
#endif


char ReSID::m_credit[];

ReSID::ReSID (sidbuilder *builder)
:sidemu(builder),
 m_context(NULL),
#ifdef HAVE_EXCEPTIONS
 m_sid(*(new(nothrow) SID)),
#else
 m_sid(*(new SID)),
#endif
 m_gain(100),
 m_status(true),
 m_locked(false)
{
    char *p = m_credit;
    m_error = "N/A";

    // Setup credits
    sprintf (p, "ReSID V%s Engine:", resid_version_string);
    p += strlen (p) + 1;
    strcpy  (p, "\tCopyright (C) 1999 Dag Lem <resid@nimrod.no>");
    p += strlen (p) + 1;
    *p = '\0';

	if (!&m_sid)
	{
		m_error  = "RESID ERROR: Unable to create sid object";
		m_status = false;
        return;
	}
    reset ();
}

bool ReSID::filter (const sid_filter_t *filter)
{
    fc_point fc[0x802];
    const    fc_point *f0 = fc;
    int      points = 0;

    if (filter == NULL)
    {   // Select default filter
        m_sid.fc_default (f0, points);
    }
    else
    {
        const sid_fc_t * const cutoff = filter->cutoff;
        points = filter->points;
 
        // Make sure there are enough filter points and they are legal
        if ((points < 2) || (points > 0x800))
            return false;

        {
            const sid_fc_t *fin, *fprev;
            const sid_fc_t  fstart = {-1, 0};
            fc_point       *fout   = fc;
            fprev = &fstart;
            fin   = cutoff;
            // Last check, make sure they are list in numerical order
            // for both axis
            while (points-- > 0)
            {
                if ((*fprev)[0] >= (*fin)[0])
                    return false;
                fout++;
                (*fout)[0] = (sound_sample) (*fin)[0];
                (*fout)[1] = (sound_sample) (*fin)[1];
                fprev      = fin++;
            }
            // Updated ReSID interpolate requires we
            // repeat the end points
            (*(fout+1))[0] = (*fout)[0];
            (*(fout+1))[1] = (*fout)[1];
            fc[0][0] = fc[1][0];
            fc[0][1] = fc[1][1];
            points   = filter->points + 2;
        }
    }

    // function from reSID
    points--;
    interpolate (f0, f0 + points, m_sid.fc_plotter (), 1.0);
    return true;
}

// Standard component options
void ReSID::reset (void)
{
    m_accessClk = 0;
    m_sid.reset ();
}

uint8_t ReSID::read (const uint_least8_t addr)
{
    event_clock_t cycles = m_context->getTime (m_accessClk);
    m_accessClk += cycles;
    if (cycles)
        m_sid.clock (cycles);
    return m_sid.read (addr);
}

void ReSID::write (const uint_least8_t addr, const uint8_t data)
{
    event_clock_t cycles = m_context->getTime (m_accessClk);
    m_accessClk += cycles;
    if (cycles)
        m_sid.clock (cycles);
    m_sid.write (addr, data);
}

int_least32_t ReSID::output (const uint_least8_t bits)
{
    event_clock_t cycles = m_context->getTime (m_accessClk);
    m_accessClk += cycles;
    if (cycles)
        m_sid.clock (cycles);
    return m_sid.output (bits); // * m_gain / 100;
}

void ReSID::filter (const bool enable)
{
	m_sid.enable_filter (enable);
}

void ReSID::voice (const uint_least8_t num, const uint_least8_t volume,
                   const bool mute)
{   // At this time only mute is supported
	m_sid.mute (num, mute);
}
    
void ReSID::gain (const int_least8_t percent)
{
    // 0 to 99 is loss, 101 - 200 is gain
    m_gain  = percent;
    m_gain += 100;
    if (m_gain > 200)
        m_gain = 200;
}

void ReSID::sampling (uint_least32_t freq)
{
    m_sid.set_sampling_parameters (1000000, SAMPLE_FAST, freq);
}

// Set execution environment and lock sid to it
void ReSID::lock (c64env *env)
{
    if (env == NULL)
    {
        m_locked  = false;
        m_context = NULL;
    }
    else
    {
        m_locked  = true;
        m_context = &env->context ();
    }
}

// Set the emulated SID model
void ReSID::model (sid2_model_t model)
{
	if (model == SID2_MOS8580)
        m_sid.set_chip_model (MOS8580);
	else
        m_sid.set_chip_model (MOS6581);
}
