/***************************************************************************
                          resid.h  -  ReSid Builder
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

#ifndef _resid_h_
#define _resid_h_

/* Since ReSID is not part of this project we are actually
 * creating a wrapper instead of implementing a SID emulation
 */

#include <vector>
#include <sidplay/sidbuilder.h>
#include <sidplay/event.h>

class ReSID: public sidemu
{
private:
    EventContext  *m_context;
    class SID     &m_sid;
    event_clock_t  m_accessClk;
    int_least32_t  m_gain;
    static char    m_credit[100];
    const  char   *m_error;
    bool           m_status;
    bool           m_locked;

public:
    ReSID  (sidbuilder *builder);
    ~ReSID (void) {;}

    // Standard component functions
    const char   *credits (void) {return m_credit;}
    void          reset   (void);
    uint8_t       read    (const uint_least8_t addr);
    void          write   (const uint_least8_t addr, const uint8_t data);
    const char   *error   (void) {return m_error;}

    // Standard SID functions
    int_least32_t output  (const uint_least8_t bits);
    void          filter  (const bool enable);
    void          voice   (const uint_least8_t num, const uint_least8_t volume,
                           const bool mute);
    void          gain    (const int_least8_t precent);

    operator bool () { return m_status; }
    static   int  devices (char *error);

    // Specific to resid
    void sampling (uint freq);
    bool filter   (const sid_filter_t *filter);
    void model    (sid2_model_t model);
    // Must lock the SID before using the standard functions.
    void lock     (c64env *env);
    bool lock     (void) const { return m_locked; }
};



/***************************************************************************
 * ReSID Builder Class
 ***************************************************************************/
// Create the SID builder object
class ReSIDBuilder: public sidbuilder
{
private:
    static const char  *ERR_FILTER_DEFINITION;
    char        m_errorBuffer[100];
    const char *m_error;
    std::vector<sidemu *> sidobjs;

public:
    ReSIDBuilder  (const char * const name);
    ~ReSIDBuilder (void);
    // true will give you the number of used devices.
    //    return values: 0 none, positive is used sids
    // false will give you all available sids.
    //    return values: 0 endless, positive is available sids.
    // use bool operator to determine error
    uint        devices (bool used);
    uint        create  (uint sids);
    sidemu     *lock    (c64env *env, sid2_model_t model);
    void        unlock  (sidemu *device);
    void        remove  (void);
    const char *error   (void) const { return m_error; }

	// Settings that effect all SIDs
    void filter   (bool enable);
    void filter   (const sid_filter_t *filter);
    void sampling (uint_least32_t freq);
};

#endif // _resid_h_
