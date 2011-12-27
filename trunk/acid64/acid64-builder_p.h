/***************************************************************************
                          acid64-builder-p.h  -  Private Implementation
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

#ifndef _acid64_builder_p_h_
#define _acid64_builder_p_h_

#include <vector>
#include <sidplay/imp/sidcobuilder.h>
#include <sidplay/event.h>
#include <sidplay/sidLazyiptr.h>
#include <sidplay/sidbuilder.h>

class Acid64Cmd;
class Acid64SID;
using namespace SIDPLAY2_NAMESPACE;

/***************************************************************************
 * ReSID Builder Class
 ***************************************************************************/
// Create the SID builder object
class Acid64BuilderPrivate: public CoBuilder<ISidBuilder>
{
protected:
    std::vector< SidLazyIPtr<ISidUnknown> > m_sidobjs;

private:
    Acid64Cmd        &m_cmd;
    std::vector<char> m_credits;
    int               m_delay;
    char              m_errorBuffer[100];
    const char       *m_error;

public:
    Acid64BuilderPrivate  (const char * name, Acid64Cmd &cmd);
    ~Acid64BuilderPrivate (void);

    // true will give you the number of used devices.
    //    return values: 0 none, positive is used sids
    // false will give you all available sids.
    //    return values: 0 endless, positive is available sids.
    // use bool operator to determine error
    uint         devices (bool used);
    uint         create  (uint sids);
    ISidUnknown *lock    (c64env *env, sid2_model_t model);
    void         unlock  (ISidUnknown &device);
    void         remove  (void);
    const char  *error   (void) const { return m_error; }
    const char  *credits (void);

protected:
    // ISidUnknown
    bool _iquery (const Iid &iid, void **implementation);
};

#endif // _acid64_builder_p_h_
