/***************************************************************************
                          sidenv.h  -  This is the environment file which
                                       defines all the standard functions to
                                       be inherited by the ICs.
                             -------------------
    begin                : Thu May 11 2000
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
#ifndef _c64env_h_
#define _c64env_h_

#include "sidtypes.h"

/*
// Enviroment functions - THESE FUNTIONS MUST BE PROVIDED
// TO ALLOW THE COMPONENTS TO SPEAK TO EACH OTHER.  ENVP
// CAN BE USED TO CREATE VERSIONS OF THESE FUNTIONS
// WHICH ACCESS MEMBER FUNTIONS OF OTHER C++ OBJECTS!
void       reset        (void);
ubyte_sidt readMemByte  (uword_sidt addr);
void       writeMemByte (uword_sidt addr, ubyte_sidt data);

// Interrupts - You must raise the interrupt(s)
// every cycle if you have not yet been serviced
void  triggerIRQ (void);
void  triggerNMI (void);
void  triggerRST (void);
void  clearIRQ   (void);

// Sidplay compatibily funtions
bool       checkBankJump  (uword addr);
ubyte_sidt readEnvMemByte (uword addr);
*/

class C64Environment
{
/*
protected:
    // Eniviroment functions
    virtual inline void  envReset             (void)
    { ::reset (); }
    virtual inline ubyte_sidt envReadMemByte  (uword_sidt addr)
    { ::readMemByte  (addr); }
    virtual inline void       envWriteMemByte (uword_sidt addr, ubyte_sidt data)
    { ::writeMemByte (addr, data); }

    // Interrupts
    virtual inline void  encTriggerIRQ (void)
    { ::triggerIRQ (); }
    virtual inline void  envTriggerNMI (void)
    { ::triggerNMI (); }
    virtual inline void  envTriggerRST (void)
    { ::triggerRST (); }
    virtual inline void  envClearIRQ   (void)
    { ::clearIRQ   (); }

    // Sidplay compatibily funtions
    virtual inline bool       envCheckBankJump   (uword_sidt addr)
    { ::checkBankJump   (); }
    virtual inline ubyte_sidt envReadMemDataByte (uword_sidt addr)
    { ::readMemDataByte (); }
    */

protected:
    C64Environment *m_envp;

    // Sidplay2 Player Environemnt
public:
    virtual ~C64Environment () {}
    virtual void setEnvironment (C64Environment *envp)
    {
        m_envp = envp;
    }

protected:
    // Eniviroment functions
    virtual inline void  envReset             (void)
    { m_envp->envReset (); }
    virtual inline ubyte_sidt envReadMemByte  (uword_sidt addr, bool useCache = false)
    { return m_envp->envReadMemByte  (addr, useCache); }
    virtual inline void       envWriteMemByte (uword_sidt addr, ubyte_sidt data, bool useCache = true)
    { m_envp->envWriteMemByte (addr, data, useCache); }

    // Interrupts
    virtual inline void  envTriggerIRQ (void)
    { m_envp->envTriggerIRQ (); }
    virtual inline void  envTriggerNMI (void)
    { m_envp->envTriggerNMI (); }
    virtual inline void  envTriggerRST (void)
    { m_envp->envTriggerRST (); }
    virtual inline void  envClearIRQ   (void)
    { m_envp->envClearIRQ ();   }

    // Sidplay compatibily funtions
    virtual inline bool       envCheckBankJump   (uword_sidt addr)
    { return m_envp->envCheckBankJump   (addr); }
    virtual inline ubyte_sidt envReadMemDataByte (uword_sidt addr, bool useCache = false)
    { return m_envp->envReadMemDataByte (addr, useCache); }
};

#endif // _c64env_h_
