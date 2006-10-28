/***************************************************************************
             hardsid.cpp  -  Hardsid support interface.
                             Created by Simon White
                             -------------------
    begin                : Thurs Jun 29 2006
    copyright            : (C) 2006 by Simon White
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
 *  Revision 1.1  2006/06/29 19:36:33  s_a_white
 *  Add emulation file for common things between platforms.
 *
 ***************************************************************************/

#include "hardsid-emu.h"

// Find the correct interface
bool HardSID::ifquery (const InterfaceID &iid, void **implementation)
{
    if (iid == ISidEmulation::iid())
        *implementation = static_cast<ISidEmulation *>(this);
    else if (iid == ISidMixer::iid())
        *implementation = static_cast<ISidMixer *>(this);
    else if (iid == IInterface::iid())
        *implementation = static_cast<ISidEmulation *>(this);
    else
        return false;
    reinterpret_cast<IInterface *>(*implementation)->ifadd ();
    return true;
}
