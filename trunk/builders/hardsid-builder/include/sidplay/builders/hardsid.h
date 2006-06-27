/***************************************************************************
               hardsid.h  -  HardSID Interface
                             -------------------
    begin                : Sat Jun 17 2006
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
 *  Revision 1.11  2006/06/27 22:08:31  s_a_white
 *  Interface class must be abstract.
 *
 *  Revision 1.10  2006/06/27 19:44:55  s_a_white
 *  Add return parameter to ifquery.
 *
 *  Revision 1.9  2006/06/27 19:17:02  s_a_white
 *  Export a create call to make a builder (eventually turn code into module)
 *
 *  Revision 1.8  2006/06/21 20:02:17  s_a_white
 *  List functions in alphabetical order.
 *
 *  Revision 1.7  2006/06/20 22:25:21  s_a_white
 *  Add interface IID export.
 *
 *  Revision 1.6  2006/06/19 20:54:10  s_a_white
 *  Move implementation out, just provide interface (like COM).
 *
 *  Revision 1.5  2005/03/22 19:10:48  s_a_white
 *  Converted windows hardsid code to work with new linux streaming changes.
 *  Windows itself does not yet support streaming in the drivers for synchronous
 *  playback to multiple sids (so cannot use MK4 to full potential).
 *
 *  Revision 1.4  2004/05/05 23:47:50  s_a_white
 *  Detect available sid devices on Unix system.
 *
 *  Revision 1.3  2003/01/23 17:48:17  s_a_white
 *  Added missed return parameter for init function prototype.
 *
 *  Revision 1.2  2002/01/30 01:42:08  jpaana
 *  Don't include config.h as it isn't always available and is included elsewhere already
 *
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 *
 ***************************************************************************/

#ifndef  _hardsid_h_
#define  _hardsid_h_

#include <sidplay/sidbuilder.h>

static const InterfaceID IID_IHardSIDBuilder =
{ 0x92b1592e, 0x7f8e, 0x47ec, {0xb9, 0x95, 0x4a, 0xd6, 0x9a, 0xa7, 0x27, 0xa1} };

class IHardSIDBuilder: virtual public ISidBuilder
{
public:
    virtual uint create  (uint sids) = 0;
    virtual uint devices (bool used) = 0;
    virtual void flush   (void) = 0;
    virtual void filter  (bool enable) = 0;
    virtual void remove  (void) = 0;
};

extern "C" bool HardSIDBuilderCreate (const char * const name,
                                      const InterfaceID &cid, void **implementation);

#endif // _hardsid_h_
