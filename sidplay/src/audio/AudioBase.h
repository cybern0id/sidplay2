/***************************************************************************
                          AudioBase.h  -  description
                             -------------------
    begin                : Sat Jul 8 2000
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
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.1  2001/01/08 16:41:43  s_a_white
 *  App and Library Seperation
 *
 *  Revision 1.4  2000/12/11 19:07:14  s_a_white
 *  AC99 Update.
 *
 ***************************************************************************/

#ifndef _AudioBase_h_
#define _AudioBase_h_

#include <string.h>
#include "AudioConfig.h"

class AudioBase
{
protected:
    AudioConfig _settings;
    char       *_errorString;
    void       *_sampleBuffer;

public:
    AudioBase ()
    {
        _errorString  = "None";
        _sampleBuffer = NULL;
    }
    virtual ~AudioBase () {;}

    // All drivers must support these
    virtual void *open  (AudioConfig &cfg, const char *name) = 0;
    // Rev 1.3 (saw) - Definition is incorrect and has been updated.
    // On a reset hardware buffers may have changed and therefore a
    // new address needs to be returned by the driver.
    virtual void *reset () = 0;
    virtual void *write () = 0;
    virtual void  close () = 0;
    void   *buffer () { return _sampleBuffer; }

    void getConfig (AudioConfig &cfg) const
    {
        cfg = _settings;
    }
    
    const char *getErrorString () const
    {
        return _errorString;
    }
};

#endif // _AudioBase_h_
