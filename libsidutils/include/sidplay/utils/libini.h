/***************************************************************************
                          libini.h  -  Header file of functions to
                                       manipulate an ini file.
                             -------------------
    begin                : Fri Apr 21 2000
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

#ifndef _libini_h_
#define _libini_h_

#include "iniconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#define INI_ADD_LIST_SUPPORT
#define INI_ADD_EXTRA_TYPES

/* Rev 1.3 Added scripting support using Swig */
#ifdef SWIG
%module libini
%include typemaps.i
%apply int    *BOTH { int    *value };
%apply long   *BOTH { long   *value };
%apply double *BOTH { double *value };
%name (ini_readString)
    int ini_readFileToBuffer    (ini_fd_t fd, buffer_init *buffer);
%name (ini_writeString)
    int ini_writeFileFromBuffer (ini_fd_t fd, buffer_init *buffer);

buffer_init *ini_createBuffer        (unsigned long size);
void         ini_deleteBuffer        (buffer_init *buffer);
int          ini_readFileToBuffer    (ini_fd_t fd, buffer_init *buffer);
int          ini_writeFileFromBuffer (ini_fd_t fd, buffer_init *buffer);
char        *ini_getBuffer           (buffer_init *buffer);
int          ini_setBuffer           (buffer_init *buffer, char *str);

%{
#include "libini.h"

typedef struct 
{
    char  *buffer;
    size_t size;
} buffer_init;

buffer_init *ini_createBuffer (unsigned int size)
{
    buffer_init *b;
    /* Allocate memory to structure */
    if (size == ( ((unsigned) -1 << 1) >> 1 ))
        return 0; /* Size is too big */
    b = malloc (sizeof (buffer_init));
    if (!b)
        return 0;

    /* Allocate memory to buffer */
    b->buffer = malloc (sizeof (char) * (size + 1));
    if (!b->buffer)
    {
        free (b);
        return 0;
    }
    b->size = size;

    /* Returns address to tcl */
    return b;
}

void ini_deleteBuffer (buffer_init *buffer)
{
    free (buffer->buffer);
    free (buffer);
}

/* tcl helper file for reading the file */
int ini_readFileToBuffer (ini_fd_t fd, buffer_init *buffer)
{
    return ini_readString (fd, buffer->buffer, buffer->size + 1);
}

/* tcl helper file for writing the file */
int ini_writeFileFromBuffer (ini_fd_t fd, buffer_init *buffer)
{
    return ini_writeString (fd, buffer->buffer);
}

/* tcl helper file for retreiving the buffer */
char *ini_getBuffer (buffer_init *buffer)
{
    return buffer->buffer;
}

/* tcl helper file for setting the buffer */
int ini_setBuffer (buffer_init *buffer, char *str)
{
    size_t len = strlen (str);
    if (len > buffer->size)
        len = buffer->size;

    memcpy (buffer->buffer, str, len);
    buffer->buffer[len] = '\0';
    return len;
}

%}
#else
    /* Rev 1.4 - Rename buffer_t to buffer_init and replaced returned
       pointer as type long to: */
    typedef void* ini_fd_t;
#endif /* SWIG */


/* Rev 1.2 Added new fuction */
ini_fd_t INI_EXPORT ini_new      (char *name);
ini_fd_t INI_EXPORT ini_open     (char *name);
int      INI_EXPORT ini_close    (ini_fd_t fd);
int      INI_EXPORT ini_flush    (ini_fd_t fd);

/* Rev 1.2 Added these functions to make life a bit easier, can still be implemented
 * through ini_writeString though. */
int INI_EXPORT ini_locateKey     (ini_fd_t fd, char *key);
int INI_EXPORT ini_locateHeading (ini_fd_t fd, char *heading);
int INI_EXPORT ini_deleteKey     (ini_fd_t fd);
int INI_EXPORT ini_deleteHeading (ini_fd_t fd);

/* Returns the number of bytes required to be able to read the key as a
 * string from the file. (1 should be added to this length to account
 * for a NULL character) */
int INI_EXPORT ini_dataLength (ini_fd_t fd);

/* Default Data Type Operations
 * Arrays implemented to help with reading, for writing you should format the
 * complete array as a string and perform a writeString. */
#ifndef SWIG
int INI_EXPORT ini_readString  (ini_fd_t fd, char *str, size_t size);
int INI_EXPORT ini_writeString (ini_fd_t fd, char *str);
#endif /* SWIG */
int INI_EXPORT ini_readInt     (ini_fd_t fd, int  *value);


#ifdef INI_ADD_EXTRA_TYPES
    /* Read Operations */
    int INI_EXPORT ini_readLong    (ini_fd_t fd, long   *value);
    int INI_EXPORT ini_readDouble  (ini_fd_t fd, double *value);

    /* Write Operations */
    int INI_EXPORT ini_writeInt    (ini_fd_t fd, int    value);
    int INI_EXPORT ini_writeLong   (ini_fd_t fd, long   value);
    int INI_EXPORT ini_writeDouble (ini_fd_t fd, double value);
#endif // INI_ADD_EXTRA_TYPES


#ifdef INI_ADD_LIST_SUPPORT
    /* Rev 1.1 Added - List Operations (Used for read operations only)
     * Be warned, once delimiters are set, every key that is read will be checked for to
     * if sub string are present.  These will incur a speed hit and therefore once a line
     * has been read and list/array functionality is not required for a while, set delimiters
     * back to NULL, until the it is required again. */

    /* Returns the number of elements in an list being seperated by the provided delimiters */
    int INI_EXPORT ini_listLength      (ini_fd_t fd);
    /* Change delimiters, default "" */
    int INI_EXPORT ini_listDelims      (ini_fd_t fd, char *delims);
    /* Set index to access in a list.  When read the index will automatically increment */
    int INI_EXPORT ini_listIndex       (ini_fd_t fd, unsigned long index);
    /* Returns the length of an indexed sub string in the list */
    int INI_EXPORT ini_listIndexLength (ini_fd_t fd);
#endif // INI_ADD_LIST_SUPPORT

#ifdef __cplusplus
}
#endif

#endif /* _libini_h_ */
