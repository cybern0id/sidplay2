/***************************************************************************
                          ini.cpp  -  Reads and writes keys to a
                                      ini file.

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
/***************************************************************************
 * $Log: not supported by cvs2svn $
 * Revision 1.10  2001/08/23 19:59:18  s_a_white
 * ini_append fix so not freeing wrong buffer.
 *
 * Revision 1.9  2001/08/17 19:23:16  s_a_white
 * Added ini_append.
 *
 * Revision 1.8  2001/08/14 22:21:21  s_a_white
 * Hash table and list removal fixes.
 *
 * Revision 1.7  2001/07/27 11:10:15  s_a_white
 * Simplified __ini_deleteAll.
 *
 * Revision 1.6  2001/07/21 09:47:12  s_a_white
 * Bug Fixes (thanks Andy):
 * *) After a flush the key and heading are now remembered.
 * *) ini_deleteAll then ini_close now correctly deletes the ini file.
 * *) ini_flush with no changes no longer destroys the ini object.
 *
 ***************************************************************************/

//*******************************************************************************************************************
// Include Files
//*******************************************************************************************************************
#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "ini.h"


//*******************************************************************************************************************
// Local Variables
//*******************************************************************************************************************
#define POLY32 0x04C11DB7L
#define MASK32 0xFFFFFFFFL
// Global buffer as some compilers produce
// code which dosen't work if it is local
char buffer[1024 * 5];


//*******************************************************************************************************************
// Function Prototypes
//*******************************************************************************************************************
static ini_t              *__ini_open            (const char *name, ini_mode_t mode);
static int                 __ini_close           (ini_t *ini, bool flush);
static void                __ini_delete          (ini_t *ini);
struct key_tag            *__ini_locate          (ini_t *ini, char *heading, char *key);
static int                 __ini_process         (ini_t *ini, FILE *file);
static int                 __ini_store           (ini_t *ini, FILE *file);


/********************************************************************************************************************
 * Function          : createCrc32
 * Parameters        : init   - initial crc starting value, pBuf - data to base crc on
 *                   : length - length in bytes of data
 * Returns           :
 * Globals Used      :
 * Globals Modified  :
 * Description       : Creates a 32 bit CRC based on the input data
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
#ifdef INI_USE_HASH_TABLE
static unsigned long createCrc32 (unsigned long init, char *pBuf, size_t length)
{
   unsigned long crc;
   bool     msb;
   size_t   l;
   unsigned char data;
   int      bits;

   // Init CRC register
   crc = init;

   for (l = 0; l < length; l++)
   {
       // Get a byte to process
       data = *pBuf++;

       // Process all the bits
       bits = 8;
       while(bits-- > 0)
       {
           // Get another bit
           msb = (crc & 0x80000000L) != 0;
           crc <<= 1;

           if(msb ^ ((bits & data) != 0))
               crc ^= POLY32;
       }
   }

   // return result which will be calculated
   // to a 32 bit boundary.  This is nessary
   // as transmission to DSPs is 32bits on
   // which means 0 padding may be required.
   return (crc&MASK32);
}
#endif // INI_USE_HASH_TABLE


/********************************************************************************************************************
 * Function          : strtrim
 * Parameters        : str - string to be trimmed
 * Returns           :
 * Globals Used      :
 * Globals Modified  :
 * Description       : Removes all char deemed to be spaces from start and end of string.
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
void strtrim (char *str)
{
    long first, last;
    first = 0;
    last  = strlen (str);

    if (!last--)
        return;

    // Clip end first
    while (isspace (str[last]) && last > 0)
        last--;
    str[last + 1] = '\0';

    // Clip beginning
    while (isspace (str[first]) && (first < last))
        first++;
    strcpy (str, str + first);
}


/********************************************************************************************************************
 * Function          : __ini_open
 * Parameters        : name - ini file to parse
 * Returns           : Pointer to ini database.
 * Globals Used      :
 * Globals Modified  :
 * Description       : Opens an ini data file and reads it's contents into a database
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
ini_t *__ini_open (const char *name, ini_mode_t mode)
{
    ini_t   *ini;
    FILE    *file = NULL;
    unsigned long length;

    if (!name)
        return 0;

    length = strlen(name);
    if (!length)
        return 0;

    // Create ini database stub
    ini = (ini_t *) malloc (sizeof (ini_t));
    if (!ini)
        goto ini_openError;
    memset (ini, 0, sizeof (ini_t));

    // Store ini filename
    ini->filename = (char *) malloc (sizeof(char) * (strlen(name) + 1));
    if (!ini->filename)
        goto ini_openError;
    strcpy (ini->filename, name);

    // Open input file
    ini->mode = mode;
    file = fopen (ini->filename, "rb");
    if (!file)
    {   // File doesn't exist and we are not allowed
        // to create one
		if (mode != INI_NEW)
            goto ini_openError;

		// Seems we can make so new one, check and
		// make sure
        file = fopen (ini->filename, "wb");
        if (!file)
            goto ini_openError;
        fclose (file);
    }

    // Open backup file
    if (ini->mode == INI_READ)
        ini->ftmp = tmpfile ();
	else
	{
        ini->filename[length - 1] = '~';
        ini->ftmp = fopen (ini->filename, "wb+");
        ini->filename[length - 1] = name[length - 1];
	}

    if (!ini->ftmp)
        goto ini_openError;
    if (!file)
        ini->newfile = true;
    else
    {   // Process existing ini file
        if (__ini_process (ini, file) < 0)
            goto ini_openError;
    }

    // Rev 1.1 Added - Changed set on open bug fix
    ini->changed = false;
return ini;

ini_openError:
    if (ini)
    {
        if (ini->ftmp)
        {   // Close and remove backup file
            fclose (ini->ftmp);
			if (ini->mode != INI_READ)
			{
                ini->filename[strlen (ini->filename) - 1] = '~';
                remove (ini->filename);
			}
        }
        if (ini->filename)
            free (ini->filename);
        free (ini);
    }

    if (file)
        fclose (file);

    return 0;
}


/********************************************************************************************************************
 * Function          : __ini_close
 * Parameters        : ini - pointer to ini file database.  force - if true, the database will be removed from
 *                   : memory even if an error occured in saving the new ini file.
 * Returns           : -1 on error, 0 on success
 * Globals Used      :
 * Globals Modified  :
 * Description       : Save any changes back to the new ini file.
 *                   : The backup file contains all the orignal data + any modifcations appended at the bottom
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
int __ini_close (ini_t *ini, bool flush)
{
    FILE *file;
    int   ret = 0;

    // Open output file
    if (ini->changed)
    {
        if (!ini->first)
            remove(ini->filename);
        else
        {
#ifdef INI_ADD_LIST_SUPPORT
            char *delims;
            // Rev 1.1 Added - Must remove delims before saving
            delims = ini->listDelims;
            ini->listDelims = NULL;
#endif // INI_ADD_LIST_SUPPORT

            // Not point writing an unchanged file
            file = fopen (ini->filename, "w");
            if (file)
            {   // Output all new headers and keys
                ret = __ini_store (ini, file);
                fflush (file);
                fclose (file);
            }

#ifdef INI_ADD_LIST_SUPPORT
            // Rev 1.1 Added - This was only a flush, so lets restore
            // the old delims
            ini->listDelims = delims;
#endif // INI_ADD_LIST_SUPPORT
            if (!file)
                return -1;
        }
    }

    // Check if the user dosent want the file closed.
    if (!flush)
        return 0;

    // Cleanup
    fclose (ini->ftmp);

	if (ini->mode != INI_READ)
	{   // If no mods were made, delete tmp file
        if (!ini->changed || ini->newfile)
		{
            ini->filename[strlen (ini->filename) - 1] = '~';
            remove (ini->filename);
		}
	}

    __ini_delete (ini);
    free (ini->filename);

    if (ini->tmpSection.heading)
        free (ini->tmpSection.heading);
    if (ini->tmpKey.key)
        free (ini->tmpKey.key);

#ifdef INI_ADD_LIST_SUPPORT
    // Rev 1.1 - Remove buffered list
    if (ini->list)
        free (ini->list);
#endif // INI_ADD_LIST_SUPPORT

    free (ini);
    return ret;
}


/********************************************************************************************************************
 * Function          : __ini_delete
 * Parameters        : ini - pointer to ini file database.
 * Returns           :
 * Globals Used      :
 * Globals Modified  :
 * Description       : Deletes the whole ini database from memory, but leaves the ini stub so the ini_close can be
 *                   : called.
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
void __ini_delete (ini_t *ini)
{   // If already deleted, don't delete it again
    if (!ini->first)
        return;

    // Go through all sections deleting them
    while (ini->first)
    {
         ini->selected = ini->first;
         __ini_deleteHeading (ini);
    }

#ifdef INI_ADD_LIST_SUPPORT
    // Rev 1.1 - Remove buffered list
    if (ini->list)
    {
        free (ini->list);
        ini->list = NULL;
    }
#endif // INI_ADD_LIST_SUPPORT

    ini->changed = true;
}


/********************************************************************************************************************
 * Function          : __ini_process
 * Parameters        : ini - pointer to ini file database,  file - ini file to read heading from
 * Returns           : -1 on error and 0 on success
 * Globals Used      : buffer
 * Globals Modified  : buffer
 * Description       : Read the ini file to determine all valid sections and keys.  Also stores the location of
 *                   : the keys data for faster accessing.
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
int __ini_process (ini_t *ini, FILE *file)
{
    char  *current, ch;
    long   pos, first, last;
    size_t count;
    bool   inSection, findNewline, newline, isEOF;
    struct key_tag *key = NULL;

    if (!ini)
        return -1;
    if (!file)
        return -1;

    // Clear out an existing ini structure
    __ini_delete (ini);
    pos   =  0;
    first = -1;
    last  = -1;
    findNewline = false;
    newline     = true;
    inSection   = false;
    isEOF       = false;

    for(;;)
    {
        fseek (file, pos, SEEK_SET);
        count   = fread (buffer, sizeof(char), sizeof (buffer), file);
        current = buffer;

        if (count <= 0)
        {
            if (feof (file))
            {
                count  = 1;
               *buffer = '\x1A';
            }
        }

        while (count--)
        {
            ch = *current++;
            switch (ch)
            {
            // Check for newline or end of file
            case '\x1A':
                isEOF = true;
                // Deliberate run on
            case '\n': case '\r': case '\f':
                inSection   = false;
                newline     = true;
                first       = -1;
                last        = -1;
                findNewline = false;
            goto __ini_processDataEnd;

            // Check for a comment
            case ';':
            case '#':
                findNewline = true;
            __ini_processDataEnd:
                // Now know keys data length
                if (key)
                {   key->length = (size_t) (pos - key->pos);
                    key         = NULL;
                }
            break;

            default:
                if (!findNewline)
                {
                    switch (ch)
                    {
                    // Check for key value
                    case '=':
                        if (!inSection)
                        {   // Make sure the key has a string content
                            last = pos;
                            if (first > 0)
                            {
                                if (!ini->selected) // Handle keys which are not in a section
                                {                                   
                                    if (!__ini_faddHeading (ini, file, 0, 0))
                                        goto __ini_processError;
                                }

                                key = __ini_faddKey (ini, file, first, last - first);
                                if (!key)
                                    goto __ini_processError;
                            }
                        }

                        findNewline = true;
                    break;

                    // Check for header (must start far left)
                    case '[':
                        if (newline)
                        {
                            first     = pos + 1;
                            inSection = true;
                        }
                        else
                            findNewline = true;
                    break;

                    // Check for header termination
                    case ']':
                        if (inSection)
                        {
                            last = pos;
                            if (first <= last) // Handle []
                            {
                                if (!__ini_faddHeading (ini, file, first, last - first))
                                    goto __ini_processError;
                            }
                        }
                        findNewline = true;
                    break;

                    default:
                        if (newline)
                            first = pos;
                    break;
                    }

                    newline = false;
                }
            break;
            }

            // Rev 1.1 Added - Exit of EOF
            if (isEOF)
                break;

        fputc (ch, ini->ftmp);
            pos++;
            if (!pos)
            {
                printf ("INI file is too large\n");
                __ini_delete (ini);
                return -1;
            }
        }

        // Exit of EOF
        if (isEOF)
            break;
    }
    return 0;

__ini_processError:
    __ini_delete (ini);
    return -1;
}


/********************************************************************************************************************
 * Function          : __ini_store
 * Parameters        : ini - pointer to ini file database,  file - ini file to read heading from
 * Returns           : -1 on error and 0 on success
 * Globals Used      :
 * Globals Modified  :
 * Description       : Writes a new ini file containing all the necessary changes
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
int __ini_store (ini_t *ini, FILE *file)
{
    struct section_tag *current_h, *selected_h;
    struct key_tag     *current_k, *selected_k;
    char  *str = NULL;
    size_t length = 0, equal_pos = 0;
    int    ret    = -1;

    if (!ini)
        return -1;
    if (!file)
        return -1;
    
    // Backup selected heading and key
    selected_h = ini->selected;
    selected_k = selected_h->selected;

    current_h = ini->first;
    while (current_h)
    {
        // Output section heading
        if (*current_h->heading)
        {
            if (fprintf (file, "[%s]\n", current_h->heading) < 0)
                goto __ini_storeError;
        }
        
        // Output the sections keys
        equal_pos = __ini_averageLengthKey (current_h);
        current_k = current_h->first;
        while (current_k)
        {
            if (((current_k->length + 1) > length) || !str)
            {   // Need more space
                if (str)
                    free (str);
                length = current_k->length + 1;
                str    = (char *) malloc (sizeof(char) * length);
                if (!str)
                    goto __ini_storeError;
            }

            {   // Output key
                char format[10];
                // Rev 1.1 Added - to support lining up of equals characters
                sprintf (format, "%%-%lus=", (unsigned long) equal_pos);
                if (fprintf (file, format, current_k->key) < 0)
                    goto __ini_storeError;
            }

            // Output keys data (point to correct keys data)
            ini->selected       = current_h;
            current_h->selected = current_k;
            if (ini_readString ((ini_fd_t) ini, str, length) < 0)
                goto __ini_storeError;

            if (fprintf (file, "%s\n", str) < 0)
                goto __ini_storeError;

            current_k = current_k->pNext;
        }

        current_h = current_h->pNext;
        if (fprintf (file, "\n") < 0)
            goto __ini_storeError;
    }
    ret = 0;

__ini_storeError:
    if (str)
        free (str);
    // Restore selected heading and key
    ini->selected           = selected_h;
    ini->selected->selected = selected_k;
    return ret;
}



//********************************************************************************************************************
//********************************************************************************************************************
// User INI File Manipulation Functions
//********************************************************************************************************************
//********************************************************************************************************************


/********************************************************************************************************************
 * Function          : ini_open
 * Parameters        : name - ini file to create
 * Returns           : Pointer to ini database.
 * Globals Used      :
 * Globals Modified  :
 * Description       : Opens an ini data file and reads it's contents into a database
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
ini_fd_t INI_LINKAGE ini_open (const char *name, const char *mode)
{
	ini_mode_t _mode;
	if (!mode)
		return NULL;
	// Convert mode
    switch (*mode)
	{
	case 'r': _mode = INI_READ;  break;
	case 'w': _mode = INI_NEW;   break;
	case 'a': _mode = INI_EXIST; break;
    default: return NULL;
	}
    return (ini_fd_t) __ini_open (name, _mode);
}


/********************************************************************************************************************
 * Function          : ini_close
 * Parameters        : ini - pointer to ini file database.
 * Returns           : -1 on error, 0 on success
 * Globals Used      :
 * Globals Modified  :
 * Description       : Call close, but make sure ini object IS deleted
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
int INI_LINKAGE ini_close (ini_fd_t fd)
{
    return __ini_close ((ini_t *) fd, true);
}


/********************************************************************************************************************
 * Function          : ini_flush
 * Parameters        : ini - pointer to ini file database.
 * Returns           : -1 on error, 0 on success
 * Globals Used      :
 * Globals Modified  :
 * Description       : Call close, but make sure ini object IS NOT deleted
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
int INI_LINKAGE ini_flush (ini_fd_t fd)
{
    return __ini_close ((ini_t *) fd, false);
}


/********************************************************************************************************************
 * Function          : ini_dataLength
 * Parameters        : ini - pointer to ini file database.  heading - heading name.  key - key name
 * Returns           : Number of bytes to read the keys data in as a string.  1 must be added to this length
 *                   : to cater for a NULL character.
 * Globals Used      :
 * Globals Modified  :
 * Description       : Number of bytes to read the keys data in as a string
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
int INI_LINKAGE ini_dataLength (ini_fd_t fd)
{
    ini_t *ini = (ini_t *) fd;
    struct key_tag *_key;
    if (!ini)
        return -1;

    // Check to make sure a section/key has
    // been asked for by the user
    if (!ini->selected)
        return -1;
    _key = ini->selected->selected;
    if (!_key)
        return -1;

#ifdef INI_ADD_LIST_SUPPORT
    if (ini->listDelims)
        return __ini_listIndexLength (ini);
#endif
    return (int) _key->length;
}


/********************************************************************************************************************
 * Function          : ini_delete
 * Parameters        : ini - pointer to ini file database.
 * Returns           :
 * Globals Used      :
 * Globals Modified  :
 * Description       : Deletes the whole ini database from memory, but leaves the ini stub so the ini_close can be
 *                   : called.
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
extern "C" int INI_LINKAGE ini_delete (ini_fd_t fd)
{
    ini_t *ini = (ini_t *) fd;
    if (!ini)
      return -1;
    __ini_delete (ini);
    return 0;
}


#ifdef INI_ADD_EXTRAS

/********************************************************************************************************************
 * Function          : ini_append
 * Parameters        : fdsrc - pointer to src ini file database to copy from.
 *                   : fddst - pointer to dst ini file database to copy to.
 * Returns           :
 * Globals Used      :
 * Globals Modified  :
 * Description       : Copies the contents of the src ini to the dst ini.  The resulting ini contains both
 *                   : headings and keys from each.  Src keys will overwrite dst keys or similar names.
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
extern "C" int INI_LINKAGE ini_append (ini_fd_t fddst, ini_fd_t fdsrc)
{
    struct section_tag *current_h;
    struct key_tag     *current_k;
    struct section_tag *src_h, *dst_h;
    struct key_tag     *src_k, *dst_k;
    char  *data   = NULL;
    int    length = 0, ret = -1;

#ifdef INI_ADD_LIST_SUPPORT
    char  *delims;
#endif

    ini_t *src = (ini_t *) fdsrc;
    ini_t *dst = (ini_t *) fddst;
    if (!(src && dst))
      return -1;

    // Backup selected heading and key
    src_h  = src->selected;
    src_k  = src_h->selected;
    dst_h  = dst->selected;
    dst_k  = dst_h->selected;

#ifdef INI_ADD_LIST_SUPPORT
    // Remove delims for proper reads
    delims = src->listDelims;
    src->listDelims = NULL;
#endif

    // Go through the src ini headings
    current_h = src->first;
    while (current_h)
    {   // Locate heading in the dst
        ini_locateHeading (dst, current_h->heading);
        // Go through the src keys under the heading
        src->selected = current_h;
        current_k = current_h->first;
        while (current_k)
        {   // Check if data buffer can hold the key
            int i = current_k->length;
            current_h->selected = current_k;
            if (i > length)
            {   // Make data buffer bigger, with some spare
                length = i + 10;
                if (data != NULL)
                    free (data);
                data = (char *) malloc (sizeof (char) * length);
                if (data == NULL)
                    goto ini_appendError;
            }
            // Locate key in dst ini file
            ini_locateKey (dst, current_k->key);
            // Copy the key from src to dst ini file
            if (ini_readString  (src, data, length) != i)
                goto ini_appendError;
            if (ini_writeString (dst, data) < 0)
                goto ini_appendError;
            // Move to next key
            current_k = current_k->pNext;
        }
        // Move to next heading
        current_h = current_h->pNext;
    }
    ret = 0;

ini_appendError:
    if (data != NULL)
        free (data);

#ifdef INI_ADD_LIST_SUPPORT
    // Restore delims
    src->listDelims = delims;
#endif
    // Restore selected headings and keys
    src->selected   = src_h;
    src_h->selected = src_k;
    dst->selected   = dst_h;
    dst_h->selected = dst_k;
    return ret;
}

#endif // INI_ADD_EXTRAS


// Add Code Modules
// Add Header Manipulation Functions
#include "headings.i"
// Add Key Manipulation Functions
#include "keys.i"
// Add Supported Datatypes
#include "types.i"
// Add List Support
#ifdef INI_ADD_LIST_SUPPORT
#   include "list.i"
#endif // INI_ADD_LIST_SUPPORT
