/***************************************************************************
                          ini.c  -  Reads and writes keys to a
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
// Rev 1.4 - Replaced use of fgetpos with ftell

//*******************************************************************************************************************
// Include Files
//*******************************************************************************************************************
#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#if defined(HAVE_MSWINDOWS) || defined(DLL_EXPORT)
// Support for DLLs
#   define INI_EXPORT __declspec(dllexport)
#endif

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
static ini_t              *__ini_open            (char *name, bool allowNew);
static int                 __ini_close           (ini_t *ini, bool force);
static void                __ini_deleteAll       (ini_t *ini);
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
 * Function          : ini_open
 * Parameters        : name - ini file to parse
 * Returns           : Pointer to ini database.
 * Globals Used      :
 * Globals Modified  :
 * Description       : Opens an ini data file and reads it's contents into a database
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
ini_t *__ini_open (char *name, bool allowNew)
{
    int      ret;
    ini_t   *ini;
    FILE    *file;
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

    // Open backup file
    ini->filename[length - 1] = '~';
    ini->ftmp = fopen (ini->filename, "w");
    if (!ini->ftmp)
        goto ini_openError;
    fclose (ini->ftmp);
    ini->ftmp = fopen (ini->filename, "rb+");
    if (!ini->ftmp)
        goto ini_openError;

    // Open input file
    ini->filename[length - 1] = name[length - 1];
    file = fopen (ini->filename, "rb");
    if (file)
    {   // Process existing ini file
        ret = __ini_process (ini, file);
        fclose (file);
        if (ret < 0)
            goto ini_openError;
    }
    else if (!allowNew)
    {   // File shoudl exist and are not allowed
        // to create a new one so exit
        goto ini_openError;
    }

    // Rev 1.1 Added - Fix chnaged set on
    // open bug
    ini->changed = false;
    return ini;

ini_openError:
    if (ini)
    {
        if (ini->filename)
            free   (ini->filename);
        if (ini->ftmp)
	{   // Close and remove backup file
            fclose (ini->ftmp);
            ini->filename[strlen (ini->filename) - 1] = '~';
            remove (ini->filename);
        }
        free (ini);
    }


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
int __ini_close (ini_t *ini, bool force)
{
    FILE *file;
    int   ret = 0;

    // Open output file
    if (ini->changed)
    {
#ifdef INI_ADD_LIST_SUPPORT
        char *delims;
        // Rev 1.1 Added - Must remove delims before saving
        delims = ini->listDelims;
        ini->listDelims = NULL;
#endif // INI_ADD_LIST_SUPPORT

        // Not point writing an unchanged file
        file = fopen (ini->filename, "w");
        if (!file)
            return -1;

        // Output all new headers and keys
        ret = __ini_store (ini, file);
        fflush (file);
        fclose (file);

        // We may want to give user chance
        // to do something else
        if (!force)
        {
#ifdef INI_ADD_LIST_SUPPORT
            // Rev 1.1 Added - This was only a flush, so lets restore
            // the old delims
            ini->listDelims = delims;
#endif // INI_ADD_LIST_SUPPORT
            return 0;
        }
    }

    // Cleanup
    fclose (ini->ftmp);

    // If no mods were made, delete tmp file
    if (!ini->changed)
    {
        ini->filename[strlen (ini->filename) - 1] = '~';
        remove (ini->filename);
    }

    __ini_deleteAll (ini);
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
 * Function          : __ini_deleteAll
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
void __ini_deleteAll (ini_t *ini)
{
    struct section_tag *current_h, *last_h;
    struct key_tag     *current_k, *last_k;

    // Scroll through all sections deleting them
    current_h = ini->first;
    while (current_h)
    {
        last_h    = current_h;
        current_h = current_h->pNext;

        // Delete Heading
        current_k = last_h->first;
        free (last_h->heading);
        free (last_h);

        // Delete Keys
        while (current_k)
        {
            last_k    = current_k;
            current_k = current_k->pNext;
            free (last_k->key);
            free (last_k);
        }

        ini->first    = NULL;
        ini->selected = NULL;
        ini->last     = NULL;
    }

#ifdef INI_ADD_LIST_SUPPORT
    // Rev 1.1 - Remove buffered list
    if (ini->list)
    {   free (ini->list);
        ini->list = NULL;
    }
#endif // INI_ADD_LIST_SUPPORT

    ini->changed = false;
    return;
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
    __ini_deleteAll (ini);
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
        {   if (feof (file))
            {   count  = 1;
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
                __ini_deleteAll (ini);
                return -1;
            }
        }

        // Exit of EOF
        if (isEOF)
            break;
    }

    return 0;

__ini_processError:
    __ini_deleteAll (ini);
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
    struct section_tag *current_h;
    struct key_tag     *current_k;
    char  *str = NULL;
    size_t length = 0, equal_pos = 0;

    if (!ini)
        return -1;
    if (!file)
        return -1;
    
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

    if (str)
        free (str);
    return 0;

__ini_storeError:
    if (str)
        free (str);
    return -1;
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
 * Description       : Opens a new ini data file
 ********************************************************************************************************************
 *  Rev   |   Date   |  By   | Comment
 * ----------------------------------------------------------------------------------------------------------------
 ********************************************************************************************************************/
ini_fd_t ini_new (char *name)
{
    return (ini_fd_t) __ini_open (name, true);
}


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
ini_fd_t ini_open (char *name)
{
    return (ini_fd_t) __ini_open (name, false);
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
int ini_close (ini_fd_t fd)
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
int ini_flush (ini_fd_t fd)
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
int ini_dataLength (ini_fd_t fd)
{
    ini_t *ini = (ini_t *) fd;
    struct key_tag *_key;
    // Check to make sure a section/key has
    // been asked for by the user
    if (!ini->selected)
        return -1;
    _key = ini->selected->selected;
    if (!_key)
        return -1;

    return (int) _key->length;
}


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
