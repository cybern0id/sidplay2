/***************************************************************************
                          match.cpp  -  Match a tune against the database 
                             -------------------
    begin                : Sun Jan 5 2003
    copyright            : (C) 2003 by Simon White
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

#include <stdio.h>
#include <string.h>
#include <iostream>
using std::cout;
using std::endl;
#include <string.h>
#include <sidplay/SidTune.h>

#define SDEBUG(x)


struct Player
{
  char name[50];
  struct id_t
  {
    bool relative;
	int offset;
    int value;
  } id[20];
  int ids;
  int offset;
  Player *next;
} player[900] = {0};
static int players = 0;
Player *search[0x100] = {0};


static void analyse (const char *file);


int decodeID (Player::id_t *id, char *in, int base = 0)
{
    int pos;
    int offset;
    int count = 0;

decodeID_next:
    pos    = 0;
    offset = 0;

    if (*in++ != '$') // handle white space better
        return 0;
    sscanf (in, "%x%n", &offset, &pos);
    in += pos;
    if (*in++ != ':') // Decode failure
        return 0;

    for (;;)
	{
        int value = 0;
        if (*in++ != '$') // Decode failure
            return count;
        id->offset = offset++;
        sscanf (in, "%x%n", &value, &pos);
        id->value = value;
        in += pos;
        id++;
        count++;

        // Check what we do next
        switch (*in)
		{
        case ':':
            in++;
            break;
        case ',': // New ID field
		  in++;
		  goto decodeID_next;
        default: // End of decoding (\n or \0)
		    goto decodeID_exit;
		}
    }

decodeID_exit:
    return count;
}


int main (int argc, char *argv[])
{
    if (argc < 3)
    {
        printf ("parameter: <database file> <sid file>\n");
        return -1;
    }

    {
        char *ret;
        char buffer[400];
	    FILE *f = fopen (argv[1], "r");
	    fgets (buffer, sizeof (buffer), f);

        do
        {
            if (!strncmp (buffer, "Player {", 8))
            {
                Player *p = &player[players++];
                char *s;
			    fgets (buffer, sizeof (buffer), f);
			    fgets (buffer, sizeof (buffer), f);
			    s = strstr (buffer, "Name=");
			    strcpy (p->name, &s[5]);
			    p->name[strlen(p->name) - 1] = '\0';
			    for (;;)
			    {
                    if (!fgets (buffer, sizeof (buffer), f))
					    break;
                    if (buffer[0] == ';')
                        continue;
                    if (buffer[0] == '}')
					    break;
                    s = strstr (buffer, "ID=");
                    if (s)
					    p->ids += decodeID (&p->id[p->ids], &s[3]);
			    }
		    }
		    ret = fgets (buffer, sizeof (buffer), f);
	    } while (ret);
    }
    SDEBUG ("Players found = " << players << endl);

    {   // setup fast search
        int count[0x100] = {0};

        for (int i =0; i < players; i++)
	    {
		    Player *p = &player[i];
		    int v = 0, c = 99999;

		    // Distrubute players evenly across the search
		    // table
		    for (int j = 0; j < p->ids; j++)
		    {
			    int v2 = p->id[j].value;
			    if ((count[v2] < c) || !v)
			    {
                    // Avoid location 0 if possible as is generally used
                    // as initialisation value for c64 ram
                    if ((v2 != 0) || !v)
				    {
				        v = v2;
				        p->offset = p->id[j].offset;
				    }
			    }
		    }

		    // Insert player in lookup table
		    p->next   = search[v];
		    search[v] = p;
	    }
    }

    analyse (argv[2]);
    return 0;
}


void analyse (const char *file)
{
    static unsigned char ram[0x10000];
    static SidTune tune(0);

    tune.load (file);
    const SidTuneInfo &info = tune.getInfo();
    tune.placeSidTuneInC64mem (ram);
    int load = info.loadAddr;
    int end  = load + info.c64dataLen - 1;

    // Do search
    struct match
    {
        Player *player;
        int    start;
    } match[100];
    int match_count = 0;
    int match_ids   = 0;

    for (int i = load; i < end; i++)
    {
        int v = ram[i];
        Player *p = search[v];

        // Have some players to check?
        while (p)
        {
            int start = i - p->offset;
            if (start >= load)
	        {
	            // Check other ids
                int ids = p->ids;
                if (ids < match_ids)
                    goto next;
	            Player::id_t *id = p->id;
	            for (int j = 0; j < ids; j++)
                {
		            int addr = start + id[j].offset;
		            if (addr > end)
		            goto next;
		            if (ram[addr] != id[j].value)
		            goto next;
	            }

                // Found a better match?
                if (ids > match_ids)
	            {
                    match_count = 0;
                    match_ids = ids;
	            }
                match[match_count].player = p;
                match[match_count].start  = start;
                match_count++;
	        }
        next:
            p = p->next;
        }
    }

    // Display all matches
    for (int i = 0; i < match_count; i++)
        printf ("%s: %s @ $%04x\n", file, match[i].player->name, match[i].start);
}
