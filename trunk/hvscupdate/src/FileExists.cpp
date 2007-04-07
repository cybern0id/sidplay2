#include "FileExists.h"

#include "config.h"

#undef HVSC_USE_STAT
#ifdef HAVE_UNISTD_H
#ifdef HAVE_SYS_TYPES_H
#ifdef HAVE_SYS_STAT_H
#define HVSC_USE_STAT
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#endif
#endif
#ifndef HVSC_USE_STAT
#include <stdio.h>
#endif

bool fileExists( const char* fileName )
{
#ifdef HVSC_USE_STAT
    struct stat fileStat;
    return ( stat( fileName, &fileStat ) == 0  &&
         S_ISREG( fileStat.st_mode )  );
#else
    FILE* f = fopen( fileName, "r+b" );
    bool exists = (f!=NULL);
    if ( exists )
        fclose( f );
    return exists;
#endif
}
