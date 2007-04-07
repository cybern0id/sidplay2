#include <stdio.h>
#include "hvscver.h"

HVSCVER atohvscver( const char *s_version )
{
	int n_major;
	unsigned int n_minor;

	n_major=0;
	n_minor=0;
	sscanf(s_version,"%d.%1u",&n_major,&n_minor);
	return MAKE_HVSCVER(n_major,n_minor);
}

void hvscvertoa( char *s_version, HVSCVER hvscver )
{
	sprintf( s_version, "%d.%1u", HVSCVER_MAJOR( hvscver ), HVSCVER_MINOR( hvscver ) );
}

int hvscvercmp( HVSCVER a, HVSCVER b )
{
	if( a < b )
	{
		return -1;
	}
	else if( a > b )
	{
		return 1;
	}
	return 0;
}
