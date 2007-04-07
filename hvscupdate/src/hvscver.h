typedef unsigned int HVSCVER;

#define MAKE_HVSCVER( major, minor ) ( ( major * 10 ) + ( minor % 10 ) )
#define HVSCVER_MAJOR( version ) ( version / 10 )
#define HVSCVER_MINOR( version ) ( version % 10 )

extern HVSCVER atohvscver( const char * );
extern void hvscvertoa( char *, HVSCVER );
extern int hvscvercmp( HVSCVER, HVSCVER );
