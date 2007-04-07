#ifndef PathCreator_h

#include <string.h>
#include "max.h"

class PathCreator
{
 private:
    const int maxPathLen;
    char* path;
        
 public:
    PathCreator(void);
    PathCreator(const char*);
    ~PathCreator(void);

    void empty(void);
    bool append(const char*);
    const char* get(void);
    char* getWritable(void);
    bool isEmpty(void)  { return (path[0]==0); }
};

#endif  // PathCreator_h
