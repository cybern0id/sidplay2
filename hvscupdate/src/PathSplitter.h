#ifndef PathSplitter_h

#include <string.h>
#include "max.h"

class PathSplitter
{
    struct charBuffer
    {
        char* ptr;
        int len;
    };
    
 private:
    const int maxPathLen;

    charBuffer splitPath;

    int splitOffset;      // offset to current directory in path
    int lastSplitOffset;  // offset to last directory in path

    bool validInput;
    
 public:
    PathSplitter(const char* pathName);
    ~PathSplitter(void);
    
    const char* getFile(void);
    
    bool isGood(void);
    bool isLastFile(void);
    
    void firstFile(void);
    void lastFile(void);
    bool nextFile(void);
    
};

#endif  // PathSplitter_h
