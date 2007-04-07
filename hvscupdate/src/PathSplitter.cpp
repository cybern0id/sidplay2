#include "PathSplitter.h"

PathSplitter::PathSplitter(const char* pathName) : maxPathLen(maxCharBufferLen)
{
    splitPath.len = 0;

    // Allocate working buffer.
    splitPath.ptr = new char[maxPathLen];
        
    // Length of source path.
    int pathLen = strlen(pathName);
    
    if (pathLen!=0 && splitPath.ptr!=0)
    {    
        // Convert each HVSC-style (!) directory separator into a zero.
        // At the same time save buffer offset to last dir/file name.
        int i = 0;
        int curLastSplitOffset = -1;
        while (pathName[i] != 0)
        {
            char c = pathName[i];
            if (c=='\\' || c=='/')
            {
                c = 0;
                lastSplitOffset = curLastSplitOffset;  // save previous
                curLastSplitOffset = -1;  // allow taking next
            }
            else
            {
                if (curLastSplitOffset < 0)
                    curLastSplitOffset = i;  // save start of file/dir name
            }
            splitPath.ptr[i++] = c;
        }

        // Make sure we take the last file.
        if (curLastSplitOffset > lastSplitOffset)
            lastSplitOffset = curLastSplitOffset;
        
        splitPath.ptr[i] = 0;
        splitPath.len = pathLen;
        
        firstFile();
    }
}

PathSplitter::~PathSplitter(void)
{
    if (splitPath.ptr != 0)
        delete[] splitPath.ptr;
}

const char* PathSplitter::getFile(void)
{
    return splitPath.ptr+splitOffset;
}

bool PathSplitter::isGood(void)
{
    return (splitPath.len!=0);
}

void PathSplitter::firstFile(void)
{
    splitOffset = 0;
    
    // Skip to next file name.
    while ((splitOffset<splitPath.len) && (splitPath.ptr[splitOffset]==0))
    {
        splitOffset++;
    };
}

void PathSplitter::lastFile(void)
{
    splitOffset = lastSplitOffset;
}

bool PathSplitter::nextFile(void)
{
    if (splitPath.ptr[splitOffset] == 0)
        return false;  // no next file
    
    // Skip current file name.
    while ((splitOffset<splitPath.len) && (splitPath.ptr[splitOffset]!=0))
    {
        splitOffset++;
    }

    // Skip to next file name.
    while ((splitOffset<splitPath.len) && (splitPath.ptr[splitOffset]==0))
    {
        splitOffset++;
    };
    
    return (splitOffset<splitPath.len);
}

bool PathSplitter::isLastFile(void)
{
    return (splitOffset==lastSplitOffset);
}

